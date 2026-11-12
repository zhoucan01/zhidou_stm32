#include "upgrade.h"
#include "modbus_port.h"
#include "at32f415_wk_config.h"
#include "at32f415.h"
#include "at32f415_usart.h"
#include "at32f415_flash.h"
#include <stdint.h>
#include <string.h>
#include "sha256.h"
#include "boot_log.h"
//flash分区说明
//0-12 ：bootloader
//13 : ota-data
//14-15: app1 data
//16-69: app1 code
//70   : app1 buffer
//71-72：app2 data
//73-126:app2 code
//127   :app2 buffer

#define APPADDRFLAG	 0x8003400U //旧版分区选择标志，仅保留调试读取，不再用于启动选择

#define MSP1_ADDR	0x8004000U
#define APP1_ADDR	(MSP1_ADDR+4U)

#define MSP2_ADDR	0x8012400U
#define APP2_ADDR	(MSP2_ADDR+4U)

#define APP1DATA1   0X08003800U//存放版本号
#define APP1DATA2   0X08003C00U//存放固件的32字节SHA-256摘要，其余字节保持原区域布局

#define APP2DATA1   0X08011C00U//存放版本号
#define APP2DATA2   0X08012000U//存放固件的32字节SHA-256摘要，其余字节保持原区域布局

uint32_t appMSP ;
uint32_t appEntry;

uint32_t binSize;//记录总字节数 最大4GB

const int protocolVersion =1;

modbus_aa_data_t packet;
 uint8_t firmware[1024];
static int32_t writeStartAddr;
uint32_t totolAddr=0;

/**
 * @brief 清理当前已经处理完的包缓存，避免下一次误用上包字段或固件尾部。
 * @note 必须先登记应答并完成本包的偏移、包号和结束判断，再调用本函数。
 *       不清理 currentPackage、totolAddr 或握手密钥，它们属于整个升级会话。
 */
static void clearCurrentPacket(void)
{
    memset(firmware, 0, sizeof(firmware));
    memset(&packet, 0, sizeof(packet));
}

//返回上一次OTA升级用的是分区1还是分区2
//返回原始标志：1表示分区1，2表示分区2；0xFF默认按分区1选择，不改写Flash
uint8_t getLastAppArea(void)
{
    uint8_t appFlag = *(volatile uint8_t *)APPADDRFLAG;
    return appFlag;
}


int erase_sector_by_number(uint32_t number)
{
    flash_status_type status;
    uint32_t address;

    if(number < 13U || number >= 128U)
        return -1;  /* 越界或属于 Bootloader */

    address = 0x08000000U + number * 1024U;
    boot_notice("Erase sector start", number, BOOT_LOG_NONE);

    flash_unlock();
    flash_flag_clear(FLASH_PRGMERR_FLAG | FLASH_EPPERR_FLAG | FLASH_ODF_FLAG);
    status = flash_sector_erase(address);
    flash_lock();

    boot_notice(status == FLASH_OPERATE_DONE ? "Erase sector OK" : "Erase sector FAILED",
                number, status);
    boot_log("ERASE sec/rc", number, status);
    return (status == FLASH_OPERATE_DONE) ? 0 : -1;
}


static volatile uint16_t currentPackage;//当前包号


/* 长度记录放在原缓冲扇区，先写长度，最后写有效标记。
 * 旧版镜像没有长度记录，不能仅凭向量表认定其通过 SHA-256 校验。 */
#define APP_A_RECORD 0x08011800u
#define APP_B_RECORD 0x0801FC00u
#define IMAGE_VALID 0x53484132u

static uint32_t flashReadWord(uint32_t address)
{
    return *(volatile uint32_t *)address;
}

static int programBytes(uint32_t address, const uint8_t *data, uint32_t size)
{
    uint32_t i;
    int ok = 1;
    flash_unlock();
    flash_flag_clear(FLASH_PRGMERR_FLAG | FLASH_EPPERR_FLAG | FLASH_ODF_FLAG);
    for(i = 0; i < size; ++i)
        if(flash_byte_program(address + i, data[i]) != FLASH_OPERATE_DONE ||
           *(volatile uint8_t *)(address + i) != data[i])
        {
            ok = 0;
            break;
        }
    flash_lock();
    return ok;
}

/* 提交前必须已擦除记录扇区；有效标记最后写，断电留下的半成品不会启动。 */
static int commitImage(uint32_t record, uint32_t length)
{
    uint32_t magic = IMAGE_VALID;
    boot_log("COMMIT", record, length);
    return programBytes(record + 4u, (const uint8_t *)&length, 4u) &&
           programBytes(record, (const uint8_t *)&magic, 4u);
}

/** 校验指定区域的长度记录及原始 BIN 的 SHA-256；B 中保存的仍是链接到 A 的固件。 */
static int imageValid(uint32_t start, uint32_t record)
{
    uint32_t length = flashReadWord(record + 4u);
    uint32_t stack = flashReadWord(start + OTA_HEADER_SIZE);
    uint32_t entry = flashReadWord(start + OTA_HEADER_SIZE + 4u);
    boot_log("CHECK addr/len", start, length);
    /* 无效向量及记录由CHECK失败判断，完整向量在跳转前输出。 */
    /* 两区保存的都是链接到 A 的 BIN，B 的向量也必须指向 A。 */
    if(stack <= 0x20000000u || stack > 0x20008000u || (stack & 7u) ||
       !(entry & 1u) || (entry & ~1u) < MSP1_ADDR + 8u ||
       length > OTA_FIRMWARE_CAPACITY || (entry & ~1u) >= MSP1_ADDR + length)
        return 0;
    return flashReadWord(record) == IMAGE_VALID && length >= 8u &&
           length <= OTA_FIRMWARE_CAPACITY &&
           verify_firmware(start + OTA_HEADER_SIZE, length, start + 1024u);
}

/* 先使目标无效，再擦除和复制；源分区始终不动，复制完成后才提交有效记录。 */
static int eraseImage(uint32_t start, uint32_t record)
{
    uint32_t sector;
    boot_notice(start == APP1DATA1 ? "Erase A start" : "Erase B start",
                BOOT_LOG_NONE, BOOT_LOG_NONE);
    if(erase_sector_by_number((record - 0x08000000u) / 1024u) != 0)
        return 0;
    for(sector = (start - 0x08000000u) / 1024u;
        sector < (start - 0x08000000u) / 1024u + 56u; ++sector)
        if(erase_sector_by_number(sector) != 0)
            return 0;
    boot_notice(start == APP1DATA1 ? "Erase A complete" : "Erase B complete",
                BOOT_LOG_NONE, BOOT_LOG_NONE);
    return 1;
}

static int copyImage(uint32_t source, uint32_t sourceRecord,
                     uint32_t target, uint32_t targetRecord)
{
    uint32_t length;
    if(!imageValid(source, sourceRecord))
        return 0;
    boot_notice(source == APP1DATA1 ? "Backup A to B" : "Restore B to A", BOOT_LOG_NONE, BOOT_LOG_NONE);
    boot_log("COPY src/dst", source, target);
    length = flashReadWord(sourceRecord + 4u);
    if(!eraseImage(target, targetRecord) ||
       !programBytes(target, (const uint8_t *)source, OTA_HEADER_SIZE + length) ||
       !verify_firmware(target + OTA_HEADER_SIZE, length, target + 1024u))
        return 0;
    return commitImage(targetRecord, length);
}

/** 手动回滚：先验证 B，再覆盖 A；B 无效时绝不擦除 A。成功后等待控制应答再启动。 */
int appRollback(void)
{
    int ok;
    modbus_port_set_ota_status(MODBUS_STATUS_FLASHING);
    modbus_port_pause_rx();
    ok = copyImage(APP2DATA1, APP_B_RECORD, APP1DATA1, APP_A_RECORD);
    modbus_port_resume_rx();
    at32bootloader_state = ok ? bootloaderIntoApp : bootloaderHandShaking;
    /* 备份有效但恢复写入失败属于 Flash 故障，不误报“双固件损坏”。 */
    modbus_port_set_ota_status(ok ? MODBUS_STATUS_IDLE :
        (imageValid(APP1DATA1, APP_A_RECORD) ? MODBUS_STATUS_IDLE :
         imageValid(APP2DATA1, APP_B_RECORD) ? MODBUS_STATUS_FORBIDDEN : MODBUS_STATUS_DAMAGED));
    boot_notice(ok ? "Rollback OK" : "Rollback FAILED", BOOT_LOG_NONE, BOOT_LOG_NONE);
    boot_log("ROLLBACK rc", ok, modbus_port_get_ota_status());
    return ok ? 0 : -1;
}

/** 固定 A 运行、B 备份：只有备份校验成功后才能擦除 A。
 * A 已损坏时保留现有 B，不用损坏的 A 覆盖唯一有效备份；双坏仍允许重新下载。 */
int appUpgradeStart(uint32_t key)
{
    int ok = 1;
    boot_notice("Preparing OTA", BOOT_LOG_NONE, BOOT_LOG_NONE);
    at32bootloader_state = bootloaderHandShaking;
    modbus_port_set_ota_status(MODBUS_STATUS_FLASHING);
    modbus_port_pause_rx();
    if(imageValid(APP1DATA1, APP_A_RECORD))
        ok = copyImage(APP1DATA1, APP_A_RECORD, APP2DATA1, APP_B_RECORD);
    if(ok)
        ok = eraseImage(APP1DATA1, APP_A_RECORD);
    modbus_port_resume_rx();
    if(!ok)
    {
        modbus_port_set_ota_status(MODBUS_STATUS_FORBIDDEN);
        return -1;
    }
    writeStartAddr = APP1DATA1;
    currentPackage = 0;
    totolAddr = binSize = 0;
    clearCurrentPacket();
    handshake.key = key;
    modbus_port_set_session_key(key);
    modbus_port_set_ota_status(MODBUS_STATUS_DOWNLOADING);
    at32bootloader_state = bootloaderUpgrading;
    return 0;
}

/** 各状态共用控制处理，操作码3为手动回滚；所有发送仍交给 modbusdeal。 */
void appControl(void)
{
    modbus_10_data_t request;
    if(modbus_reply_pending() || modbus_port_get_10(&request) != 1)
        return;
    boot_log("CMD query/op", request.is_query, request.operation);
    if(request.is_query)
    {
        if(at32bootloader_state == bootloaderWating)
            at32bootloader_state = bootloaderHandShaking;
        (void)modbus_set_10_reply(1);
    }
    else if(request.operation == 1u)
    {
        if(appUpgradeStart(request.key) == 0)
            (void)modbus_set_10_reply(0);
        else
            (void)modbus_set_10_reply(1); /* 失败回传当前状态，不伪造控制成功。 */
    }
    else if(request.operation == 3u)
    {
        if(appRollback() == 0)
            (void)modbus_set_10_reply(0);
        else
            (void)modbus_set_10_reply(1);
    }
    else if(request.operation == 0u)
        (void)modbus_set_10_reply(0);
}

void appUpgrade(void)
{
    uint32_t calculated_crc32;
    /* 上一条应答未发送成功时不取下一包，避免覆盖待发应答。 */
    if(modbus_reply_pending())
        return;
    appControl();
    if(at32bootloader_state != bootloaderUpgrading || modbus_reply_pending())
        return;
    if(modbus_port_get_aa(&packet, firmware, sizeof(firmware)) == 1)
    {
        boot_log("PKT no/last", packet.packet, packet.total_packets);
        boot_log("LEN frame/data", packet.received_frame_size, packet.firmware_size);
        if(packet.protocol != protocolVersion)//验证协议版本
        {
            //返回04
            (void)modbus_set_ack(0x04, packet.packet);
            clearCurrentPacket();
            return;
        }
        if(packet.key != handshake.key)//验证密钥
        {
            //返回05
            (void)modbus_set_ack(0x05, packet.packet);
            clearCurrentPacket();
            return;
        }
        if(packet.packet != currentPackage || packet.packet > packet.total_packets)//核验包号，total仍为末包包号
        {
            //返回06
            (void)modbus_set_ack(0x06, packet.packet);
            clearCurrentPacket();
            return;
        }
        calculated_crc32 = modbus_crc32(firmware, packet.firmware_size);
        boot_log("CRC32 rx/calc", packet.received_crc32, calculated_crc32);
        if(calculated_crc32 != packet.received_crc32)
        //验证crc32，只计算固件
        {
            //返回02
            (void)modbus_set_ack(0x02, packet.packet);
            clearCurrentPacket();
            return;
        }
        if(packet.firmware_length != packet.firmware_size)//验证固件长度
        {
            //返回08
            (void)modbus_set_ack(0x08, packet.packet);
            clearCurrentPacket();
            return;
        }
        /* 单包长度正确不代表整镜像不越界；用减法判断，避免加法溢出。 */
        if(totolAddr > OTA_IMAGE_CAPACITY || packet.firmware_size > OTA_IMAGE_CAPACITY - totolAddr ||
           (packet.packet == packet.total_packets && totolAddr + packet.firmware_size < OTA_HEADER_SIZE + 8u))
        {
            (void)modbus_set_ack(MODBUS_ACK_TOO_LONG, packet.packet);
            clearCurrentPacket();
            return;
        }
        //前面的都过了才会写入
        //使用本次开始OTA时确定并成功擦除的目标，不能写入另一分区。
        boot_log("WRITE addr/len", writeStartAddr + totolAddr, packet.firmware_size);
        flash_unlock();                         /* 解锁 */
        flash_flag_clear(FLASH_PRGMERR_FLAG | FLASH_EPPERR_FLAG | FLASH_ODF_FLAG);
        for(size_t i=0;i<packet.firmware_size;i++)
        {
            if(flash_byte_program((writeStartAddr+i)+totolAddr, firmware[i]) != FLASH_OPERATE_DONE)
            {
                flash_lock();
                boot_log("WRITE BAD", writeStartAddr + totolAddr + i, firmware[i]);
                /* 写入失败返回 09，不推进地址偏移或包号，由 modbusdeal 统一发送。 */
                (void)modbus_set_ack(MODBUS_ACK_FLASH_WRITE, packet.packet);
                clearCurrentPacket();
                /* 部分字节可能已写入，不继续本会话；新开始指令会重新擦除。 */
                at32bootloader_state = bootloaderHandShaking;
                modbus_port_set_ota_status(MODBUS_STATUS_IDLE);
                return;
            }
        }
        totolAddr += packet.firmware_size;//地址偏移
        binSize += packet.firmware_size; /* 整包成功后才统一推进长度，失败不重复累计。 */
        flash_lock();   /* 上锁 */

        //这一包写入完成，需求包号加1
        currentPackage++;
        boot_log("WRITE OK", binSize, currentPackage);
        if(currentPackage>packet.total_packets)//下一个需求的包号大于当前的包号就跳出
        {

            boot_log("RX done", binSize, binSize - OTA_HEADER_SIZE);
            /* 原始固件的长度写入独立记录；断电或摘要失败均留待启动恢复。 */
            if(verify_firmware(MSP1_ADDR, binSize - OTA_HEADER_SIZE, APP1DATA2) &&
               !commitImage(APP_A_RECORD, binSize - OTA_HEADER_SIZE))
            {
                (void)modbus_set_ack(MODBUS_ACK_FLASH_WRITE, packet.packet);
                at32bootloader_state = bootloaderIntoApp;
                clearCurrentPacket();
                return;
            }
            binSize=0;
            flash_lock();
            at32bootloader_state = bootloaderIntoApp;//跳转待下一个阶段
        }
        //回传应答码1；长度记录提交出错时上面已经返回09，不覆盖待发应答。
        (void)modbus_set_ack(MODBUS_ACK_OK, packet.packet);
        /* 应答已保存包号，且本包字段已全部使用完毕，现在才能清零缓存。 */
        clearCurrentPacket();
    }
}

/**
 * @brief 检查应用向量：MSP 位于 32 KB SRAM 内且 8 字节对齐，入口为分区内 Thumb 地址。
 * @note 允许初始栈顶等于 SRAM 结束地址 0x20008000，但不允许等于 SRAM 起点。
 */
static int appVectorValid(uint32_t base, uint32_t end, uint32_t stack, uint32_t entry)
{
    if(stack <= 0x20000000u || stack > 0x20008000u || (stack & 7u) != 0)
        return 0;
    if((entry & 1u) == 0 || (entry & ~1u) < base + 8u || (entry & ~1u) >= end)
        return 0;
    return 1;
}

/* 设置 MSP 后不能再访问 Bootloader 的 C 栈。两个参数由 r0/r1 传入，
 * 在汇编桩中完成栈、屏蔽寄存器和执行模式交接，最后直接跳转应用入口。
 * 应用 Reset_Handler 不应返回；LR 置无效地址，避免意外返回旧栈。
 */
#if defined(__CC_ARM)
__asm void appJump(uint32_t stack, uint32_t entry)
{
    MSR MSP, r0
    MOVS r2, #0
    MSR CONTROL, r2
    MSR BASEPRI, r2
    MSR FAULTMASK, r2
    ISB
    LDR r3, =0xFFFFFFFF
    MOV lr, r3
    CPSIE i
    BX r1
}
#else
__attribute__((naked, noreturn)) static void appJump(
    uint32_t stack __attribute__((unused)), uint32_t entry __attribute__((unused)))
{
    __asm volatile(
        "msr msp, r0\n"
        "movs r2, #0\n"
        "msr control, r2\n"
        "msr basepri, r2\n"
        "msr faultmask, r2\n"
        "isb\n"
        "ldr r3, =0xFFFFFFFF\n"
        "mov lr, r3\n"
        "cpsie i\n"
        "bx r1\n");
}
#endif

/**
 * @brief 固定从 A 启动：SHA-256 校验失败先从 B 恢复，再交接向量表、中断和栈。
 * @note 只能在主循环/普通线程调用；待发送应答未完成、固件不可恢复或向量无效时直接返回。
 *       不再使用旧分区选择标志；恢复失败留在 Bootloader，应用自行初始化时钟和外设。
 *       应用 SystemInit 必须保留A 区 0x08004000 的 VTOR，不能重设为 Bootloader 向量表。
 */
void intoApp(void)
{
    uint32_t vectorBase, appEnd, groups, i;

    if(__get_IPSR() != 0 || modbus_reply_pending())
        return;

    /* A 校验失败时恢复 B；恢复写入失败不反复擦写，留在 Bootloader 等待指令。 */
    if(!imageValid(APP1DATA1, APP_A_RECORD))
    {
        if(appRollback() != 0)
            return;
    }
    vectorBase = MSP1_ADDR;
    appEnd = 0x08011800u;

    /* 向量表第一个字是初始栈顶，第二个字是 Reset_Handler 地址。 */
    appMSP = *(volatile uint32_t *)vectorBase;
    appEntry = *(volatile uint32_t *)(vectorBase + 4u);
    if(!appVectorValid(vectorBase, appEnd, appMSP, appEntry))
    {
        at32bootloader_state = bootloaderHandShaking;
        modbus_port_set_ota_status(MODBUS_STATUS_DAMAGED);
        return;
    }

    boot_notice("Starting APP in A", BOOT_LOG_NONE, BOOT_LOG_NONE);
    boot_log("JUMP msp/pc", appMSP, appEntry);
    boot_log_flush();
    /* 应答已发送完成，停止 Bootloader 串口和 DMA，释放 RS485 总线。 */
    modbus_port_pause_rx();
    __disable_irq();
    usart_enable(USART3, FALSE);
    gpio_bits_reset(GPIOB, GPIO_PINS_2);
    flash_lock();

    /* 关闭旧 SysTick，清除 SysTick/PendSV 待处理状态。 */
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL = 0;
    SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk | SCB_ICSR_PENDSVCLR_Msk;

    /* 按芯片实际实现的中断组数关闭 NVIC，并清除所有外部待处理中断。 */
    groups = (SCnSCB->ICTR & SCnSCB_ICTR_INTLINESNUM_Msk) + 1u;
    if(groups > 8u)
        groups = 8u;
    for(i = 0; i < groups; ++i)
    {
        NVIC->ICER[i] = 0xFFFFFFFFu;
        NVIC->ICPR[i] = 0xFFFFFFFFu;
    }

    SCB->VTOR = vectorBase;
    __DSB();
    __ISB();
    appJump(appMSP, appEntry); /* 设置应用栈后不再执行普通 C 语句。 */
}

//固件验证函数，计算完整固件的SHA-256，与原签名区域中保存的摘要比较

sha256_ctx my_hasher;

// ================= 2. 固件SHA-256校验函数 =================
/**
 * @param app_start_addr 原始固件起始地址 (如 MSP1_ADDR)
 * @param fw_len         你定义的固件长度变量 (实际写入的数据长度)
 * @param hash_addr      32字节SHA-256摘要存放地址 (如 APP1DATA2，原签名区域)
 * @return 1 成功，0 失败
 */
int verify_firmware(uint32_t app_start_addr, uint32_t fw_len, uint32_t hash_addr)
{
    uint8_t my_hash[32];
    int valid;
    boot_log("SHA addr/len", app_start_addr, fw_len);
    if(fw_len < 8u || fw_len > OTA_FIRMWARE_CAPACITY)
        return 0; /* 防止不完整镜像或异常长度导致Flash越界读取。 */

    // --- 第一步：直接读取 Flash 中的实际固件数据，计算 SHA-256 ---
    sha256_init(&my_hasher);
    // 哈希范围仍为 app_start_addr 起的 fw_len 字节，不使用额外的 256 字节中转区。
    // sha256_update 内部按 64 字节块处理，只读取 Flash，不修改固件内容。
    sha256_update(&my_hasher, (const void *)app_start_addr, fw_len);

    // 计算出最终 32 字节哈希
    sha256_final(&my_hasher, my_hash);

    // --- 第二步：与原签名区域前32字节保存的SHA-256摘要比较 ---
    // 仅校验固件完整性，不进行数字签名或来源认证。
    valid = memcmp(my_hash, (const void *)hash_addr, sizeof(my_hash)) == 0;
    boot_log_hex("SHA stored", (const uint8_t *)hash_addr, sizeof(my_hash));
    boot_log_hex("SHA calc", my_hash, sizeof(my_hash));
    boot_notice(valid ? "SHA256 PASS" : "SHA256 FAIL", BOOT_LOG_NONE, BOOT_LOG_NONE);
    boot_log("SHA ok/addr", valid, hash_addr);
    return valid;
}
