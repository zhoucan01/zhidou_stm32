#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <stdio.h>
#include "upgrade.h"
#include "modbus_port.h"

typedef enum { FLASH_PROGRAM_ERROR=1, FLASH_OPERATE_DONE=3 } flash_status_type;
#define FLASH_PRGMERR_FLAG 4u
#define FLASH_EPPERR_FLAG 16u
#define FLASH_ODF_FLAG 32u
#define BASE 0x08000000u
#define FLAG_ADDR (BASE+0x3400u)
static uint8_t memory[128*1024], incoming[1024];
static modbus_aa_data_t next_packet;
static modbus_10_data_t next_control;
static int packet_ready, control_ready, ack, reply_count, paused, pauses, resumes;
static int erase_count, first_sector, last_sector, fail_sector, fail_after, flag_failure;
static int data_writes, hash_calls, hash_ok, locked, sticky_error;
static uint32_t hash_base, hash_length, hash_addr;
static uint16_t status;
modbus_10_data_t handshake;
bootloader_state_t at32bootloader_state;

static const uint8_t *model_pointer(uint32_t address) { return memory + address - BASE; }
static uint32_t model_word(uint32_t address) { uint32_t value;memcpy(&value,model_pointer(address),4);return value; }
static uint8_t model_read(uint32_t address)
{
    assert(address>=BASE && address<BASE+sizeof(memory));
    return memory[address-BASE];
}
void flash_unlock(void) { locked=0; }
void flash_lock(void) { locked=1; }
void flash_flag_clear(uint32_t flags) { (void)flags;sticky_error=0; }
flash_status_type flash_sector_erase(uint32_t address)
{
    unsigned sector=(address-BASE)/1024;
    assert(!locked && address>=BASE && sector<128 && (paused || sector==13));
    ++erase_count;last_sector=(int)sector;
    if(erase_count==1)first_sector=(int)sector;
    if((int)sector==fail_sector || sticky_error) { sticky_error=1;return FLASH_PROGRAM_ERROR; }
    memset(memory+sector*1024,0xff,1024);
    return FLASH_OPERATE_DONE;
}
flash_status_type flash_byte_program(uint32_t address,uint8_t value)
{
    unsigned offset=address-BASE;
    assert(!locked && address>=BASE && offset<sizeof(memory));
    if(address==FLAG_ADDR)
    {
        if(flag_failure==1) return FLASH_PROGRAM_ERROR;
        if(flag_failure==2) return FLASH_OPERATE_DONE; /* 状态成功但实际未写入。 */
    }
    else
    {
        if(data_writes==fail_after || sticky_error) { sticky_error=1;return FLASH_PROGRAM_ERROR; }
        ++data_writes;
    }
    if((memory[offset]&value)!=value)return FLASH_PROGRAM_ERROR;
    memory[offset]&=value;
    return FLASH_OPERATE_DONE;
}
int modbus_reply_pending(void) { return ack!=0; }
int modbus_set_ack(uint8_t code,uint16_t packet_number)
{
    (void)packet_number;assert(ack==0);ack=code;return 1;
}
int modbus_set_10_reply(uint8_t query) { (void)query;++reply_count;return 1; }
void modbus_port_set_ota_status(uint16_t value) { status=value; }
void modbus_port_set_session_key(uint32_t key) { assert(key==handshake.key); }
void modbus_port_pause_rx(void)
{
    paused=1;++pauses;packet_ready=control_ready=0;
}
void modbus_port_resume_rx(void) { assert(paused);paused=0;++resumes; }
int modbus_port_get_10(modbus_10_data_t *out)
{
    if(!control_ready)return 0;
    *out=next_control;control_ready=0;return 1;
}
int modbus_port_get_aa(modbus_aa_data_t *out,uint8_t *data,size_t capacity)
{
    if(!packet_ready)return 0;
    assert(next_packet.firmware_size<=capacity);
    *out=next_packet;memcpy(data,incoming,next_packet.firmware_size);packet_ready=0;return 1;
}
uint32_t modbus_crc32(const uint8_t *data,size_t size) { (void)data;(void)size;return 0x42; }
int verify_firmware(uint32_t base,uint32_t size,uint32_t digest)
{
    uint32_t sum=0,i;
    assert(size>=8 && size<=OTA_FIRMWARE_CAPACITY);
    ++hash_calls;hash_base=base;hash_length=size;hash_addr=digest;
    for(i=0;i<size;++i)sum+=model_read(base+i);
    return hash_ok && sum==model_word(digest);
}

#include "upgrade_logic_under_test.inc"


static void reset_model(void)
{
    memset(memory,0xff,sizeof(memory));
    memset(&handshake,0,sizeof(handshake));
    packet_ready=control_ready=ack=reply_count=paused=pauses=resumes=0;
    erase_count=first_sector=last_sector=data_writes=hash_calls=flag_failure=sticky_error=0;
    fail_sector=fail_after=-1;hash_ok=locked=1;status=MODBUS_STATUS_IDLE;
    currentPackage=0;totolAddr=binSize=0;writeStartAddr=0;
    at32bootloader_state=bootloaderHandShaking;
}
static void put_word(uint32_t address,uint32_t value) { memcpy(memory+address-BASE,&value,4); }
static void seed_image(uint32_t start,uint32_t record,uint8_t fill)
{
    uint32_t i,sum=0;
    memset(memory+start-BASE,0,OTA_HEADER_SIZE+16);
    memset(memory+start+OTA_HEADER_SIZE-BASE,fill,16);
    put_word(start+OTA_HEADER_SIZE,0x20008000u);
    put_word(start+OTA_HEADER_SIZE+4,MSP1_ADDR+9u);
    for(i=0;i<16;++i)sum+=model_read(start+OTA_HEADER_SIZE+i);
    put_word(start+1024,sum);
    put_word(record+4,16);put_word(record,IMAGE_VALID);
}
static void send_packet(unsigned number,unsigned total,size_t size,const uint8_t *bytes)
{
    assert(ack==0 && size<=sizeof(incoming));
    memcpy(incoming,bytes,size);memset(&next_packet,0,sizeof(next_packet));
    next_packet.protocol=protocolVersion;next_packet.key=handshake.key;
    next_packet.packet=(uint16_t)number;next_packet.total_packets=(uint16_t)total;
    next_packet.firmware_size=next_packet.firmware_length=size;next_packet.received_crc32=0x42;
    packet_ready=1;appUpgrade();
}
int main(void)
{
    uint8_t saved[2064],image[2064];
    reset_model();seed_image(APP1DATA1,APP_A_RECORD,0x11);
    memcpy(saved,model_pointer(APP1DATA1),sizeof(saved));
    assert(appUpgradeStart(123)==0 && writeStartAddr==APP1DATA1);
    assert(imageValid(APP2DATA1,APP_B_RECORD) && !imageValid(APP1DATA1,APP_A_RECORD));
    assert(!memcmp(saved,model_pointer(APP2DATA1),sizeof(saved)) && !paused && pauses==resumes);
    /* 下载中复位：A尚无有效记录，从B恢复后字节和长度一致。 */
    assert(appRollback()==0 && imageValid(APP1DATA1,APP_A_RECORD));
    assert(!memcmp(saved,model_pointer(APP1DATA1),sizeof(saved)));
    /* 备份擦除或复制失败，不擦除仍有效的A。 */
    reset_model();seed_image(APP1DATA1,APP_A_RECORD,0x22);fail_sector=72;
    assert(appUpgradeStart(1)==-1 && imageValid(APP1DATA1,APP_A_RECORD) && !paused);
    reset_model();seed_image(APP1DATA1,APP_A_RECORD,0x22);fail_after=10;
    assert(appUpgradeStart(1)==-1 && imageValid(APP1DATA1,APP_A_RECORD));
    /* A坏、B好时开始新下载必须保留B。 */
    reset_model();seed_image(APP2DATA1,APP_B_RECORD,0x33);
    memcpy(saved,model_pointer(APP2DATA1),sizeof(saved));
    assert(appUpgradeStart(7)==0 && imageValid(APP2DATA1,APP_B_RECORD));
    assert(!memcmp(saved,model_pointer(APP2DATA1),sizeof(saved)));
    /* B坏时手动回滚不可破坏有效A，也不能误报双坏。 */
    reset_model();seed_image(APP1DATA1,APP_A_RECORD,0x44);
    assert(appRollback()==-1 && imageValid(APP1DATA1,APP_A_RECORD) && status==MODBUS_STATUS_IDLE);
    reset_model();assert(appRollback()==-1 && status==MODBUS_STATUS_DAMAGED);
    /* 恢复写入失败：B仍有效，A没有有效标记。 */
    reset_model();seed_image(APP2DATA1,APP_B_RECORD,0x55);fail_after=20;
    assert(appRollback()==-1 && status==MODBUS_STATUS_FORBIDDEN);
    assert(imageValid(APP2DATA1,APP_B_RECORD) && !imageValid(APP1DATA1,APP_A_RECORD));
    /* 新下载只写A，最后一包成功才提交长度记录。 */
    reset_model();seed_image(APP2DATA1,APP_B_RECORD,0x66);
    memcpy(image,model_pointer(APP2DATA1),sizeof(image));
    assert(appUpgradeStart(7)==0);
    send_packet(0,2,1024,image);assert(ack==MODBUS_ACK_OK);ack=0;
    send_packet(1,2,1024,image+1024);assert(ack==MODBUS_ACK_OK);ack=0;
    assert(!imageValid(APP1DATA1,APP_A_RECORD));
    send_packet(2,2,16,image+2048);assert(ack==MODBUS_ACK_OK);
    assert(imageValid(APP1DATA1,APP_A_RECORD) && model_word(APP_A_RECORD+4)==16);
    /* 摘要不符时不提交；后续启动通过备份恢复。 */
    ack=0;assert(appUpgradeStart(7)==0);image[2063]^=1;
    send_packet(0,2,1024,image);ack=0;send_packet(1,2,1024,image+1024);ack=0;
    send_packet(2,2,16,image+2048);assert(!imageValid(APP1DATA1,APP_A_RECORD));
    ack=0;assert(appRollback()==0 && imageValid(APP1DATA1,APP_A_RECORD));
    /* 短末包、乱序、超容量不会写入。 */
    assert(appUpgradeStart(7)==0);send_packet(0,0,16,image);assert(ack==MODBUS_ACK_TOO_LONG);ack=0;
    send_packet(1,2,16,image);assert(ack==MODBUS_ACK_SEQUENCE);ack=0;
    totolAddr=OTA_IMAGE_CAPACITY;send_packet(0,2,16,image);assert(ack==MODBUS_ACK_TOO_LONG);
    /* 操作码3也在升级态生效，控制成功之后才允许跳转。 */
    ack=0;control_ready=1;memset(&next_control,0,sizeof(next_control));next_control.operation=3;
    appUpgrade();assert(reply_count==1 && at32bootloader_state==bootloaderIntoApp);
    puts("OTA safety scenarios passed");return 0;
}
