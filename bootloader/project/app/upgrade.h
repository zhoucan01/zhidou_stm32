#ifndef UPGRADE__H
#define UPGRADE__H

#include <stdint.h>
#include "modbus.h"

/* 兼容旧调试接口的分区标志常量；启动不再读取该标志，始终运行 A。 */
#define OTA_APP_AREA_1 1u
#define OTA_APP_AREA_2 2u
#define OTA_APP_AREA_DEFAULT 0xFFu /* 擦除态：选择时等同分区1，不提前改写Flash。 */
#define OTA_HEADER_SIZE 2048u
#define OTA_IMAGE_CAPACITY (56u * 1024u) /* 含版本区、摘要区，不能写入缓冲扇区。 */
#define OTA_FIRMWARE_CAPACITY (OTA_IMAGE_CAPACITY - OTA_HEADER_SIZE)


typedef enum
{
    bootloaderWating = 0,
    bootloaderHandShaking = 1,
    bootloaderUpgrading =2,
    bootloaderIntoApp = 3
}bootloader_state_t;

/* main.c 中定义的共享业务变量，升级模块通过声明访问同一份数据。 */
extern modbus_10_data_t handshake;
extern bootloader_state_t at32bootloader_state;

uint8_t getLastAppArea(void);
int erase_sector_by_number(uint32_t number);
/* 开始或重新开始升级：先备份有效 A 到 B，再擦除 A，成功后清零本次包号/长度/偏移。
 * 成功返回0；失败返回-1，不发送控制成功确认，不启动未完成固件。
 */
int appUpgradeStart(uint32_t key);
void appUpgrade(void);
/* 查询/开始下载/手动回滚共用控制入口；普通上下文调用。 */
void appControl(void);
/* 校验 B 后恢复 A；成功0，失败-1且保留源 B，发送由公共层处理。 */
int appRollback(void);
/* 普通上下文调用；应答未发完或应用向量无效则返回，成功跳转不返回。 */
void intoApp(void);
/* 比较完整固件的 SHA-256 与原签名区域前 32 字节保存的摘要。 */
int verify_firmware(uint32_t app_start_addr, uint32_t fw_len, uint32_t hash_addr);

#endif


