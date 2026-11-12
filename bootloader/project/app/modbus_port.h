#ifndef BOOTLOADER_MODBUS_PORT_H
#define BOOTLOADER_MODBUS_PORT_H

#include "modbus.h"

/* USART3 DMA+IDLE 接口，实现位于 wk_usart.c 的用户代码区，无需 RTOS。
 * 一帧最多 1024 字节固件加 22 字节协议开销。
 * 队列共 4 槽，提供 3 个完整帧待处理槽和 1 个中断发布槽。
 */
#define MODBUS_PORT_FRAME_SIZE 1046u
#define MODBUS_PORT_QUEUE_DEPTH 4u
#define MODBUS_RX_TIMEOUT_US 3500u /* 半帧最后一次接收活动后的静默超时。 */
/* DMA 单块长度与最大帧长分离，满传输后切换双缓冲，半帧可跨块累计。 */
#define MODBUS_PORT_DMA_SIZE (MODBUS_PORT_FRAME_SIZE + 1u)
extern uint8_t modbus_port_dma_buffer[MODBUS_PORT_DMA_SIZE];

typedef struct
{
  volatile uint32_t queue_overflow;
  volatile uint32_t receive_error;
  volatile uint32_t frame_error;
  volatile uint32_t responses;
  volatile uint32_t transmit_error;
  volatile uint32_t crc16_error;     /* 完整候选帧的 CRC16 错误次数。 */
  volatile uint32_t valid_frames;    /* CRC16 及字段格式通过的完整帧数。 */
  volatile uint32_t register_writes; /* 0x10 写请求成功更新寄存器缓存的次数。 */
  volatile uint32_t error_reply_overflow; /* 接收错误应答队列已满时的丢弃次数。 */
  volatile uint32_t receive_timeout; /* 半帧超过 3.5 ms 未收完整的次数。 */
} modbus_port_stats_t;

extern modbus_port_stats_t modbus_port_stats;

/* 初始化固定协议、本地状态、接收队列和诊断计数，成功返回 0。
 * 在 wk_usart3_init() 前、接收中断未启用时调用。
 * 无配置和回调参数，不决定 Flash 分区或执行业务操作。
 */
int modbus_port_init(void);
/* DMA/USART3 中断共用入口：读取本块后切换双缓冲，在中断内组帧、校验 CRC16，
 * 通过后更新 E7～E9 接收寄存器和完整帧队列；E6 为本地状态，不接受主机覆盖。
 * 两个中断必须同抢占优先级。
 * 调用者不得提前清除 IDLE，禁止使用 RDBF 中断与 DMA 抢读 DT。
 */
void modbus_port_irq(void);
/* 读取保持寄存器快照：0xE6 为 AT32 本地状态，0xE7 密钥高16位，
 * 0xE8 密钥低16位，0xE9 操作码。查询请求不会改写该缓存。
 * 成功返回 1，地址范围/参数错误返回 -1。不会消费接收队列。
 */
int modbus_port_read_registers(uint16_t start, uint16_t count, uint16_t *out);
/* AT32 设置本地 OTA 状态：1 空闲、2 下载中、其他值禁止 OTA。
 * 仅更新 RAM 状态，不执行升级操作；初始化后调用，初始化前设置不生效。
 * 主机查询应答和 230 寄存器均读取这一状态。
 */
void modbus_port_set_ota_status(uint16_t status);
/* 获取本地 OTA 状态，初始化后默认空闲，初始化前返回禁止 OTA。 */
uint16_t modbus_port_get_ota_status(void);
/* 发送一条已登记的 0xAA 接收错误应答：0x03 超时或 0x07 CRC16 错误，附加 CRC16。
 * 普通上下文调用：1 已发送，0 暂无待发送，-1 发送失败并保留待重试。
 * 由 modbusdeal() 统一调度，禁止在中断调用；取数接口不再隐式发送。
 * 0x10 的 CRC 错误不应答，因为协议尚未定义控制错误应答格式。
 */
int modbus_port_reply_errors(void);
/* 检查 3.5 ms 半帧超时，登记 0x03 应答并清理残帧，由 modbusdeal() 调度。
 * 未收齐包号的残帧只能清理，不应答；不在此函数内发送。
 * 必须周期调用，实际处理时刻受调度周期影响。
 */
void modbus_port_check_timeout(void);
/* 发送指定 0xAA 应答码和包号，内部组帧、计算 CRC16 并控制 RS485 方向。
 * 成功返回 1，未初始化/参数无效/发送失败返回 -1。
 * 普通上下文单一发送者调用，不消费接收队列，不自动更新状态或执行重启。
 */
int modbus_port_reply_aa(modbus_ack_t ack, uint16_t packet);
/* 以下为业务和公共发送层的适配接口，发送仍由 modbusdeal 统一调度。 */
int modbus_port_reply_10(uint8_t is_query);
/* 处理已通过 CRC 的 0x11 版本分块查询，普通上下文发送，失败保留重试。 */
int modbus_port_reply_version(void);
void modbus_port_set_session_key(uint32_t key);
int modbus_port_check_aa_crc32(const modbus_aa_data_t *packet, const uint8_t *firmware, size_t size);
/* 普通上下文暂停/恢复接收，用于分区擦除；恢复时丢弃暂停前的半帧。 */
void modbus_port_pause_rx(void);
void modbus_port_resume_rx(void);
/* 原子取走中断登记的有效 0x10 重启请求标记：1 有请求，0 无请求。
 * 仅供 modbusdeal() 使用，不消费正常帧队列。
 */
int modbus_port_take_reset_request(void);
/* 从队列按接收顺序复制中断已经处理好的 0x10 或 0xAA 数据，成功返回 1，暂无匹配数据返回 0。
 * 若队首有效帧是另一功能码，不跳过也不丢弃，请交替调用两个接口。
 * 数据复制到调用方内存，不保留指向 DMA/接收队列的悬空指针。
 * 参数/固件容量错误返回 -1 并保留该帧，可更换缓冲区重试。
 * CRC16 或格式错误的帧已在中断丢弃；取数不重复校验，CRC32 仅保存。
 * 取数不自动应答、不校验会话包序、不写 Flash、不复位，公共处理由 modbusdeal() 调度。
 * 在普通任务/主循环中由单一消费者调用。
 */
int modbus_port_get_10(modbus_10_data_t *out);
int modbus_port_get_aa(modbus_aa_data_t *out, uint8_t *firmware, size_t capacity);


#endif
