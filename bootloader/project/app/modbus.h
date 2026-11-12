#ifndef BOOTLOADER_MODBUS_H
#define BOOTLOADER_MODBUS_H

#include <stddef.h>
#include <stdint.h>

#define MODBUS_ADDRESS             0x02u
#define MODBUS_FUNCTION_CONTROL    0x10u
#define MODBUS_FUNCTION_DATA       0xAAu
#define MODBUS_REGISTER_START      0x00E6u
#define MODBUS_STATUS_IDLE         0x0001u
#define MODBUS_STATUS_DOWNLOADING  0x0002u
#define MODBUS_STATUS_FLASHING     0x0003u
#define MODBUS_STATUS_FORBIDDEN    0x0000u
#define MODBUS_STATUS_DAMAGED      0x0055u /* 无可启动固件，每100ms主动上报。 */
#define MODBUS_RESPONSE_CAPACITY   14u
#define MODBUS_MAX_PAYLOAD         1024u

/* 本工程固定：普通字段和 CRC32 大端，CRC16 小端；CRC 算法使用本模块实现。
 * 不再保存运行时配置或存储回调；升级、写入和复位由现有业务层负责。
 */

typedef enum
{
  MODBUS_ACK_OK = 0x01,
  MODBUS_ACK_FIRMWARE_CRC = 0x02,
  MODBUS_ACK_RX_TIMEOUT = 0x03, /* 半帧接收静默超时，阈值 3.5 ms。 */
  MODBUS_ACK_TOO_SHORT = MODBUS_ACK_RX_TIMEOUT, /* 保留旧名称兼容，含义已改为超时。 */
  MODBUS_ACK_VERSION = 0x04,
  MODBUS_ACK_KEY = 0x05,
  MODBUS_ACK_SEQUENCE = 0x06,
  MODBUS_ACK_FRAME_CRC = 0x07,
  MODBUS_ACK_TOO_LONG = 0x08,
  MODBUS_ACK_FLASH_WRITE = 0x09 /* 固件写入或分区标志擦写/读回确认失败。 */
} modbus_ack_t;

typedef struct
{
  uint32_t key;    /* 已接受的会话密钥，供握手查询应答使用。 */
  uint16_t status; /* AT32 本地 OTA 状态，与 230 寄存器及查询应答共用。 */
} modbus_t;

/* 0x10 查询/控制帧字段；is_query 非零表示查询，此时无操作码，operation 为 0。
 * 此处读取的是主机发送值，不代表从机本地状态或已接受的操作。
 */
typedef struct
{
  uint16_t register_start; /* 寄存器起始地址，本协议为 0x00E6（十进制 230）。 */
  uint16_t register_count; /* 寄存器数量：查询为 3，控制写入为 4；每个寄存器占 2 字节。 */
  uint16_t ota_status;     /* 主机请求中携带的状态字段，仅留存；不能覆盖 AT32 本地 OTA 状态。 */
  uint32_t key;            /* 请求携带的 4 字节会话密钥，协议约定为时间戳。 */
  uint16_t operation;      /* 控制操作码：0 无操作，1 开始 OTA，2 重启，3 手动回滚；查询帧无此字段，置 0。 */
  uint8_t is_query;        /* 帧类型标记：1 表示 14 字节查询帧，0 表示 20 字节控制写入帧。 */
  uint16_t received_crc16; /* 收到的 CRC16 字段；DMA/IDLE 接收中断负责验证，此处保留接收值。 */
} modbus_10_data_t;

/* 0xAA 数据帧字段；固件内容通过单独的调用方缓冲区返回。 */
typedef struct
{
  uint16_t data_length;    /* 整帧长度 L，包含地址、帧头和两个 CRC 字段，等于 L1+22。 */
  uint16_t protocol;       /* 主机发送的协议号，取数接口仅保存，不检查是否为支持的版本。 */
  uint32_t key;            /* 数据包携带的 4 字节会话密钥，应与开始 OTA 时的密钥一致。 */
  uint16_t firmware_length; /* 帧内独立的固件数据长度 L1，偏移 10，占 2 字节。 */
  uint16_t total_packets;  /* 本协议表示末包包号，即包数减1；不在首包锁定该字段。 */
  uint16_t packet;         /* 当前数据包的包号；从 0 或 1 开始由收发双方约定。 */
  size_t received_frame_size; /* 解析窗口提供的帧字节数；当前窗口按 L 切分，不是独立物理帧长。 */
  size_t firmware_size;    /* 从实际帧窗口长度减 22 计算的固件字节数，也是实际复制长度。 */
  uint32_t received_crc32; /* 收到的 CRC32 字段，仅保存。 */
  uint16_t received_crc16; /* 收到的 CRC16 字段；DMA/IDLE 接收中断负责验证。 */
} modbus_aa_data_t;

/* 在 data 的 *cursor 位置起扫描同一接收段。
 * 成功返回 1，frame 和 frame_size 输出帧头和长度符合格式的帧的地址及长度，cursor 前进到帧尾；
 * 无完整有效帧返回 0，参数错误返回 -1。返回的指针依赖输入数据的生命周期。
 * 此函数不执行业务、不发送应答，也不跨接收段保留半帧。
 */
int modbus_find_frame(const modbus_t *ctx, const uint8_t *data, size_t size,
                      size_t *cursor, const uint8_t **frame, size_t *frame_size);

/* 读取单个 0x10 帧字段，只检查帧格式和长度，不计算接收 CRC，不改变会话或执行业务。
 * 成功返回 1，参数无效或帧非法返回 -1；失败时不修改 out。
 */
int modbus_read_10(const modbus_t *ctx, const uint8_t *frame, size_t size,
                    modbus_10_data_t *out);

/* 读取单个 0xAA 帧并复制固件，不改变会话，也不校验业务层密钥/包序。
 * firmware 为调用方缓冲区，capacity 为可写字节数；成功返回 1。
 * 参数、帧格式或容量无效返回 -1；接收 CRC 字段仅保存；失败时不修改输出。
 * firmware 可等于 frame+16，此时原地解析，不复制固件；其他重叠方式不支持。
 */
int modbus_read_aa(const modbus_t *ctx, const uint8_t *frame, size_t size,
                    modbus_aa_data_t *out, uint8_t *firmware, size_t capacity);

/* 计算 size 字节数据的 CRC16 数值，不负责校验字段的发送字节序。 */
uint16_t modbus_crc16(const uint8_t *data, size_t size);
/* 使用硬件 CRC 计算 size 字节的 CRC32/ISO-HDLC，调用前完成 wk_crc_init()。
 * 每次计算独立复位，逐字节输入，末尾不补零；不负责线上 CRC 字节序。
 * 不可与其他使用 CRC 外设的代码并发调用；接收 ISR 不自动调用本函数。
 */
uint32_t modbus_crc32(const uint8_t *data, size_t size);

/* 普通上下文周期调用的公共处理入口：执行有效重启请求、发送 CRC16 错误应答。
 * 不消费正常 0x10/0xAA 队列，不改变现有 OTA 状态机；后续通用操作在此扩展。
 * 包含系统复位和阻塞串口发送，禁止在中断中调用。
 */
void modbusdeal(void);
/* 设置待发送的 0xAA 应答，不立即操作串口，由 modbusdeal() 统一发送。
 * code 为 0x01～0x09，packet 为对应包号。1 登记成功，0 上一条尚未发送，-1 参数无效。
 * 仅由普通业务上下文调用；失败发送保留重试，成功发送后清除。
 */
int modbus_set_ack(uint8_t code, uint16_t packet);
/* 登记 0x10 应答：is_query=1 查询状态，0 控制写入确认；由 modbusdeal 发送。
 * 返回 1 成功、0 上一条控制应答未发送；不执行操作码。
 */
int modbus_set_10_reply(uint8_t is_query);
/* 是否仍有业务/控制应答待发送；业务取下一包或跳转前用于防止丢失应答。 */
int modbus_reply_pending(void);
/* 组装查询（14 字节）或控制确认（8 字节）；失败返回 -1。 */
int modbus_build_10_reply(const modbus_t *ctx, uint8_t is_query, uint8_t *out, size_t capacity);
/* 仅对本包 firmware[0..size-1] 计算硬件 CRC32 并比较，不包含协议头或长度字段。
 * 返回1通过、0不匹配、-1参数无效；只在普通业务上下文调用。
 */
int modbus_check_aa_crc32(const modbus_aa_data_t *packet,
                          const uint8_t *firmware, size_t size);

/* 只组装 7 字节 0xAA 应答并生成 CRC16，不发送、不执行业务。
 * ack 为协议应答码 0x01～0x09，packet 为本次应答包号。
 * 成功返回 7，参数/容量无效返回 -1，输出至少 7 字节。
 */
int modbus_build_aa_reply(const modbus_t *ctx, modbus_ack_t ack, uint16_t packet,
                          uint8_t *out, size_t capacity);

#endif
