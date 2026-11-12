#include "modbus.h"
#include "modbus_port.h"
#include "wk_crc.h"
#include <string.h>
#include "boot_log.h"

/* 业务只登记应答，串口发送统一在 modbusdeal()，不覆盖尚未发送的上一条。 */
static uint8_t pending_ack_code;
static uint16_t pending_ack_packet;
static uint8_t pending_10_reply;
static uint8_t pending_reset; /* 重启确认发送失败时保留请求，下次重试。 */

/**
 * @brief 按协议固定的大端字节序读取无符号整数，避免未对齐访问。
 * @param p 字段起始地址；调用方保证至少有 width 字节可读。
 * @param width 字段宽度，范围为 1～4 字节。
 * @return 读取到的整数值。
 */
static uint32_t read_value(const uint8_t *p, unsigned int width)
{
  uint32_t value = 0;
  unsigned int i;
  for(i = 0; i < width; ++i)
    value = (value << 8) | p[i];
  return value;
}

/* 接收 CRC16 固定低字节在前，不使用普通字段的大端读取函数。 */
static uint16_t read_crc16(const uint8_t *p)
{
  return (uint16_t)((uint16_t)p[1] << 8 | p[0]);
}

/**
 * @brief 将整数按协议固定的大端字节序写入缓冲区。
 * @param p 输出地址；调用方保证至少有 width 字节可写。
 * @param value 待写数值，仅写入低 width 字节。
 * @param width 字段宽度，范围为 1～4 字节。
 */
static void write_value(uint8_t *p, uint32_t value, unsigned int width)
{
  unsigned int i;
  for(i = 0; i < width; ++i)
  {
    p[width - 1u - i] = (uint8_t)value;
    value >>= 8;
  }
}

/**
 * @brief 计算 CRC16/MODBUS：初值 0xFFFF，反射多项式 0xA001，无最终异或。
 * @param data 原始数据地址；size 非零时必须有效。
 * @param size 参与校验的字节数，不包含待生成的 CRC16 字段。
 * @return CRC16 数值；发送时按固定协议低字节在前，本函数不负责组帧。
 */
uint16_t modbus_crc16(const uint8_t *data, size_t size)
{
  uint16_t crc = 0xFFFFu;
  unsigned int bit;
  size_t i;
  for(i = 0; i < size; ++i)
  {
    crc ^= data[i];
    for(bit = 0; bit < 8; ++bit)
      crc = (uint16_t)((crc >> 1) ^ ((crc & 1u) ? 0xA001u : 0u));
  }
  return crc;
}

/**
 * @brief 使用 AT32 硬件 CRC 计算 CRC32/ISO-HDLC。
 * @param data 原始数据地址；size 非零时必须有效。
 * @param size 参与校验的字节数。
 * @return CRC32 数值，初值及最终异或值均为 0xFFFFFFFF。
 * @note 调用前完成 CRC 时钟使能及 wk_crc_init()；硬件配置为 32 位、
 *       多项式 0x04C11DB7、按字节输入翻转、全字输出翻转。
 *       每次从初值重新计算，按 8 位访问 DT，不补零、不强转输入为字数组。
 *       CRC 外设为共享资源，不可与其他 CRC 硬件计算并发或互相抢占。
 */
uint32_t modbus_crc32(const uint8_t *data, size_t size)
{
  volatile uint8_t *crc_input = (volatile uint8_t *)&CRC->dt;
  size_t i;
  crc_data_reset();
  for(i = 0; i < size; ++i)
    *crc_input = data[i];
  return crc_data_get() ^ 0xFFFFFFFFu;
}

/** @brief 仅计算本包固件内容的硬件 CRC32，不包含任何协议头或长度字段。 */
int modbus_check_aa_crc32(const modbus_aa_data_t *packet,
                          const uint8_t *firmware, size_t size)
{
  if(packet == NULL || firmware == NULL || size != packet->firmware_size ||
     size != packet->firmware_length || size == 0)
    return -1;
  return modbus_crc32(firmware, size) == packet->received_crc32;
}

/**
 * @brief 在应答末尾追加 CRC16。
 * @param ctx 已初始化的协议上下文。
 * @param response 应答缓冲区，必须预留至少 size+2 字节空间。
 * @param size 已组装的应答长度，不包含 CRC16。
 * @return 追加 CRC16 后的完整应答长度。
 */
static int add_crc(const modbus_t *ctx, uint8_t *response, size_t size)
{
  uint16_t crc = modbus_crc16(response, size);
  (void)ctx;
  response[size] = (uint8_t)crc;
  response[size + 1] = (uint8_t)(crc >> 8);
  return (int)(size + 2);
}

/**
 * @brief 组装 7 字节数据应答：地址、功能码、应答码、两字节包号、CRC16。
 * @param ctx 已初始化的协议上下文。
 * @param out 输出缓冲区，至少可容纳 7 字节。
 * @param ack 当前数据包的处理结果。
 * @param packet 本次应答对应的包号。
 * @return 完整应答长度，固定为 7。
 */
static int data_response(const modbus_t *ctx, uint8_t *out,
                         modbus_ack_t ack, uint16_t packet)
{
  out[0] = MODBUS_ADDRESS;
  out[1] = MODBUS_FUNCTION_DATA;
  out[2] = (uint8_t)ack;
  write_value(out + 3, packet, 2);
  return add_crc(ctx, out, 5);
}

/**
 * @brief 按协议组装指定应答码及包号的 0xAA 应答，统一处理字节序和 CRC16。
 * @param ctx 已初始化上下文；ack 为应答码；packet 为对应包号。
 * @param out 输出缓冲区；capacity 为容量，至少 7 字节。
 * @return 7 表示组装成功，-1 表示参数无效；本函数不操作串口。
 */
int modbus_build_aa_reply(const modbus_t *ctx, modbus_ack_t ack, uint16_t packet,
                          uint8_t *out, size_t capacity)
{
  if(ctx == NULL || out == NULL || capacity < 7 ||
     ack < MODBUS_ACK_OK || ack > MODBUS_ACK_FLASH_WRITE)
    return -1;
  return data_response(ctx, out, ack, packet);
}

/** @brief 组装 0x10 应答，查询值来自 AT32 本地会话状态，不使用主机状态字段。 */
int modbus_build_10_reply(const modbus_t *ctx, uint8_t is_query, uint8_t *out, size_t capacity)
{
  size_t size = is_query ? 14u : 8u;
  if(ctx == NULL || out == NULL || capacity < size)
    return -1;
  out[0] = MODBUS_ADDRESS;
  out[1] = MODBUS_FUNCTION_CONTROL;
  write_value(out + 2, MODBUS_REGISTER_START, 2);
  write_value(out + 4, is_query ? 3u : 4u, 2);
  if(is_query)
  {
    write_value(out + 6, ctx->status, 2);
    write_value(out + 8, ctx->key, 4);
  }
  return add_crc(ctx, out, size - 2);
}

/**
 * @brief 根据当前候选帧头推导本项目自定义协议的完整帧长。
 * @param ctx 保留统一内部接口；帧格式固定，不读取运行时配置。
 * @param data 当前扫描位置的数据地址。
 * @param available 从当前位置起实际剩余的字节数。
 * @return 查询帧返回 14，控制帧返回 20，数据帧直接返回整帧长度 L；无法识别返回 0。
 * @note 推导长度不代表已收到完整帧，也不代表 CRC 校验通过。
 */
static size_t request_length(const modbus_t *ctx, const uint8_t *data,
                             size_t available)
{
  uint32_t count;
  (void)ctx;
  if(available < 2 || data[0] != MODBUS_ADDRESS)
    return 0;
  if(data[1] == MODBUS_FUNCTION_CONTROL)
  {
    if(available < 6 || read_value(data + 2, 2) != MODBUS_REGISTER_START)
      return 0;
    count = read_value(data + 4, 2);
    return count == 3 ? 14u : (count == 4 ? 20u : 0u);
  }
  if(data[1] == MODBUS_FUNCTION_DATA && available >= 4)
  {
    count = read_value(data + 2, 2);
    return count >= 23 ? (size_t)count : 0u;
  }
  return 0;
}

/**
 * @brief 从当前接收段扫描下一个有效完整帧，只定位数据，不触发业务回调。
 * @param ctx 本地协议上下文；字段使用固定的大端字节序。
 * @param data 输入数据段；size 为段长。
 * @param cursor 输入起始偏移，输出下次扫描位置。
 * @param frame 输出帧起始地址；frame_size 输出完整帧长。
 * @return 1 为找到有效帧，0 为扫描结束，-1 为参数错误。
 */
int modbus_find_frame(const modbus_t *ctx, const uint8_t *data, size_t size,
                      size_t *cursor, const uint8_t **frame, size_t *frame_size)
{
  size_t length;
  if(ctx == NULL || data == NULL || cursor == NULL || frame == NULL ||
     frame_size == NULL || *cursor > size)
    return -1;
  *frame = NULL;
  *frame_size = 0;
  while(*cursor < size)
  {
    length = request_length(ctx, data + *cursor, size - *cursor);
    if(length != 0 && length <= size - *cursor)
    {
      *frame = data + *cursor;
      *frame_size = length;
      *cursor += length;
      return 1;
    }
    ++*cursor;
  }
  return 0;
}

/**
 * @brief 将完整 0x10 请求解码到调用方结构体，不执行查询、升级或复位动作。
 * @param ctx 本地协议上下文；frame/size 为输入完整帧及长度。
 * @param out 输出字段；查询帧的 operation 固定为 0。
 * @return 1 表示成功，-1 表示参数或帧格式无效。
 */
int modbus_read_10(const modbus_t *ctx, const uint8_t *frame, size_t size,
                    modbus_10_data_t *out)
{
  modbus_10_data_t value;
  if(ctx == NULL || frame == NULL || out == NULL ||
     (size != 14 && size != 20) || frame[0] != MODBUS_ADDRESS ||
     frame[1] != MODBUS_FUNCTION_CONTROL || request_length(ctx, frame, size) != size)
    return -1;
  if(size == 20 && read_value(frame + 6, 4) != 8)
    return -1;
  memset(&value, 0, sizeof(value));
  value.register_start = (uint16_t)read_value(frame + 2, 2);
  value.register_count = (uint16_t)read_value(frame + 4, 2);
  value.is_query = (uint8_t)(size == 14);
  value.ota_status = (uint16_t)read_value(frame + (value.is_query ? 6 : 10), 2);
  value.key = read_value(frame + (value.is_query ? 8 : 12), 4);
  value.received_crc16 = (uint16_t)read_crc16(frame + size - 2);
  if(!value.is_query)
    value.operation = (uint16_t)read_value(frame + 16, 2);
  *out = value;
  return 1;
}

/**
 * @brief 将完整 0xAA 请求解码，并把固件内容复制到调用方缓冲区。
 * @param ctx 本地协议上下文；frame/size 为输入完整帧及长度。
 * @param out 输出协议号、密钥、包数、包号和固件长度。
 * @param firmware 固件输出缓冲区；允许等于 frame+16，此时只解析字段，不重复复制。
 *                 其他情况不可与输入帧或 out 重叠。
 * @param capacity 固件输出容量，至少为独立固件长度 L1（整帧长度 L 减 22）。
 * @return 1 表示成功，-1 表示参数/格式/容量无效；不计算接收 CRC。
 * @note 只检查格式和长度，原样保存收到的 CRC 字段，不校验会话状态、协议号值、密钥或包序。
 */
int modbus_read_aa(const modbus_t *ctx, const uint8_t *frame, size_t size,
                    modbus_aa_data_t *out, uint8_t *firmware, size_t capacity)
{
  modbus_aa_data_t value;
  uint32_t length;
  if(ctx == NULL || frame == NULL || out == NULL || firmware == NULL ||
     size < 23 ||
     frame[0] != MODBUS_ADDRESS || frame[1] != MODBUS_FUNCTION_DATA ||
     request_length(ctx, frame, size) != size)
    return -1;
  length = read_value(frame + 2, 2);
  /* 实际输入窗口、主机声明的整帧 L、固件 L1 三者交叉检查，避免越界复制。 */
  if(size - 22 > MODBUS_MAX_PAYLOAD || size - 22 > capacity ||
     (size_t)read_value(frame + 10, 2) != size - 22)
    return -1;
  memset(&value, 0, sizeof(value));
  value.data_length = (uint16_t)length;
  value.protocol = (uint16_t)read_value(frame + 4, 2);
  value.key = read_value(frame + 6, 4);
  value.firmware_length = (uint16_t)read_value(frame + 10, 2);
  value.total_packets = (uint16_t)read_value(frame + 12, 2);
  value.packet = (uint16_t)read_value(frame + 14, 2);
  value.received_frame_size = size;
  value.firmware_size = size - 22;
  value.received_crc32 = read_value(frame + length - 6, 4);
  value.received_crc16 = (uint16_t)read_crc16(frame + length - 2);
  /* 队列内原地解析时固件已在目标位置；外部取数仍保持复制语义。 */
  if(firmware != frame + 16)
    memcpy(firmware, frame + 16, value.firmware_size);
  *out = value;
  return 1;
}

/**
 * @brief 登记一条业务应答，交给 modbusdeal() 延迟发送。
 * @param code 应答码 0x01～0x09；packet 为该应答对应的数据包号。
 * @return 1 登记成功，0 上一条仍待发送，-1 应答码无效；不执行实际发送。
 */
int modbus_set_ack(uint8_t code, uint16_t packet)
{
  if(code < MODBUS_ACK_OK || code > MODBUS_ACK_FLASH_WRITE)
    return -1;
  if(pending_ack_code != 0)
    return 0;
  boot_log("ACK code/no", code, packet);
  pending_ack_packet = packet;
  pending_ack_code = code;
  return 1;
}

int modbus_set_10_reply(uint8_t is_query)
{
  if(pending_10_reply != 0)
    return 0;
  pending_10_reply = is_query ? 1u : 2u;
  return 1;
}

int modbus_reply_pending(void)
{
  return pending_ack_code != 0 || pending_10_reply != 0 || pending_reset != 0;
}

/**
 * @brief 公共处理：重启先回控制确认，发送完成才复位；其余检测半帧超时、发送接收错误应答及业务应答。
 * @note 应答实际发送成功才清除标记，失败时下次调用继续尝试；禁止中断调用。
 */
void modbusdeal(void)
{
  if(modbus_port_take_reset_request())
  {
    boot_notice("RX: reboot", BOOT_LOG_NONE, BOOT_LOG_NONE);
    pending_reset = 1;
  }
  if(pending_reset)
  {
    /* reply_10 成功表示最后停止位已发完、RS485已释放，不能提前复位截断应答。 */
    if(modbus_port_reply_10(0) == 1)
    {
      pending_reset = 0;
      boot_notice("Reboot ACK sent; resetting", BOOT_LOG_NONE, BOOT_LOG_NONE);
      boot_log("RESET", 2, 0);
      boot_log_flush();
      nvic_system_reset();
    }
    return;
  }
  modbus_port_check_timeout();
  if(modbus_port_reply_errors() != 0)
    return;
  if(modbus_port_reply_version() != 0)
    return;
  if(pending_10_reply != 0)
  {
    if(modbus_port_reply_10(pending_10_reply == 1u) == 1)
      pending_10_reply = 0;
    return;
  }
  if(pending_ack_code != 0 &&
     modbus_port_reply_aa((modbus_ack_t)pending_ack_code, pending_ack_packet) == 1)
    pending_ack_code = 0;
  /* 后续通用协议操作在此扩展。 */
}
