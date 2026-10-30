#include "modbusTask.h"

#include "freertos_app.h"
#include "wk_dma.h"
#include "wk_usart.h"
#include "wk_debug.h"
#include "log.h"
#include "AT8236.h"
#include "LED.h"
#include "current.h"
#include "getBusVol.h"
#include "Tempurature.h"
#include "FT.h"
#include "moter28.h"
#include "moter42.h"
#include "moter57.h"
#include "enc.h"
#include "tmux1308.h"
#include "totolCtl.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

/* 寄存器布局严格遵循 站号2_AT32F415CBT7_Modbus协议.md：
 *   线圈    0x0000~0x0009  共 10
 *   离散输入 0x0000~0x000F  共 16
 *   输入寄存器 0x0000~0x0027 共 40（文档标准）+ 0x0028~0x004F 共 40（V2 追加）= 80
 *   保持寄存器 0x0000~0x0066 共 103 */
#define MB_COIL_COUNT              10U
#define MB_DISCRETE_COUNT          16U
#define MB_INPUT_REG_COUNT          80U
#define MB_HOLD_REG_COUNT          103U
#define MB_RX_QUEUE_DEPTH            4U
#define MB_RESPONSE_BUFFER_SIZE    256U
/* Must exactly match wk_usart3_init(): USART3 is 1 Mbit/s, 8N1. */
#define MB_BAUD_RATE           1000000U

/* 抬升"下降一段"触发值：向保持寄存器 0x0003 写入本值与主站 ESP32S3
 * ModbusMap.h 的 MODBUS_RISE_MOTOR_TIMED_DOWN_HOLD 保持一致。写入后触发一次
 * 固定段长下降（段长度写在下位机 moter42.c），不占用独立寄存器。 */
#define MB_RISE_TIMED_DOWN_HOLD  ((int16_t)-1000)

/* 抬升"上行一段"触发值：向保持寄存器 0x0003 写入本值，与主站 ESP32S3
 * ModbusMap.h 的 MODBUS_RISE_MOTOR_TIMED_UP_HOLD 保持一致。写入后触发一次
 * 固定段长"上行一段"（从最低位向上走，段长度写在下位机 moter42.c）。
 * -601 与 -1000 同属魔数：抬升转速合法区间为 ±600rpm，超出必然不是真速度。 */
#define MB_RISE_TIMED_UP_HOLD    ((int16_t)-601)

/* 临时调试开关（2026-10-13）：置 1 时，站2 保持寄存器 0x0002 / 线圈 0x0008 的压制速度按
 * "发多少=多少 rpm" 解释（内部乘 10 转为 rpm×10）；串口上位机输入框 0~1000 即 0~600rpm（>350 段被机械上限压到 350rpm），
 * 超过 350 由 pressMoterUp/Down 按机械上限 PRESS_MOTER_MAX_RPM_X10(=3500) 自动钳位。
 * 置 0 恢复 main 语义：寄存器值本身即转速×10（1000=100.0rpm）。
 * 注意：ESP32 主站全流程自组帧依赖 ×10 语义，回归全流程验证前须改回 0。 */
#define PRESS_CMD_VALUE_IS_RPM     1
#if PRESS_CMD_VALUE_IS_RPM
#define PRESS_CMD_RPM_SCALE        10
#else
#define PRESS_CMD_RPM_SCALE        1
#endif

/* 压制速度低档接口（2026-10-19）：|寄存器值| < 50 时，该值本身即 rpm×10（2026-10-22 边界由 100 收到 50）
 * （0~49 对应 0~4.9rpm，便于上位机细调极低速压制）；
 * |寄存器值| >= 50 保持原语义：值=整数 rpm，内部 ×PRESS_CMD_RPM_SCALE。
 * 例：8→0.8rpm，49→4.9rpm，50→50rpm，1000→350rpm（机械上限钳位）。
 * >1000 不除以 10（仍按 值=rpm 解释后由机械上限钳位，如 2000→350rpm）。 */
#define PRESS_CMD_LOW_RANGE_LIMIT  50
static int pressCmdValueToRpmX10(int16_t v)
{
  if(v > -PRESS_CMD_LOW_RANGE_LIMIT && v < PRESS_CMD_LOW_RANGE_LIMIT)
  {
    return (int)v;
  }
  return (int)v * PRESS_CMD_RPM_SCALE;
}

/* 抬升转速接口（2026-10-19 重新打开）：站2 保持寄存器 0x0003 / 线圈 0x0009 的数值直接
 * = 转速 rpm（内部 ×10 交给 riseMoterUp/DownAtSpeed()），符号给出方向
 * （>0 上行 / <0 下行 / 0 停止）；上限由 moter42.c 的 riseMoterSpeedLimit() 按
 * RISE_MOTER_MAX_RPM_X10(6000=600rpm) 钳位。
 * 写 -1000（MB_RISE_TIMED_DOWN_HOLD）仍为“下降一段”保留值（优先判断）。
 * 注意：主站 ESP32 的 riseMoterUp/Down(23) 按本语义 = 23rpm（原为固定 90rpm）。 */
#define RISE_CMD_RPM_SCALE   10

/* 抬升值 → rpm×10（取绝对值后 ×10，钳到 int16 上限）。 */
static uint16_t riseCmdValueToRpmX10(int16_t value)
{
  int32_t rpmX10 = (value < 0) ? -(int32_t)value : (int32_t)value;
  rpmX10 *= RISE_CMD_RPM_SCALE;
  if(rpmX10 > 32767) rpmX10 = 32767;
  return (uint16_t)rpmX10;
}

#define MB_LOG_STAGE_Q  0x01U
#define MB_LOG_STAGE_T  0x02U
#define MB_LOG_STAGE_C  0x04U
#define MB_LOG_STAGE_V  0x08U
#define MB_LOG_STAGE_S  0x10U
#define MB_LOG_STAGE_D  0x20U

/* Suppress UART5 diagnostics for this exact Modbus request. Set ENABLE to 0
 * to show every request, or edit the byte list to filter another command. */
#define MODBUS_DEBUG_FILTER_ENABLE  1U
#define MODBUS_DEBUG_FILTER_BYTES  0x02U, 0x04U, 0x00U, 0x00U, 0x00U, 0x50U, 0xF0U, 0x05U
static const uint8_t modbusDebugFilter[] = { MODBUS_DEBUG_FILTER_BYTES };

/* Modbus RTU t3.5 turn-around delay (MODBUS over Serial Line V1.02 §2.2).
 * For baud > 19200 the standard fixes t3.5 at 1.75 ms; with the 1 ms tick
 * resolution of wk_timebase_get() we round up to 2 ms. */
#define MB_T35_MS               2U

/* Bus-release confirmation retries: after t3.5, if the DMA write position
 * still advances (master hasn't released DE), restart the wait. */
#define MB_BUS_RELEASE_RETRIES  3U

#define MB_EX_ILLEGAL_FUNCTION       1U
#define MB_EX_ILLEGAL_ADDRESS        2U
#define MB_EX_ILLEGAL_VALUE          3U

typedef struct
{
  uint16_t length;
  uint8_t data[MODBUS_RX_BUFFER_SIZE];
} modbus_rx_slot_t;

static modbus_rx_slot_t rxQueue[MB_RX_QUEUE_DEPTH];
static volatile uint8_t rxHead;
static volatile uint8_t rxTail;
static uint8_t coils[MB_COIL_COUNT];
static uint16_t holdingRegs[MB_HOLD_REG_COUNT];
static uint8_t response[MB_RESPONSE_BUFFER_SIZE];

/* USART3 RX uses DMA1 channel1 in non-circular mode. On each IDLE interrupt
 * the DMA is stopped, the received length is read from dtcnt, the bytes are
 * handed to Modbus_RxIdleCallback(), then the DMA buffer is reloaded and the
 * channel is restarted for the next frame. This mirrors the proven V1
 * (NUVELL_V1) approach and avoids the circular-DMA position tracking that
 * caused data-order corruption. */
static uint8_t usart3DmaBuffer[MODBUS_RX_BUFFER_SIZE];
static bool modbusPortInitialized;

volatile uint32_t modbusRxQueueOverflow;
volatile uint32_t modbusCrcErrorCount;
volatile uint32_t modbusFrameErrorCount;
/* 统计：CRC 校验通过的有效请求帧总数、实际发送的应答帧总数。
 * 供 UART5 调试口 NB+stat 指令周期性打印，用于观察总线流量与应答比。 */
volatile uint32_t modbusRxTotalCount;
volatile uint32_t modbusReplyTotalCount;

/* Timestamp (1 ms tick) of the last USART3 IDLE event, set in
 * modbusUsart3IdleIsr(). Used by modbusWaitBusRelease() to enforce the
 * t3.5 turn-around delay before driving DE high. */
static volatile uint32_t modbusLastRxTick;

static uint16_t requestLength(const uint8_t *data, uint16_t available);
static uint8_t supportedFunction(uint8_t function);
static uint16_t processRequest(const uint8_t *request, uint8_t broadcast);

static bool modbusDebugSuppress(const uint8_t *data, uint16_t length)
{
#if MODBUS_DEBUG_FILTER_ENABLE
  return length == sizeof(modbusDebugFilter) &&
         memcmp(data, modbusDebugFilter, sizeof(modbusDebugFilter)) == 0;
#else
  (void)data;
  (void)length;
  return false;
#endif
}

static void modbusDebugLogFrame(const uint8_t *data, uint16_t length,
                                uint8_t stages)
{
  uint16_t i;
  uint16_t textLength = 0U;
  uint8_t stageLength = 0U;
  char frameText[56];
  char stageText[7];

  for(i = 0U; i < length && i < 16U; i++)
  {
    textLength = (uint16_t)(textLength + (uint16_t)sprintf(
      &frameText[textLength], i == 0U ? "%02X" : " %02X", (unsigned int)data[i]));
  }
  if(length > 16U)
  {
    (void)sprintf(&frameText[textLength], " ...");
  }
  else
  {
    frameText[textLength] = '\0';
  }

  if((stages & MB_LOG_STAGE_Q) != 0U) stageText[stageLength++] = 'Q';
  if((stages & MB_LOG_STAGE_T) != 0U) stageText[stageLength++] = 'T';
  if((stages & MB_LOG_STAGE_C) != 0U) stageText[stageLength++] = 'C';
  if((stages & MB_LOG_STAGE_V) != 0U) stageText[stageLength++] = 'V';
  if((stages & MB_LOG_STAGE_S) != 0U) stageText[stageLength++] = 'S';
  if((stages & MB_LOG_STAGE_D) != 0U) stageText[stageLength++] = 'D';
  stageText[stageLength] = '\0';

  log_d("Modbus RX[%u]=%s stages=%s", (unsigned int)length, frameText, stageText);
}

/* TMR9 1 ms time base (wk_system.c). Used for the t3.5 turn-around delay. */
extern uint32_t wk_timebase_get(void);

/* UART5 retains the legacy screen/host protocol during the RTU migration. */
extern volatile bool moter28Run;
extern volatile bool moter28Dir;

/* Millisecond delay via the TMR9 time base (wk_system.c). */
static void modbusDelayMs(uint32_t ms)
{
  uint32_t start = wk_timebase_get();
  while((wk_timebase_get() - start) < ms)
  {
  }
}

/* Wait for the Modbus t3.5 turn-around delay to elapse since the last
 * received byte (IDLE event), then confirm the bus is idle by checking
 * that the DMA write position does not advance over one character time.
 * Returns true if the bus is confirmed idle, false if the bus-release
 * check timed out after MB_BUS_RELEASE_RETRIES attempts (the response
 * is sent anyway to avoid blocking the slave forever). */
static bool modbusWaitBusRelease(void)
{
  uint8_t retries;
  for(retries = 0U; retries < MB_BUS_RELEASE_RETRIES; retries++)
  {
    /* 1. t3.5 turn-around: at least MB_T35_MS since last IDLE. */
    uint32_t elapsed = wk_timebase_get() - modbusLastRxTick;
    if(elapsed < MB_T35_MS)
    {
      modbusDelayMs(MB_T35_MS - elapsed);
    }
    /* 2. Bus-release confirmation: snapshot DMA position, wait one
     *    character time (10 bits at 8N1), re-read. If the position
     *    advanced, the master is still driving — restart t3.5. */
    {
      uint16_t pos1 = (uint16_t)(MODBUS_RX_BUFFER_SIZE - DMA1_CHANNEL1->dtcnt);
      uint16_t pos2;
      volatile uint32_t char_cycles = (system_core_clock / MB_BAUD_RATE) * 10U;
      while(char_cycles-- != 0U)
      {
        __NOP();
      }
      pos2 = (uint16_t)(MODBUS_RX_BUFFER_SIZE - DMA1_CHANNEL1->dtcnt);
      if(pos1 == pos2)
      {
        return true;
      }
    }
  }
  return false;
}

static void rs485TransmitEnable(bool enable)
{
  if(enable)
  {
    gpio_bits_set(USART3SEL_GPIO_PORT, USART3SEL_PIN);
  }
  else
  {
    gpio_bits_reset(USART3SEL_GPIO_PORT, USART3SEL_PIN);
  }
}

static void rs485DelayThreeCharacters(void)
{
  /* 8N1 is 10 bits/character. The loop overhead makes this delay no shorter
   * than the required 3 character times. */
  volatile uint32_t cycles = (system_core_clock / MB_BAUD_RATE) * 30U;
  while(cycles-- != 0U)
  {
    __NOP();
  }
}

/* ---------- RS485 版本号打印（每次复位/上电发送一次） ----------
 * 在 USART3 RS485 口上同时发送两种格式：
 *   1) ASCII 文本行（便于串口助手直接查看）
 *   2) 二进制版本帧（与 legacy modbus.c §5.4 兼容）
 *      [0x76][FW ver×4][CRC16_Calc(5B) LE][0x6E]，共 8 字节
 * 版本帧使用 legacy 的非标准 CRC16（多项式 0x04C11DB7，4 字节小端拼字），
 * 与标准 Modbus CRC16 (0xA001) 不同，此处重新实现以保持帧兼容。 */
static uint16_t modbusVersionCRC16(const uint8_t *data, uint32_t len)
{
  uint32_t crc = 0xFFFFFFFFUL;
  uint32_t word;
  uint8_t  i;

  while(len >= 4U)
  {
    word = (uint32_t)data[0] |
           ((uint32_t)data[1] << 8) |
           ((uint32_t)data[2] << 16) |
           ((uint32_t)data[3] << 24);
    crc ^= word;
    for(i = 0U; i < 32U; i++)
    {
      if(crc & 0x80000000UL)
      {
        crc = (crc << 1) ^ 0x04C11DB7UL;
      }
      else
      {
        crc = crc << 1;
      }
    }
    data += 4U;
    len -= 4U;
  }

  if(len > 0U)
  {
    word = 0UL;
    for(i = 0U; i < 4U; i++)
    {
      if(i < (uint8_t)len)
      {
        word |= (uint32_t)data[i] << (8U * i);
      }
    }
    crc ^= word;
    for(i = 0U; i < 32U; i++)
    {
      if(crc & 0x80000000UL)
      {
        crc = (crc << 1) ^ 0x04C11DB7UL;
      }
      else
      {
        crc = crc << 1;
      }
    }
  }
  return (uint16_t)(crc & 0xFFFFUL);
}

/* 通过 RS485 (USART3) 发送 ASCII 文本版本行。 */
static void modbusSendVersionAscii(void)
{
  char versionText[64];
  /* 使用宏构建版本字符串，避免硬编码版本号。 */
  (void)sprintf(versionText,
                "\r\nFW:%u.%u.%u.%u HW:%u.%u.%u.%u PROTO:%u.%u.%u\r\n",
                FW_VERSION_MAJOR, FW_VERSION_MINOR, FW_VERSION_PATCH, FW_VERSION_BUILD,
                HW_VERSION_MAJOR, HW_VERSION_MINOR, HW_VERSION_PATCH, HW_VERSION_BOARD,
                PROTO_VERSION_MAJOR, PROTO_VERSION_MINOR, PROTO_VERSION_PATCH);
  modbusRs485Send((const uint8_t *)versionText, (uint16_t)strlen(versionText));
}

/* 通过 RS485 (USART3) 发送二进制版本帧（legacy §5.4 格式）。 */
static void modbusSendVersionFrame(void)
{
  uint8_t frame[8];
  uint16_t crc;

  frame[0] = 0x76U; /* MODBUS_HEAD_VERSION */
  frame[1] = FW_VERSION_MAJOR;
  frame[2] = FW_VERSION_MINOR;
  frame[3] = FW_VERSION_PATCH;
  frame[4] = FW_VERSION_BUILD;
  crc = modbusVersionCRC16(frame, 5U);
  frame[5] = (uint8_t)(crc & 0xFFU);
  frame[6] = (uint8_t)((crc >> 8) & 0xFFU);
  frame[7] = 0x6EU; /* MODBUS_FRAME_TAIL_VER */
  modbusRs485Send(frame, 8U);
}

/* 通过 RS485 口打印三个版本号（ASCII + 二进制帧），每次复位/上电调用。 */
void modbusPrintVersion(void)
{
  modbusSendVersionAscii();
  modbusSendVersionFrame();
}

void modbusinit(void)
{
  if(modbusPortInitialized)
  {
    return;
  }
  /* USART3 format, RX DMA request and IDLE interrupt are configured exactly
   * once by wk_usart3_init(). This module owns only PB2 state and the DMA
   * memory buffer; do not override the BSP's 1000000/8N1 configuration.
   * MB_BAUD_RATE must track wk_usart.c's usart_init(USART3, ...) and
   * code.ATWP's USART3<b>BaudRate</b>, otherwise the 3-character RS485
   * release delay below is wrong and the last byte gets truncated. */
  rs485TransmitEnable(false);

  /* DMA1_CH1 mode and request selection are initialized once by
   * wk_dma1_channel1_init().  Workbench's generated main() initially enables
   * this channel with its placeholder address/length (both are zero here).
   * DMA address/count registers must only be replaced while the channel is
   * disabled; otherwise IDLE can occur with an invalid buffer even though the
   * USART interrupt itself is working. */
  dma_channel_enable(DMA1_CHANNEL1, FALSE);
  wk_dma_channel_config(DMA1_CHANNEL1, (uint32_t)&USART3->dt,
                        (uint32_t)usart3DmaBuffer, MODBUS_RX_BUFFER_SIZE);
  dma_channel_enable(DMA1_CHANNEL1, TRUE);
  modbusPortInitialized = true;

  /* 版本帧打印已移至 my_task01_func 首次运行时执行。原来在 modbusinit()
   * 中直接发送，但 modbusinit() 在 main.c 中于 wk_freertos_init()（创建
   * 队列 + 启动调度器）之前被调用，此时若 IDLE 中断因上位机数据或 RS485
   * 回环触发，xQueueSendFromISR 会因 modbusData_handle==NULL 而 HardFault。
   * 移到任务上下文中发送可彻底避免该竞态。 */
}

void modbusUsart3IdleIsr(void)
{
  uint16_t rxLength;

  /* Stop DMA first so dtcnt remains stable while the received length is
   * read. This is the V1 (NUVELL_V1) pattern: non-circular DMA, stop → read
   * length → clear IDLE → process → reload → restart. It avoids the
   * circular-DMA position tracking that caused data-order corruption
   * (circular left shift). */
  dma_channel_enable(DMA1_CHANNEL1, FALSE);
  rxLength = (uint16_t)(MODBUS_RX_BUFFER_SIZE - DMA1_CHANNEL1->dtcnt);

  /* Clear the IDLE flag AFTER stopping DMA. usart_flag_clear() reads STS
   * then DT; if DMA is still running, reading DT steals a byte from the
   * receive register and corrupts the DMA buffer contents (every byte ends
   * up with bit7 set and wrong length). V1 clears IDLE here, after DMA is
   * stopped, so the DT read is harmless. The caller (USART3_IRQHandler)
   * must NOT clear IDLE before calling this function. */
  usart_flag_clear(USART3, USART_IDLEF_FLAG);

  if(rxLength > 0U && rxLength <= MODBUS_RX_BUFFER_SIZE)
  {
    /* Record the time of this IDLE event for the t3.5 turn-around delay
     * enforced by modbusWaitBusRelease() before the next response. */
    modbusLastRxTick = wk_timebase_get();
    Modbus_RxIdleCallback(usart3DmaBuffer, rxLength);
  }

  /* Reload the same buffer for the next frame and restart DMA. */
  DMA1_CHANNEL1->dtcnt = MODBUS_RX_BUFFER_SIZE;
  DMA1_CHANNEL1->maddr = (uint32_t)usart3DmaBuffer;
  dma_channel_enable(DMA1_CHANNEL1, TRUE);
}

bool modbusRs485Send(const uint8_t *data, uint16_t length)
{
  uint16_t i;

  if(data == NULL || length == 0U)
  {
    return false;
  }

  /* Enforce Modbus t3.5 turn-around delay and confirm the master has
   * released the bus before driving DE high (prevents bus contention). */
  modbusWaitBusRelease();
  rs485TransmitEnable(true);
  for(i = 0U; i < length; i++)
  {
    while(usart_flag_get(USART3, USART_TDBE_FLAG) == RESET)
    {
    }
    usart_data_transmit(USART3, data[i]);
  }
  while(usart_flag_get(USART3, USART_TDC_FLAG) == RESET)
  {
  }
  rs485DelayThreeCharacters();
  rs485TransmitEnable(false);
  return true;
}

uint16_t Modbus_CRC16(const uint8_t *data, uint16_t length)
{
  uint16_t crc = 0xFFFFU;
  uint8_t bit;

  while(length-- != 0U)
  {
    crc ^= *data++;
    for(bit = 0U; bit < 8U; bit++)
    {
      crc = (crc & 1U) != 0U ? (uint16_t)((crc >> 1) ^ 0xA001U) : (uint16_t)(crc >> 1);
    }
  }
  return crc;
}

void Modbus_RxIdleCallback(const uint8_t *data, uint16_t length)
{
  BaseType_t higherPriorityTaskWoken = pdFALSE;
  uint8_t next;
  uint16_t slot;

  /* V1 (NUVELL_V1) pattern: ISR only copies the received bytes into the next
   * free queue slot and signals the task. All frame assembly, length
   * validation and CRC checking happen in the task context (processBurst),
   * never in the ISR. This keeps the ISR short and avoids the data-order
   * corruption caused by the previous rxAssembleBuffer/requestLength logic. */
  if(data == NULL || length == 0U || length > MODBUS_RX_BUFFER_SIZE)
  {
    modbusFrameErrorCount++;
    return;
  }

  /* modbusinit() 在 main.c 中于 wk_freertos_init()（创建队列 + 启动调度器）
   * 之前被调用。此时 IDLE 中断已使能，若上位机在复位后立即发送数据，
   * DMA 会收到数据并触发 IDLE 中断，但 modbusData_handle 还是 NULL。
   * 直接调用 xQueueSendFromISR(NULL, ...) 会访问空指针导致 HardFault，
   * 表现为复位后 USART3 RX 有波形但收不到数据/无应答。
   * 此处检查队列句柄：若为 NULL（调度器未就绪），丢弃本帧即可——
   * DMA 会在 modbusUsart3IdleIsr() 尾部被重启，不影响后续接收。 */
  if(modbusData_handle == NULL)
  {
    modbusFrameErrorCount++;
    return;
  }

  next = (uint8_t)((rxHead + 1U) % MB_RX_QUEUE_DEPTH);
  if(next == rxTail)
  {
    modbusRxQueueOverflow++;
    return;
  }

  slot = rxHead;
  memcpy(rxQueue[slot].data, data, length);
  rxQueue[slot].length = length;
  if(xQueueSendFromISR(modbusData_handle, &slot, &higherPriorityTaskWoken) != pdPASS)
  {
    modbusRxQueueOverflow++;
    return;
  }
  rxHead = next;
  portYIELD_FROM_ISR(higherPriorityTaskWoken);
}

static uint16_t floatWord(float value, uint8_t highWord)
{
  uint32_t raw;
  memcpy(&raw, &value, sizeof(raw));
  return highWord != 0U ? (uint16_t)(raw >> 16) : (uint16_t)raw;
}

static uint8_t readDiscrete(uint16_t address)
{
  /* 离散输入 0x0000~0x000F 严格遵循 站号2_AT32F415CBT7_Modbus协议.md：
   *   0 抬升高位 / 1 抬升低位 / 2 压制高位 / 3 压制低位
   *   4 压制头在位 / 5 水箱在位 / 6 水箱高液位 / 7 水箱低液位
   *   8 废水箱在位 / 9 阀门开到位 / 10 阀门关到位 / 11 点卤盒在位
   *   12 溢出报警 / 13 上盖在位 / 14 滤网在位 / 15 成型盒在位 */
  switch(address)
  {
    case 0U:  return riseMoterHighPositionGet();
    case 1U:  return riseMoterLowPositionGet();
    case 2U:  return pressMoterHighPositionGet();
    case 3U:  return pressMoterLowPositionGet();
    case 4U:  return tmux1308Data0Values[pressaAvail] != 0U;
    case 5U:  return tmux1308Data0Values[waterTankAvail] != 0U;
    case 6U:  return tmux1308Data0Values[waterHighPos] != 0U;
    case 7U:  return tmux1308Data2Values[waterPos] > 2800U;
    case 8U:  return tmux1308Data0Values[wasteBoxAvail] != 0U;
    case 9U:  return valveOnPos;
    case 10U: return valveOffPos;
    case 11U: return tmux1308Data0Values[brineAvail] != 0U;
    case 12U: return tmux1308Data2Values[overFlow] != 0U;
    case 13U: return tmux1308Data0Values[lidAvail] != 0U;
    case 14U: return 0U; /* 滤网在位：硬件未实现 */
    case 15U: return tmux1308Data1Values[modeAvail] != 0U;
    default:  return 0U;
  }
}

static void buildInputSnapshot(uint16_t *snapshot)
{
  uint16_t i;
  uint16_t discreteMask = 0U;
  uint16_t coilMask = 0U;
  uint16_t deviceStatus = 0U;
  uint16_t faultCode = 0U;
  uint16_t modbusRxDepth;
  uint16_t legacyCmdDepth = 0U; /* V2 无旧协议命令队列，固定为 0 */

  /* getState task refreshes these shared measurement caches every 20 ms.
   * A Modbus response only snapshots them, avoiding long ADC scans in the
   * RS485 request/response path. */

  for(i = 0U; i < MB_DISCRETE_COUNT; i++)
  {
    if(readDiscrete(i) != 0U)
    {
      discreteMask |= (uint16_t)(1U << i);
    }
  }
  for(i = 0U; i < MB_COIL_COUNT; i++)
  {
    if(coils[i] != 0U)
    {
      coilMask |= (uint16_t)(1U << i);
    }
  }

  memset(snapshot, 0, MB_INPUT_REG_COUNT * sizeof(snapshot[0]));

  /* ===== 0x0000~0x0027：严格遵循 站号2_AT32F415CBT7_Modbus协议.md ===== */

  /* 0x0000~0x0001: NTC1 原始 ADC 值（uint32，高字在前） */
  snapshot[0] = (uint16_t)((uint32_t)tmux1308Data2Values[NTC1] >> 16);
  snapshot[1] = (uint16_t)tmux1308Data2Values[NTC1];
  /* 0x0002~0x0003: 母线电压 float32，高字在前 */
  snapshot[2] = floatWord(busVol, 1U);
  snapshot[3] = floatWord(busVol, 0U);
  /* 0x0004~0x0005: 电流 float32，高字在前（母线电流） */
  snapshot[4] = floatWord(currentData.busAmpere, 1U);
  snapshot[5] = floatWord(currentData.busAmpere, 0U);
  /* 0x0006: 抬升电机设定值镜像（保持寄存器 0x0003） */
  snapshot[6] = holdingRegs[3];
  /* 0x0007~0x0008: 累计圈数 float32，高字在前（压制编码器
	真实脉冲/1000） */
  snapshot[7] = floatWord((float)getPressMoterTurns() / 1000.0f, 1U);
  snapshot[8] = floatWord((float)getPressMoterTurns() / 1000.0f, 0U);

  /* 0x0009: 设备状态字
   *   bit0 Modbus 就绪 / bit1 存在非零执行器命令
   *   bit2 到位/在位/液位/报警有效 / bit3 存在通信诊断异常 */
  deviceStatus = 0x0001U; /* bit0: Modbus 协议已就绪 */
  if(coilMask != 0U || holdingRegs[0] != 0U || holdingRegs[1] != 0U ||
     holdingRegs[2] != 0U || holdingRegs[3] != 0U)
  {
    deviceStatus |= 0x0002U; /* bit1: 存在非零执行器命令 */
  }
  if(discreteMask != 0U)
  {
    deviceStatus |= 0x0004U; /* bit2: 到位/在位/液位/报警有效 */
  }
  if(modbusRxQueueOverflow != 0U || modbusCrcErrorCount != 0U ||
     modbusFrameErrorCount != 0U)
  {
    deviceStatus |= 0x0008U; /* bit3: 存在通信诊断异常 */
  }
  snapshot[9] = deviceStatus;

  /* 0x000A: 故障码
   *   bit0 接收队列溢出 / bit1 CRC 错误 / bit2 帧错误 / bit3 旧协议溢出 */
  if(modbusRxQueueOverflow != 0U) faultCode |= 0x0001U;
  if(modbusCrcErrorCount != 0U) faultCode |= 0x0002U;
  if(modbusFrameErrorCount != 0U) faultCode |= 0x0004U;
  snapshot[10] = faultCode;

  /* 0x000B~0x000C: 固件版本 {主, 次, 修订, 构建} */
  snapshot[11] = (uint16_t)((uint16_t)(FW_VERSION_MAJOR << 8) | FW_VERSION_MINOR);
  snapshot[12] = (uint16_t)((uint16_t)(FW_VERSION_PATCH << 8) | FW_VERSION_BUILD);

  /* 0x000D: 保留 */
  snapshot[13] = 0U;

  /* 0x000E: 全部 16 个离散输入位图 */
  snapshot[14] = discreteMask;
  /* 0x000F: 全部 10 个线圈命令状态位图，bit0~bit9 对应线圈 0x0000~0x0009 */
  snapshot[15] = coilMask;

  /* 0x0010: 清水泵速度命令镜像（保持寄存器 0x0000） */
  snapshot[16] = holdingRegs[0];
  /* 0x0011: 蠕动泵速度命令镜像（保持寄存器 0x0001） */
  snapshot[17] = holdingRegs[1];
  /* 0x0012: 压制电机有符号速度命令镜像（保持寄存器 0x0002） */
  snapshot[18] = holdingRegs[2];
  /* 0x0013: 抬升电机有符号命令镜像（保持寄存器 0x0003） */
  snapshot[19] = holdingRegs[3];

  /* 0x0014: 队列深度（高字节旧协议队列，低字节 Modbus 接收队列） */
  modbusRxDepth = (uint16_t)((rxHead - rxTail + MB_RX_QUEUE_DEPTH) % MB_RX_QUEUE_DEPTH);
  snapshot[20] = (uint16_t)((legacyCmdDepth << 8) | (uint8_t)modbusRxDepth);

  /* 0x0015~0x0016: Modbus 接收队列溢出累计次数 uint32，高字在前 */
  snapshot[21] = (uint16_t)(modbusRxQueueOverflow >> 16);
  snapshot[22] = (uint16_t)modbusRxQueueOverflow;
  /* 0x0017~0x0018: Modbus CRC 错误累计次数 uint32，高字在前 */
  snapshot[23] = (uint16_t)(modbusCrcErrorCount >> 16);
  snapshot[24] = (uint16_t)modbusCrcErrorCount;
  /* 0x0019~0x001A: Modbus 帧错误累计次数 uint32，高字在前 */
  snapshot[25] = (uint16_t)(modbusFrameErrorCount >> 16);
  snapshot[26] = (uint16_t)modbusFrameErrorCount;
  /* 0x001B~0x001C: 旧协议命令队列溢出累计次数 uint32，高字在前（V2 无，固定 0） */
  snapshot[27] = 0U;
  snapshot[28] = 0U;

  /* 0x001D: TMUX1308 D0 的 8 路原始数字输入位图 */
  {
    uint16_t d0Mask = 0U;
    for(i = 0U; i < 8U; i++)
    {
      if(tmux1308Data0Values[i] != 0U) d0Mask |= (uint16_t)(1U << i);
    }
    snapshot[29] = d0Mask;
  }
  /* 0x001E: TMUX1308 D1 的 8 路原始数字输入位图 */
  {
    uint16_t d1Mask = 0U;
    for(i = 0U; i < 8U; i++)
    {
      if(tmux1308Data1Values[i] != 0U) d1Mask |= (uint16_t)(1U << i);
    }
    snapshot[30] = d1Mask;
  }

  /* 0x001F~0x0026: TMUX1308 通道 0~7 的原始 12 位 ADC 值（D2 组） */
  for(i = 0U; i < 8U; i++)
  {
    snapshot[31U + i] = tmux1308Data2Values[i];
  }

  /* 0x0027: 板载电流采样 ADC 原始值 */
  snapshot[39] = currentData.busRaw;

  /* ===== 0x0028~0x004F：V2 追加功能（文档未定义，往后追加） ===== */

  /* 0x0028~0x002F: 多路电流 float32，高字在前
   *   busAmpere / ledAmpere / brakeAmpere / strainGageAmpere */
  snapshot[40] = floatWord(currentData.busAmpere, 1U);
  snapshot[41] = floatWord(currentData.busAmpere, 0U);
  snapshot[42] = floatWord(currentData.ledAmpere, 1U);
  snapshot[43] = floatWord(currentData.ledAmpere, 0U);
  snapshot[44] = floatWord(currentData.brakeAmpere, 1U);
  snapshot[45] = floatWord(currentData.brakeAmpere, 0U);
  snapshot[46] = floatWord(currentData.strainGageAmpere, 1U);
  snapshot[47] = floatWord(currentData.strainGageAmpere, 0U);

  /* 0x0030~0x0037: 抬升双电机四路电流 float32，高字在前
   *   riseRightB / riseRightA / riseLeftB / riseLeftA */
  snapshot[48] = floatWord(currentData.riseRightBAmpere, 1U);
  snapshot[49] = floatWord(currentData.riseRightBAmpere, 0U);
  snapshot[50] = floatWord(currentData.riseRightAAmpere, 1U);
  snapshot[51] = floatWord(currentData.riseRightAAmpere, 0U);
  snapshot[52] = floatWord(currentData.riseLeftBAmpere, 1U);
  snapshot[53] = floatWord(currentData.riseLeftBAmpere, 0U);
  snapshot[54] = floatWord(currentData.riseLeftAAmpere, 1U);
  snapshot[55] = floatWord(currentData.riseLeftAAmpere, 0U);

  /* 0x0038: 42 抬升电机当前电流幅值（回读） */
  snapshot[56] = StepperPulse42_GetAmplitude();
  /* 0x0039: 57 压制电机电流幅值 */
  snapshot[57] = StepperPulse57_8412_GetAmplitude();
  /* 0x003A: 复位按键电平 */
  snapshot[58] = getTotolRstLevel() ? 1U : 0U;

  /* 0x003B~0x003E: 编码器位置 int32，高字在前
   *   抬升左 / 抬升右 / 压制右 */
  {
    uint32_t left = (uint32_t)encRiseMoterLEncPositionGet();
    uint32_t right = (uint32_t)encRiseMoterREncPositionGet();
    uint32_t press = (uint32_t)encPressMoterREncPositionGet();
    snapshot[59] = (uint16_t)(left >> 16);
    snapshot[60] = (uint16_t)left;
    snapshot[61] = (uint16_t)(right >> 16);
    snapshot[62] = (uint16_t)right;
    snapshot[63] = (uint16_t)(press >> 16);
    snapshot[64] = (uint16_t)press;
  }

  /* 0x0041~0x0046: NTC2/NTC3/NTCBack 原始 ADC 值（uint32，高字在前） */
  snapshot[65] = (uint16_t)((uint32_t)tmux1308Data2Values[NTC2] >> 16);
  snapshot[66] = (uint16_t)tmux1308Data2Values[NTC2];
  snapshot[67] = (uint16_t)((uint32_t)tmux1308Data2Values[NTC3] >> 16);
  snapshot[68] = (uint16_t)tmux1308Data2Values[NTC3];
  snapshot[69] = (uint16_t)((uint32_t)tmux1308Data2Values[NTCBacK] >> 16);
  snapshot[70] = (uint16_t)tmux1308Data2Values[NTCBacK];

  /* 0x0047~0x004A: 流量计脉冲累计值 uint32，高字在前
   *   清水 / 点卤 */
  {
    uint32_t waterPulse = waterFTPulseGet();
    uint32_t brinePulse = brineFTPulseGet();
    snapshot[71] = (uint16_t)(waterPulse >> 16);
    snapshot[72] = (uint16_t)waterPulse;
    snapshot[73] = (uint16_t)(brinePulse >> 16);
    snapshot[74] = (uint16_t)brinePulse;
  }

  /* 0x004B~0x004E: 57 绕组 A/B ADC 零点基准 + ADC 采集状态
   *   CurrA_Zero / CurrB_Zero / 采集就绪标志 / 采样数 */
  snapshot[75] = StepperPulse57_8412_CurrA_Zero;
  snapshot[76] = StepperPulse57_8412_CurrB_Zero;
  snapshot[77] = StepperPulse57_8412_AdcCaptureReady() ? 1U : 0U;
  snapshot[78] = StepperPulse57_8412_AdcCaptureCount();

  /* 0x004F: 保留 */
  snapshot[79] = 0U;
}

static void writeCoil(uint16_t address, uint8_t on)
{
  /* 线圈 0x0000~0x0009（共 10 个）严格遵循 站号2_AT32F415CBT7_Modbus协议.md。
   * 与常规 Modbus 习惯不同：地址 3~4 写 0 不会执行"反向动作"，
   * 开/关被拆成两个独立地址，因此 FC0F 一次性清零不会关断这些执行器。 */
  coils[address] = on != 0U;
  switch(address)
  {
    case 0U: /* 清水泵：0=停止，1=运行（速度=PUMP_DEFAULT_WATER_PERCENT） */
      if(on != 0U) waterPumpOn(PUMP_DEFAULT_WATER_PERCENT);
      else waterPumpOff();
      break;
    case 1U: /* 点卤泵：0=停止，1=运行（速度=PUMP_DEFAULT_BRINE_PERCENT） */
      if(on != 0U) brinePumpOn(PUMP_DEFAULT_BRINE_PERCENT);
      else brinePumpOff();
      break;
    case 2U: /* 蠕动泵：0=停止，1=运行（速度=PUMP_DEFAULT_HOSE_PERCENT，可为负=反向） */
      if(on != 0U) hosePumpOn(PUMP_DEFAULT_HOSE_PERCENT);
      else hosePumpOff();
      break;
    case 3U: /* 阀门：0=关闭，1=开启 */
      if(on != 0U) valveOn();
      else valveOff();
      break;
    case 4U: /* LED：0=关闭，1=开启 */
      if(on != 0U) LEDOn();
      else LEDOff();
      break;
    case 5U: /* 预留执行器：仅保存状态 */
    case 6U:
    case 7U:
      break;
    case 8U: /* 压制电机命令状态：由保持寄存器 0x0002 更新 */
      if(on != 0U)
      {
        int v = pressCmdValueToRpmX10((int16_t)holdingRegs[2]);
        if(v > 0) pressMoterUp(v);
        else if(v < 0) pressMoterDown(v);
        else pressMoterStop();
      }
      else
      {
        pressMoterStop();
      }
      break;
    case 9U: /* 抬升电机命令状态：由保持寄存器 0x0003 更新（数值=转速 rpm） */
      if(on != 0U)
      {
        int16_t v = (int16_t)holdingRegs[3];
        /* 魔数触发（优先判断）：-1000 = 下行一段；-601 = 从最低位向上走"上行一段" */
        if(v == MB_RISE_TIMED_DOWN_HOLD)
        {
          (void)riseMoterDownSegmentDefault();
        }
        else if(v == MB_RISE_TIMED_UP_HOLD)
        {
          (void)riseMoterUpSegmentDefault();
        }
        else if(v > 0)
        {
          riseMoterUpAtSpeed(riseCmdValueToRpmX10(v));
        }
        else if(v < 0)
        {
          riseMoterDownAtSpeed(riseCmdValueToRpmX10(v));
        }
        else
        {
          riseMoterStop();
        }
      }
      else
      {
        riseMoterStop();
      }
      break;
    default: break;
  }
}

static void writeHoldingRegister(uint16_t address, uint16_t value)
{
  int16_t signedValue = (int16_t)value;
  holdingRegs[address] = value;

  /* 保持寄存器 0x0000~0x0003 严格遵循 站号2_AT32F415CBT7_Modbus协议.md。
   * 0x0004 = 抬升电流档（0=低档，非 0=高档）；0x0005~0x0066 为参数/预留区，
   * 仅保存于 RAM，上电清零。 */
  switch(address)
  {
    case 0U: /* 清水泵速度：0~1000，0 停止 */
      if(value == 0U) waterPumpOff();
      else waterPumpOn((int8_t)(value > 1000U ? 100 : value / 10U));
      break;
    case 1U: /* 蠕动泵速度：0~255，超过 255 按 255 */
      if(value == 0U) hosePumpOff();
      else hosePumpOn((int8_t)(value > 255U ? 255 : value));
      break;
    case 2U: /* 压制电机有符号速度：正=Up，负=Down，0=Stop */
    {
      int speedX10 = pressCmdValueToRpmX10(signedValue);
      if(speedX10 > 0) pressMoterUp(speedX10);
      else if(speedX10 < 0) pressMoterDown(speedX10);
      else pressMoterStop();
      break;
    }
    case 3U: /* 抬升电机有符号命令：正=Up，负=Down，0=Stop（数值直接=转速 rpm，上限见 moter42.c）。
              * 魔数（|值| 超出 ±600 合法转速，只作动作触发，优先判断）：
              *   -1000（MB_RISE_TIMED_DOWN_HOLD）= 从当前停点向下走固定段长"下降一段"；
              *   -601 （MB_RISE_TIMED_UP_HOLD）  = 从最低位向上走固定段长"上行一段"。 */
      if(signedValue == MB_RISE_TIMED_DOWN_HOLD)
      {
        (void)riseMoterDownSegmentDefault();
      }
      else if(signedValue == MB_RISE_TIMED_UP_HOLD)
      {
        (void)riseMoterUpSegmentDefault();
      }
      else if(signedValue > 0)
      {
        riseMoterUpAtSpeed(riseCmdValueToRpmX10(signedValue));
      }
      else if(signedValue < 0)
      {
        riseMoterDownAtSpeed(riseCmdValueToRpmX10(signedValue));
      }
      else
      {
        riseMoterStop();
      }
      break;
    case 4U: /* 抬升电流档：0 = 低档（默认：上位机手动点动、开机自检）；
              * 非 0 = 高档（压制流程带载：归位下行 / -601 / -1000 / 回高位上行）。 */
      if(value != 0U) riseMoterSetLoadGear(1U);
      else riseMoterSetLoadGear(0U);
      break;
    default: break; /* 0x0005~0x0066 参数/预留区：仅保存，不驱动硬件 */
  }
}

static uint16_t exceptionResponse(uint8_t function, uint8_t exception)
{
  response[0] = MODBUS_SLAVE_ADDRESS;
  response[1] = (uint8_t)(function | 0x80U);
  response[2] = exception;
  return 3U;
}

static uint16_t inputSnapshot[MB_INPUT_REG_COUNT];
static uint16_t processRequest(const uint8_t *request, uint8_t broadcast)
{
  uint8_t function = request[1];
  uint16_t address = (uint16_t)(((uint16_t)request[2] << 8) | request[3]);
  uint16_t quantity;
  uint16_t value;
  uint16_t i;
  uint16_t payloadLength;
  
  uint16_t limit;

  switch(function)
  {
    case 0x01U:
    case 0x02U:
      quantity = (uint16_t)(((uint16_t)request[4] << 8) | request[5]);
      limit = function == 0x01U ? MB_COIL_COUNT : MB_DISCRETE_COUNT;
      if(quantity == 0U || quantity > 2000U) payloadLength = exceptionResponse(function, MB_EX_ILLEGAL_VALUE);
      else if(address >= limit || quantity > (uint16_t)(limit - address)) payloadLength = exceptionResponse(function, MB_EX_ILLEGAL_ADDRESS);
      else
      {
        response[0] = MODBUS_SLAVE_ADDRESS;
        response[1] = function;
        response[2] = (uint8_t)((quantity + 7U) / 8U);
        memset(&response[3], 0, response[2]);
        for(i = 0U; i < quantity; i++)
        {
          uint8_t state = function == 0x01U ? coils[address + i] : readDiscrete(address + i);
          if(state != 0U) response[3U + i / 8U] |= (uint8_t)(1U << (i % 8U));
        }
        payloadLength = (uint16_t)(3U + response[2]);
      }
      break;

    case 0x03U:
    case 0x04U:
      quantity = (uint16_t)(((uint16_t)request[4] << 8) | request[5]);
      limit = function == 0x03U ? MB_HOLD_REG_COUNT : MB_INPUT_REG_COUNT;
      if(quantity == 0U || quantity > 125U) payloadLength = exceptionResponse(function, MB_EX_ILLEGAL_VALUE);
      else if(address >= limit || quantity > (uint16_t)(limit - address)) payloadLength = exceptionResponse(function, MB_EX_ILLEGAL_ADDRESS);
      else
      {
        if(function == 0x04U) buildInputSnapshot(inputSnapshot);
        response[0] = MODBUS_SLAVE_ADDRESS;
        response[1] = function;
        response[2] = (uint8_t)(quantity * 2U);
        for(i = 0U; i < quantity; i++)
        {
          value = function == 0x03U ? holdingRegs[address + i] : inputSnapshot[address + i];
          response[3U + i * 2U] = (uint8_t)(value >> 8);
          response[4U + i * 2U] = (uint8_t)value;
        }
        payloadLength = (uint16_t)(3U + quantity * 2U);
      }
      break;

    case 0x05U:
      value = (uint16_t)(((uint16_t)request[4] << 8) | request[5]);
      if(address >= MB_COIL_COUNT) payloadLength = exceptionResponse(function, MB_EX_ILLEGAL_ADDRESS);
      else if(value != 0x0000U && value != 0xFF00U) payloadLength = exceptionResponse(function, MB_EX_ILLEGAL_VALUE);
      else { writeCoil(address, value == 0xFF00U); memcpy(response, request, 6U); payloadLength = 6U; }
      break;

    case 0x06U:
      value = (uint16_t)(((uint16_t)request[4] << 8) | request[5]);
      if(address >= MB_HOLD_REG_COUNT) payloadLength = exceptionResponse(function, MB_EX_ILLEGAL_ADDRESS);
      else { writeHoldingRegister(address, value); memcpy(response, request, 6U); payloadLength = 6U; }
      break;

    case 0x0FU:
      quantity = (uint16_t)(((uint16_t)request[4] << 8) | request[5]);
      if(quantity == 0U || quantity > 1968U || request[6] != (uint8_t)((quantity + 7U) / 8U)) payloadLength = exceptionResponse(function, MB_EX_ILLEGAL_VALUE);
      else if(address >= MB_COIL_COUNT || quantity > (uint16_t)(MB_COIL_COUNT - address)) payloadLength = exceptionResponse(function, MB_EX_ILLEGAL_ADDRESS);
      else
      {
        for(i = 0U; i < quantity; i++) writeCoil(address + i, (uint8_t)((request[7U + i / 8U] >> (i % 8U)) & 1U));
        memcpy(response, request, 6U);
        payloadLength = 6U;
      }
      break;

    case 0x10U:
      quantity = (uint16_t)(((uint16_t)request[4] << 8) | request[5]);
      if(quantity == 0U || quantity > 123U || request[6] != (uint8_t)(quantity * 2U)) payloadLength = exceptionResponse(function, MB_EX_ILLEGAL_VALUE);
      else if(address >= MB_HOLD_REG_COUNT || quantity > (uint16_t)(MB_HOLD_REG_COUNT - address)) payloadLength = exceptionResponse(function, MB_EX_ILLEGAL_ADDRESS);
      else
      {
        for(i = 0U; i < quantity; i++) writeHoldingRegister(address + i, (uint16_t)(((uint16_t)request[7U + i * 2U] << 8) | request[8U + i * 2U]));
        memcpy(response, request, 6U);
        payloadLength = 6U;
      }
      break;

    default:
      payloadLength = exceptionResponse(function, MB_EX_ILLEGAL_FUNCTION);
      break;
  }
  return broadcast != 0U ? 0U : payloadLength;
}

static uint8_t supportedFunction(uint8_t function)
{
  switch(function)
  {
    case 0x01U: case 0x02U: case 0x03U: case 0x04U:
    case 0x05U: case 0x06U: case 0x0FU: case 0x10U:
      return 1U;
    default:
      return 0U;
  }
}

static uint16_t requestLength(const uint8_t *data, uint16_t available)
{
  if(available < 2U) return 0U;
  switch(data[1])
  {
    case 0x01U: case 0x02U: case 0x03U: case 0x04U: case 0x05U: case 0x06U: return 8U;
    case 0x0FU: case 0x10U: return available >= 7U ? (uint16_t)(9U + data[6]) : 0U;
    /* Unknown functions are fixed 8-byte requests.  After CRC validation the
     * request reaches processRequest(), which generates function|0x80, 0x01. */
    default: return 8U;
  }
}

static void processBurst(const uint8_t *data, uint16_t length, uint8_t *debugStages)
{
  uint16_t offset = 0U;

  if(debugStages != NULL)
  {
    *debugStages = 0U;
  }

  /* V1 (NUVELL_V1) per-byte parsing: every error path advances offset by
   * exactly one byte, never by an estimated frame length and never breaks
   * out of the loop. This guarantees that noise or a damaged frame can at
   * most swallow its own bytes; the next valid frame in the same DMA burst
   * is still aligned and parsed. */
  while(offset < length)
  {
    uint16_t available = (uint16_t)(length - offset);
    uint16_t frameLength;
    uint16_t crc;
    uint16_t replyLength;
    uint8_t address = data[offset];

    /* 1. Address must be the slave address or broadcast (0). Otherwise skip
     *    one byte so leading noise cannot hide a later valid frame. */
    if(address != MODBUS_SLAVE_ADDRESS && address != 0U)
    {
      offset++;
      continue;
    }

    /* 2. Need at least 2 bytes to inspect the function code. */
    if(available < 2U)
    {
      modbusFrameErrorCount++;
      break;
    }

    /* 3. Function code must be in the supported whitelist. An unsupported
     *    function with a valid address is still a protocol error: skip one
     *    byte here; if a full valid-length frame with bad function code
     *    survives CRC, processRequest() returns exception 0x01. */
    if(supportedFunction(data[offset + 1U]) == 0U)
    {
      modbusFrameErrorCount++;
      offset++;
      continue;
    }

    /* 4. Determine the expected frame length from the function code. */
    frameLength = requestLength(&data[offset], available);
    if(frameLength == 0U || frameLength > available)
    {
      modbusFrameErrorCount++;
      offset++;
      continue;
    }

    /* 5. CRC check (Modbus RTU uses little-endian CRC at frame tail). */
    crc = (uint16_t)(data[offset + frameLength - 2U] |
                     ((uint16_t)data[offset + frameLength - 1U] << 8));
    if(crc != Modbus_CRC16(&data[offset], (uint16_t)(frameLength - 2U)))
    {
      if(debugStages != NULL) *debugStages |= MB_LOG_STAGE_C;
      modbusCrcErrorCount++;
      offset++;
      continue;
    }

    /* 6. Valid frame: process and send the response. */
    if(debugStages != NULL) *debugStages |= MB_LOG_STAGE_V;
    modbusRxTotalCount++;
    replyLength = processRequest(&data[offset], address == 0U);
    if(replyLength != 0U)
    {
      crc = Modbus_CRC16(response, replyLength);
      response[replyLength++] = (uint8_t)crc;
      response[replyLength++] = (uint8_t)(crc >> 8);
      if(debugStages != NULL) *debugStages |= MB_LOG_STAGE_S;
      (void)modbusRs485Send(response, replyLength);
      if(debugStages != NULL) *debugStages |= MB_LOG_STAGE_D;
      modbusReplyTotalCount++;
    }
    /* A valid frame was consumed; continue at the following byte/frame. */
    offset = (uint16_t)(offset + frameLength);
  }
}

void Modbus_Poll(void)
{
  uint16_t slot;

  while(xQueueReceive(modbusData_handle, &slot, 0U) == pdPASS)
  {
    if(slot >= MB_RX_QUEUE_DEPTH)
    {
      modbusFrameErrorCount++;
      continue;
    }
    if(!modbusDebugSuppress(rxQueue[slot].data, rxQueue[slot].length))
    {
      uint8_t stages = (uint8_t)(MB_LOG_STAGE_Q | MB_LOG_STAGE_T);
      processBurst(rxQueue[slot].data, rxQueue[slot].length, &stages);
      modbusDebugLogFrame(rxQueue[slot].data, rxQueue[slot].length, stages);
    }
    else
    {
      processBurst(rxQueue[slot].data, rxQueue[slot].length, NULL);
    }
    rxTail = (uint8_t)((slot + 1U) % MB_RX_QUEUE_DEPTH);
  }
}

void my_task01_func(void *pvParameters)
{
  uint16_t slot;
  static bool versionPrinted = false;

  (void)pvParameters;
  /* main() performs the one-time USART3/PB2/DMA setup before the scheduler
   * starts.  This task only waits for and processes completed RTU frames. */

  /* 版本帧在任务上下文（调度器已启动、队列已创建）中发送，避免
   * modbusinit() 在调度器就绪前发送时 IDLE 中断触发 xQueueSendFrom_ISR(NULL)
   * 导致 HardFault。仅首次运行时发送一次。 */
  if(!versionPrinted)
  {
    versionPrinted = true;
    modbusPrintVersion();
  }

  for(;;)
  {
    if(xQueueReceive(modbusData_handle, &slot, portMAX_DELAY) == pdPASS)
    {
      if(slot < MB_RX_QUEUE_DEPTH)
      {
        if(!modbusDebugSuppress(rxQueue[slot].data, rxQueue[slot].length))
        {
          uint8_t stages = (uint8_t)(MB_LOG_STAGE_Q | MB_LOG_STAGE_T);
          processBurst(rxQueue[slot].data, rxQueue[slot].length, &stages);
          modbusDebugLogFrame(rxQueue[slot].data, rxQueue[slot].length, stages);
        }
        else
        {
          processBurst(rxQueue[slot].data, rxQueue[slot].length, NULL);
        }
        rxTail = (uint8_t)((slot + 1U) % MB_RX_QUEUE_DEPTH);
      }
      else
      {
        modbusFrameErrorCount++;
      }
    }
  }
}
