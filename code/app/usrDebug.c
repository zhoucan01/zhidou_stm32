#include "usrDebug.h"

/* 非关键串口诊断信息和心跳输出辅助函数。 */
#include "wk_dma.h"
#include "wk_usart.h"
#include "at32f415_usart.h"
#include "at32f415_dma.h"
#include "log.h"
#include <string.h>
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"

#if (USRDEBUG_AT_ENABLE != 0U)
#include "moter28.h"
#include "moter42.h"
#include "moter57.h"
#include "AT8236.h"
#include "LED.h"
#include "ws2812.h"
#include "totolCtl.h"
#include "getBusVol.h"
#include "current.h"
#include "enc.h"
#include "FT.h"
#include "tmux1308.h"
#include "Tempurature.h"
#include "wk_debug.h"
#include "../task/modbusTask.h"
#include <stdlib.h>

/* moter28.c 里的全局步进计数（未在 moter28.h 中声明，这里补一个 extern）。
 * 单位是"步"（一圈 4096 步），值与 moter28RunStep() 同步递增。 */
extern int32_t currTurns;
#endif

/* UART5 RX uses DMA1 channel2 in non-circular mode. On each IDLE interrupt
 * the DMA is stopped, the received length is read from dtcnt, the bytes are
 * latched into a local frame buffer, then the DMA buffer is reloaded and the
 * channel is restarted for the next frame. This mirrors the proven V1
 * (NUVELL_V1) approach used by modbusUsart3IdleIsr() for USART3/DMA1_CH1,
 * and avoids the circular-DMA position tracking that caused data-order
 * corruption. */
static uint8_t uart5DmaBuffer[USRDEBUG_RX_BUF_SIZE];

/* 帧队列：ISR 写入空槽，usrDebugGetFrame() 取出已写入的槽。
 * 解决上位机连发多条查询时单帧握手丢帧的问题。
 * 4 个固定帧槽，每槽最大 256 字节，调试口连发 11 条查询通常合并成 1-2 帧，足够。 */
#define USRDEBUG_RING_FRAME_MAX   4U
static uint8_t  ringFrames[USRDEBUG_RING_FRAME_MAX][USRDEBUG_RX_BUF_SIZE];
static volatile uint16_t ringLens[USRDEBUG_RING_FRAME_MAX];
static volatile uint16_t ringHead = 0U;   /* ISR 下一个写入槽 */
static volatile uint16_t ringTail = 0U;   /* poll 下一个读取槽 */
static volatile uint16_t ringCount = 0U; /* 队列中帧数 */

static bool uart5PortInitialized = false;

/* 毫秒级时基（实现在 wk_system.c，声明在 wk_debug.h 的用户代码区）。 */
extern uint32_t wk_timebase_get(void);

void usrDebugInit(void)
{
  if(uart5PortInitialized)
  {
    return;
  }

  /* DMA1_CH2 mode and request selection are initialized once by
   * wk_dma1_channel2_init(). Workbench's generated main() initially enables
   * this channel with its placeholder address/length (both are zero here).
   * DMA address/count registers must only be replaced while the channel is
   * disabled; otherwise IDLE can occur with an invalid buffer even though the
   * USART interrupt itself is working. 与 modbusinit() 对 DMA1_CH1 的处理一致。 */
  dma_channel_enable(DMA1_CHANNEL2, FALSE);
  wk_dma_channel_config(DMA1_CHANNEL2, (uint32_t)&UART5->dt,
                        (uint32_t)uart5DmaBuffer, USRDEBUG_RX_BUF_SIZE);
  dma_channel_enable(DMA1_CHANNEL2, TRUE);
  uart5PortInitialized = true;

  log_d("UART5 DMA RX init: buf=%u @0x%08lX", (unsigned int)USRDEBUG_RX_BUF_SIZE,
        (unsigned long)(uint32_t)uart5DmaBuffer);
}

void usrDebugUart5IdleIsr(void)
{
  uint16_t rxLength;

  /* Stop DMA first so dtcnt remains stable while the received length is
   * read. This is the V1 (NUVELL_V1) pattern: non-circular DMA, stop → read
   * length → clear IDLE → process → reload → restart. It avoids the
   * circular-DMA position tracking that caused data-order corruption
   * (circular left shift). 与 modbusUsart3IdleIsr() 完全一致。 */
  dma_channel_enable(DMA1_CHANNEL2, FALSE);
  rxLength = (uint16_t)(USRDEBUG_RX_BUF_SIZE - DMA1_CHANNEL2->dtcnt);

  /* Clear the IDLE flag AFTER stopping DMA. usart_flag_clear() reads STS
   * then DT; if DMA is still running, reading DT steals a byte from the
   * receive register and corrupts the DMA buffer contents (every byte ends
   * up with bit7 set and wrong length). V1 clears IDLE here, after DMA is
   * stopped, so the DT read is harmless. The caller (UART5_IRQHandler)
   * must NOT clear IDLE before calling this function. */
  usart_flag_clear(UART5, USART_IDLEF_FLAG);

  if(rxLength > 0U && rxLength <= USRDEBUG_RX_BUF_SIZE)
  {
    /* 帧队列：写入下一个空槽，队列满时丢弃该帧（调试口流量低，极少发生）。 */
    if(ringCount < USRDEBUG_RING_FRAME_MAX)
    {
      uint16_t i;
      uint16_t slot = ringHead;
      for(i = 0U; i < rxLength; i++)
      {
        ringFrames[slot][i] = uart5DmaBuffer[i];
      }
      ringLens[slot] = rxLength;
      ringHead = (uint16_t)((slot + 1U) % USRDEBUG_RING_FRAME_MAX);
      ringCount++;
    }
  }

  /* Reload the same buffer for the next frame and restart DMA. */
  DMA1_CHANNEL2->dtcnt = USRDEBUG_RX_BUF_SIZE;
  DMA1_CHANNEL2->maddr = (uint32_t)uart5DmaBuffer;
  dma_channel_enable(DMA1_CHANNEL2, TRUE);
}

bool usrDebugGetFrame(uint8_t *outData, uint16_t *outLen)
{
  uint16_t len;
  uint16_t i;
  uint16_t slot;

  if(outData == NULL || outLen == NULL)
  {
    return false;
  }

  if(ringCount == 0U)
  {
    return false;
  }

  slot = ringTail;
  len = ringLens[slot];
  if(len > USRDEBUG_RX_BUF_SIZE)
  {
    len = USRDEBUG_RX_BUF_SIZE;
  }
  for(i = 0U; i < len; i++)
  {
    outData[i] = ringFrames[slot][i];
  }
  *outLen = len;

  /* 取走后推进 tail，允许 ISR 复用该槽。 */
  ringTail = (uint16_t)((slot + 1U) % USRDEBUG_RING_FRAME_MAX);
  ringCount--;
  return true;
}

void usrDebugWrite(const uint8_t *data, uint16_t len)
{
  uint16_t i;

  if(data == NULL)
  {
    return;
  }
  for(i = 0U; i < len; i++)
  {
    wk_debug_putc(data[i]);
  }
}

#if (USRDEBUG_AT_ENABLE != 0U)

/* ============================ AT 指令解析 ============================ */

#define USRDEBUG_AT_INTERVAL_MS   10U
#define USRDEBUG_HEARTBEAT_INTERVAL_MS 1000U
/* 超过该间隔没有新数据则认为"半截行"作废，重新同步 */
#define USRDEBUG_AT_RESYNC_MS   1000U

/* 行缓冲：跨帧累积，直到遇到 '\n' 或 '\r' 才作为一整条指令处理 */
static char     atLineBuf[USRDEBUG_AT_LINE_MAX];
static uint16_t atLineLen = 0U;
static uint8_t  atLineOverflow = 0U;   /* 本行太长，丢弃直到下一个换行 */
static uint32_t atLastDataMs = 0U;
static uint8_t  atHaveDataMs = 0U;

/* Modbus 统计周期打印开关与定时。
 * NB+stat=on 打开，NB+stat=off 关闭，NB+stat=now 立即打印一次。
 * 打开时每 10 秒通过 UART5 输出一行统计信息。 */
static uint8_t  statPrintEnabled = 0U;
static uint32_t statLastPrintMs = 0U;
#define USRDEBUG_STAT_PRINT_INTERVAL_MS  10000U

/* 打印一行 Modbus 统计信息到 UART5。
 * 字段：rx=有效请求帧总数, reply=应答帧总数, crcErr=CRC错误数,
 *       frameErr=地址/功能码/长度错误数, qOvf=队列满丢弃数。 */
static void usrDebugPrintModbusStat(void)
{
  (void)printf("NB+stat,rx=%lu,reply=%lu,crcErr=%lu,frameErr=%lu,qOvf=%lu\r\n",
               (unsigned long)modbusRxTotalCount,
               (unsigned long)modbusReplyTotalCount,
               (unsigned long)modbusCrcErrorCount,
               (unsigned long)modbusFrameErrorCount,
               (unsigned long)modbusRxQueueOverflow);
}

/* ---- 57 电流表标定：周期打印转速/幅值/位置/编码器增量 ----
 * NB+m57=watch,on | NB+m57=watch,off | NB+m57=watch,<周期ms>（默认 50ms）
 *
 * 每周期打印一行：
 *   NB+m57=watch,cur=<瞬时指令转速×10>,amp=<当前幅值%>,
 *                pos=<指令微步位置>,denc=<本周期压制编码器增量>,
 *                adcA/adcB=<绕组原始ADC>,zA/zB=<零点基准>,
 *                ia=<绕组A电流A>,ib=<绕组B电流A>
 * cur 取瞬时指令值而不是目标值：Run() 走 8 秒 S 曲线，途中两者不同。
 * denc 是电机实际走过的编码器脉冲数，定速时应保持平稳；掉速或跳变即为
 * 该时刻的丢步/抖动，用来定位 SP_57_8412_AmpTable 的问题转速区间。 */
static uint8_t  m57WatchEnabled  = 0U;
static uint16_t m57WatchPeriodMs = 50U;
static uint32_t m57WatchLastMs   = 0U;
static int32_t  m57WatchLastEnc  = 0;

#define USRDEBUG_M57_WATCH_PERIOD_MIN_MS    10U
#define USRDEBUG_M57_WATCH_PERIOD_MAX_MS  1000U

/* 开启打印时重取基准，避免第一行的 denc 从上电以来的累计值起算。 */
static void usrDebugM57WatchArm(void)
{
  m57WatchLastEnc = encPressMoterREncPositionGet();
  m57WatchLastMs  = (uint32_t)xTaskGetTickCount();
}

static void usrDebugM57WatchPrint(void)
{
  int32_t encNow = encPressMoterREncPositionGet();
  int32_t encDelta = encNow - m57WatchLastEnc;

  /* 57 绕组电流：ADC1 注入组在 40kHz 中断里把 pdt1/pdt2 锁存到 adcv[]，
   * 这里按零点基准换算成安培。只作趋势判据，不参与控制。 */
  getMoter57Curr();
  m57WatchLastEnc = encNow;
  (void)printf("NB+m57=watch,cur=%u,amp=%u,pos=%ld,denc=%ld,"
               "adcA=%u,adcB=%u,zA=%u,zB=%u,ia=%.3f,ib=%.3f\r\n",
               (unsigned)StepperPulse57_8412_GetCurrentRPM(),
               (unsigned)StepperPulse57_8412_GetAmplitude(),
               (long)StepperPulse57_8412_GetPosition(),
               (long)encDelta,
               (unsigned)getMoter57AdcA(),
               (unsigned)getMoter57AdcB(),
               (unsigned)StepperPulse57_8412_CurrA_Zero,
               (unsigned)StepperPulse57_8412_CurrB_Zero,
               (double)moter57CurrA,
               (double)moter57CurrB);
}

/* ---- 57 电流表标定：自动往返扫描 ----
 * NB+m57=sweep,<止rpm>,<加速ms>,<保持ms>,<循环次数>
 *
 * 每轮时序：静止起步 → S 曲线加速到 <止rpm> → 保持 <保持ms> →
 *           硬停（切电流） → 保持 <保持ms> → 下一轮
 * <循环次数> = 0 表示无限循环，直到收到 NB+m57=stop。
 *
 * 一轮就覆盖整张 SP_57_8412_AmpTable，且每轮都含一次静止起步，
 * 用来暴露低档电流不足导致的起步丢步。注意硬停后电机靠摩擦自由滑行，
 * 停止那一刻的 log 会出现 denc 仍在涨而 pos 冻结，判读时要跳过该行。 */
#define USRDEBUG_M57_SWEEP_IDLE     0U
#define USRDEBUG_M57_SWEEP_RAMPING  1U
#define USRDEBUG_M57_SWEEP_HOLD     2U
#define USRDEBUG_M57_SWEEP_PAUSE    3U

#define USRDEBUG_M57_SWEEP_ACCEL_MIN_MS     500U
#define USRDEBUG_M57_SWEEP_ACCEL_MAX_MS   30000U
#define USRDEBUG_M57_SWEEP_HOLD_MIN_MS      100U
#define USRDEBUG_M57_SWEEP_HOLD_MAX_MS    60000U
/* 加速超时保护：S 曲线走完后 IsRamping() 应清零，超时则强制进入保持。 */
#define USRDEBUG_M57_SWEEP_GUARD_EXTRA_MS  3000U

static uint8_t  m57SweepState   = USRDEBUG_M57_SWEEP_IDLE;
static uint32_t m57SweepPhaseMs = 0U;
static uint32_t m57SweepAccelMs = 8000U;
static uint32_t m57SweepHoldMs  = 1000U;
static uint32_t m57SweepCycles  = 0U;
static signed int m57SweepRpm   = 0;

/* 该方向是否已到限位：仅供调试口查询/上报，不承担保护职责。
 * 实际拦截在底层 StepperPulse57_8412_RunAtSpeed()（main 的限位实现）。
 *   rpm_x10 — 指令转速×10，正=下行，负=上行
 *   返回 1=该方向已到限位 */
static uint8_t usrDebugM57LimitBlocked(signed int rpm_x10)
{
  if(rpm_x10 > 0)
  {
    return pressMoterLowPositionGet();
  }
  if(rpm_x10 < 0)
  {
    return pressMoterHighPositionGet();
  }
  return 1U;
}

/* 中止扫描并立即停机；手动 run/stop 命令也会调它。 */
static void usrDebugM57SweepAbort(void)
{
  m57SweepState = USRDEBUG_M57_SWEEP_IDLE;
}

static void usrDebugM57SweepStart(signed int rpm_x10, unsigned int accelMs,
                                  uint32_t holdMs, uint32_t cycles)
{
  m57SweepRpm     = rpm_x10;
  m57SweepAccelMs = accelMs;
  m57SweepHoldMs  = holdMs;
  m57SweepCycles  = cycles;
  m57SweepPhaseMs = (uint32_t)xTaskGetTickCount();
  StepperPulse57_8412_RunAutoTableEx(rpm_x10, accelMs);
  m57SweepState = USRDEBUG_M57_SWEEP_RAMPING;
}

static void usrDebugM57SweepPoll(uint32_t nowMs)
{
  if(m57SweepState == USRDEBUG_M57_SWEEP_IDLE)
  {
    return;
  }

  /* 限位保护：撞到限位后 pressMoterMonitor() 已把电机停住，
   * 这里中止扫描并报出触发方向，避免状态机空转。 */
  if(usrDebugM57LimitBlocked(m57SweepRpm) != 0U)
  {
    StepperPulse57_8412_Stop();
    usrDebugM57SweepAbort();
    (void)printf("NB+m57=sweep,abort,%s\r\n", (m57SweepRpm < 0) ? "high" : "low");
    return;
  }

  if(m57SweepState == USRDEBUG_M57_SWEEP_RAMPING)
  {
    if(StepperPulse57_8412_IsRamping() == 0U ||
       (nowMs - m57SweepPhaseMs) >=
         (m57SweepAccelMs + USRDEBUG_M57_SWEEP_GUARD_EXTRA_MS))
    {
      m57SweepPhaseMs = nowMs;
      m57SweepState = USRDEBUG_M57_SWEEP_HOLD;
    }
    return;
  }

  if(m57SweepState == USRDEBUG_M57_SWEEP_HOLD)
  {
    if((nowMs - m57SweepPhaseMs) >= m57SweepHoldMs)
    {
      StepperPulse57_8412_Stop();
      m57SweepPhaseMs = nowMs;
      m57SweepState = USRDEBUG_M57_SWEEP_PAUSE;
    }
    return;
  }

  /* PAUSE：停机等待结束后决定是否再跑一轮。 */
  if((nowMs - m57SweepPhaseMs) >= m57SweepHoldMs)
  {
    if(m57SweepCycles != 0U)
    {
      if(m57SweepCycles <= 1U)
      {
        m57SweepState = USRDEBUG_M57_SWEEP_IDLE;
        return;
      }
      m57SweepCycles--;
    }
    m57SweepPhaseMs = nowMs;
    StepperPulse57_8412_RunAutoTableEx(m57SweepRpm, m57SweepAccelMs);
    m57SweepState = USRDEBUG_M57_SWEEP_RAMPING;
  }
}

/* ---- 42 抬升电机标定：周期打印转速/双幅值/位置/双编码器增量/左电机电流 ----
 * NB+rise=watch,on | NB+rise=watch,off | NB+rise=watch,<周期ms>（默认 50ms）
 *
 * 每周期打印一行：
 *   NB+rise=watch,cur=<瞬时指令转速×10>,ampL=<左幅值%>,ampR=<右幅值%>,
 *                 pos=<指令微步位置>,dencL=<左编码器增量>,dencR=<右编码器增量>,
 *                 ia=<左电机A相电流A>,ib=<左电机B相电流A>,
 *                 iaR=<右电机A相电流A>,ibR=<右电机B相电流A>,
 *                 dir=<+1上行/-1下行>,ramp=<1=S曲线进行中>,hi=<高位>,lo=<低位>,
 *                 encL=<左编码器绝对位置>,encR=<右编码器绝对位置>,
 *                 dev=<encL-encR 左右偏差>
 * cur 取 S 曲线瞬时指令值而不是目标值：抬升走 5 秒 S 曲线，途中两者不同。
 * dencL/dencR 是两侧本周期实际走过的编码器脉冲数，正常时两侧应同步平稳；一侧掉速
 * 或两侧分叉即该时刻的丢步/抖动，用来定位 SP_42_Left/RightAmpTable 的问题转速区间。
 * dev 是两侧绝对位置之差：同向模式看左右是否跟得上，反向模式应恒定不变。
 * hi/lo 用来确认上行到高位、下行到低位时停机逻辑是否按预期触发，ramp 看 S 曲线是否走完。
 * ia/ib/iaR/ibR 来自 TMUX1308 D1，每 20ms 刷新一次，只作趋势判据
 * （不具备 57 的 40kHz 波形分辨率）。 */
static uint8_t  riseWatchEnabled  = 0U;
static uint16_t riseWatchPeriodMs = 50U;
static uint32_t riseWatchLastMs   = 0U;
static int32_t  riseWatchLastEncL = 0;
static int32_t  riseWatchLastEncR = 0;

#define USRDEBUG_RISE_WATCH_PERIOD_MIN_MS    10U
#define USRDEBUG_RISE_WATCH_PERIOD_MAX_MS  1000U

/* rise=wave 突发采样的默认值/上限。 */
#define USRDEBUG_RISE_WAVE_DEFAULT_N            1024U
#define USRDEBUG_RISE_WAVE_DEFAULT_INTERVAL_US    50U
#define USRDEBUG_RISE_WAVE_MAX_INTERVAL_US       200U
/* 整段窗口封顶 200 ms：超了就压缩采样间隔，避免长时间堵住 Modbus 快照。 */
#define USRDEBUG_RISE_WAVE_MAX_WINDOW_US      200000U

/* 开启打印时重取基准，避免第一行的 denc 从上电以来的累计值起算。 */
static void usrDebugRiseWatchArm(void)
{
  riseWatchLastEncL = encRiseMoterLEncPositionGet();
  riseWatchLastEncR = encRiseMoterREncPositionGet();
  riseWatchLastMs   = (uint32_t)xTaskGetTickCount();
}

static void usrDebugRiseWatchPrint(void)
{
  int32_t encLNow = encRiseMoterLEncPositionGet();
  int32_t encRNow = encRiseMoterREncPositionGet();
  int32_t encLDelta = encLNow - riseWatchLastEncL;
  int32_t encRDelta = encRNow - riseWatchLastEncR;

  riseWatchLastEncL = encLNow;
  riseWatchLastEncR = encRNow;
  (void)printf("NB+rise=watch,cur=%u,ampL=%u,ampR=%u,pos=%ld,dencL=%ld,dencR=%ld,"
               "ia=%.3f,ib=%.3f,iaR=%.3f,ibR=%.3f,dir=%d,ramp=%u,hi=%u,lo=%u,encL=%ld,encR=%ld,dev=%ld\r\n",
               (unsigned)StepperPulse42_GetCurrentRPM(),
               (unsigned)StepperPulse42_GetLeftAmplitude(),
               (unsigned)StepperPulse42_GetRightAmplitude(),
               (long)StepperPulse42_GetPosition(),
               (long)encLDelta,
               (long)encRDelta,
               (double)currentData.riseLeftAAmpere,
               (double)currentData.riseLeftBAmpere,
               (double)currentData.riseRightAAmpere,
               (double)currentData.riseRightBAmpere,
               (int)StepperPulse42_GetDirection(),
               (unsigned)StepperPulse42_IsRamping(),
               (unsigned)riseMoterHighPositionGet(),
               (unsigned)riseMoterLowPositionGet(),
               (long)encLNow,
               (long)encRNow,
               (long)(encLNow - encRNow));
}

/* 应答一条指令：收到什么回什么，后面跟 ",OK"，再补 CRLF。
 * 例如收到 "NB+moter28 = up" → 回 "NB+moter28=up,OK\r\n" */
static void usrDebugAtAck(const char *cmd, const char *unit)
{
  (void)printf("%s,%s\r\n", cmd, unit);
}

static void usrDebugAtError(const char *reason)
{
  (void)printf("NB+ERR,%s\r\n", reason);
}

/* 去掉字符串首尾空白，返回首指针，原地截断尾部 */
static char *usrDebugAtTrim(char *s)
{
  char *end;
  char *p = s;

  if(s == NULL)
  {
    return NULL;
  }
  while(*p == ' ' || *p == '\t')
  {
    p++;
  }
  end = p + strlen(p);
  while(end > p && (end[-1] == ' ' || end[-1] == '\t'))
  {
    end--;
  }
  *end = '\0';
  return p;
}

/* 大小写不敏感比较（ARMCC5 无 strcasecmp） */
static uint8_t usrDebugAtEqIgnoreCase(const char *a, const char *b)
{
  uint8_t ca;
  uint8_t cb;

  while(*a != '\0' && *b != '\0')
  {
    ca = (uint8_t)*a;
    cb = (uint8_t)*b;
    if(ca >= 'A' && ca <= 'Z') ca = (uint8_t)(ca + ('a' - 'A'));
    if(cb >= 'A' && cb <= 'Z') cb = (uint8_t)(cb + ('a' - 'A'));
    if(ca != cb)
    {
      return 0U;
    }
    a++;
    b++;
  }
  return ((*a == '\0') && (*b == '\0')) ? 1U : 0U;
}

/* "NB+" 前缀，大小写不敏感 */
static uint8_t usrDebugAtHasPrefix(const char *s)
{
  return (uint8_t)((s[0] == 'N' || s[0] == 'n') &&
                   (s[1] == 'B' || s[1] == 'b') &&
                   (s[2] == '+'));
}

/* 解析 "键 = 值"：定位 '='，把 '=' 换成 '\0' 便于分别比较。
 * 返回 0 成功；1 没有 '='；2 空键；3 空值。 */
static uint8_t usrDebugAtSplit(char *line, char **outKey, char **outVal)
{
  char *eq = strchr(line, '=');

  if(eq == NULL)
  {
    return 1U;
  }
  *eq = '\0';
  *outKey = usrDebugAtTrim(line);
  *outVal = usrDebugAtTrim(eq + 1);
  if(**outKey == '\0')
  {
    return 2U;
  }
  if(**outVal == '\0')
  {
    return 3U;
  }
  return 0U;
}

/* 把字符串解析为 long，跳过前导空白。ARMCC5 的 stdlib 有 atol，
 * 但它不跳前导空白，这里先 trim 再调。失败时 *ok=0。 */
static long usrDebugAtParseLong(const char *s, uint8_t *ok)
{
  char *end = NULL;
  long v;

  if(s == NULL || *s == '\0' || ok == NULL)
  {
    if(ok != NULL) *ok = 0U;
    return 0L;
  }
  v = strtol(s, &end, 10);
  if(end != NULL && *end == '\0')
  {
    *ok = 1U;
    return v;
  }
  *ok = 0U;
  return 0L;
}

/* 把字符串解析为 0~100 的 unsigned char（电流幅值百分比）。
 * 越界自动限幅；非数字返回 0 且 *ok=0。 */
static unsigned char usrDebugAtParseAmp(const char *s, uint8_t *ok)
{
  long v = usrDebugAtParseLong(s, ok);

  if(*ok == 0U)
  {
    return 0U;
  }
  if(v < 0L) v = 0L;
  if(v > 100L) v = 100L;
  return (unsigned char)v;
}

/* 把字符串解析为 -100~100 的 int8（泵速度百分比）。 */
static int8_t usrDebugAtParsePumpSpeed(const char *s, uint8_t *ok)
{
  long v = usrDebugAtParseLong(s, ok);

  if(*ok == 0U)
  {
    return 0;
  }
  if(v < -100L) v = -100L;
  if(v > 100L) v = 100L;
  return (int8_t)v;
}

/* 把字符串解析为 0~255 的 uint8（WS2812 RGB 分量）。 */
static uint8_t usrDebugAtParseU8(const char *s, uint8_t *ok)
{
  long v = usrDebugAtParseLong(s, ok);

  if(*ok == 0U)
  {
    return 0U;
  }
  if(v < 0L) v = 0L;
  if(v > 255L) v = 255L;
  return (uint8_t)v;
}

/* 抬升电流通道名 -> 下标：la/lb/ra/rb 或 0~3。成功返回 1 并置 *ok=1。 */
static uint8_t usrDebugAtParseRiseWaveCh(const char *s, uint8_t *ch, uint8_t *ok)
{
  uint8_t numOk = 0U;
  long v;

  if(ch == NULL || ok == NULL)
  {
    return 0U;
  }
  *ok = 0U;
  if(s == NULL)
  {
    return 0U;
  }
  if(usrDebugAtEqIgnoreCase(s, "la") != 0U) { *ch = 0U; *ok = 1U; return 1U; }
  if(usrDebugAtEqIgnoreCase(s, "lb") != 0U) { *ch = 1U; *ok = 1U; return 1U; }
  if(usrDebugAtEqIgnoreCase(s, "ra") != 0U) { *ch = 2U; *ok = 1U; return 1U; }
  if(usrDebugAtEqIgnoreCase(s, "rb") != 0U) { *ch = 3U; *ok = 1U; return 1U; }
  v = usrDebugAtParseLong(s, &numOk);
  if(numOk != 0U && v >= 0L && v <= 3L)
  {
    *ch = (uint8_t)v;
    *ok = 1U;
    return 1U;
  }
  return 0U;
}

/* 一次性切分逗号字段并原地去除空白。
 * 不能每次按索引重复切分：第一次把逗号改成 '\0' 后，后续字段将无法搜索。
 * 例如 "run,3850,30" 必须一次得到 ["run", "3850", "30"]。 */
static uint8_t usrDebugAtSplitFields(char *val, char **fields, uint8_t maxFields)
{
  uint8_t count = 0U;
  char *p = val;

  if(val == NULL || fields == NULL || maxFields == 0U)
  {
    return 0U;
  }

  while(count < maxFields)
  {
    char *comma = strchr(p, ',');
    fields[count++] = usrDebugAtTrim(p);
    if(comma == NULL)
    {
      break;
    }
    *comma = '\0';
    p = comma + 1;
  }
  return count;
}

/* ---- 泵短命令别名（与 cbt7 工程兼容），与 "NB+" 键值命令共存 ----
 *
 *   W<n> / H<n> / B<n>  清水 / 蠕动 / 点卤泵，档位 n = 1~10 → 10%~100%
 *   <n> = 0 关闭对应泵；S 三泵全停
 * 大小写不敏感；行尾可用 \n / \r / \r\n。
 * 只有"字母 + 纯档位数字"或单独的 S 会被认领，其它一律返回 0 交给 NB+ 解析，
 * 因此不会影响任何 NB+ 指令。速度仍走 AT8236 的 0.5s S 曲线。
 * 成功回 "NB+OK,<命令>"；档位 >10 或超过两位数字回 NB+ERR,short-gear。
 * 反向调速请用 NB+pump=hose,<负值>。 */
static uint8_t usrDebugShortCommand(char *line)
{
  char letter;
  const char *p;
  uint8_t gear = 0U;
  uint8_t digits = 0U;
  uint8_t speed;

  if(line == NULL || line[0] == '\0')
  {
    return 0U;
  }

  letter = line[0];
  if(letter == 'w' || letter == 'W') letter = 'W';
  else if(letter == 'h' || letter == 'H') letter = 'H';
  else if(letter == 'b' || letter == 'B') letter = 'B';
  else if(letter == 's' || letter == 'S') letter = 'S';
  else return 0U;

  p = line + 1;
  if(*p != '\0')
  {
    while(*p >= '0' && *p <= '9')
    {
      gear = (uint8_t)(gear * 10U + (uint8_t)(*p - '0'));
      digits++;
      p++;
      if(digits > 2U)
      {
        usrDebugAtError("short-gear");
        return 1U;
      }
    }
    if(digits == 0U || *p != '\0')
    {
      return 0U;   /* 不是纯档位数字，交给 NB+ 解析 */
    }
    if(gear > 10U)
    {
      usrDebugAtError("short-gear");
      return 1U;
    }
  }
  else if(letter != 'S')
  {
    return 0U;   /* W/H/B 必须带档位 */
  }

  if(letter == 'S')
  {
    waterPumpOff();
    brinePumpOff();
    hosePumpOff();
    (void)printf("NB+OK,S\r\n");
    return 1U;
  }

  if(gear == 0U)
  {
    if(letter == 'W')      waterPumpOff();
    else if(letter == 'H') hosePumpOff();
    else                   brinePumpOff();
    (void)printf("NB+OK,%c0\r\n", letter);
    return 1U;
  }

  speed = (uint8_t)(gear * 10U);
  if(letter == 'W')      waterPumpOn((int8_t)speed);
  else if(letter == 'H') hosePumpOn((int8_t)speed);
  else                   brinePumpOn((int8_t)speed);

  (void)printf("NB+OK,%c%u\r\n", letter, (unsigned)gear);
  return 1U;
}

/* 处理一条完整的（已去首尾空白的）指令行 */
static void usrDebugAtHandleLine(char *line)
{
  char *key;
  char *val;
  char *body;
  uint8_t r;

  if(*line == '\0')
  {
    /* 空行静默：串口工具行尾 "\r\n" 会多产生一条空行，回错只会刷屏。 */
    return;
  }
  /* 先尝试泵短命令（W8/H5/B10/S），未命中再走 NB+ 键值命令。 */
  if(usrDebugShortCommand(line) != 0U)
  {
    return;
  }
  if(usrDebugAtHasPrefix(line) == 0U)
  {
    usrDebugAtError("prefix");
    return;
  }
  body = line + 3;   /* 跳过 "NB+" */

  r = usrDebugAtSplit(body, &key, &val);
  if(r == 1U) { usrDebugAtError("no-eq");  return; }
  if(r == 2U) { usrDebugAtError("no-key"); return; }
  if(r == 3U) { usrDebugAtError("no-val"); return; }

  /* ============== 28 步进 / 阀门 ============== */
  if(usrDebugAtEqIgnoreCase(key, "moter28") != 0U)
  {
    if(usrDebugAtEqIgnoreCase(val, "up") != 0U)
    {
      moter28Forward();
      usrDebugAtAck("NB+moter28=up", "OK");
    }
    else if(usrDebugAtEqIgnoreCase(val, "down") != 0U)
    {
      moter28Backward();
      usrDebugAtAck("NB+moter28=down", "OK");
    }
    else if(usrDebugAtEqIgnoreCase(val, "stop") != 0U)
    {
      moter28Stop();
      usrDebugAtAck("NB+moter28=stop", "OK");
    }
    else if(usrDebugAtEqIgnoreCase(val, "turn") != 0U)
    {
      /* 返回当前步数（一圈 4096 步） */
      (void)printf("NB+moter28=turn,%ld\r\n", (long)currTurns);
    }
    else if(usrDebugAtEqIgnoreCase(val, "pos") != 0U)
    {
      /* 返回阀门软件到位标志：on=开位，off=关位。 */
      (void)printf("NB+moter28=pos,on=%u,off=%u\r\n",
                   valveOnPos ? 1U : 0U, valveOffPos ? 1U : 0U);
    }
    else
    {
      usrDebugAtError("moter28-val");
    }
    return;
  }

  if(usrDebugAtEqIgnoreCase(key, "valve") != 0U)
  {
    if(usrDebugAtEqIgnoreCase(val, "on") != 0U)
    {
      valveOn();
      usrDebugAtAck("NB+valve=on", "OK");
    }
    else if(usrDebugAtEqIgnoreCase(val, "off") != 0U)
    {
      valveOff();
      usrDebugAtAck("NB+valve=off", "OK");
    }
    else
    {
      usrDebugAtError("valve-val");
    }
    return;
  }

  /* ============== 57 步进电机 ============== */
  if(usrDebugAtEqIgnoreCase(key, "m57") != 0U)
  {
    char *fields[8];
    uint8_t fieldCount = usrDebugAtSplitFields(val, fields, 8U);
    char *sub = (fieldCount > 0U) ? fields[0] : NULL;
    if(sub == NULL) { usrDebugAtError("m57-val"); return; }

    if(usrDebugAtEqIgnoreCase(sub, "stop") != 0U)
    {
      usrDebugM57SweepAbort();
      StepperPulse57_8412_Stop();
      usrDebugAtAck("NB+m57=stop", "OK");
    }
    else if(usrDebugAtEqIgnoreCase(sub, "amp") != 0U)
    {
      uint8_t ok = 0U;
      char *a = (fieldCount > 1U) ? fields[1] : NULL;
      unsigned char amp = usrDebugAtParseAmp(a, &ok);
      if(ok == 0U) { usrDebugAtError("amp"); return; }
      StepperPulse57_8412_SetAmplitude(amp);
      usrDebugAtAck("NB+m57=amp", "OK");
    }
    else if(usrDebugAtEqIgnoreCase(sub, "info") != 0U)
    {
      (void)printf("NB+m57=info,pos=%ld,ang=%d,rpm=%u,amp=%u\r\n",
                   (long)StepperPulse57_8412_GetPosition(),
                   (int)StepperPulse57_8412_GetCurrentAngle(),
                   (unsigned)StepperPulse57_8412_GetCurrentRPM(),
                   (unsigned)StepperPulse57_8412_GetAmplitude());
    }
    else if(usrDebugAtEqIgnoreCase(sub, "run") != 0U)
    {
      uint8_t ok1 = 0U;
      uint8_t ok2 = 0U;
      char *rpmArg = (fieldCount > 1U) ? fields[1] : NULL;
      char *ampArg = (fieldCount > 2U) ? fields[2] : NULL;
      long rpm = usrDebugAtParseLong(rpmArg, &ok1);
      unsigned char amp;

      if(ok1 == 0U) { usrDebugAtError("m57-run-arg"); return; }

      /* 手动运行会接管电机，先中止可能正在跑的扫描。 */
      usrDebugM57SweepAbort();

      /* 限位查询：低位只能上转，高位只能下转（rpm 负=上行）；实际拦截在底层。 */
      if(usrDebugM57LimitBlocked((signed int)rpm) != 0U)
      {
        (void)printf("NB+m57=run,blocked,%s\r\n", (rpm < 0L) ? "high" : "low");
        return;
      }

      /* "auto" 表示按 SP_57_8412_AmpTable 查表设幅值。
       * 不能用数值 101 表达：usrDebugAtParseAmp() 会把 >100 钳到 100，
       * 那反而变成"固定满幅值运行"，正好绕过查表。 */
      if(ampArg != NULL && usrDebugAtEqIgnoreCase(ampArg, "auto") != 0U)
      {
        StepperPulse57_8412_RunAutoTable((signed int)rpm);
        usrDebugAtAck("NB+m57=run", "auto");
        return;
      }

      amp = usrDebugAtParseAmp(ampArg, &ok2);
      if(ok2 == 0U) { usrDebugAtError("m57-run-arg"); return; }
      StepperPulse57_8412_Run((signed int)rpm, amp);
      usrDebugAtAck("NB+m57=run", "OK");
    }
    else if(usrDebugAtEqIgnoreCase(sub, "watch") != 0U)
    {
      char *arg = (fieldCount > 1U) ? fields[1] : NULL;

      if(arg == NULL) { usrDebugAtError("m57-watch-arg"); return; }
      if(usrDebugAtEqIgnoreCase(arg, "off") != 0U)
      {
        m57WatchEnabled = 0U;
        usrDebugAtAck("NB+m57=watch", "off");
      }
      else if(usrDebugAtEqIgnoreCase(arg, "on") != 0U)
      {
        m57WatchEnabled = 1U;
        usrDebugM57WatchArm();
        (void)printf("NB+m57=watch,on,%u\r\n", (unsigned)m57WatchPeriodMs);
      }
      else
      {
        uint8_t ok = 0U;
        long period = usrDebugAtParseLong(arg, &ok);

        if(ok == 0U) { usrDebugAtError("m57-watch-arg"); return; }
        if(period < (long)USRDEBUG_M57_WATCH_PERIOD_MIN_MS)
        {
          period = (long)USRDEBUG_M57_WATCH_PERIOD_MIN_MS;
        }
        if(period > (long)USRDEBUG_M57_WATCH_PERIOD_MAX_MS)
        {
          period = (long)USRDEBUG_M57_WATCH_PERIOD_MAX_MS;
        }
        m57WatchPeriodMs = (uint16_t)period;
        m57WatchEnabled = 1U;
        usrDebugM57WatchArm();
        (void)printf("NB+m57=watch,on,%u\r\n", (unsigned)m57WatchPeriodMs);
      }
    }
    else if(usrDebugAtEqIgnoreCase(sub, "sweep") != 0U)
    {
      uint8_t okRpm = 0U;
      uint8_t okAccel = 0U;
      uint8_t okHold = 0U;
      uint8_t okCycles = 0U;
      long rpm = usrDebugAtParseLong((fieldCount > 1U) ? fields[1] : NULL, &okRpm);
      long accelMs = usrDebugAtParseLong((fieldCount > 2U) ? fields[2] : NULL, &okAccel);
      long holdMs = usrDebugAtParseLong((fieldCount > 3U) ? fields[3] : NULL, &okHold);
      long cycles = usrDebugAtParseLong((fieldCount > 4U) ? fields[4] : NULL, &okCycles);

      /* rpm 正=下行，负=上行；方向决定被哪一侧限位挡住。 */
      if(okRpm == 0U || rpm == 0L || okAccel == 0U ||
         okHold == 0U || okCycles == 0U)
      {
        usrDebugAtError("m57-sweep-arg");
        return;
      }
      if(accelMs < (long)USRDEBUG_M57_SWEEP_ACCEL_MIN_MS)
      {
        accelMs = (long)USRDEBUG_M57_SWEEP_ACCEL_MIN_MS;
      }
      if(accelMs > (long)USRDEBUG_M57_SWEEP_ACCEL_MAX_MS)
      {
        accelMs = (long)USRDEBUG_M57_SWEEP_ACCEL_MAX_MS;
      }
      if(holdMs < (long)USRDEBUG_M57_SWEEP_HOLD_MIN_MS)
      {
        holdMs = (long)USRDEBUG_M57_SWEEP_HOLD_MIN_MS;
      }
      if(holdMs > (long)USRDEBUG_M57_SWEEP_HOLD_MAX_MS)
      {
        holdMs = (long)USRDEBUG_M57_SWEEP_HOLD_MAX_MS;
      }
      if(cycles < 0L)
      {
        cycles = 0L;
      }

      /* 限位查询：低位只能上转，高位只能下转；实际拦截在底层。 */
      if(usrDebugM57LimitBlocked((signed int)rpm) != 0U)
      {
        (void)printf("NB+m57=sweep,blocked,%s\r\n", (rpm < 0L) ? "high" : "low");
        return;
      }

      usrDebugM57SweepStart((signed int)rpm, (unsigned int)accelMs,
                            (uint32_t)holdMs, (uint32_t)cycles);
      (void)printf("NB+m57=sweep,on,%ld,%ld,%ld,%ld\r\n",
                   rpm, accelMs, holdMs, cycles);
    }
    else
    {
      usrDebugAtError("m57-val");
    }
    return;
  }

  /* ============== 42 抬升电机 ============== */
  if(usrDebugAtEqIgnoreCase(key, "m42") != 0U)
  {
    char *fields[3];
    uint8_t fieldCount = usrDebugAtSplitFields(val, fields, 3U);
    char *sub = (fieldCount > 0U) ? fields[0] : NULL;
    if(sub == NULL) { usrDebugAtError("m42-val"); return; }

    if(usrDebugAtEqIgnoreCase(sub, "stop") != 0U)
    {
      StepperPulse42_Stop();
      usrDebugAtAck("NB+m42=stop", "OK");
    }
    else if(usrDebugAtEqIgnoreCase(sub, "amp") != 0U)
    {
      uint8_t okLeft = 0U;
      uint8_t okRight = 0U;
      unsigned char leftAmp = usrDebugAtParseAmp((fieldCount > 1U) ? fields[1] : NULL, &okLeft);
      unsigned char rightAmp;
      if(okLeft == 0U) { usrDebugAtError("amp"); return; }
      if(fieldCount > 2U)
      {
        rightAmp = usrDebugAtParseAmp(fields[2], &okRight);
        if(okRight == 0U) { usrDebugAtError("amp-right"); return; }
        StepperPulse42_SetAmplitudeLeftRight(leftAmp, rightAmp);
      }
      else
      {
        /* 兼容旧命令 NB+m42=amp,<幅值>：左右同时设置。 */
        StepperPulse42_SetAmplitude(leftAmp);
      }
      usrDebugAtAck("NB+m42=amp", "OK");
    }
    else if(usrDebugAtEqIgnoreCase(sub, "info") != 0U)
    {
      (void)printf("NB+m42=info,pos=%ld,ang=%d,ampL=%u,ampR=%u\r\n",
                   (long)StepperPulse42_GetPosition(),
                   (int)StepperPulse42_GetCurrentAngle(),
                   (unsigned)StepperPulse42_GetLeftAmplitude(),
                   (unsigned)StepperPulse42_GetRightAmplitude());
    }
    else if(usrDebugAtEqIgnoreCase(sub, "run") != 0U)
    {
      uint8_t ok1 = 0U, ok2 = 0U;
      long rpm = usrDebugAtParseLong((fieldCount > 1U) ? fields[1] : NULL, &ok1);
      unsigned char amp = usrDebugAtParseAmp((fieldCount > 2U) ? fields[2] : NULL, &ok2);
      if(ok1 == 0U || ok2 == 0U) { usrDebugAtError("m42-run-arg"); return; }
      StepperPulse42_Run((signed int)rpm, amp);
      usrDebugAtAck("NB+m42=run", "OK");
    }
    else if(usrDebugAtEqIgnoreCase(sub, "same") != 0U)
    {
      uint8_t ok1 = 0U, ok2 = 0U;
      long rpm = usrDebugAtParseLong((fieldCount > 1U) ? fields[1] : NULL, &ok1);
      unsigned char amp = usrDebugAtParseAmp((fieldCount > 2U) ? fields[2] : NULL, &ok2);
      if(ok1 == 0U || ok2 == 0U) { usrDebugAtError("m42-same-arg"); return; }
      StepperPulse42_RunSameDirection((signed int)rpm, amp);
      usrDebugAtAck("NB+m42=same", "OK");
    }
    else
    {
      usrDebugAtError("m42-val");
    }
    return;
  }

  /* ============== 抬升机构 ============== */
  if(usrDebugAtEqIgnoreCase(key, "rise") != 0U)
  {
    char *fields[4];
    uint8_t fieldCount = usrDebugAtSplitFields(val, fields, 4U);
    char *sub = (fieldCount > 0U) ? fields[0] : NULL;
    uint8_t speedOk = 0U;
    long speed = (fieldCount > 1U) ? usrDebugAtParseLong(fields[1], &speedOk) : 0L;

    if(sub == NULL) { usrDebugAtError("rise-val"); return; }
    if(fieldCount > 1U &&
       (usrDebugAtEqIgnoreCase(sub, "up") != 0U || usrDebugAtEqIgnoreCase(sub, "down") != 0U) &&
       (speedOk == 0U || speed < 1L || speed > 6000L))
    {
      usrDebugAtError("rise-speed");
      return;
    }

    if(usrDebugAtEqIgnoreCase(sub, "up") != 0U)
    {
      if(fieldCount > 1U) riseMoterUpAtSpeed((uint16_t)speed);
      else riseMoterUp();
      usrDebugAtAck("NB+rise=up", "OK");
    }
    else if(usrDebugAtEqIgnoreCase(sub, "down") != 0U)
    {
      if(fieldCount > 1U) riseMoterDownAtSpeed((uint16_t)speed);
      else riseMoterDown();
      usrDebugAtAck("NB+rise=down", "OK");
    }
    else if(usrDebugAtEqIgnoreCase(sub, "stop") != 0U)
    {
      riseMoterStop();
      usrDebugAtAck("NB+rise=stop", "OK");
    }
    else if(usrDebugAtEqIgnoreCase(sub, "pos") != 0U)
    {
      (void)printf("NB+rise=pos,high=%u,low=%u\r\n",
                   (unsigned)riseMoterHighPositionGet(),
                   (unsigned)riseMoterLowPositionGet());
    }
    else if(usrDebugAtEqIgnoreCase(sub, "mode") != 0U)
    {
      char *mode = (fieldCount > 1U) ? fields[1] : NULL;
      if(mode != NULL && usrDebugAtEqIgnoreCase(mode, "same") != 0U)
      {
        riseMoterSetSameDirection(1U);
        usrDebugAtAck("NB+rise=mode", "same");
      }
      else if(mode != NULL && usrDebugAtEqIgnoreCase(mode, "opposite") != 0U)
      {
        riseMoterSetSameDirection(0U);
        usrDebugAtAck("NB+rise=mode", "opposite");
      }
      else
      {
        usrDebugAtError("rise-mode");
      }
    }
    else if(usrDebugAtEqIgnoreCase(sub, "amp") != 0U)
    {
      uint8_t okLeft = 0U;
      uint8_t okRight = 0U;
      unsigned char leftAmp = usrDebugAtParseAmp((fieldCount > 1U) ? fields[1] : NULL, &okLeft);
      unsigned char rightAmp;
      if(okLeft == 0U) { usrDebugAtError("rise-amp"); return; }
      if(fieldCount > 2U)
      {
        rightAmp = usrDebugAtParseAmp(fields[2], &okRight);
        if(okRight == 0U) { usrDebugAtError("rise-amp-right"); return; }
        riseMoterSetAmplitudeLeftRight(leftAmp, rightAmp);
      }
      else
      {
        riseMoterSetAmplitude(leftAmp);
      }
      usrDebugAtAck("NB+rise=amp", "OK");
    }
    else if(usrDebugAtEqIgnoreCase(sub, "gear") != 0U)
    {
      /* 抬升电流档：0=低档（普通上/下行），非 0=高档（压制流程带载）。
       * 只选档位，幅值数值见 moter42.c；档位粘滞保存，段动作会强制高档。 */
      uint8_t gear = 1U;
      if(fieldCount > 1U && fields[1][0] == '0')
      {
        gear = 0U;
      }
      riseMoterSetLoadGear(gear);
      (void)printf("NB+rise=gear,gear=%u,ampL=%u,ampR=%u\r\n",
                   (unsigned)riseMoterGetLoadGear(),
                   (unsigned)riseMoterGetLeftAmplitude(),
                   (unsigned)riseMoterGetRightAmplitude());
    }
    else if(usrDebugAtEqIgnoreCase(sub, "zero") != 0U)
    {
      /* 重新标定抬升四路电流的零电流基线：必须在电机静止（零电流）时执行。 */
      uint16_t zeros[4];

      currentCalibrateRiseZero();
      currentRiseZeroGetArray(zeros);
      (void)printf("NB+rise=zero,lA=%u,lB=%u,rA=%u,rB=%u,OK\r\n",
                   (unsigned)zeros[0], (unsigned)zeros[1],
                   (unsigned)zeros[2], (unsigned)zeros[3]);
    }
    else if(usrDebugAtEqIgnoreCase(sub, "peak") != 0U)
    {
      /* 在窗口内高速采样抬升四路电流，返回左右电机相电流幅值（A）。
       * NB+rise=peak[,<窗口ms>]，窗口默认 RISE_CURRENT_PEAK_DEFAULT_MS。
       * 代价：占用 TMUX1308 地址线并阻塞本任务整个窗口（几十~几百 ms）。 */
      rise_current_peak_t pk;
      uint16_t window = RISE_CURRENT_PEAK_DEFAULT_MS;
      double dev = 0.0;

      if(fieldCount > 1U)
      {
        if(speedOk == 0U || speed < (long)RISE_CURRENT_PEAK_MIN_MS ||
           speed > (long)RISE_CURRENT_PEAK_MAX_MS)
        {
          usrDebugAtError("rise-peak-arg");
          return;
        }
        window = (uint16_t)speed;
      }
      if(currentCaptureRisePeak(window, &pk) == 0U)
      {
        usrDebugAtError("rise-peak-fail");
        return;
      }
      if((pk.left + pk.right) > 0.0f)
      {
        dev = 100.0 * ((double)pk.left - (double)pk.right) /
              (0.5 * ((double)pk.left + (double)pk.right));
      }
      (void)printf("NB+rise=peak,win=%u,iaL=%.4f,ibL=%.4f,iaR=%.4f,ibR=%.4f,"
                   "ampL=%.4f,ampR=%.4f,dev=%.1f%%\r\n",
                   (unsigned)window,
                   (double)pk.leftA, (double)pk.leftB,
                   (double)pk.rightA, (double)pk.rightB,
                   (double)pk.left, (double)pk.right, dev);
    }
    else if(usrDebugAtEqIgnoreCase(sub, "wave") != 0U)
    {
      /* 抬升单通道高速突发采样：把 12 位原始 ADC 连续吐出来，用于看相电流波形。
       *   NB+rise=wave,<ch>[,<n>[,<us>]]
       *     ch : la/lb/ra/rb 或 0~3（0=左A 1=左B 2=右A 3=右B）
       *     n  : 点数 1~RISE_WAVE_MAX_SAMPLES（默认 1024）
       *     us : 采样间隔 0~200us，0=尽量快（默认 50）
       * 换算：I(A) = (raw - zero) * 0.0008，zero 在返回的 hdr 行里。
       * 代价：整段独占地址线且不 yield，这段窗口内 Modbus 快照不刷新。 */
      uint8_t ch = 0U;
      uint8_t okCh = 0U;
      uint8_t okArg = 0U;
      uint16_t want = USRDEBUG_RISE_WAVE_DEFAULT_N;
      uint16_t intervalUs = USRDEBUG_RISE_WAVE_DEFAULT_INTERVAL_US;
      uint16_t got;
      uint32_t fs;
      uint32_t totalUs;
      uint32_t i;
      uint16_t *buf;
      rise_wave_info_t info;
      int32_t encL0;
      int32_t encL1;
      int32_t encR0;
      int32_t encR1;

      if(fieldCount < 2U)
      {
        usrDebugAtError("rise-wave-ch");
        return;
      }
      if(usrDebugAtParseRiseWaveCh(fields[1], &ch, &okCh) == 0U)
      {
        usrDebugAtError("rise-wave-ch");
        return;
      }
      if(fieldCount > 2U)
      {
        long v = usrDebugAtParseLong(fields[2], &okArg);
        if(okArg == 0U || v < 1L)
        {
          usrDebugAtError("rise-wave-n");
          return;
        }
        if(v > (long)RISE_WAVE_MAX_SAMPLES)
        {
          v = (long)RISE_WAVE_MAX_SAMPLES;
        }
        want = (uint16_t)v;
      }
      if(fieldCount > 3U)
      {
        long v = usrDebugAtParseLong(fields[3], &okArg);
        if(okArg == 0U || v < 0L)
        {
          usrDebugAtError("rise-wave-us");
          return;
        }
        if(v > (long)USRDEBUG_RISE_WAVE_MAX_INTERVAL_US)
        {
          v = (long)USRDEBUG_RISE_WAVE_MAX_INTERVAL_US;
        }
        intervalUs = (uint16_t)v;
      }
      totalUs = (uint32_t)want * (uint32_t)intervalUs;
      if(totalUs > USRDEBUG_RISE_WAVE_MAX_WINDOW_US)
      {
        intervalUs = (uint16_t)(USRDEBUG_RISE_WAVE_MAX_WINDOW_US / (uint32_t)want);
      }

      buf = (uint16_t *)pvPortMalloc((size_t)want * sizeof(uint16_t));
      if(buf == NULL)
      {
        usrDebugAtError("rise-wave-nomem");
        return;
      }
      encL0 = encRiseMoterLEncPositionGet();
      encR0 = encRiseMoterREncPositionGet();
      got = currentCaptureRiseWave(ch, buf, want, intervalUs, &info);
      encL1 = encRiseMoterLEncPositionGet();
      encR1 = encRiseMoterREncPositionGet();
      if(got == 0U)
      {
        vPortFree(buf);
        usrDebugAtError("rise-wave-fail");
        return;
      }
      /* got<=2048，*1000000 不会溢出 32 位；若 RISE_WAVE_MAX_SAMPLES 调过 4294 要改这里。 */
      fs = (info.elapsedUs != 0U)
             ? (uint32_t)(((uint32_t)got * 1000000U) / info.elapsedUs)
             : 0U;
      (void)printf("NB+rise=wave,hdr,ch=%u,zero=%u,a_per_count=0.0008,n=%u,req_us=%u,"
                   "elapsed_us=%lu,fs=%lu,encL0=%ld,encL1=%ld,encR0=%ld,encR1=%ld\r\n",
                   (unsigned)info.ch, (unsigned)info.zero, (unsigned)got,
                   (unsigned)intervalUs, (unsigned long)info.elapsedUs, (unsigned long)fs,
                   (long)encL0, (long)encL1, (long)encR0, (long)encR1);
      for(i = 0U; i < got; i++)
      {
        if((i % 16U) == 0U)
        {
          (void)printf("NB+rise=wave,d");
        }
        (void)printf(",%u", (unsigned)buf[i]);
        if(((i % 16U) == 15U) || ((i + 1U) == got))
        {
          (void)printf("\r\n");
        }
      }
      (void)printf("NB+rise=wave,done,got=%u\r\n", (unsigned)got);
      vPortFree(buf);
    }
    else if(usrDebugAtEqIgnoreCase(sub, "seg") != 0U)
    {
      /* 抬升"下降一段"：NB+rise=seg,<编码器计数>，从当前停点向下走（不在最低位即可）。 */
      if(fieldCount < 2U || speedOk == 0U || speed < 1L)
      {
        usrDebugAtError("rise-seg-arg");
        return;
      }
      if(riseMoterDownSegment((uint32_t)speed) != 0U)
      {
        usrDebugAtAck("NB+rise=seg", "OK");
      }
      else
      {
        usrDebugAtError("rise-seg-reject");
      }
    }
    else if(usrDebugAtEqIgnoreCase(sub, "upseg") != 0U)
    {
      /* 抬升"上行一段"：NB+rise=upseg,<编码器计数>，只能从最低位启动。 */
      if(fieldCount < 2U || speedOk == 0U || speed < 1L)
      {
        usrDebugAtError("rise-upseg-arg");
        return;
      }
      if(riseMoterUpSegment((uint32_t)speed) != 0U)
      {
        usrDebugAtAck("NB+rise=upseg", "OK");
      }
      else
      {
        usrDebugAtError("rise-upseg-reject");
      }
    }

    else if(usrDebugAtEqIgnoreCase(sub, "segpos") != 0U)
    {
      (void)printf("NB+rise=segpos,active=%u,fault=%u,counts=%lu\r\n",
                   (unsigned)riseMoterSegmentActiveGet(),
                   (unsigned)riseMoterSegmentFaultGet(),
                   (unsigned long)riseMoterSegmentCountsGet());
    }
    else if(usrDebugAtEqIgnoreCase(sub, "watch") != 0U)
    {
      char *arg = (fieldCount > 1U) ? fields[1] : NULL;

      if(arg == NULL) { usrDebugAtError("rise-watch-arg"); return; }
      if(usrDebugAtEqIgnoreCase(arg, "off") != 0U)
      {
        riseWatchEnabled = 0U;
        usrDebugAtAck("NB+rise=watch", "off");
      }
      else if(usrDebugAtEqIgnoreCase(arg, "on") != 0U)
      {
        riseWatchEnabled = 1U;
        usrDebugRiseWatchArm();
        (void)printf("NB+rise=watch,on,%u\r\n", (unsigned)riseWatchPeriodMs);
      }
      else
      {
        uint8_t ok = 0U;
        long period = usrDebugAtParseLong(arg, &ok);

        if(ok == 0U) { usrDebugAtError("rise-watch-arg"); return; }
        if(period < (long)USRDEBUG_RISE_WATCH_PERIOD_MIN_MS)
        {
          period = (long)USRDEBUG_RISE_WATCH_PERIOD_MIN_MS;
        }
        if(period > (long)USRDEBUG_RISE_WATCH_PERIOD_MAX_MS)
        {
          period = (long)USRDEBUG_RISE_WATCH_PERIOD_MAX_MS;
        }
        riseWatchPeriodMs = (uint16_t)period;
        riseWatchEnabled = 1U;
        usrDebugRiseWatchArm();
        (void)printf("NB+rise=watch,on,%u\r\n", (unsigned)riseWatchPeriodMs);
      }
    }
    else
    {
      usrDebugAtError("rise-val");
    }
    return;
  }

  /* ============== 压制机构 ============== */
  if(usrDebugAtEqIgnoreCase(key, "press") != 0U)
  {
    char *fields[2];
    uint8_t fieldCount = usrDebugAtSplitFields(val, fields, 2U);
    int speed = 2000;
    if(fieldCount >= 2U)
    {
      uint8_t speedOk = 0U;
      speed = (int)usrDebugAtParseLong(fields[1], &speedOk);
      if(speedOk == 0U || speed < 1 || speed > 6000)
      {
        usrDebugAtError("press-speed");
        return;
      }
    }
    if(usrDebugAtEqIgnoreCase(fields[0], "up") != 0U)
    {
      pressMoterUp(speed);
      usrDebugAtAck("NB+press=up", "OK");
    }
    else if(usrDebugAtEqIgnoreCase(fields[0], "down") != 0U)
    {
      pressMoterDown(speed);
      usrDebugAtAck("NB+press=down", "OK");
    }
    else if(usrDebugAtEqIgnoreCase(fields[0], "stop") != 0U)
    {
      pressMoterStop();
      usrDebugAtAck("NB+press=stop", "OK");
    }
    else if(usrDebugAtEqIgnoreCase(val, "pos") != 0U)
    {
      (void)printf("NB+press=pos,high=%u,low=%u\r\n",
                   (unsigned)pressMoterHighPositionGet(),
                   (unsigned)pressMoterLowPositionGet());
    }
    else
    {
      usrDebugAtError("press-val");
    }
    return;
  }

  /* ============== 泵 ============== */
  if(usrDebugAtEqIgnoreCase(key, "pump") != 0U)
  {
    char *fields[2];
    uint8_t fieldCount = usrDebugAtSplitFields(val, fields, 2U);
    char *sub = (fieldCount > 0U) ? fields[0] : NULL;
    if(sub == NULL) { usrDebugAtError("pump-val"); return; }

    if(usrDebugAtEqIgnoreCase(sub, "off") != 0U)
    {
      waterPumpOff();
      brinePumpOff();
      hosePumpOff();
      usrDebugAtAck("NB+pump=off", "OK");
    }
    else
    {
      uint8_t ok = 0U;
      int8_t speed = usrDebugAtParsePumpSpeed((fieldCount > 1U) ? fields[1] : NULL, &ok);
      if(ok == 0U) { usrDebugAtError("pump-speed"); return; }

      if(usrDebugAtEqIgnoreCase(sub, "water") != 0U)
      {
        if(speed == 0) waterPumpOff();
        else waterPumpOn(speed);
        usrDebugAtAck("NB+pump=water", "OK");
      }
      else if(usrDebugAtEqIgnoreCase(sub, "brine") != 0U)
      {
        if(speed == 0) brinePumpOff();
        else brinePumpOn(speed);
        usrDebugAtAck("NB+pump=brine", "OK");
      }
      else if(usrDebugAtEqIgnoreCase(sub, "hose") != 0U)
      {
        if(speed == 0) hosePumpOff();
        else hosePumpOn(speed);
        usrDebugAtAck("NB+pump=hose", "OK");
      }
      else
      {
        usrDebugAtError("pump-name");
      }
    }
    return;
  }

  /* ============== RST / LED / WS2812 ============== */
  if(usrDebugAtEqIgnoreCase(key, "rst") != 0U)
  {
    if(usrDebugAtEqIgnoreCase(val, "on") != 0U)
    {
      moterEN();
      usrDebugAtAck("NB+rst=on", "OK");
    }
    else if(usrDebugAtEqIgnoreCase(val, "off") != 0U)
    {
      moterDIS();
      usrDebugAtAck("NB+rst=off", "OK");
    }
    else
    {
      usrDebugAtError("rst-val");
    }
    return;
  }

  if(usrDebugAtEqIgnoreCase(key, "led") != 0U)
  {
    if(usrDebugAtEqIgnoreCase(val, "on") != 0U)
    {
      LEDOn();
      usrDebugAtAck("NB+led=on", "OK");
    }
    else if(usrDebugAtEqIgnoreCase(val, "off") != 0U)
    {
      LEDOff();
      usrDebugAtAck("NB+led=off", "OK");
    }
    else
    {
      usrDebugAtError("led-val");
    }
    return;
  }

  if(usrDebugAtEqIgnoreCase(key, "ws2812") != 0U)
  {
    char *fields[3];
    uint8_t fieldCount = usrDebugAtSplitFields(val, fields, 3U);
    uint8_t ok1 = 0U, ok2 = 0U, ok3 = 0U;
    uint8_t r = usrDebugAtParseU8((fieldCount > 0U) ? fields[0] : NULL, &ok1);
    uint8_t g = usrDebugAtParseU8((fieldCount > 1U) ? fields[1] : NULL, &ok2);
    uint8_t b = usrDebugAtParseU8((fieldCount > 2U) ? fields[2] : NULL, &ok3);
    if(ok1 == 0U || ok2 == 0U || ok3 == 0U) { usrDebugAtError("ws2812-arg"); return; }
    ws2812SetRgb(r, g, b);
    usrDebugAtAck("NB+ws2812", "OK");
    return;
  }

  /* ============== Modbus 统计打印开关 ============== */
  if(usrDebugAtEqIgnoreCase(key, "stat") != 0U)
  {
    if(usrDebugAtEqIgnoreCase(val, "on") != 0U)
    {
      statPrintEnabled = 1U;
      statLastPrintMs = wk_timebase_get();
      usrDebugPrintModbusStat();
      usrDebugAtAck("NB+stat=on", "OK");
    }
    else if(usrDebugAtEqIgnoreCase(val, "off") != 0U)
    {
      statPrintEnabled = 0U;
      usrDebugAtAck("NB+stat=off", "OK");
    }
    else if(usrDebugAtEqIgnoreCase(val, "now") != 0U)
    {
      usrDebugPrintModbusStat();
      usrDebugAtAck("NB+stat=now", "OK");
    }
    else
    {
      usrDebugAtError("stat-val");
    }
    return;
  }

  /* ============== 诊断查询 ============== */
  if(usrDebugAtEqIgnoreCase(key, "diag") != 0U)
  {
    if(usrDebugAtEqIgnoreCase(val, "bus") != 0U)
    {
      (void)printf("NB+diag=bus,vol=%.2f,amp=%.3f\r\n",
                   (double)busVol, (double)currentData.busAmpere);
    }
    else if(usrDebugAtEqIgnoreCase(val, "enc") != 0U)
    {
      (void)printf("NB+diag=enc,riseL=%ld,riseR=%ld,pressR=%ld\r\n",
                   (long)encRiseMoterLEncPositionGet(),
                   (long)encRiseMoterREncPositionGet(),
                   (long)encPressMoterREncPositionGet());
    }
    else if(usrDebugAtEqIgnoreCase(val, "temp") != 0U)
    {
      (void)printf("NB+diag=temp,ntc1=%.1f,ntc2=%.1f,ntc3=%.1f,ntcB=%.1f\r\n",
                   (double)getTemp(tmux1308Data2Values[NTC1]),
                   (double)getTemp(tmux1308Data2Values[NTC2]),
                   (double)getTemp(tmux1308Data2Values[NTC3]),
                   (double)getTemp(tmux1308Data2Values[NTCBacK]));
    }
    else if(usrDebugAtEqIgnoreCase(val, "flow") != 0U)
    {
      (void)printf("NB+diag=flow,water=%lu,brine=%lu\r\n",
                   (unsigned long)waterFTPulseGet(),
                   (unsigned long)brineFTPulseGet());
    }
    else if(usrDebugAtEqIgnoreCase(val, "curr") != 0U)
    {
      /* 全部 7 路电流：母线/LED/抱闸/应变片/右抬升A,B/左抬升A,B */
      (void)printf("NB+diag=curr,bus=%.3f,led=%.3f,brake=%.3f,strain=%.3f,rR_A=%.3f,rR_B=%.3f,rL_A=%.3f,rL_B=%.3f\r\n",
                   (double)currentData.busAmpere,
                   (double)currentData.ledAmpere,
                   (double)currentData.brakeAmpere,
                   (double)currentData.strainGageAmpere,
                   (double)currentData.riseRightAAmpere,
                   (double)currentData.riseRightBAmpere,
                   (double)currentData.riseLeftAAmpere,
                   (double)currentData.riseLeftBAmpere);
    }
    else if(usrDebugAtEqIgnoreCase(val, "tmux") != 0U)
    {
      /* D0 数字量 8 通道 */
      (void)printf("NB+diag=tmux,d0=%u,%u,%u,%u,%u,%u,%u,%u\r\n",
                   (unsigned)tmux1308Data0Values[0], (unsigned)tmux1308Data0Values[1],
                   (unsigned)tmux1308Data0Values[2], (unsigned)tmux1308Data0Values[3],
                   (unsigned)tmux1308Data0Values[4], (unsigned)tmux1308Data0Values[5],
                   (unsigned)tmux1308Data0Values[6], (unsigned)tmux1308Data0Values[7]);
      /* D1 模拟量 8 通道 */
      (void)printf("NB+diag=tmux,d1=%u,%u,%u,%u,%u,%u,%u,%u\r\n",
                   (unsigned)tmux1308Data1Values[0], (unsigned)tmux1308Data1Values[1],
                   (unsigned)tmux1308Data1Values[2], (unsigned)tmux1308Data1Values[3],
                   (unsigned)tmux1308Data1Values[4], (unsigned)tmux1308Data1Values[5],
                   (unsigned)tmux1308Data1Values[6], (unsigned)tmux1308Data1Values[7]);
      /* D2 模拟量 8 通道 */
      (void)printf("NB+diag=tmux,d2=%u,%u,%u,%u,%u,%u,%u,%u\r\n",
                   (unsigned)tmux1308Data2Values[0], (unsigned)tmux1308Data2Values[1],
                   (unsigned)tmux1308Data2Values[2], (unsigned)tmux1308Data2Values[3],
                   (unsigned)tmux1308Data2Values[4], (unsigned)tmux1308Data2Values[5],
                   (unsigned)tmux1308Data2Values[6], (unsigned)tmux1308Data2Values[7]);
    }
    else
    {
      usrDebugAtError("diag-val");
    }
    return;
  }

  usrDebugAtError("key");
}

/* 收到一个字符：累积到行缓冲，遇到 \n 或 \r 就处理一整行。
 * 兼容 \n、\r、\r\n 三种行结束方式。 */
static void usrDebugAtFeedChar(char c)
{
  if(c == '\n' || c == '\r')
  {
    if(atLineOverflow != 0U)
    {
      atLineOverflow = 0U;
      atLineLen = 0U;
      usrDebugAtError("line-overflow");
      return;
    }
    atLineBuf[atLineLen] = '\0';
    usrDebugAtHandleLine(usrDebugAtTrim(atLineBuf));
    atLineLen = 0U;
    return;
  }

  if(atLineLen < (USRDEBUG_AT_LINE_MAX - 1U))
  {
    atLineBuf[atLineLen] = c;
    atLineLen++;
  }
  else
  {
    atLineOverflow = 1U;
    atLineLen = 0U;
  }
}

void usrDebugAtPoll(void)
{
  /* 所有局部变量改为 static，避免在 task04 栈上分配。
   * task04 栈仅 512 字=2048 字节，printf 内部缓冲较大，
   * 局部变量走 static/全局可显著降低栈消耗。 */
  static uint8_t  frame[USRDEBUG_RX_BUF_SIZE];
  static uint16_t frameLen = 0U;
  static uint16_t i;
  static uint32_t nowMs;
  static uint32_t diagLastMs = 0U;
  static uint32_t heartbeatLastMs = 0U;
  static uint8_t  atPollFirst = 1U;

  nowMs = (uint32_t)xTaskGetTickCount();

  /* 诊断：第一次进入无条件打印，确认函数被调用。 */
  if(atPollFirst != 0U)
  {
    atPollFirst = 0U;
    log_d("usrDebugAtPoll enter t=%lu", (unsigned long)nowMs);
  }

  /* 长时间没有数据 → 丢掉可能残留的半截行，重新同步 */
  if(atHaveDataMs != 0U && (nowMs - atLastDataMs) > USRDEBUG_AT_RESYNC_MS)
  {
    atLineLen = 0U;
    atLineOverflow = 0U;
  }

  /* 一次 poll 取完队列里所有帧（连发时不积压） */
  while(usrDebugGetFrame(frame, &frameLen) == true)
  {
    atLastDataMs = nowMs;
    atHaveDataMs = 1U;

    for(i = 0U; i < frameLen; i++)
    {
      usrDebugAtFeedChar((char)frame[i]);
    }
  }

  /* UART5 调试链路心跳：用于上位机确认 MCU 任务、串口发送路径仍在运行。 */
  if((wk_timebase_get() - heartbeatLastMs) >= USRDEBUG_HEARTBEAT_INTERVAL_MS)
  {
    heartbeatLastMs = wk_timebase_get();
    (void)printf("NB+heartbeat=ok\r\n");
  }

  /* Modbus 统计周期打印：开关打开后每 10 秒输出一次。
   * 放在帧解析之后，避免与指令应答交错输出。 */
  if(statPrintEnabled != 0U &&
     (nowMs - statLastPrintMs) >= USRDEBUG_STAT_PRINT_INTERVAL_MS)
  {
    statLastPrintMs = nowMs;
    usrDebugPrintModbusStat();
  }

  /* 57 电流表标定周期打印：同样放在帧解析之后，避免与指令应答交错。 */
  if(m57WatchEnabled != 0U &&
     (nowMs - m57WatchLastMs) >= (uint32_t)m57WatchPeriodMs)
  {
    m57WatchLastMs = nowMs;
    usrDebugM57WatchPrint();
  }

  /* 42 抬升电机标定周期打印：与 57 同一位点，两者可同时打开。 */
  if(riseWatchEnabled != 0U &&
     (nowMs - riseWatchLastMs) >= (uint32_t)riseWatchPeriodMs)
  {
    riseWatchLastMs = nowMs;
    usrDebugRiseWatchPrint();
  }

  /* 57 电流表标定自动扫描：状态机时序推进（每 10ms 一次）。 */
  usrDebugM57SweepPoll(nowMs);

  /* 诊断：每 10 秒打印一次 DMA/队列状态，帮助定位 RX 不工作问题。 */
  if((nowMs - diagLastMs) >= 10000U)
  {
    diagLastMs = nowMs;
    log_d("diag dtcnt=%lu ring=%u atLen=%u",
          (unsigned long)DMA1_CHANNEL2->dtcnt,
          (unsigned)ringCount, (unsigned)atLineLen);
  }

  /* 诊断：第一次进入函数末尾打印，确认 printf 不卡死、函数能返回。 */
  {
    static uint8_t atPollExitFirst = 1U;
    if(atPollExitFirst != 0U)
    {
      atPollExitFirst = 0U;
      log_d("usrDebugAtPoll exit");
    }
  }
}

#else  /* USRDEBUG_AT_ENABLE == 0 */

void usrDebugAtPoll(void)
{
  /* 功能关闭：清掉队列里所有帧，避免缓冲被占满后新帧被丢弃 */
  uint8_t  frame[USRDEBUG_RX_BUF_SIZE];
  uint16_t frameLen = 0U;

  while(usrDebugGetFrame(frame, &frameLen) == true)
  {
    /* 丢弃 */
  }
}

#endif /* USRDEBUG_AT_ENABLE */
