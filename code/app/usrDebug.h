#ifndef __USRDEBUG_H
#define __USRDEBUG_H
/* Debug UART and periodic heartbeat output API. */

#include "at32f415_wk_config.h"
#include <stdint.h>
#include <stdbool.h>

/*
 * UART5 调试口 DMA 接收处理（仿照 USART3 / modbusTask.c 的方式）
 *
 * 硬件：UART5 TX=PC12 RX=PD2，1 Mbit/s 8N1（由 wk_uart5_init() 配置）
 *   接收：DMA1_CHANNEL2（FLEX_UART5_RX）+ USART IDLE 空闲中断
 *   发送：阻塞式（wk_debug_putc / printf 重定向，见 wk_debug.c）
 *
 * Workbench 生成的 main() 用占位符（地址 0、长度 0）使能 DMA1_CHANNEL2，
 * 本模块的 usrDebugInit() 负责把它重新配置为真实缓冲区并重启，
 * 与 modbusinit() 对 DMA1_CHANNEL1 的处理方式完全一致。
 *
 * UART5_IRQHandler() 检测到 IDLE 后调用 usrDebugUart5IdleIsr()，
 * 该函数关 DMA → 读 dtcnt 取本帧长度 → 清 IDLE → 回调 → 重载 → 重启，
 * 与 modbusUsart3IdleIsr() 的 V1 (NUVELL_V1) 模式一致。
 */

/* UART5 RX DMA 缓冲区大小（与 USART3 的 MODBUS_RX_BUFFER_SIZE 对齐） */
#define USRDEBUG_RX_BUF_SIZE  256U

/* ---------------------------------------------------------------------------
 * AT 风格调试指令开关（类 AT 指令，行以 '\n' 或 '\r' 结束，大小写不敏感）
 *
 *   USRDEBUG_AT_ENABLE = 1  打开：解析下面这些指令并执行/应答
 *   USRDEBUG_AT_ENABLE = 0  关闭：usrDebugAtPoll() 变成空操作，不解析任何数据
 *
 * 指令清单（"NB+" 前缀，'=' 前后的空格可有可无，大小写不敏感）。
 *   值部分多参数用逗号分隔，如 NB+m57=run,1100,50
 *
 * -- 28 步进 / 阀门 --
 *   NB+moter28 = up              -> moter28Forward()      OK
 *   NB+moter28 = down            -> moter28Backward()     OK
 *   NB+moter28 = stop            -> moter28Stop()         OK
 *   NB+moter28 = turn            -> 返回 currTurns        turn,<数值>
 *   NB+moter28 = pos             -> 返回 valveOnPos/valveOffPos
 *   NB+valve   = on              -> valveOn()             OK
 *   NB+valve   = off             -> valveOff()            OK
 *
 * -- 57 步进电机 --
 *   NB+m57 = run,<rpm_x10>,<amp> -> StepperPulse57_8412_Run(rpm,amp)  OK
 *   NB+m57 = run,<rpm_x10>,auto  -> 按 SP_57_8412_AmpTable 查表运行     run,auto
 *   NB+m57 = stop                -> StepperPulse57_8412_Stop()        OK
 *   NB+m57 = amp,<amp>           -> StepperPulse57_8412_SetAmplitude  OK
 *   NB+m57 = info                -> 返回 pos,ang,rpm,amp
 *   NB+m57 = watch,on|off|<ms>   -> 57 电流表标定周期打印（默认 50ms）  OK
 *                                   打印 cur=瞬时指令转速×10,amp=幅值%,
 *                                   pos=指令微步,denc=本周期编码器增量,
 *                                   adcA/adcB=绕组原始ADC,zA/zB=零点基准,
 *                                   ia/ib=绕组电流A（adcA 静止时应≈zA）
 *   NB+m57 = sweep,<rpm>,<加速ms>,<保持ms>,<循环次数>
 *                                -> 标定自动往返扫描：静止起步 → S 曲线
 *                                   加速到 <rpm> → 保持 → 硬停 → 保持 →
 *                                   下一轮；<循环次数> 0=无限，直到 stop
 *
 * -- 42 抬升电机（双电机反向）--
 *   NB+m42 = run,<rpm_x10>,<amp> -> StepperPulse42_Run(rpm,amp)       OK
 *   NB+m42 = same,<rpm_x10>,<amp>-> StepperPulse42_RunSameDirection   OK
 *   NB+m42 = stop                -> StepperPulse42_Stop()             OK
 *   NB+m42 = amp,<amp>           -> 左右同时设置幅值                  OK
 *   NB+m42 = amp,<left>,<right>  -> 分别设置左/右 42 电机幅值         OK
 *   NB+m42 = info                -> 返回 pos,ang
 *
 * -- 抬升机构（限位/编码器保护）--
 *   NB+rise = up                 -> riseMoterUp()          OK
 *   NB+rise = down               -> riseMoterDown()        OK
 *   NB+rise = up,<rpm_x10>       -> 带保护上行，速度 1~6000
 *   NB+rise = down,<rpm_x10>     -> 带保护下行，速度 1~6000
 *   NB+rise = mode,same          -> 两台抬升电机机械同向
 *   NB+rise = mode,opposite      -> 两台抬升电机机械反向（默认）
 *   NB+rise = auto,on|off        -> 抬升幅值来源：查表(默认)/固定值；无参回读
 *   NB+rise = amp,<amp>          -> 两台抬升电机相同电流幅值（0~100，切手动模式）
 *   NB+rise = amp,<left>,<right> -> 分别设置左/右抬升电流幅值（0~100，切手动模式）
 *   NB+rise = watch,on|off|<ms>  -> 42 电流表标定周期打印（默认 50ms）  OK
 *                                   打印 cur=瞬时指令转速×10,ampL/ampR=左/右幅值%,
 *                                   pos=指令微步,dencL/dencR=左/右编码器增量,
 *                                   ia/ib=左电机 A/B 相电流 A
 *   NB+rise = zero               -> 重标定抬升四路电流零基线（电机须静止） OK
 *   NB+rise = peak[,<ms>]        -> 窗口内高速采样，回左右相电流幅值       peak,...
 *   NB+rise = wave,<ch>[,<n>[,<us>]] -> 单通道突发采样原始ADC(看波形)      wave,...
 *                                    ch=la/lb/ra/rb 或 0~3；n 默认 1024；us 默认 50
 *                                    先回 hdr 行，再跟若干 d 行(每行16个原始计数)，最后 done
 *   NB+rise = stop               -> riseMoterStop()        OK
 *   NB+rise = pos                -> 返回 high,low
 *
 * -- 压制机构 --
 *   NB+press = up                -> pressMoterUp()         OK
 *   NB+press = down              -> pressMoterDown()       OK
 *   NB+press = stop              -> pressMoterStop()       OK
 *   NB+press = pos               -> 返回 high,low
 *
 * -- 泵（speed: -100~100, 0=停止）--
 *   NB+pump = water,<speed>       -> waterPumpOn/Off        OK
 *   NB+pump = brine,<speed>       -> brinePumpOn/Off        OK
 *   NB+pump = hose,<speed>        -> hosePumpOn/Off         OK
 *   NB+pump = off                 -> 三泵全停               OK
 *
 * -- RST / LED / WS2812 --
 *   NB+rst = on                   -> moterEN()              OK
 *   NB+rst = off                  -> moterDIS()             OK
 *   NB+led = on                   -> LEDOn()                OK
 *   NB+led = off                  -> LEDOff()               OK
 *   NB+ws2812 = <r>,<g>,<b>       -> ws2812SetRgb(r,g,b)    OK
 *
 * -- 诊断查询 --
 *   NB+diag = bus                  -> 返回 vol,amp
 *   NB+diag = enc                  -> 返回 riseL,riseR,pressR
 *   NB+diag = temp                 -> 返回 ntc1,ntc2,ntc3,ntcB
 *   NB+diag = flow                 -> 返回 water,brine
 *
 * -- Modbus 统计 --
 *   NB+stat = on                   -> 打开 10s 周期统计打印（立即打印一次） OK
 *   NB+stat = off                  -> 关闭周期统计打印                       OK
 *   NB+stat = now                  -> 立即打印一次统计信息                   OK
 *   打印格式: NB+stat,rx=<有效帧>,reply=<应答帧>,crcErr=<CRC错>,
 *             frameErr=<帧错>,qOvf=<队列满丢弃>
 *
 * 解析失败（前缀/键/参数不对）应答 NB+ERR,<原因>\r\n
 * ------------------------------------------------------------------------- */
#define USRDEBUG_AT_ENABLE  1U

/* AT 单行最大长度（超出则丢弃该行，等下一个换行重新同步） */
#define USRDEBUG_AT_LINE_MAX  64U

/* 初始化 UART5 DMA 接收：重配 DMA1_CHANNEL2 为本地缓冲区并重启。
 * 在所有外设初始化完成后（wk_uart5_init() 之后）调用一次。
 * 重复调用安全：已初始化时直接返回。 */
void usrDebugInit(void);

/* UART5 空闲中断处理：在 UART5_IRQHandler() 中 IDLE 标志置位时调用。
 * 注意：调用者不要在本函数之前清 IDLE 标志，本函数内部在关 DMA 之后清，
 * 与 modbusUsart3IdleIsr() 的顺序一致（避免 DMA 运行中读 DT 偷字节）。 */
void usrDebugUart5IdleIsr(void);

/* 获取最近一帧接收到的数据指针与长度（拷贝到调用者缓冲区）。
 * 返回 true 表示有新帧；false 表示无新帧（上次已取走）。 */
bool usrDebugGetFrame(uint8_t *outData, uint16_t *outLen);

/* AT 风格调试指令轮询：从 usrDebugGetFrame() 取帧 → 按行切分 → 解析执行 → 应答。
 * 需周期性调用（建议放在 usrDubug 任务里，10~20ms 一次）。
 * 距离上次调用超过 1s 时重置解析状态（丢掉半截行）。
 * USRDEBUG_AT_ENABLE = 0 时本函数为空操作。 */
void usrDebugAtPoll(void);

/* 通过 UART5 发送一段原始字节（阻塞式，printf 同一条链路）。 */
void usrDebugWrite(const uint8_t *data, uint16_t len);

#endif /* __USRDEBUG_H */
