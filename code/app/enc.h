#ifndef ENC_H
#define ENC_H
/* Encoder initialization, position counters, and interrupt callback APIs. */

#include <stdint.h>

/*
 * 编码器接线及计数方式：B 相接定时器输入捕获，在 B 相上升沿读取 A 相 GPIO
 * 判断方向。位置为有符号脉冲数；B 相脉冲数只递增，适合统计总脉冲。
 *
 * 抬升电机左编码器：PA11 = A 相 GPIO，PA10 = B 相 / TMR1_CH3。
 * 抬升电机右编码器：PA9  = A 相 GPIO，PA8  = B 相 / TMR1_CH1。
 * 压制电机编码器：    PC9  = A 相 GPIO，PC8  = B 相 / TMR3_CH3。
 */

/* 仅清零软件计数；GPIO、TMR 和 NVIC 由 BSP 初始化。 */
void encInit(void);

uint8_t encRiseMoterLEncARead(void);
int32_t encRiseMoterLEncPositionGet(void);
void encRiseMoterLEncPositionReset(void);
uint32_t encRiseMoterLEncBPulseGet(void);
void encRiseMoterLEncBPulseReset(void);

uint8_t encRiseMoterREncARead(void);
int32_t encRiseMoterREncPositionGet(void);
void encRiseMoterREncPositionReset(void);
uint32_t encRiseMoterREncBPulseGet(void);
void encRiseMoterREncBPulseReset(void);

uint8_t encPressMoterREncARead(void);
int32_t encPressMoterREncPositionGet(void);
void encPressMoterREncPositionReset(void);
uint32_t encPressMoterREncBPulseGet(void);
void encPressMoterREncBPulseReset(void);

/* 仅由相应定时器捕获中断服务函数调用。 */
/* TMR1_CH1 = left lift encoder B(PA8); TMR1_CH3 = right B(PA10). */
void encRiseMoterLEncTmr1Ch1Irq(void);
void encRiseMoterREncTmr1Ch3Irq(void);
void encPressMoterREncTmr3Ch3Irq(void);
/* Workbench EXINT handlers call these; no timer-capture IRQ is used. */
void encRiseMoterLEncExintIrq(void);
void encRiseMoterREncExintIrq(void);
void encPressMoterREncExintIrq(void);

int getPressMoterTurns(void);

#endif
