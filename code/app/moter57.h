#ifndef MOTER57_H
#define MOTER57_H
/* 57-step motor, press-axis motion, and winding-current capture API. */

#include "at32f415_wk_config.h"
#include "wk_tmr.h"
#include "wk_gpio.h"
#include "wk_system.h"
#include "wk_adc.h"
/*
 * 步进电机微步进控制算法（HAL 库版本）
 *
 * 硬件：STM32F103C8T6 + DRV8412
 *   PA6 = TIM3_CH1 (CCR1) → 绕组1电流
 *   PA7 = TIM3_CH2 (CCR2) → 绕组2电流
 *
 * TIM3 配置：Prescaler=0, Period=3199, CounterMode=UP
 *   系统时钟 HSI/2×PLL16 = 64MHz，APB1定时器时钟=64MHz
 *   PWM 频率 = 64MHz / 3200 = 20000 Hz
 *   PWM 中心值 = 1600（50%占空比 = 零电流）
 *   PWM 范围 0~3199
 *
 * 转速公式：
 *   RPM = 3000 × StepRate / (2048 × Divider)
 *   StepRate: 64~512（×64定点，即 1.0~8.0 步/次）
 *   Divider:  1~255（中断分频）
 *   最大 RPM = 750RPM
 */

/* 初始化步进电机算法（在 MX_TIM3_Init 之后调用） */
void StepperPulse57_8412_Init(void);

/* TIM3 中断回调（在 HAL_TIM_PeriodElapsedCallback 的 TIM3 分支中调用） */
void StepperPulse57_8412_Tick(void);

/* 持续运行（速度模式）
 *   rpm_x10      — 转速 ×10，正=正转，负=反转，0=停止（如 -1100=110RPM反转, 5=0.5RPM）
 *   amplitude    — 电流幅值百分比 0~100
 *   accelTime_ms — 加速时间ms（>0时启用S形加速，0=立即启动）
 *                  S曲线公式: s = 3x² - 2x³，加速度两端缓、中间快
 *                  由 TMR4 中断每 10ms 自动更新，无需在主循环调用。
 *   startRpm_x10 — 起步转速 ×10（S形加速起始速度，0=使用默认300RPM）
 *                  速度<200RPM时电流从10线性增到设定值
 */
void StepperPulse57_8412_RunAtSpeed(signed int rpm_x10, unsigned char amplitude, unsigned int accelTime_ms, unsigned int startRpm_x10);

/* 简化版持续运行（无加速，直接启动）
 *   rpm_x10   — 转速 ×10，正=正转，负=反转，0=停止
 *   amplitude — 电流幅值百分比 0~100
 */
void StepperPulse57_8412_Run(signed int rpm_x10, unsigned char amplitude);

/* 按 SP_57_8412_AmpTable 自动查表运行（电流幅值由转速决定）。
 * 与 pressMoterUp/Down 走同一条查表路径，供调试口验证电流表用。
 *   rpm_x10   — 转速 ×10，正=正转，负=反转，0=停止
 */
void StepperPulse57_8412_RunAutoTable(signed int rpm_x10);

/* 同上，但可指定 S 曲线加速时间。
 *   accelTime_ms — 加速/减速时间 ms，0 表示沿用默认 8000
 * 速度按压制机构机械上限 PRESS_MOTER_MAX_RPM_X10 钳位，防止误设超速。 */
void StepperPulse57_8412_RunAutoTableEx(signed int rpm_x10,
                                        unsigned int accelTime_ms);

/* S曲线更新函数；已由 TMR4 中断自动每 10ms 调用。 */
void StepperPulse57_8412_RampUpdate(void);

/* 转到指定角度（位置模式，走完自动停）
 *   angle     — 目标角度，正=正转，负=反转，支持多圈（如 720=正转2圈）
 *   rpm_x10   — 转速 ×10（如 1100=110RPM）
 *   amplitude — 电流幅值百分比 0~100
 */
void StepperPulse57_8412_MoveToAngle(signed int angle, unsigned int rpm_x10, unsigned char amplitude);

/* 立即停止，零电流 */
void StepperPulse57_8412_Stop(void);

/* 缓停：先S曲线降速到爬行速度，再在 STOP_FADE_TIME_MS 内把电流斜降为零
 * （压制到位/手动停止用；急停仍用 Stop()）。 */
void StepperPulse57_8412_StopRamped(void);

/*
 * 压制机构行程控制：上行以 PB12 高位限位停止；下行按 PC8/PC9 编码器的
 * 绝对脉冲数运行到 moter57LowEncNum 后停止。运动中 Modbus 压制高/低位均为 0。
 */
extern volatile uint32_t moter57LowEncNum;
void pressMoterUp(int v);
void pressMoterDown(int v);
void pressMoterStop(void);
uint8_t pressMoterHighPositionGet(void);
uint8_t pressMoterLowPositionGet(void);

/* 运行中调整电流幅值 0~100 */
void StepperPulse57_8412_SetAmplitude(unsigned char amplitude);

/* 获取当前绝对位置（微步，支持负值） */
signed long StepperPulse57_8412_GetPosition(void);

/* 获取当前累积角度（支持多圈） */
signed int StepperPulse57_8412_GetCurrentAngle(void);

/* 获取当前转速（rpm_x10），调试用 */
unsigned int StepperPulse57_8412_GetCurrentRPM(void);

/* 获取当前电流幅值百分比 0~100，调试用 */
unsigned char StepperPulse57_8412_GetAmplitude(void);

/* 获取当前方向（1=正转，-1=反转），调试用 */
signed char StepperPulse57_8412_GetDirection(void);

/* 获取S曲线加速进行中标志（1=加速中），调试用 */
unsigned char StepperPulse57_8412_IsRamping(void);

/* 电机使能标志（1=运行中，0=停止），用于判断 MoveToAngle 是否完成 */
extern volatile unsigned char StepperPulse57_8412_Enable;

/* ADC 零电流基准（初始化时采样 20 次平均得到）
 * pressMoterCurrA_Zero = ADC1_IN11(PC1) 绕组A电流零点
 * pressMoterCurrB_Zero = ADC1_IN12(PC2) 绕组B电流零点
 * 运行时实际电流 = 当前采样值 - 零点基准 */
extern volatile unsigned short StepperPulse57_8412_CurrA_Zero;
extern volatile unsigned short StepperPulse57_8412_CurrB_Zero;

/* ADC burst capture. Samples are recorded in ADC1_IRQHandler at 40 kHz and
 * can be retrieved after StepperPulse57_8412_AdcCaptureReady() returns 1. */
#define STEPPER57_ADC_CAPTURE_MAX_SAMPLES  1024U
unsigned char StepperPulse57_8412_AdcCaptureStart(unsigned short sampleCount);
unsigned char StepperPulse57_8412_AdcCaptureReady(void);
unsigned short StepperPulse57_8412_AdcCaptureCount(void);
void StepperPulse57_8412_AdcCaptureGetSample(unsigned short index,
                                               unsigned short *adcA,
                                               unsigned short *adcB);



/* 读取 57 电机绕组 A/B 电流（触发一次 ADC 采样并减去零点基准） */
void getMoter57Curr(void);
//更新57注入组的两个ADC值
void updateMoter57CurAdc(void);

/* 绕组 A/B 电流值（安培，由 getMoter57Curr() 更新，供上位机上报读取） */
extern float moter57CurrA;
extern float moter57CurrB;

/* 读取绕组 A/B 的 ADC 原始值（注入组锁存值，调试用） */
unsigned short getMoter57AdcA(void);
unsigned short getMoter57AdcB(void);

#endif // MOTER57_H
