#ifndef __STEPPER_PULSE_57_8412_H__
#define __STEPPER_PULSE_57_8412_H__

#include "main.h"
#include "tim.h"

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
 *                  需在 task10ms 中调用 StepperPulse57_8412_RampUpdate()
 *   startRpm_x10 — 起步转速 ×10（S形加速起始速度，0=使用默认300RPM）
 *                  速度<200RPM时电流从10线性增到设定值
 */
void StepperPulse57_8412_RunAtSpeed(signed int rpm_x10, unsigned char amplitude, unsigned int accelTime_ms, unsigned int startRpm_x10);

/* 简化版持续运行（无加速，直接启动）
 *   rpm_x10   — 转速 ×10，正=正转，负=反转，0=停止
 *   amplitude — 电流幅值百分比 0~100
 */
void StepperPulse57_8412_Run(signed int rpm_x10, unsigned char amplitude);

/* S曲线加速更新（在 task10ms 中调用，每10ms更新一次速度） */
void StepperPulse57_8412_RampUpdate(void);

/* 转到指定角度（位置模式，走完自动停）
 *   angle     — 目标角度，正=正转，负=反转，支持多圈（如 720=正转2圈）
 *   rpm_x10   — 转速 ×10（如 1100=110RPM）
 *   amplitude — 电流幅值百分比 0~100
 */
void StepperPulse57_8412_MoveToAngle(signed int angle, unsigned int rpm_x10, unsigned char amplitude);

/* 立即停止，零电流 */
void StepperPulse57_8412_Stop(void);

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
unsigned char StepperPulse57_8412_GetAmplitude(void);

/* 获取当前方向（1=正转，-1=反转），调试用 */
signed char StepperPulse57_8412_GetDirection(void);

/* 获取S曲线加速进行中标志（1=加速中），调试用 */
unsigned char StepperPulse57_8412_IsRamping(void);

/* 电机使能标志（1=运行中，0=停止），用于判断 MoveToAngle 是否完成 */
extern volatile unsigned char StepperPulse57_8412_Enable;

#endif /* __STEPPER_PULSE_57_8412_H__ */
