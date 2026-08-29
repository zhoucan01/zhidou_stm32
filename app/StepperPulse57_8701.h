#ifndef __STEPPER_PULSE_57_8701_H__
#define __STEPPER_PULSE_57_8701_H__

#include "main.h"
#include "tim.h"

/*
 * 4路步进电机微步进控制算法
 *
 * 硬件：STM32F103C8T6
 *   PA0 = TIM2_CH1 (CCR1) → A+ 绕组
 *   PA1 = TIM2_CH2 (CCR2) → B+ 绕组
 *   PA2 = TIM2_CH3 (CCR3) → A- 绕组（互补）
 *   PA3 = TIM2_CH4 (CCR4) → B- 绕组（互补）
 *
 * TIM2 配置：Prescaler=0, Period=3199, CounterMode=UP
 *   PWM 频率 = 64MHz / 3200 = 20000 Hz
 *   PWM 中心值 = 1600（50%占空比 = 零电流）
 *   PWM 范围 0~3199
 *
 * 4路相位关系（H桥互补驱动）：
 *   CH1 = sin(θ)       → A+
 *   CH2 = cos(θ)       → B+
 *   CH3 = -sin(θ)      → A-
 *   CH4 = -cos(θ)      → B-
 *
 * 转速公式（与 StepperPulse57 一致）：
 *   RPM = 3000 × StepRate / (2048 × Divider)
 */

/* 初始化（在 MX_TIM2_Init 之后调用） */
void StepperPulse57_8701_Init(void);

/* TIM2 中断回调（在 HAL_TIM_PeriodElapsedCallback 的 TIM2 分支中调用） */
void StepperPulse57_8701_Tick(void);

/* 持续运行（速度模式）
 *   rpm_x10   — 转速 ×10，正=正转，负=反转，0=停止
 *   amplitude — 电流幅值百分比 0~100
 */
void StepperPulse57_8701_RunAtSpeed(signed int rpm_x10, unsigned char amplitude);

/* S曲线加速启动（在 task10ms 中调用 RampUpdate 更新）
 *   rpm_x10   — 目标转速 ×10，正=正转，负=反转
 *   amplitude — 电流幅值 0~100
 *   accel_ms  — 加速时间（毫秒）
 */
void StepperPulse57_8701_RunAtSpeedS(signed int rpm_x10, unsigned char amplitude, unsigned int accel_ms);

/* S曲线加速更新（在 task10ms 中调用） */
void StepperPulse57_8701_RampUpdate(void);

/* 转到指定角度（位置模式，走完自动停）
 *   angle     — 目标角度，正=正转，负=反转，支持多圈
 *   rpm_x10   — 转速 ×10
 *   amplitude — 电流幅值百分比 0~100
 */
void StepperPulse57_8701_MoveToAngle(signed int angle, unsigned int rpm_x10, unsigned char amplitude);

/* 立即停止，零电流 */
void StepperPulse57_8701_Stop(void);

/* 电机自锁（保持当前位置，持续通电产生保持力矩）
 *   holdDuty — 保持电流占空比 0~100（如 30 = 30%电流）
 * 停止推进，PH 保持方向电平，EN 输出固定 PWM
 */
void StepperPulse57_8701_Lock(unsigned char holdDuty);

/* 释放自锁，零电流 */
void StepperPulse57_8701_Unlock(void);

/* 运行中调整电流幅值 0~100 */
void StepperPulse57_8701_SetAmplitude(unsigned char amplitude);

/* 获取当前绝对位置（微步，支持负值） */
signed long StepperPulse57_8701_GetPosition(void);

/* 获取当前转速 ×10（如 5000 = 500RPM） */
unsigned int StepperPulse57_8701_GetCurrentRPM(void);

/* 电机使能标志（1=运行中，0=停止） */
extern volatile unsigned char StepperPulse57_8701_Enable;

#endif /* __STEPPER_PULSE_57_8701_H__ */
