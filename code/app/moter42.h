#ifndef MOTER42_H
#define MOTER42_H
/* Dual 42-step motor control, protected rise motion, and pairing mode API. */

#include "at32f415_wk_config.h"
#include "wk_tmr.h"
#include "wk_gpio.h"
#include "wk_system.h"

/*
 * 42步进双电机微步进控制算法（参考 moter57 控制模式）
 *
 * 硬件：AT32F415 + DRV8412 类驱动
 *   电机1: TMR4_CH1/PB6 + TMR4_CH2/PB7（绕组A/B）
 *   电机2: TMR5_CH1/PF4 + TMR5_CH2/PF5（绕组A/B）
 *
 * TMR4/TMR5 均为 period=3749, 40kHz PWM 更新
 *   PWM 中心值 = 1875（50%占空比 = 零电流）
 *   PWM 范围 0~3750
 *
 * 双电机特性：
 *   两个电机共用同一相位累加器，由同一个 Tick 函数驱动，
 *   电机2 相位 = -电机1 相位（index2 = -index1），实现两电机
 *   同时运动、方向始终相反。
 *   输入速度 rpm_x10 的符号即电机1的方向，电机2自动反向。
 *
 * 转速公式（同 moter57）：
 *   RPM = 60000 × StepRate / (2048 × Divider)
 *   StepRate: 64~512（×64定点，即 1.0~8.0 步/次）
 *   Divider:  1~255（中断分频）
 */

/* 初始化双电机算法（在 wk_tmr4_init/wk_tmr5_init 之后调用） */
void StepperPulse42_Init(void);

/* 中断回调（在 TMR4_GLOBAL_IRQHandler 的 OVF 分支中调用，
 * 单一函数同时驱动电机1与电机2） */
void StepperPulse42_Tick(void);

/* 持续运行（速度模式）
 *   rpm_x10      — 转速 ×10，正=电机1正转/电机2反转，负=电机1反转/电机2正转，0=停止
 *   amplitude    — 电流幅值百分比 0~100
 *   accelTime_ms — 加速时间ms（>0时启用S形加速，0=立即启动）
 *                  S曲线公式: s = 3x² - 2x³，由 TMR4 中断每 10ms 自动更新
 *   startRpm_x10 — 起步转速 ×10（S形加速起始速度，0=使用默认300RPM）
 */
void StepperPulse42_RunAtSpeed(signed int rpm_x10, unsigned char amplitude, unsigned int accelTime_ms, unsigned int startRpm_x10);
void StepperPulse42_RunAtSpeedSameDirection(signed int rpm_x10, unsigned char amplitude, unsigned int accelTime_ms, unsigned int startRpm_x10);

/* 简化版持续运行（无加速，直接启动）
 *   rpm_x10   — 转速 ×10，正=电机1正转/电机2反转，0=停止
 *   amplitude — 电流幅值百分比 0~100
 */
void StepperPulse42_Run(signed int rpm_x10, unsigned char amplitude);
void StepperPulse42_RunSameDirection(signed int rpm_x10, unsigned char amplitude);

/* S曲线更新函数；已由 TMR4 中断自动每 10ms 调用。 */
void StepperPulse42_RampUpdate(void);

/* 转到指定角度（位置模式，走完自动停）
 *   angle     — 目标角度（电机1视角），正=电机1正转/电机2反转
 *   rpm_x10   — 转速 ×10
 *   amplitude — 电流幅值百分比 0~100
 */
void StepperPulse42_MoveToAngle(signed int angle, unsigned int rpm_x10, unsigned char amplitude);

/* 立即停止双电机，零电流 */
void StepperPulse42_Stop(void);

/*
 * 抬升机构控制。两台 42 步进电机由同一相位发生器驱动；按当前绕组接线，
 * 相同电相位会得到相反的机械转向。运行中
 * 上行时各侧高位触发后单独停止步进并保持励磁，两侧均到位后停止并清零编码器。
 * 持续核对左右编码器的绝对脉冲数。riseMoterHighEncNum 是下行目标脉冲数，
 * 必须在调用 riseMoterDown() 前设为非零值。
 */
extern volatile const uint32_t riseMoterHighEncNum;
void riseMoterUp(void);
void riseMoterDown(void);
/* 可调速度的抬升动作：速度单位 rpm×10，取值 1~6000；
 * 上行固定为正向、下行固定为反向，限位/编码器保护仍然有效。 */
void riseMoterUpAtSpeed(uint16_t rpm_x10);
void riseMoterDownAtSpeed(uint16_t rpm_x10);
/* 设置两台抬升 42 电机的机械方向模式：0=反向，1=同向。
 * 设置在下一次抬升上/下行启动时生效。 */
void riseMoterSetSameDirection(uint8_t sameDirection);
uint8_t riseMoterGetSameDirection(void);
/* 抬升机构专用电流幅值（0~100%）：上行与下行共用，可在运行中修改。
 * 默认分两档：低档（普通上/下行、上位机手动点动、开机自检）、
 * 高档（压制流程带载：-1000/-601 段动作、归位下行、回高位上行）；
 * 档位由保持寄存器 0x0004 选择（见 riseMoterSetLoadGear），
 * 数值见 moter42.c 的 RISE_MOTER_LEFT/RIGHT_AMPLITUDE[_LOADED]。 */
void riseMoterSetAmplitude(uint8_t amplitude);
void riseMoterSetAmplitudeLeftRight(uint8_t leftAmplitude, uint8_t rightAmplitude);
uint8_t riseMoterGetLeftAmplitude(void);
uint8_t riseMoterGetRightAmplitude(void);
/* 抬升电流档：0=低档，非 0=高档（幅值数值见 moter42.c 的
 * RISE_MOTER_*_AMPLITUDE[_LOADED]）。由保持寄存器 0x0004 设置，写后立即生效；
 * 段动作（一定带载）会强制切高档。 */
void riseMoterSetLoadGear(uint8_t gear);
uint8_t riseMoterGetLoadGear(void);
void riseMoterStop(void);
uint8_t riseMoterLowPositionGet(void);
uint8_t riseMoterHighPositionGet(void);

/* 抬升"下降一段"：从当前停点向下走固定编码器距离后停止。
 *   counts — 段长度，单位为编码器计数（1000 计数/圈），
 *            必须大于 0 且小于全行程 riseMoterHighEncNum。
 * 返回 1 = 已启动或已在执行中（重传幂等）；
 *      0 = 被拒绝（已在最低位 / 长度为 0 / 不小于全行程），此时故障位为 1。
 * 段结束时零电流停止，位置靠丝杆机械自锁保持。 */
uint8_t riseMoterDownSegment(uint32_t counts);
/* 抬升"下降一段"默认入口：段长度固定写在下位机（RISE_MOTER_SEGMENT_COUNTS，
 * 编码器计数）。语义与 riseMoterDownSegment() 完全一致，供 0x0003 写 -1000 触发。 */
uint8_t riseMoterDownSegmentDefault(void);

/* 抬升"上行一段"：从最低位向上走固定编码器距离后停止（压制流程排水定位用）。
 *   counts — 段长度，单位为编码器计数（1000 计数/圈），
 *            必须大于 0 且小于全行程 riseMoterHighEncNum。
 * 返回 1 = 已启动或已在执行中（重传幂等）；
 *      0 = 被拒绝（不在最低位 / 长度为 0 / 不小于全行程），此时故障位为 1。
 * 段结束时零电流停止，位置靠丝杆机械自锁保持。 */
uint8_t riseMoterUpSegment(uint32_t counts);
/* 抬升"上行一段"默认入口：段长度固定写在下位机（RISE_MOTER_UP_SEGMENT_COUNTS，
 * 编码器计数）。语义与 riseMoterUpSegment() 完全一致，供 0x0003 写 -601 触发。 */
uint8_t riseMoterUpSegmentDefault(void);
/* 1=正在执行"上行一段"/"下降一段" */
uint8_t riseMoterSegmentActiveGet(void);
/* 1=上一次"上行一段"/"下降一段"被拒绝或中途触到低位 */
uint8_t riseMoterSegmentFaultGet(void);
/* 最近一次设定的段长度（编码器计数） */
uint32_t riseMoterSegmentCountsGet(void);

/* 运行中将左右两台 42 电机的电流幅值同时设为相同值（0~100）。 */
void StepperPulse42_SetAmplitude(unsigned char amplitude);

/* 运行中分别设置左右两台 42 电机的电流幅值（0~100）。
 * 左侧：TMR4 CH1/CH2；右侧：TMR5 CH1/CH2。 */
void StepperPulse42_SetAmplitudeLeftRight(unsigned char leftAmplitude,
                                          unsigned char rightAmplitude);

/* 获取电机1绝对位置（微步，支持负值；电机2位置 = -电机1位置） */
signed long StepperPulse42_GetPosition(void);

/* 获取电机1当前累积角度（电机2角度 = -电机1角度） */
signed int StepperPulse42_GetCurrentAngle(void);

/* 获取当前转速（rpm_x10，电机1视角），调试用 */
unsigned int StepperPulse42_GetCurrentRPM(void);

/* 获取当前电流幅值百分比 0~100，调试用 */
unsigned char StepperPulse42_GetAmplitude(void);
unsigned char StepperPulse42_GetLeftAmplitude(void);
unsigned char StepperPulse42_GetRightAmplitude(void);

/* 获取电机1当前方向（1=正转，-1=反转；电机2方向相反），调试用 */
signed char StepperPulse42_GetDirection(void);

/* 获取S曲线加速进行中标志（1=加速中），调试用 */
unsigned char StepperPulse42_IsRamping(void);

/* 电机使能标志（1=运行中，0=停止），用于判断 MoveToAngle 是否完成 */
extern volatile unsigned char StepperPulse42_Enable;

#endif // MOTER42_H
