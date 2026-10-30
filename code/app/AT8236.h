#ifndef AT8236_H
#define AT8236_H
/* Public API for the AT8236-driven water, brine, and hose pumps. */

#include <stdbool.h>
#include <stdint.h>

/*
 * 三路泵的编译期默认速度（占空比百分比，0~100）。
 * 线圈（FC05/FC0F）启动时使用这里的目标速度：
 * 改宏 -> 重新编译烧录 -> 泵的默认转速即改变。
 * 保持寄存器 0x0000（清水泵）/0x0001（蠕动泵）仍可按上位机下发的值实时调速。
 * PUMP_DEFAULT_HOSE_PERCENT 同时决定蠕动泵的默认方向（正数=A 向，负数=B 向）。
 * 宏值与 AT8236.c 的加减速斜坡无关，只决定最终目标占空比。
 */
#define PUMP_DEFAULT_WATER_PERCENT  (100)
#define PUMP_DEFAULT_BRINE_PERCENT  (100)
#define PUMP_DEFAULT_HOSE_PERCENT   (90)

/*
 * speed 为占空比百分比，合法范围 -100～100，越界值会自动限幅。
 * 软管泵：正值为 A 向，负值为 B 向；清水泵和卤水泵仅使用绝对值。
 * 清水泵/卤水泵加减速为 S 曲线（smoothstep，标称约 0.5s），由 pumpDecSpdTick() 推进。
 *
 * 蠕动泵（软管泵）沿用 main 的线性斜坡控制方式（每 tick 减 PUMP_RAMP_STEP）：
 *   启动：直接把当前方向通道拉到 100% 满占空比，再逐步降到目标；
 *   换向：把另一侧清零后在目标通道重启同一段 100% 起步斜坡；
 *   停泵：立即清零两路并关闭 CH1/CH2（无减速斜坡）。
 * 清水泵/卤水泵：单向，从当前占空比 S 曲线过渡到目标（含 0 与目标之间）；
 *   CH3/CH4 全程保持使能，PC10=1 时 100% PWM 才是 AT8236 的静止输出。
 *
 * 注意：卤水泵和清水泵 AT8236 的另一输入为公共 PC10（moterRST）。
 * PC10 不由本模块管理。外部改变其电平后，须立即调用 pumpSharedInputSync()：
 * 静止泵保持静止；运行泵保留目标速度，并从静止 PWM 重新斜坡至目标速度。
 */
void brinePumpOn(int8_t speed);
void brinePumpOff(void);

void waterPumpOn(int8_t speed);
void waterPumpOff(void);

void hosePumpOn(int8_t speed);
void hosePumpOff(void);

/* 目标速度回读（百分比 -100~100，0 表示停止）。用于上位机状态查询。 */
int8_t brinePumpGetSpeed(void);
int8_t waterPumpGetSpeed(void);
int8_t hosePumpGetSpeed(void);

/* 单向泵是否已完全停止（目标为 0 且减速斜坡已结束）。
 * 供需要等减速结束再继续的逻辑使用；清水定量停泵当前不依赖它（与 main 一致）。 */
uint8_t brinePumpIsIdle(void);
uint8_t waterPumpIsIdle(void);

/* 固定周期调用，用于推进正在进行的 PWM 斜坡。 */
void pumpDecSpdTick(void);

/* PC10（moterRST）改变后立即调用，按新电平重新开始清水泵和卤水泵斜坡。 */
void pumpSharedInputSync(void);

#endif /* AT8236_H */
