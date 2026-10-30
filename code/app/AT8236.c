#include "AT8236.h"
#include "FT.h"
/* AT8236 水泵驱动：管理 PWM 目标、缓停斜坡和公共 H 桥输入电平补偿。 */

#include "wk_tmr.h"

/*
 * AT8236 的另一路输入由公共 PC10（moterRST）提供，PC10 的所有权属于
 * 其他模块。本文件绝不改动 PC10；但在 PC10 改变后，外部必须调用
 * pumpSharedInputSync()。若 PC10 已翻转，运行中的泵会从静止 PWM 重新
 * 斜坡至原目标速度；静止的泵保持静止。
 */

/*
 * TMR2 的周期值为 14999。清水泵/点卤泵使用 S 形斜坡（smoothstep）加减速；
 * 蠕动泵沿用 main 的线性斜坡：启动即到 100% 满占空比，之后每个 tick 减
 * PUMP_RAMP_STEP，最多 100 次 tick 从 100% 过渡到目标占空比。
 */
#define PUMP_RAMP_STEP          (150U)
#define PUMP_DUTY_PERCENT_MAX   (100)

/* 清水泵/点卤泵 S 曲线时长：pumpDecSpdTick() 名义 1ms 一次，0.5s = 500 个 tick。 */
#define PUMP_RAMP_TICKS         (500U)

/* 单路单向泵的 S 曲线斜坡状态。duty 均为 TMR 计数值。 */
typedef struct
{
    uint16_t current_duty;    /* 当前逻辑驱动占空比：0 为静止，period 为全速 */
    uint16_t target_duty;     /* 本次 S 曲线终点占空比 */
    uint16_t ramp_start_duty; /* 本次 S 曲线起点占空比 */
    uint16_t ramp_elapsed;    /* 本次 S 曲线已用 tick 数 */
    bool     ramp_active;     /* true：S 曲线尚未走完 */
} pump_control_t;

/* TMR2 通道映射：CH1/CH2=软管泵正反转，CH3=清水泵，CH4=卤水泵。 */
static pump_control_t brine_pump = { 0U, 0U, 0U, 0U, false };
static pump_control_t water_pump = { 0U, 0U, 0U, 0U, false };
/* GPIO 初始化时 PC10 被置低，因此初值为 false。 */
static bool shared_input_was_high = false;

/* 目标速度（百分比，-100~100，0 表示停止）。仅用于上位机状态回读。 */
static int8_t brine_pump_speed = 0;
static int8_t water_pump_speed = 0;
static int8_t hose_pump_speed  = 0;

static int8_t pump_clamp_speed(int8_t speed)
{
    /* 防止调用者传入越界百分比导致比较值超过定时器周期。 */
    if (speed > PUMP_DUTY_PERCENT_MAX)
    {
        return PUMP_DUTY_PERCENT_MAX;
    }

    if (speed < -PUMP_DUTY_PERCENT_MAX)
    {
        return -PUMP_DUTY_PERCENT_MAX;
    }

    return speed;
}

static uint16_t pump_duty_from_percent(int8_t speed)
{
    int16_t magnitude = pump_clamp_speed(speed);

    /* 单向泵仅使用绝对值；软管泵的方向在 hosePumpOn() 中单独处理。 */
    if (magnitude < 0)
    {
        magnitude = -magnitude;
    }

    return (uint16_t)(((uint32_t)magnitude * TMR2->pr) / PUMP_DUTY_PERCENT_MAX);
}

static bool pump_shared_input_is_high(void)
{
    return gpio_output_data_bit_read(moterRST_GPIO_PORT, moterRST_PIN) != RESET;
}

static uint32_t pump_shared_compare_value(uint16_t duty)
{
    /*
     * AT8236 的另一输入固定为 PC10：
     * PC10=0 时，PWM 高电平驱动电机，比较值等于逻辑驱动占空比；
     * PC10=1 时，PWM 低电平驱动电机，比较值必须反相。
     * 所以 duty=0（静止）在 PC10=0 时输出 0%，PC10=1 时输出 100%。
     */
    return pump_shared_input_is_high() ? (TMR2->pr - duty) : duty;
}

/* smoothstep：s = 3x^2 - 2x^3（Q16 定点），两端平缓、中间快。 */
static uint16_t pump_smoothstep(uint16_t start, uint16_t end,
                                uint16_t elapsed, uint16_t total)
{
    uint64_t x;
    uint64_t x2;
    uint64_t x3;
    uint64_t s;
    uint32_t span;

    if (elapsed >= total)
    {
        return end;
    }

    x  = ((uint64_t)elapsed * 65536UL) / total;      /* Q16: 0~65536 */
    x2 = x * x;                                      /* Q32 */
    x3 = x2 * x;                                     /* Q48 */
    s  = (3UL * x2 - 2UL * x3 / 65536UL) / 65536UL;  /* Q16: 0~65536 */

    if (end >= start)
    {
        span = (uint32_t)(end - start);
        return (uint16_t)((uint32_t)start + (uint32_t)((uint64_t)span * s / 65536UL));
    }

    span = (uint32_t)(start - end);
    return (uint16_t)((uint32_t)start - (uint32_t)((uint64_t)span * s / 65536UL));
}

static void pump_apply_shared_duty(const pump_control_t *pump,
                                   volatile uint32_t *compare)
{
    *compare = pump_shared_compare_value(pump->current_duty);
}

/*
 * 清水泵/点卤泵共用公共输入（PC10）的单向 S 曲线斜坡。
 *   brinePumpOn/waterPumpOn  ：从当前占空比 S 曲线过渡到目标（含 0 到目标冷启动）；
 *   brinePumpOff/waterPumpOff：从当前占空比 S 曲线降到 0，到 0 即 AT8236 静止态；
 *   PC10 翻转                ：运行中的泵从静止重新 S 曲线起动到原目标。
 * CH3/CH4 全程保持使能：PC10=1 时 100% PWM 才是 AT8236 的静止输出。
 */
static void pump_shared_start(pump_control_t *pump, volatile uint32_t *compare,
                              uint16_t target_duty)
{
    pump->ramp_start_duty = pump->current_duty;
    pump->target_duty     = target_duty;
    pump->ramp_elapsed    = 0U;
    pump->ramp_active     = (pump->current_duty != target_duty);
    pump_apply_shared_duty(pump, compare);
}

static void pump_shared_ramp_tick(pump_control_t *pump, volatile uint32_t *compare)
{
    if (pump->ramp_active)
    {
        pump->ramp_elapsed++;
        if (pump->ramp_elapsed >= PUMP_RAMP_TICKS)
        {
            pump->current_duty = pump->target_duty;
            pump->ramp_active  = false;
        }
        else
        {
            pump->current_duty = pump_smoothstep(pump->ramp_start_duty,
                                                 pump->target_duty,
                                                 pump->ramp_elapsed,
                                                 PUMP_RAMP_TICKS);
        }
    }

    /* 即使斜坡已结束也重新映射，可在 PC10 翻转后恢复原来的泵状态。 */
    pump_apply_shared_duty(pump, compare);
}

static void pump_restart_shared_ramp(pump_control_t *pump)
{
    /* 目标为 0 代表静止或正在停泵，PC10 翻转后保持 AT8236 的静止输入组合。 */
    if (pump->target_duty == 0U)
    {
        pump->current_duty = 0U;
        pump->ramp_active  = false;
        return;
    }

    /* 运行中的泵从新 PC10 电平对应的静止 PWM 重新 S 曲线起动。 */
    pump->current_duty    = 0U;
    pump->ramp_start_duty = 0U;
    pump->ramp_elapsed    = 0U;
    pump->ramp_active     = true;
}

/*
 * 蠕动泵（软管泵 / TMR2 CH1+CH2 H 桥）线性斜坡 —— 与 main 控制方式完全一致。
 *
 * 启动：直接把当前方向通道拉到 100% 满占空比，之后每个 tick 减 PUMP_RAMP_STEP
 *       逐步降到目标占空比（破静摩擦起步）；换向只把另一侧清零后在目标通道重启
 *       同一段 100% 起步斜坡；停泵立即清零两路并关闭 CH1/CH2（无减速斜坡）。
 */
typedef struct
{
    uint16_t current_duty; /* 保留字段，实际占空比读写见通道比较寄存器 */
    uint16_t target_duty;  /* 最终逻辑驱动占空比 */
    uint16_t ramp_step;    /* 每次 pumpDecSpdTick() 的下降步进 */
    bool     ramp_active;  /* true：尚未到达目标值，需要继续斜坡 */
} hose_pump_t;

static hose_pump_t hose_pump = { 0U, 0U, PUMP_RAMP_STEP, false };

static void hose_start_ramp(hose_pump_t *pump, volatile uint32_t *compare,
                            uint16_t target_duty)
{
    /* 软管泵不使用 PC10，保留原有的 100% 起动、逐步降至目标的逻辑。 */
    pump->target_duty = target_duty;
    *compare = TMR2->pr;
    pump->ramp_active = (*compare != target_duty);
}

static void hose_update_ramp(hose_pump_t *pump, volatile uint32_t *compare)
{
    uint32_t current_duty;
    uint32_t next_duty;

    if (!pump->ramp_active)
    {
        return;
    }

    current_duty = *compare;
    if (current_duty <= pump->target_duty)
    {
        *compare = pump->target_duty;
        pump->ramp_active = false;
        return;
    }

    next_duty = current_duty - pump->ramp_step;
    /* 最后一步可能不足一个步进，直接落在目标值，不允许越过目标。 */
    if (next_duty <= pump->target_duty)
    {
        next_duty = pump->target_duty;
        pump->ramp_active = false;
    }

    *compare = next_duty;
}

void brinePumpOn(int8_t speed)
{
    /* 卤水泵连接 TMR2 CH4（PA3）；公共输入 PC10 不在此处控制。 */
    tmr_channel_enable(TMR2, TMR_SELECT_CHANNEL_4, TRUE);
    brine_pump_speed = pump_clamp_speed(speed);
    pump_shared_start(&brine_pump, &TMR2->c4dt, pump_duty_from_percent(speed));
}

void brinePumpOff(void)
{
    /* S 曲线降到 0；CH4 保持使能，PC10=1 时 100% PWM 才是静止输入。 */
    brine_pump_speed = 0;
    pump_shared_start(&brine_pump, &TMR2->c4dt, 0U);
}

void waterPumpOn(int8_t speed)
{
    /* 清水泵连接 TMR2 CH3（PA2）；公共输入 PC10 不在此处控制。 */
    tmr_channel_enable(TMR2, TMR_SELECT_CHANNEL_3, TRUE);
    water_pump_speed = pump_clamp_speed(speed);
    pump_shared_start(&water_pump, &TMR2->c3dt, pump_duty_from_percent(speed));
}

void waterPumpOff(void)
{
    /* S 曲线降到 0；CH3 保持使能，PC10=1 时 100% PWM 才是静止输入。 */
    water_pump_speed = 0;
    pump_shared_start(&water_pump, &TMR2->c3dt, 0U);
    /* 停泵即把清水流量计计数清零（与 main 一致）：本次定量结束归零，
     * 下一次定量从 0 重新起算。 */
    clearWaterFT();
}

void hosePumpOn(int8_t speed)
{
    uint16_t target_duty = pump_duty_from_percent(speed);

    tmr_channel_enable(TMR2, TMR_SELECT_CHANNEL_1, TRUE);
    tmr_channel_enable(TMR2, TMR_SELECT_CHANNEL_2, TRUE);

    /* 速度符号记录方向：正=A 路，负=B 路（绝对值即占空比）。 */
    hose_pump_speed = pump_clamp_speed(speed);

    /* speed 为正（或 0）时使用 A 路；负值使用 B 路。切向前先将另一侧清零，
     * 防止 H 桥两路同时输出。 */
    if (speed >= 0)
    {
        TMR2->c2dt = 0U;
        hose_start_ramp(&hose_pump, &TMR2->c1dt, target_duty);
    }
    else
    {
        TMR2->c1dt = 0U;
        hose_start_ramp(&hose_pump, &TMR2->c2dt, target_duty);
    }
}

void hosePumpOff(void)
{
    /* 软管泵为 H 桥控制，停止时必须同时关闭 CH1 和 CH2。 */
    hose_pump_speed = 0;
    hose_pump.target_duty = 0U;
    hose_pump.ramp_active = false;
    TMR2->c1dt = 0U;
    TMR2->c2dt = 0U;
    tmr_channel_enable(TMR2, TMR_SELECT_CHANNEL_1, FALSE);
    tmr_channel_enable(TMR2, TMR_SELECT_CHANNEL_2, FALSE);
}

int8_t brinePumpGetSpeed(void) { return brine_pump_speed; }
int8_t waterPumpGetSpeed(void) { return water_pump_speed; }
int8_t hosePumpGetSpeed(void)  { return hose_pump_speed; }

/* 单向泵是否已完全停止：目标为 0 且 S 曲线斜坡已结束。 */
static uint8_t pump_is_idle(const pump_control_t *pump)
{
    return (pump->ramp_active || pump->current_duty != 0U) ? 0U : 1U;
}

uint8_t brinePumpIsIdle(void) { return pump_is_idle(&brine_pump); }
uint8_t waterPumpIsIdle(void) { return pump_is_idle(&water_pump); }

void pumpDecSpdTick(void)
{
    /* 此函数应由固定周期的定时器中断或主循环周期任务调用。 */
    pumpSharedInputSync();
    pump_shared_ramp_tick(&brine_pump, &TMR2->c4dt);
    pump_shared_ramp_tick(&water_pump, &TMR2->c3dt);

    /* 软管泵运行时仅有一个方向通道非零，据此更新正在使用的通道。 */
    if (TMR2->c1dt != 0U)
    {
        hose_update_ramp(&hose_pump, &TMR2->c1dt);
    }
    else
    {
        hose_update_ramp(&hose_pump, &TMR2->c2dt);
    }
}

void pumpSharedInputSync(void)
{
    bool shared_input_is_high = pump_shared_input_is_high();

    /*
     * 仅在 PC10 实际翻转时重启斜坡。重复调用本函数不会影响正在运行的泵。
     * 即使外部遗漏本函数，下一次 pumpDecSpdTick() 也会发现该变化。
     */
    if (shared_input_is_high != shared_input_was_high)
    {
        shared_input_was_high = shared_input_is_high;
        pump_restart_shared_ramp(&brine_pump);
        pump_restart_shared_ramp(&water_pump);
    }

    /* 立即输出新 PC10 电平对应的静止或当前运行 PWM。 */
    pump_apply_shared_duty(&brine_pump, &TMR2->c4dt);
    pump_apply_shared_duty(&water_pump, &TMR2->c3dt);
}
