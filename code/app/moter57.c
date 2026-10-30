#include "moter57.h"

/* 57 步进电机驱动：负责脉冲、方向、电流幅值、限位和压制运动。 */
#include "enc.h"

/* 机械换算�?�?= 256微步 × 50减速比 = 12800�?*/
#define STEPS_PER_REV  12800

/* PWM 参数 */
#define SP_57_8412_PWM_CENTER     1875   /* 50%占空�?= 零电�?*/
#define SP_57_8412_PWM_RANGE      3750   /* 0~3750 */
#define SP_57_8412_SINE_POINTS    256

/* 运行模式 */
#define SP_57_8412_MODE_IDLE     0
#define SP_57_8412_MODE_STEPS    1   /* 走指定步数后�?*/
#define SP_57_8412_MODE_RUN      2   /* 持续运行 */

/* S形加速参�?*/
#define SP_57_8412_START_RPM_X10   3000    /* 启动速度 300RPM(×10)，从静止可可靠启�?*/
#define SP_57_8412_PWM_UPDATE_HZ   40000UL
#define SP_57_8412_RAMP_PERIOD_MS  10UL
#define SP_57_8412_RAMP_TICKS      (SP_57_8412_PWM_UPDATE_HZ * SP_57_8412_RAMP_PERIOD_MS / 1000UL)

/* 减振参数（上机实测后可调）：
 * AMP_SLEW_PER_TICK  每 10ms 电流幅值最大变化 %，0=关闭斜率限制（立即跟随目标）
 * STOP_COAST_RPM_X10 停止斜坡阶段1先把转速 S 曲线降到该值（×10 RPM）
 * STOP_RAMP_TIME_MS  停止斜坡阶段1 减速时长（实测 300ms 时 170RPM 停机丢步，改 1000ms）
 * STOP_FADE_TIME_MS  停止斜坡阶段2 电流从当前值斜降到 0 的时长 */
#define SP_57_8412_AMP_SLEW_PER_TICK   2
#define SP_57_8412_STOP_COAST_RPM_X10  500U
#define SP_57_8412_STOP_RAMP_TIME_MS  1000U
#define SP_57_8412_STOP_FADE_TIME_MS   60U
/* 1 = StopRamped does S-curve decel + current fade; 0 = plain Stop(). */
#define SP_57_8412_STOP_RAMPED_ENABLE  1

/* S曲线起步速度（方案B）：默认起步速度改为按目标速度比例动态计算，
 * 取 max(目标/START_FRACTION, START_FLOOR) 并限制在 START_CEIL 以内，
 * 避免 300RPM 以下的低速压制动作从 300RPM 速度阶跃启动。
 * 传 startRpm_x10 > 0 时仍优先使用用户指定值。 */
#define SP_57_8412_START_FRACTION       3
#define SP_57_8412_START_FLOOR_RPM_X10  500U
#define SP_57_8412_START_CEIL_RPM_X10   3000U

/* 压制机构默认运行参数：30 rpm、10% 电流幅值。 */
#define PRESS_MOTER_RPM_X10          2000
#define PRESS_MOTER_MAX_RPM_X10       3500   /* 350rpm：实测 373rpm 起必失步（24V 母线电压天花板），表内 350rpm 以上档位不再可达 */
/* 大于 100 表示启用 SP_57_8412_AmpTable 自动查表。 */
#define PRESS_MOTER_AUTO_AMPLITUDE    101U

/* 转�?电流幅值查�?
 * 按转速区�?rpm_x10)分段，每段对应一个电流幅�?
 * 电机转速落�?[rpm, 下一行rpm) 区间内时使用对应幅�?
 * 最后一行为最高转速上限，超过则用用户设定�?
 * 修改表项可自由插入新的转速电流区�?
 */
static const struct {
	unsigned int  rpm_x10;   /* 区间下限转�?×10 */
	unsigned char amplitude;  /* 该区间电流幅�?*/
} SP_57_8412_AmpTable[] = {
	{ 0,    20 },   /* 0~25RPM:    20% */
	{ 250,  22 },   /* 25~50RPM:   22% */
	{ 500,  25 },   /* 50~75RPM:   25% */
	{ 750,  27 },   /* 75~100RPM:  27% */
	{ 1000, 30 },   /* 100~125RPM: 30% */
	{ 1250, 32 },   /* 125~150RPM: 32% */
	{ 1500, 35 },   /* 150~175RPM: 35% */
	{ 1750, 38 },   /* 175~200RPM: 38% */
	{ 2000, 52 },   /* 200~225RPM: 52% */
	{ 2250, 58 },   /* 225~237.5RPM: 58% */
	{ 2375, 64 },   /* 237.5~250RPM: 64% */
	/* 200RPM 以上先整体抬高幅值：先保证不丢步，标定完成后再逐点下调 */
	{ 2500, 70 },   /* 250~262.5RPM: 70% */
	{ 2625, 75 },   /* 262.5~275RPM: 75% */
	{ 2750, 81 },   /* 275~287.5RPM: 81% */
	{ 2875, 86 },   /* 287.5~300RPM: 86% */
	{ 3000, 88 },   /* 300~312.5RPM: 88% */
	{ 3125, 90 },   /* 312.5~325RPM: 90% */
	{ 3250, 91 },   /* 325~337.5RPM: 91% */
	{ 3375, 92 },   /* 337.5~350RPM: 92% */
	{ 3500, 92 },   /* 350~360RPM: 92% */
	{ 3600, 100 },  /* 360RPM 以上：直接满幅 */
	{ 3750, 100 },  /* 375~400RPM: 100% */
	{ 4000, 100 },  /* 400~425RPM: 100% */
	{ 4250, 100 },  /* 425~450RPM: 100% */
	{ 4500, 100 },  /* 450~475RPM: 100% */
	{ 4750, 100 },  /* 475~500RPM: 100% */
	{ 5000, 100 },  /* 500~525RPM: 100% */
	{ 5250, 100 },  /* 525~550RPM: 100% */
	{ 5500, 100 },  /* 550~575RPM: 100% */
	{ 5750, 100 },  /* 575~600RPM: 100% */
	{ 6000, 100 },  /* 600RPM 以上：同样满幅 100%（原 0=标记行已弃用） */
};
#define SP_57_8412_AMP_TABLE_SIZE  (sizeof(SP_57_8412_AmpTable) / sizeof(SP_57_8412_AmpTable[0]))

/* 256点正弦表，值域 0~3699
 * 1850 = 50%占空�?= 零电�?
 * 3699 = 100%占空�?= 正向满电�?
 * 0    = 0%占空�?  = 反向满电�?
 *
 * 公式: 1850 +  1849* sin(2π * i / 256)
 */
static const uint16_t SP_57_8412_SineTable256[256] = {
	1875,1921,1967,2013,2059,2105,2150,2196,2241,2286,2331,2375,2419,2463,2507,2550,
	2593,2635,2677,2718,2759,2799,2839,2878,2917,2955,2992,3029,3064,3100,3134,3168,
	3201,3233,3264,3295,3324,3353,3381,3408,3434,3459,3483,3506,3529,3550,3570,3589,
	3607,3624,3640,3655,3669,3682,3694,3704,3714,3722,3730,3736,3741,3745,3748,3749,
	3750,3749,3748,3745,3741,3736,3730,3722,3714,3704,3694,3682,3669,3655,3640,3624,
	3607,3589,3570,3550,3529,3506,3483,3459,3434,3408,3381,3353,3324,3295,3264,3233,
	3201,3168,3134,3100,3064,3029,2992,2955,2917,2878,2839,2799,2759,2718,2677,2635,
	2593,2550,2507,2463,2419,2375,2331,2286,2241,2196,2150,2105,2059,2013,1967,1921,
	1875,1829,1783,1737,1691,1645,1600,1554,1509,1464,1419,1375,1331,1287,1243,1200,
	1157,1115,1073,1032,991,951,911,872,833,795,758,721,686,650,616,582,
	549,517,486,455,426,397,369,342,316,291,267,244,221,200,180,161,
	143,126,110,95,81,68,56,46,36,28,20,14,9,5,2,1,
	0,1,2,5,9,14,20,28,36,46,56,68,81,95,110,126,
	143,161,180,200,221,244,267,291,316,342,369,397,426,455,486,517,
	549,582,616,650,686,721,758,795,833,872,911,951,991,1032,1073,1115,
	1157,1200,1243,1287,1331,1375,1419,1464,1509,1554,1600,1645,1691,1737,1783,1829
};

/*
 * 微步进状态（内部变量�?
 */
volatile unsigned char StepperPulse57_8412_Enable    = 0;
volatile unsigned short StepperPulse57_8412_CurrA_Zero = 0;  /* IN11 零点基准 */
volatile unsigned short StepperPulse57_8412_CurrB_Zero = 0;  /* IN12 零点基准 */
static unsigned char  SP_57_8412_Amplitude           = 60;
static unsigned char  SP_57_8412_TargetAmplitude     = 60;  /* 用户设定的目标电流幅�?*/
/* 幅值斜坡的实际目标：AUTO 模式按转速查表，手动模式为用户值。 */
static unsigned char  SP_57_8412_TargetAmplitudeReal = 60;
static unsigned short SP_57_8412_Index               = 0;
static unsigned char  SP_57_8412_Divider             = 0;
static unsigned int   SP_57_8412_StepCount           = 0;
static unsigned int   SP_57_8412_TargetSteps         = 0;
static signed char    SP_57_8412_Direction           = 1;
static unsigned char  SP_57_8412_Mode                = SP_57_8412_MODE_IDLE;
static signed long    SP_57_8412_Position            = 0;
static unsigned char  SP_57_8412_SpeedDivider        = 6;
static unsigned short SP_57_8412_StepRate            = 64;
static unsigned short SP_57_8412_StepAccum           = 0;


/* S形加减速状态（�?TMR4 中断�?10ms 更新）�?*/
static volatile unsigned char  SP_57_8412_RampActive          = 0;
static volatile unsigned int   SP_57_8412_RampTime            = 0;
static volatile unsigned int   SP_57_8412_RampTotal           = 0;
static volatile unsigned int   SP_57_8412_RampStartRPM        = 0;
static volatile unsigned int   SP_57_8412_RampTargetRPM       = 0;
static volatile unsigned int   SP_57_8412_CurrentRPM          = 0;
static unsigned short          SP_57_8412_RampTickCounter     = 0;

/* 幅值斜率状态：SP_57_8412_Amplitude 每 10ms 向目标幅值限速逼近。 */
static volatile unsigned char  SP_57_8412_AmpSlewActive       = 0;
/* 停止斜坡阶段2：电流幅值斜降（0.1%/tick 计数，负值=正在下降）。 */
static volatile signed short   SP_57_8412_StopFadeLevel       = 0;
static volatile unsigned char  SP_57_8412_StopFading          = 0;
/* 停止斜坡流程进行中标志（RunAtSpeed 需要打断它重新启动）。 */
static volatile unsigned char  SP_57_8412_StopUseRamp         = 0;
/* Phase 2 fade step (0.1% amplitude per 10 ms tick), computed by StopRamped. */
static volatile unsigned char  SP_57_8412_StopFadeStep        = 1;

/* 压制机构位置状态，供 Modbus FC02 地址 2/3 读取。 */
 volatile uint8_t pressMoterHighPosition;
 volatile uint8_t pressMoterLowPosition;
 volatile uint8_t pressMoterRunning;
static volatile int8_t pressMoterDirection;

volatile uint32_t moter57LowEncNum = 75000;

static uint8_t pressMoterLowTargetReached(void)
{
	int32_t position = encPressMoterREncPositionGet();
	uint32_t pulses = (uint32_t)(position < 0 ? -position : position);
	return (pulses >= moter57LowEncNum) ? 1U : 0U;
}

/* 位置状态只由高位开关和当前编码器目标决定，不随运动命令清零。 */
static void pressMoterPositionUpdate(void)
{
	if(gpio_input_data_bit_read(pressHighPos_GPIO_PORT, pressHighPos_PIN) == RESET)
	{
		  pressMoterHighPosition = 1U;
	}
	else
	{
		pressMoterHighPosition = 0U;
	}
	if(pressMoterLowTargetReached() != 0U)
	{
		pressMoterLowPosition = 1U;
	}
	else
	{
		pressMoterLowPosition = 0U;
	}
}

static void pressMoterMonitor(void)
{
	pressMoterPositionUpdate();
	/* 所有启动入口均受保护，不依赖上层运行标志。底层正转为下行。 */
	if(StepperPulse57_8412_Enable == 0U)
	{
		return;
	}

	/* 上行：PB12 高位限位有效。 */
	if(pressMoterHighPosition == 1 && SP_57_8412_Direction < 0)
	{
		StepperPulse57_8412_StopRamped();
		pressMoterRunning = 0U;
		/* PB12 高位是压制编码器的机械零点。 */
		encPressMoterREncPositionReset();
		encPressMoterREncBPulseReset();
		pressMoterPositionUpdate();
		return;
	}

	if(pressMoterLowPosition == 1 && SP_57_8412_Direction > 0)
	{
		/* 下行：压制编码器已经达到指定脉冲数。 */
		StepperPulse57_8412_StopRamped();
		pressMoterRunning = 0U;
		
	}
}

/*
 * 根据目标 RPM 计算 SpeedDivider �?StepRate
 *
 * rpm_x10 �?×10 定点转速（�?5=0.5RPM, 1100=110.0RPM�?
 * RPM = 3375 × StepRate / (2048 × Divider)
 *   StepRate 范围 64~512（即 1.0~8.0 步）
 *   SpeedDivider 范围 1~255
 *   最�?RPM = 30000 × 512 / 2048 = 7500(×10) = 750RPM
 */
static void SP_57_8412_CalcRPM(unsigned int rpm_x10, unsigned char *divider, unsigned short *stepRate)
{
	unsigned short bestSR = 64;
	unsigned char bestDiv = 255;
	unsigned long bestErr = 0xFFFFFFFF;
	unsigned short i;

	if (rpm_x10 == 0)
	{
		*divider = 255;
		*stepRate = 64;
		return;
	}

	/* Divider = 3000 × StepRate × 10 / (2048 × rpm_x10)
	 * 实际RPM×10 = 30000 × StepRate / (2048 × Divider) */
	for (i = 64; i <= 512; i++)
	{
		/* AT32F415 TMR4 update rate is 40 kHz. The STM32F103 version used
		 * 30000 here for a 20 kHz update rate. */
		unsigned long idealDiv = (60000UL * i) / (2048UL * rpm_x10);
		unsigned long actualRPMx10, err;

		if (idealDiv < 1) idealDiv = 1;
		if (idealDiv > 255) idealDiv = 255;

		actualRPMx10 = (60000UL * i) / (2048UL * idealDiv);
		err = (actualRPMx10 > rpm_x10) ? (actualRPMx10 - rpm_x10) : (rpm_x10 - actualRPMx10);

		if (err < bestErr)
		{
			bestErr = err;
			bestSR = i;
			bestDiv = (unsigned char)idealDiv;
		}
	}

	*divider = bestDiv;
	*stepRate = bestSR;
}

/* 根据当前转速获取电流幅值（区间内线性插值）
 * 区间表按转速升序排列，命中区间后在上下限幅值之间按转速线性插值
 * 末行幅值为0为标记行，表示用用户设定值
 */
static unsigned char SP_57_8412_CalcAmplitude(unsigned int rpm_x10)
{
	unsigned char i;
	unsigned int  prevRpm, curRpm;
	int           prevAmp, curAmp, amp;
	for (i = 0; i < SP_57_8412_AMP_TABLE_SIZE; i++)
	{
		if (rpm_x10 < SP_57_8412_AmpTable[i].rpm_x10)
		{
			/* 低于第一行下限，用第一行幅�?*/
			if (i == 0)
				return SP_57_8412_AmpTable[0].amplitude;
			prevRpm = SP_57_8412_AmpTable[i - 1].rpm_x10;
			curRpm = SP_57_8412_AmpTable[i].rpm_x10;
			prevAmp = (int)SP_57_8412_AmpTable[i - 1].amplitude;
			curAmp = (int)SP_57_8412_AmpTable[i].amplitude;
			if (curAmp == 0)
			{
				/* 上限行是标记行：区间内保持原幅值，越过上限后才用用户设定值 */
				return (unsigned char)prevAmp;
			}
			if (prevAmp == 0)
			{
				/* 下限行是标记行：直接用用户设定值 */
				return SP_57_8412_TargetAmplitude;
			}
			if (curRpm <= prevRpm)
			{
				/* 区间宽度异常：退回阶梯取值 */
				return (unsigned char)prevAmp;
			}
			/* 区间内线性插值：amp = prev + (cur-prev)*(rpm-prev)/(curRpm-prevRpm)，四舍五入 */
			amp = prevAmp + ((curAmp - prevAmp) * (int)(rpm_x10 - prevRpm) + (int)(curRpm - prevRpm) / 2)
			      / (int)(curRpm - prevRpm);
			return (unsigned char)amp;
		}
		if (SP_57_8412_AmpTable[i].amplitude == 0)
		{
			/* 标记行：用用户设定�?*/
			return SP_57_8412_TargetAmplitude;
		}
	}
	/* 超过所有区间，用用户设定�?*/
	return SP_57_8412_TargetAmplitude;
}

static void SP_57_8412_ApplyRPM(unsigned int rpm_x10, signed char direction)
{
	unsigned char div;
	unsigned short sr;
	SP_57_8412_Direction = direction;
	SP_57_8412_CalcRPM(rpm_x10, &div, &sr);
	SP_57_8412_SpeedDivider = div;
	SP_57_8412_StepRate = sr;
	SP_57_8412_CurrentRPM = rpm_x10;
	/* 幅值统一走斜率限制：这里只写目标，实际 SP_57_8412_Amplitude
	 * 由 Tick 内的 SP_57_8412_AmpSlewUpdate 每 10ms 向目标逼近。 */
	if (SP_57_8412_TargetAmplitude <= 100U)
		SP_57_8412_TargetAmplitudeReal = SP_57_8412_TargetAmplitude;
	else
	{
		SP_57_8412_TargetAmplitudeReal = SP_57_8412_CalcAmplitude(rpm_x10);
		/* 标记行会在超出表范围时返回用户设定值(自动模式下达 101)，钳到 100%。 */
		if (SP_57_8412_TargetAmplitudeReal > 100U)
			SP_57_8412_TargetAmplitudeReal = 100U;
	}
}

/*
 * 初始化步进电机算�?
 * �?MX_TIM3_Init() 之后调用
 * 配置 PA6/PA7 为零电流，启�?TIM3 中断
 * 初始化时�?ADC1_IN11(PC1)/ADC1_IN12(PC2) 各采�?20 次求平均�?
 * 作为绕组 A/B 电流的零点基准（电机静止时执行）
 */
void StepperPulse57_8412_Init(void)
{
	unsigned char i;
	unsigned long sumA = 0;
	unsigned long sumB = 0;

	/* 先启�?PWM 通道输出 */
	tmr_channel_enable(TMR4, TMR_SELECT_CHANNEL_3, TRUE);
	tmr_channel_enable(TMR4, TMR_SELECT_CHANNEL_4, TRUE);

	/* 初始：零电流 */
	tmr_channel_value_set(TMR4, TMR_SELECT_CHANNEL_3, SP_57_8412_PWM_CENTER);
	tmr_channel_value_set(TMR4, TMR_SELECT_CHANNEL_4, SP_57_8412_PWM_CENTER);

	/* 使能 TIM3 Update 中断并启动定时器 */

	/* ADC 零点标定：电机静止时�?IN11/IN12 各采�?20 次求平均 */
	for (i = 0; i < 20; i++)
	{
		sumA += adc_read_channel(ADC_CHANNEL_11);
		sumB += adc_read_channel(ADC_CHANNEL_12);
	}
	StepperPulse57_8412_CurrA_Zero = (unsigned short)(sumA / 20);
	StepperPulse57_8412_CurrB_Zero = (unsigned short)(sumB / 20);

	StepperPulse57_8412_Enable = 0;
	SP_57_8412_Mode            = SP_57_8412_MODE_IDLE;
}
/*
 * TIM3 中断回调 �?�?HAL_TIM_PeriodElapsedCallback �?TIM3 分支中调�?
 *
 * 每个 TIM3 周期触发一次，分频后更�?CCR1/CCR2，推进一步正弦相位�?
 * 低速时也持续刷�?CCR，保持电流波形连续�?
 * S形加减速由本函数内�?10ms 节拍自动更新�?
 */
/* Current amplitude slew: SP_57_8412_Amplitude moves toward the real target
 * by at most AMP_SLEW_PER_TICK percent every 10 ms, smoothing the torque
 * steps that excite low-speed vibration. Runs in the TMR4 ISR. */
static void StepperPulse57_8412_AmpSlewUpdate(void)
{
#if SP_57_8412_AMP_SLEW_PER_TICK > 0
	signed int diff;

	/* Frozen during the whole stop ramp (phase 1 decel + phase 2 fade); the
	 * fade path writes Amplitude directly, and the AUTO table must not raise
	 * torque back while the motor is stopping. */
	if (SP_57_8412_StopFading || SP_57_8412_StopUseRamp)
		return;

	if (SP_57_8412_Amplitude == SP_57_8412_TargetAmplitudeReal)
		return;

	diff = (signed int)SP_57_8412_TargetAmplitudeReal - (signed int)SP_57_8412_Amplitude;
	if (diff > SP_57_8412_AMP_SLEW_PER_TICK)
		diff = SP_57_8412_AMP_SLEW_PER_TICK;
	else if (diff < -SP_57_8412_AMP_SLEW_PER_TICK)
		diff = -SP_57_8412_AMP_SLEW_PER_TICK;
	SP_57_8412_Amplitude = (unsigned char)((signed int)SP_57_8412_Amplitude + diff);
#else
	SP_57_8412_Amplitude = SP_57_8412_TargetAmplitudeReal;
#endif
}

/* Stop ramp driver, called every 10 ms from Tick while StopUseRamp is set.
 * Phase 1 is driven by RampUpdate (S curve down to the coast speed); when it
 * completes, phase 2 fades the winding current to zero, then calls Stop(). */
static void StepperPulse57_8412_StopRampTick(void)
{
#if SP_57_8412_STOP_RAMPED_ENABLE
	if (!SP_57_8412_StopUseRamp)
		return;

	if (!SP_57_8412_StopFading)
	{
		if (SP_57_8412_RampActive)
			return; /* phase 1 still decelerating */

		/* Phase 1 finished: start the current fade from the present level. */
		SP_57_8412_StopFading = 1;
		SP_57_8412_StopFadeLevel =
			(signed short)((unsigned int)SP_57_8412_Amplitude * 10U);
		return;
	}

	SP_57_8412_StopFadeLevel -= SP_57_8412_StopFadeStep;
	if (SP_57_8412_StopFadeLevel <= 0)
		StepperPulse57_8412_Stop();
	else
		SP_57_8412_Amplitude = (unsigned char)(SP_57_8412_StopFadeLevel / 10U);
#endif
}

void StepperPulse57_8412_Tick(void)
{
	uint16_t valA, valB, idx;

	pressMoterMonitor();

	if (!StepperPulse57_8412_Enable)
		return;

	/* TMR4 runs at 40 kHz. Update the S curve at 10 ms intervals. */
	if (++SP_57_8412_RampTickCounter >= SP_57_8412_RAMP_TICKS)
	{
		SP_57_8412_RampTickCounter = 0;
		StepperPulse57_8412_RampUpdate();
		StepperPulse57_8412_AmpSlewUpdate();
		StepperPulse57_8412_StopRampTick();
	}

	/* 分频：每 SpeedDivider 次中断才推进一�?*/
	if (++SP_57_8412_Divider < SP_57_8412_SpeedDivider)
	{
		/* 即使未推进一步，也刷�?CCR 保持电流连续 */
		idx = SP_57_8412_Index;
		valA = SP_57_8412_SineTable256[idx];
		valB = SP_57_8412_SineTable256[(idx + SP_57_8412_SINE_POINTS / 4) % SP_57_8412_SINE_POINTS];
		/* 按幅值缩�?*/
		valA = SP_57_8412_PWM_CENTER + (uint16_t)((int16_t)(valA - SP_57_8412_PWM_CENTER) * SP_57_8412_Amplitude / 100);
		valB = SP_57_8412_PWM_CENTER + (uint16_t)((int16_t)(valB - SP_57_8412_PWM_CENTER) * SP_57_8412_Amplitude / 100);
		tmr_channel_value_set(TMR4, TMR_SELECT_CHANNEL_3, valA);
		tmr_channel_value_set(TMR4, TMR_SELECT_CHANNEL_4, valB);
		return;
	}
	SP_57_8412_Divider = 0;

	/* 分数步进：StepRate �?×64 定点�?*/
	SP_57_8412_StepAccum += SP_57_8412_StepRate;
	while (SP_57_8412_StepAccum >= 64)
	{
		SP_57_8412_StepAccum -= 64;
		if (SP_57_8412_Direction >= 0)
			SP_57_8412_Index = (SP_57_8412_Index + 1) & 0xFF;
		else
			SP_57_8412_Index = (SP_57_8412_Index - 1) & 0xFF;
	}

	/* 绕组1 = sin(θ)，绕�? = sin(θ+90°) = cos(θ) */
	idx = SP_57_8412_Index;
	valA = SP_57_8412_SineTable256[idx];
	valB = SP_57_8412_SineTable256[(idx + SP_57_8412_SINE_POINTS / 4) % SP_57_8412_SINE_POINTS];

	/* 按幅值百分比缩放 */
	valA = SP_57_8412_PWM_CENTER + (uint16_t)((int16_t)(valA - SP_57_8412_PWM_CENTER) * SP_57_8412_Amplitude / 100);
	valB = SP_57_8412_PWM_CENTER + (uint16_t)((int16_t)(valB - SP_57_8412_PWM_CENTER) * SP_57_8412_Amplitude / 100);

	tmr_channel_value_set(TMR4, TMR_SELECT_CHANNEL_3, valA);
	tmr_channel_value_set(TMR4, TMR_SELECT_CHANNEL_4, valB);

	/* 累计步数和绝对位�?*/
	SP_57_8412_StepCount++;
	if (SP_57_8412_Direction >= 0)
		SP_57_8412_Position++;
	else
		SP_57_8412_Position--;

	/* 步数模式：走到目标步数则自动停止 */
	if (SP_57_8412_Mode == SP_57_8412_MODE_STEPS &&
		SP_57_8412_TargetSteps != 0 &&
		SP_57_8412_StepCount >= SP_57_8412_TargetSteps)
	{
		StepperPulse57_8412_StopRamped();
	}
}

/* 停止，零电流 */
void StepperPulse57_8412_Stop(void)
{
	SP_57_8412_RampActive   = 0;
	SP_57_8412_CurrentRPM   = 0;
	SP_57_8412_RampTickCounter = 0;
	SP_57_8412_StopFading   = 0;
	SP_57_8412_StopFadeLevel = 0;
	SP_57_8412_AmpSlewActive = 0;
	SP_57_8412_StopUseRamp  = 0;
	StepperPulse57_8412_Enable = 0;
	SP_57_8412_Mode            = SP_57_8412_MODE_IDLE;
	tmr_channel_value_set(TMR4, TMR_SELECT_CHANNEL_3, SP_57_8412_PWM_CENTER);
	tmr_channel_value_set(TMR4, TMR_SELECT_CHANNEL_4, SP_57_8412_PWM_CENTER);
}

/* Ramped stop: S-curve deceleration down to a low coast speed, then fade the
 * winding current to zero, then zero-current stop. Callers keep their normal
 * Running-flag handling; Tick finishes the sequence in the background. */
void StepperPulse57_8412_StopRamped(void)
{
#if SP_57_8412_STOP_RAMPED_ENABLE
	unsigned int fadeStep;

	if (!StepperPulse57_8412_Enable)
	{
		StepperPulse57_8412_Stop();
		return;
	}

	/* Re-entry guard: while a stop ramp is already in progress, keep its
	 * timeline. A repeated StopRamped() (e.g. Monitor hitting the limit
	 * every 10 ms while the ramp decelerates) would otherwise restart the
	 * S-curve each call and the fade phase would never run. */
	if (SP_57_8412_StopUseRamp)
		return;

	/* Phase 2 fade rate: amplitude falls by fadeStep (0.1% units) per 10 ms
	 * tick; STOP_FADE_TIME_MS/10 ticks finish the fade. */
	fadeStep = ((unsigned int)SP_57_8412_Amplitude * 10U) /
	           (SP_57_8412_STOP_FADE_TIME_MS / 10U);
	if (fadeStep == 0U)
		fadeStep = 1U;
	SP_57_8412_StopFadeStep = (unsigned char)fadeStep;

	if (SP_57_8412_RampActive || SP_57_8412_CurrentRPM > SP_57_8412_STOP_COAST_RPM_X10)
	{
		/* Phase 1: S curve from the current speed down to the coast speed. */
		SP_57_8412_RampStartRPM  = SP_57_8412_CurrentRPM;
		SP_57_8412_RampTargetRPM = SP_57_8412_STOP_COAST_RPM_X10;
		SP_57_8412_RampTime      = 0;
		SP_57_8412_RampTotal     = SP_57_8412_STOP_RAMP_TIME_MS;
		SP_57_8412_RampActive    = 1;
		SP_57_8412_RampTickCounter = 0;
		SP_57_8412_ApplyRPM(SP_57_8412_CurrentRPM, SP_57_8412_Direction);
	}

	/* Stop as if running free; MODE_STEPS must not re-trigger the target. */
	SP_57_8412_TargetSteps = 0;
	SP_57_8412_Mode        = SP_57_8412_MODE_RUN;
	/* Keep at least one amplitude percent of holding torque while fading, so
	 * the load cannot ratchet the mechanism back during the coast phase. */
	SP_57_8412_Amplitude = (SP_57_8412_Amplitude != 0U) ? SP_57_8412_Amplitude : 1U;
	SP_57_8412_StopUseRamp = 1;
#else
	StepperPulse57_8412_Stop();
#endif
}

void pressMoterUp(int v)
{
	uint32_t irqMask;
	unsigned int speed = (v < 0) ? (unsigned int)(-v) : (unsigned int)v;
	if(speed == 0U) speed = PRESS_MOTER_RPM_X10;
	if(speed > PRESS_MOTER_MAX_RPM_X10) speed = PRESS_MOTER_MAX_RPM_X10;
	irqMask = __get_PRIMASK();
	__disable_irq();
	/* 若高位已经触发，无需再次向上运行。 */
	if(gpio_input_data_bit_read(pressHighPos_GPIO_PORT, pressHighPos_PIN) == RESET)
	{
		StepperPulse57_8412_Stop();
		pressMoterRunning = 0U;
		encPressMoterREncPositionReset();
		encPressMoterREncBPulseReset();
		pressMoterPositionUpdate();
		__set_PRIMASK(irqMask);
		return;
	}

	/* 同向运行时走变速路径；换向或从静止起步时才需要先停再启。 */
	if (StepperPulse57_8412_Enable != 0U && SP_57_8412_Direction < 0)
	{
		/* 已在下行：只改目标速度，S 曲线从当前转速渐变到新速度，
		 * 不打断脉冲分频、不重置斜坡计时，否则速度会瞬间跳变。 */
		StepperPulse57_8412_RunAtSpeed(-(int)speed, PRESS_MOTER_AUTO_AMPLITUDE, 8000U, 0U);
		pressMoterDirection = 1;
		pressMoterRunning = 1U;
		__set_PRIMASK(irqMask);
		return;
	}

	/* 确保由停止状态重新开始，避免复用上一次下行的脉冲计数。 */
	StepperPulse57_8412_Stop();
	pressMoterPositionUpdate();
	pressMoterDirection = 1;
	pressMoterRunning = 1U;
	StepperPulse57_8412_Run(-(int)speed, PRESS_MOTER_AUTO_AMPLITUDE);
	__set_PRIMASK(irqMask);
}

void pressMoterDown(int v)
{
	uint32_t irqMask;
	unsigned int speed = (v < 0) ? (unsigned int)(-v) : (unsigned int)v;
	if(speed == 0U) speed = PRESS_MOTER_RPM_X10;
	if(speed > PRESS_MOTER_MAX_RPM_X10) speed = PRESS_MOTER_MAX_RPM_X10;
	irqMask = __get_PRIMASK();
	__disable_irq();
	pressMoterPositionUpdate();
	/* 已达到目标（包括目标为 0）时禁止再次下行。 */
	if(pressMoterLowPosition != 0U)
	{
		StepperPulse57_8412_Stop();
		pressMoterRunning = 0U;
		__set_PRIMASK(irqMask);
		return;
	}

	if (StepperPulse57_8412_Enable != 0U && SP_57_8412_Direction > 0)
	{
		/* 已在上行：只改目标速度，S 曲线从当前转速渐变到新速度，
		 * 不打断脉冲分频、不重置斜坡计时，否则速度会瞬间跳变。 */
		StepperPulse57_8412_RunAtSpeed((int)speed, PRESS_MOTER_AUTO_AMPLITUDE, 8000U, 0U);
		pressMoterDirection = -1;
		pressMoterRunning = 1U;
		__set_PRIMASK(irqMask);
		return;
	}

	/* 下行目标以 PB12 高位归零后的压制编码器绝对值为准。 */
	StepperPulse57_8412_Stop();
	pressMoterDirection = -1;
	pressMoterRunning = 1U;
	StepperPulse57_8412_Run((int)speed, PRESS_MOTER_AUTO_AMPLITUDE);
	__set_PRIMASK(irqMask);
}

void pressMoterStop(void)
{
	StepperPulse57_8412_StopRamped();
	pressMoterRunning = 0U;
	/* 手动停止时保留最后一个已确认的高/低位状态。 */
}

uint8_t pressMoterHighPositionGet(void)
{
	return pressMoterHighPosition;
}

uint8_t pressMoterLowPositionGet(void)
{
	return pressMoterLowPosition;
}

/* 运行中调整电流幅�?*/
void StepperPulse57_8412_SetAmplitude(unsigned char amplitude)
{
	SP_57_8412_TargetAmplitude = amplitude;
	SP_57_8412_TargetAmplitudeReal = (amplitude <= 100U) ? amplitude : SP_57_8412_TargetAmplitude;
	SP_57_8412_AmpSlewActive = 1;
}

/* 获取绝对位置（微步） */
signed long StepperPulse57_8412_GetPosition(void)
{
	return SP_57_8412_Position;
}

/* 获取当前累积角度 */
signed int StepperPulse57_8412_GetCurrentAngle(void)
{
	return (signed int)(SP_57_8412_Position * 360 / STEPS_PER_REV);
}

/* 获取当前转速（rpm_x10），调试�?*/
unsigned int StepperPulse57_8412_GetCurrentRPM(void)
{
	return SP_57_8412_CurrentRPM;
}

/* 获取当前电流幅值百分比，调试用 */
unsigned char StepperPulse57_8412_GetAmplitude(void)
{
	return SP_57_8412_Amplitude;
}

/* 获取当前方向�?=正转�?1=反转），调试�?*/
signed char StepperPulse57_8412_GetDirection(void)
{
	return SP_57_8412_Direction;
}

/* 获取S曲线加速进行中标志，调试用 */
unsigned char StepperPulse57_8412_IsRamping(void)
{
	return SP_57_8412_RampActive;
}

/*
 * 持续运行（速度模式�?
 *   rpm_x10      �?转�?×10，正=正转，负=反转�?=停止
 *   amplitude    �?电流幅�?0~100
 *   accelTime_ms �?加速时间ms�?0时启用S形加速，0=立即启动�?
 *   startRpm_x10 �?起步转�?×10（S形加速起始速度�?=使用默认300RPM�?
 */
static void SP_57_8412_RunAtSpeedUnchecked(signed int rpm_x10, unsigned char amplitude, unsigned int accelTime_ms, unsigned int startRpm_x10)
{
	signed char direction;
	unsigned int startRPM;

	if (rpm_x10 == 0)
	{
		StepperPulse57_8412_Stop();
		return;
	}

	/* 重新启动会打断进行中的停止斜坡，直接走正常启动路径。 */
	SP_57_8412_StopUseRamp = 0;
	SP_57_8412_StopFading  = 0;

	direction = (rpm_x10 > 0) ? 1 : -1;
	if (rpm_x10 < 0)
		rpm_x10 = -rpm_x10;

	/* 起步转速：用户指定 > 0 用用户值，否则用默�?*/
	startRPM = (startRpm_x10 > 0) ? startRpm_x10 : SP_57_8412_START_RPM_X10;
	if (startRpm_x10 == 0)
	{
		startRPM = (unsigned int)rpm_x10 / SP_57_8412_START_FRACTION;
		if (startRPM < SP_57_8412_START_FLOOR_RPM_X10)
			startRPM = SP_57_8412_START_FLOOR_RPM_X10;
		if (startRPM > SP_57_8412_START_CEIL_RPM_X10)
			startRPM = SP_57_8412_START_CEIL_RPM_X10;
		if ((unsigned int)rpm_x10 <= startRPM)
			startRPM = (unsigned int)rpm_x10;
	}

	SP_57_8412_TargetAmplitude = amplitude;
	SP_57_8412_TargetAmplitudeReal = (amplitude <= 100U)
	                                 ? amplitude
	                                 : SP_57_8412_CalcAmplitude((unsigned int)rpm_x10);

	/* 设置了加速时�?�?S形加�?*/
	if (accelTime_ms > 0)
	{
		/* 如果电机未运行，从起步速度开始加�?*/
		if (!StepperPulse57_8412_Enable || SP_57_8412_Direction != direction)
		{
			if ((unsigned int)rpm_x10 <= startRPM)
			{
				/* 目标速度 �?起步速度，直接以目标速度启动 */
				SP_57_8412_RampStartRPM  = (unsigned int)rpm_x10;
				SP_57_8412_CurrentRPM    = (unsigned int)rpm_x10;
				SP_57_8412_RampActive    = 0;
			}
			else
			{
				SP_57_8412_RampStartRPM  = startRPM;
				SP_57_8412_CurrentRPM    = startRPM;
				SP_57_8412_RampActive    = 1;
			}
		}
		else
		{
			/* 电机已在运行，从当前速度继续加�?*/
			if ((unsigned int)rpm_x10 == SP_57_8412_CurrentRPM)
			{
				/* 目标速度等于当前速度，无需更新曲线�?*/
				SP_57_8412_RampActive = 0;
			}
			else
			{
				SP_57_8412_RampStartRPM = SP_57_8412_CurrentRPM;
				SP_57_8412_RampActive   = 1;
			}
		}

		SP_57_8412_RampTargetRPM = (unsigned int)rpm_x10;
		SP_57_8412_RampTotal     = accelTime_ms;
		SP_57_8412_RampTime      = 0;
		SP_57_8412_RampTickCounter = 0;

		/* 设置初始速度 */
		SP_57_8412_ApplyRPM(SP_57_8412_CurrentRPM, direction);

		SP_57_8412_Mode        = SP_57_8412_MODE_RUN;
		SP_57_8412_TargetSteps = 0;

		if (!StepperPulse57_8412_Enable)
		{
			SP_57_8412_Index     = 0;
			SP_57_8412_Divider   = 0;
			SP_57_8412_StepAccum = 0;
			SP_57_8412_StepCount = 0;
			/* 从零电流起坡，消除启动瞬间 0→目标幅值的阶跃 */
			SP_57_8412_Amplitude = 0;
			StepperPulse57_8412_Enable = 1;
		}
	}
	else
	{
		/* 无加速时�?�?直接启动 */
		SP_57_8412_RampActive = 0;
		SP_57_8412_ApplyRPM((unsigned int)rpm_x10, direction);
		SP_57_8412_Mode        = SP_57_8412_MODE_RUN;
		SP_57_8412_TargetSteps = 0;

		if (!StepperPulse57_8412_Enable)
		{
			SP_57_8412_Index     = 0;
			SP_57_8412_Divider   = 0;
			SP_57_8412_StepAccum = 0;
			SP_57_8412_StepCount = 0;
			SP_57_8412_Amplitude = 0;
			StepperPulse57_8412_Enable = 1;
		}
	}
}

/* 简化版持续运行（无加速，直接启动�?*/
/* 底层启动也检查位置，并防止启动过程中被中断停止后重新使能。 */
void StepperPulse57_8412_RunAtSpeed(signed int rpm_x10, unsigned char amplitude, unsigned int accelTime_ms, unsigned int startRpm_x10)
{
	uint32_t irqMask = __get_PRIMASK();
	__disable_irq();
	pressMoterPositionUpdate();
	if((rpm_x10 > 0 && pressMoterLowPosition != 0U) ||
	   (rpm_x10 < 0 && pressMoterHighPosition != 0U))
	{
		StepperPulse57_8412_Stop();
		pressMoterRunning = 0U;
	}
	else
	{
		SP_57_8412_RunAtSpeedUnchecked(rpm_x10, amplitude, accelTime_ms, startRpm_x10);
	}
	__set_PRIMASK(irqMask);
}

void StepperPulse57_8412_Run(signed int rpm_x10, unsigned char amplitude)
{
	/* 末参传 0：起步速度由 RunAtSpeed 按目标速度动态计算（目标/3，
	 * 钳位 [50,300]RPM），避免低速压制指令直接以 300RPM 速度阶跃启动。 */
	StepperPulse57_8412_RunAtSpeed(rpm_x10, amplitude, 8000, 0);
}

/* 按 SP_57_8412_AmpTable 自动查表运行（幅值由转速决定）。
 * 幅值传查表标记值(>100)，由 ApplyRPM 按当前转速查表得到实际幅值；
 * 与 pressMoterUp/Down 走同一条受保护的启动路径，供调试口验证电流表用。
 *   rpm_x10 — 转速×10，正=正转，负=反转，0=停止 */
void StepperPulse57_8412_RunAutoTable(signed int rpm_x10)
{
	StepperPulse57_8412_RunAutoTableEx(rpm_x10, 0U);
}

/* 同上，但可指定 S 曲线加速时间；0 表示沿用默认 8000ms。
 * 速度按压制机构机械上限 PRESS_MOTER_MAX_RPM_X10 钳位。 */
void StepperPulse57_8412_RunAutoTableEx(signed int rpm_x10,
                                        unsigned int accelTime_ms)
{
	signed int rpm = rpm_x10;
	signed char direction = 1;

	if (rpm < 0)
	{
		rpm = -rpm;
		direction = -1;
	}
	if ((unsigned int)rpm > PRESS_MOTER_MAX_RPM_X10)
		rpm = (signed int)PRESS_MOTER_MAX_RPM_X10;

	if (rpm == 0)
	{
		StepperPulse57_8412_Stop();
		return;
	}

	if (direction < 0)
		rpm = -rpm;

	if (accelTime_ms == 0U)
		accelTime_ms = 8000U;

	StepperPulse57_8412_RunAtSpeed(rpm, PRESS_MOTER_AUTO_AMPLITUDE,
	                               accelTime_ms, 300U);
}

/*
 * S曲线加速更�?�?�?task10ms 中调�?
 * �?10ms 更新一次当�?RPM
 *
 * S曲线公式（smoothstep）：s = 3p² - 2p³
 *   p = elapsed / total�?�?�?
 *   Q16定点运算�?4位中间变量（主循环中安全，非ISR�?
 */
void StepperPulse57_8412_RampUpdate(void)
{
	unsigned int curRPM;
	signed long rpmDelta;

	if (!SP_57_8412_RampActive)
		return;

	SP_57_8412_RampTime += 10;
	if (SP_57_8412_RampTime >= SP_57_8412_RampTotal)
	{
		/* 加速完成，到达目标速度 */
		SP_57_8412_ApplyRPM(SP_57_8412_RampTargetRPM, SP_57_8412_Direction);
		SP_57_8412_CurrentRPM = SP_57_8412_RampTargetRPM;
		SP_57_8412_RampActive = 0;
		return;
	}

	/* S曲线（smoothstep）：s = 3p² - 2p³, Q16定点, 64位中间运�?*/
	{
		uint64_t x = ((uint64_t)SP_57_8412_RampTime * 65536UL) / SP_57_8412_RampTotal;  /* 0~65536, Q16 */
		uint64_t x2 = x * x;                                        /* Q32 */
		uint64_t x3 = x2 * x;                                       /* Q48 */
		uint64_t s = (3UL * x2 - 2UL * x3 / 65536UL) / 65536UL;   /* 0~65536, Q16 */

		rpmDelta = (signed long)SP_57_8412_RampTargetRPM - (signed long)SP_57_8412_RampStartRPM;
		curRPM = (unsigned int)((signed long)SP_57_8412_RampStartRPM + rpmDelta * (signed long)s / 65536L);
	}

	SP_57_8412_CurrentRPM = curRPM;
	SP_57_8412_ApplyRPM(curRPM, SP_57_8412_Direction);
}

/*
 * 转到指定角度（位置模式）
 *   angle     �?目标角度，正=正转，负=反转
 *   rpm_x10   �?转�?×10
 *   amplitude �?电流幅�?0~100
 */
void StepperPulse57_8412_MoveToAngle(signed int angle, unsigned int rpm_x10, unsigned char amplitude)
{
	signed long targetPos;
	long diff;

	if (rpm_x10 == 0)
		return;

	targetPos = (signed long)angle * STEPS_PER_REV / 360;
	diff = targetPos - SP_57_8412_Position;

	if (diff == 0)
		return;

	SP_57_8412_TargetSteps = (unsigned int)(diff > 0 ? diff : -diff);
	SP_57_8412_Amplitude = 0;
	SP_57_8412_TargetAmplitude = amplitude;
	SP_57_8412_TargetAmplitudeReal = (amplitude <= 100U) ? amplitude : SP_57_8412_CalcAmplitude(rpm_x10);
	SP_57_8412_AmpSlewActive = 1;
	/* Starting a move interrupts any pending stop ramp. */
	SP_57_8412_StopUseRamp = 0;
	SP_57_8412_StopFading = 0;
	SP_57_8412_ApplyRPM(rpm_x10, (diff > 0) ? 1 : -1);
	SP_57_8412_Index       = 0;
	SP_57_8412_Divider     = 0;
	SP_57_8412_StepAccum   = 0;
	SP_57_8412_StepCount   = 0;
	SP_57_8412_Mode        = SP_57_8412_MODE_STEPS;
	StepperPulse57_8412_Enable = 1;
}
/* Written by ADC1_IRQHandler and read from foreground code. */
static volatile uint16_t adcv[3];
static uint16_t SP_57_8412_CaptureA[STEPPER57_ADC_CAPTURE_MAX_SAMPLES];
static uint16_t SP_57_8412_CaptureB[STEPPER57_ADC_CAPTURE_MAX_SAMPLES];
static volatile uint16_t SP_57_8412_CaptureIndex;
static volatile uint16_t SP_57_8412_CaptureCount;
static volatile unsigned char SP_57_8412_CaptureActive;
static volatile unsigned char SP_57_8412_CaptureReady;

unsigned char StepperPulse57_8412_AdcCaptureStart(unsigned short sampleCount)
{
	if (sampleCount == 0U || sampleCount > STEPPER57_ADC_CAPTURE_MAX_SAMPLES ||
		SP_57_8412_CaptureActive)
		return 0;

	SP_57_8412_CaptureReady = 0;
	SP_57_8412_CaptureIndex = 0;
	SP_57_8412_CaptureCount = sampleCount;
	SP_57_8412_CaptureActive = 1;
	return 1;
}

unsigned char StepperPulse57_8412_AdcCaptureReady(void)
{
	return SP_57_8412_CaptureReady;
}

unsigned short StepperPulse57_8412_AdcCaptureCount(void)
{
	return SP_57_8412_CaptureCount;
}

void StepperPulse57_8412_AdcCaptureGetSample(unsigned short index,
	unsigned short *adcA, unsigned short *adcB)
{
	if (index < SP_57_8412_CaptureCount)
	{
		*adcA = SP_57_8412_CaptureA[index];
		*adcB = SP_57_8412_CaptureB[index];
	}
}

void updateMoter57CurAdc(void)
{
    adcv[0]=ADC1->pdt1;
	adcv[1]=ADC1->pdt2;
	adcv[2]=ADC1->pdt3;

	/* The ISR only stores data while capturing; serial transmission happens
	 * later from the foreground loop. */
	if (SP_57_8412_CaptureActive)
	{
		uint16_t index = SP_57_8412_CaptureIndex;
		SP_57_8412_CaptureA[index] = adcv[0];
		SP_57_8412_CaptureB[index] = adcv[1];
		index++;
		SP_57_8412_CaptureIndex = index;
		if (index >= SP_57_8412_CaptureCount)
		{
			SP_57_8412_CaptureActive = 0;
			SP_57_8412_CaptureReady = 1;
		}
	}
}

float moter57CurrA=0;
float moter57CurrB=0;
void getMoter57Curr(void)
{
	/* Convert to signed before subtracting the zero offset. Otherwise values
	 * below the offset wrap around as a very large positive unsigned value. */
	moter57CurrA = ((int32_t)adcv[0] - (int32_t)StepperPulse57_8412_CurrA_Zero) * 0.002015f;
	moter57CurrB = ((int32_t)adcv[1] - (int32_t)StepperPulse57_8412_CurrB_Zero) * 0.002015f;
}
unsigned short getMoter57AdcA(void)
{
	return adcv[0];
}
unsigned short getMoter57AdcB(void)
{
	return adcv[1];
}


void moter57FastStop(void)
{
	
}
