#include "StepperPulse57_8412.h"

/*
 * 步进电机微步进控制算法（HAL 库版本）
 *
 * 适配 zhidou-f103c8t6 工程：
 *   系统时钟: HSI/2×PLL16 = 64MHz，APB1定时器时钟=64MHz
 *   TIM3: Prescaler=0, Period=3199, UP mode, 20000 Hz
 *   PWM 中心值 1600，范围 0~3199
 *   转速系数: 3000/2048 ≈ 1.4648（由 20000Hz 推导）
 *
 * S形加速：参考 StepperPulse57_8412mos 架构
 *   加速逻辑放在 RampUpdate() 中，由主循环 task10ms 调用
 *   TIM3 中断只负责 PWM 波形生成，不含加速运算
 *
 * 命名规范：
 *   外部函数: StepperPulse57_8412_XXX (在 .h 中声明)
 *   内部函数/变量: SP_57_8412_XXX (static, 仅本文件可见)
 */

/* 机械换算：1圈 = 256微步 × 50减速比 = 12800步 */
#define STEPS_PER_REV  12800

/* PWM 参数 */
#define SP_57_8412_PWM_CENTER     1600   /* 50%占空比 = 零电流 */
#define SP_57_8412_PWM_RANGE      3200   /* 0~3199 */
#define SP_57_8412_SINE_POINTS    256

/* 运行模式 */
#define SP_57_8412_MODE_IDLE     0
#define SP_57_8412_MODE_STEPS    1   /* 走指定步数后停 */
#define SP_57_8412_MODE_RUN      2   /* 持续运行 */

/* S形加速参数 */
#define SP_57_8412_START_RPM_X10   3000    /* 启动速度 300RPM(×10)，从静止可可靠启动 */

/* 转速-电流幅值查表
 * 按转速区间(rpm_x10)分段，每段对应一个电流幅值
 * 电机转速落在 [rpm, 下一行rpm) 区间内时使用对应幅值
 * 最后一行为最高转速上限，超过则用用户设定值
 * 修改表项可自由插入新的转速电流区间
 */
static const struct {
	unsigned int  rpm_x10;   /* 区间下限转速 ×10 */
	unsigned char amplitude;  /* 该区间电流幅值 */
} SP_57_8412_AmpTable[] = {
	{ 0,    20 },   /* 0~25RPM:    20% */
	{ 250,  22 },   /* 25~50RPM:   22% */
	{ 500,  25 },   /* 50~75RPM:   25% */
	{ 750,  28 },   /* 75~100RPM:  28% */
	{ 1000, 30 },   /* 100~125RPM: 30% */
	{ 1250, 33 },   /* 125~150RPM: 33% */
	{ 1500, 35 },   /* 150~175RPM: 35% */
	{ 1750, 38 },   /* 175~200RPM: 38% */
	{ 2000, 42 },   /* 200~225RPM: 42% */
	{ 2250, 45 },   /* 225~237.5RPM: 50% */
	{ 2375, 48 },   /* 237.5~250RPM: 55% */
	{ 2500, 52 },   /* 250~262.5RPM: 60% */
	{ 2625, 55 },   /* 262.5~275RPM: 65% */
	{ 2750, 58 },   /* 275~287.5RPM: 70% */
	{ 2875, 62 },   /* 287.5~300RPM: 73% */
	{ 3000, 67 },   /* 300~312.5RPM: 76% */
	{ 3125, 78 },   /* 312.5~325RPM: 78% */
	{ 3250, 79 },   /* 325~337.5RPM: 79% */
	{ 3375, 80 },   /* 337.5~350RPM: 80% */
	{ 3500, 99 },   /* 350~375RPM: 82% */
	{ 3750, 99 },   /* 375~400RPM: 84% */
	{ 4000, 99 },   /* 400~425RPM: 86% */
	{ 4250, 99 },   /* 425~450RPM: 87% */
	{ 4500, 99 },   /* 450~475RPM: 88% */
	{ 4750, 99 },   /* 475~500RPM: 89% */
	{ 5000, 99 },   /* 500~525RPM: 90% */
	{ 5250, 99 },   /* 525~550RPM: 91% */
	{ 5500, 99 },   /* 550~575RPM: 92% */
	{ 5750, 99 },   /* 575~600RPM: 92% */
	{ 6000, 0   },  /* 600RPM+: 用用户设定值(0=标记) */
};
#define SP_57_8412_AMP_TABLE_SIZE  (sizeof(SP_57_8412_AmpTable) / sizeof(SP_57_8412_AmpTable[0]))

/* 256点正弦表，值域 0~3199
 * 1600 = 50%占空比 = 零电流
 * 3199 = 100%占空比 = 正向满电流
 * 0    = 0%占空比   = 反向满电流
 *
 * 公式: 1600 + 1599 * sin(2π * i / 256)
 */
static const uint16_t SP_57_8412_SineTable256[256] = {
	1600,1639,1678,1718,1757,1796,1835,1873,1912,1950,1989,2026,2064,2102,2139,2175,
	2212,2248,2284,2319,2354,2388,2422,2455,2488,2521,2553,2584,2614,2644,2674,2703,
	2731,2758,2785,2811,2836,2861,2884,2907,2930,2951,2972,2991,3010,3028,3045,3062,
	3077,3092,3106,3118,3130,3141,3151,3160,3168,3175,3182,3187,3191,3195,3197,3199,
	3199,3199,3197,3195,3191,3187,3182,3175,3168,3160,3151,3141,3130,3118,3106,3092,
	3077,3062,3045,3028,3010,2991,2972,2951,2930,2907,2884,2861,2836,2811,2785,2758,
	2731,2703,2674,2644,2614,2584,2553,2521,2488,2455,2422,2388,2354,2319,2284,2248,
	2212,2175,2139,2102,2064,2026,1989,1950,1912,1873,1835,1796,1757,1718,1678,1639,
	1600,1561,1522,1482,1443,1404,1365,1327,1288,1250,1211,1174,1136,1098,1061,1025,
	988,952,916,881,846,812,778,745,712,679,647,616,586,556,526,497,
	469,442,415,389,364,339,316,293,270,249,228,209,190,172,155,138,
	123,108,94,82,70,59,49,40,32,25,18,13,9,5,3,1,
	1,1,3,5,9,13,18,25,32,40,49,59,70,82,94,108,
	123,138,155,172,190,209,228,249,270,293,316,339,364,389,415,442,
	469,497,526,556,586,616,647,679,712,745,778,812,846,881,916,952,
	988,1025,1061,1098,1136,1174,1211,1250,1288,1327,1365,1404,1443,1482,1522,1561
};

/*
 * 微步进状态（内部变量）
 */
volatile unsigned char StepperPulse57_8412_Enable    = 0;
static unsigned char  SP_57_8412_Amplitude           = 60;
static unsigned char  SP_57_8412_TargetAmplitude      = 60;  /* 用户设定的目标电流幅值 */
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

/* S形加速状态（由主循环 RampUpdate 更新，非中断） */
static unsigned char  SP_57_8412_RampActive          = 0;   /* 加速中标志 */
static unsigned int   SP_57_8412_RampTime            = 0;   /* 已用时间 ms */
static unsigned int   SP_57_8412_RampTotal           = 0;   /* 总加速时间 ms */
static unsigned int   SP_57_8412_RampStartRPM        = 0;   /* 起始 rpm_x10 */
static unsigned int   SP_57_8412_RampTargetRPM       = 0;   /* 目标 rpm_x10 */
static unsigned int   SP_57_8412_CurrentRPM          = 0;   /* 当前 rpm_x10 */

/*
 * 根据目标 RPM 计算 SpeedDivider 和 StepRate
 *
 * rpm_x10 为 ×10 定点转速（如 5=0.5RPM, 1100=110.0RPM）
 * RPM = 3375 × StepRate / (2048 × Divider)
 *   StepRate 范围 64~512（即 1.0~8.0 步）
 *   SpeedDivider 范围 1~255
 *   最大 RPM = 30000 × 512 / 2048 = 7500(×10) = 750RPM
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
		unsigned long idealDiv = (30000UL * i) / (2048UL * rpm_x10);
		unsigned long actualRPMx10, err;

		if (idealDiv < 1) idealDiv = 1;
		if (idealDiv > 255) idealDiv = 255;

		actualRPMx10 = (30000UL * i) / (2048UL * idealDiv);
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

/* 根据当前转速查表获取电流幅值
 * 遍历区间表，找到转速所在区间返回对应幅值
 * 末行幅值为0表示用用户设定值
 */
static unsigned char SP_57_8412_CalcAmplitude(unsigned int rpm_x10)
{
	unsigned char i;
	for (i = 0; i < SP_57_8412_AMP_TABLE_SIZE; i++)
	{
		if (rpm_x10 < SP_57_8412_AmpTable[i].rpm_x10)
		{
			/* 低于第一行下限，用第一行幅值 */
			return (i == 0) ? SP_57_8412_AmpTable[0].amplitude : SP_57_8412_AmpTable[i - 1].amplitude;
		}
		if (SP_57_8412_AmpTable[i].amplitude == 0)
		{
			/* 标记行：用用户设定值 */
			return SP_57_8412_TargetAmplitude;
		}
	}
	/* 超过所有区间，用用户设定值 */
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
	SP_57_8412_StepAccum = 0;
	SP_57_8412_CurrentRPM = rpm_x10;
	SP_57_8412_Amplitude = SP_57_8412_CalcAmplitude(rpm_x10);
}

/*
 * 初始化步进电机算法
 * 在 MX_TIM3_Init() 之后调用
 * 配置 PA6/PA7 为零电流，启动 TIM3 中断
 */
void StepperPulse57_8412_Init(void)
{
	/* 先复位板载 DRV8412（nRESET 低有效）：热启动时确保 H 桥内部状态清零 */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	{
		GPIO_InitTypeDef gpio = {0};
		gpio.Pin = GPIO_PIN_15;
		gpio.Mode = GPIO_MODE_OUTPUT_PP;
		gpio.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(GPIOA, &gpio);
	}
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);  /* 拉低=复位 DRV8412 */
	HAL_Delay(10);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);    /* 拉高=释放复位 */

	/* 先启动 PWM 通道输出 */
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

	/* 初始：零电流 */
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, SP_57_8412_PWM_CENTER);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, SP_57_8412_PWM_CENTER);

	/* 使能 TIM3 Update 中断并启动定时器 */
	__HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE);
	__HAL_TIM_ENABLE(&htim3);

	StepperPulse57_8412_Enable = 0;
	SP_57_8412_Mode            = SP_57_8412_MODE_IDLE;
}

/*
 * TIM3 中断回调 — 在 HAL_TIM_PeriodElapsedCallback 的 TIM3 分支中调用
 *
 * 每个 TIM3 周期触发一次，分频后更新 CCR1/CCR2，推进一步正弦相位。
 * 低速时也持续刷新 CCR，保持电流波形连续。
 * 注意：S形加速逻辑不在此处，由主循环 RampUpdate 更新 SpeedDivider/StepRate。
 */
void StepperPulse57_8412_Tick(void)
{
	uint16_t valA, valB, idx;

	if (!StepperPulse57_8412_Enable)
		return;

	/* 分频：每 SpeedDivider 次中断才推进一步 */
	if (++SP_57_8412_Divider < SP_57_8412_SpeedDivider)
	{
		/* 即使未推进一步，也刷新 CCR 保持电流连续 */
		idx = SP_57_8412_Index;
		valA = SP_57_8412_SineTable256[idx];
		valB = SP_57_8412_SineTable256[(idx + SP_57_8412_SINE_POINTS / 4) % SP_57_8412_SINE_POINTS];
		/* 按幅值缩放 */
		valA = SP_57_8412_PWM_CENTER + (uint16_t)((int16_t)(valA - SP_57_8412_PWM_CENTER) * SP_57_8412_Amplitude / 100);
		valB = SP_57_8412_PWM_CENTER + (uint16_t)((int16_t)(valB - SP_57_8412_PWM_CENTER) * SP_57_8412_Amplitude / 100);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, valA);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, valB);
		return;
	}
	SP_57_8412_Divider = 0;

	/* 分数步进：StepRate 为 ×64 定点值 */
	SP_57_8412_StepAccum += SP_57_8412_StepRate;
	while (SP_57_8412_StepAccum >= 64)
	{
		SP_57_8412_StepAccum -= 64;
		if (SP_57_8412_Direction >= 0)
			SP_57_8412_Index = (SP_57_8412_Index + 1) & 0xFF;
		else
			SP_57_8412_Index = (SP_57_8412_Index - 1) & 0xFF;
	}

	/* 绕组1 = sin(θ)，绕组2 = sin(θ+90°) = cos(θ) */
	idx = SP_57_8412_Index;
	valA = SP_57_8412_SineTable256[idx];
	valB = SP_57_8412_SineTable256[(idx + SP_57_8412_SINE_POINTS / 4) % SP_57_8412_SINE_POINTS];

	/* 按幅值百分比缩放 */
	valA = SP_57_8412_PWM_CENTER + (uint16_t)((int16_t)(valA - SP_57_8412_PWM_CENTER) * SP_57_8412_Amplitude / 100);
	valB = SP_57_8412_PWM_CENTER + (uint16_t)((int16_t)(valB - SP_57_8412_PWM_CENTER) * SP_57_8412_Amplitude / 100);

	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, valA);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, valB);

	/* 累计步数和绝对位置 */
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
		StepperPulse57_8412_Enable = 0;
		SP_57_8412_Mode            = SP_57_8412_MODE_IDLE;
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, SP_57_8412_PWM_CENTER);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, SP_57_8412_PWM_CENTER);
	}
}

/* 停止，零电流 */
void StepperPulse57_8412_Stop(void)
{
	SP_57_8412_RampActive   = 0;
	SP_57_8412_CurrentRPM   = 0;
	StepperPulse57_8412_Enable = 0;
	SP_57_8412_Mode            = SP_57_8412_MODE_IDLE;
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, SP_57_8412_PWM_CENTER);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, SP_57_8412_PWM_CENTER);
}

/* 运行中调整电流幅值 */
void StepperPulse57_8412_SetAmplitude(unsigned char amplitude)
{
	SP_57_8412_TargetAmplitude = amplitude;
	SP_57_8412_Amplitude = amplitude;
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

/* 获取当前转速（rpm_x10），调试用 */
unsigned int StepperPulse57_8412_GetCurrentRPM(void)
{
	return SP_57_8412_CurrentRPM;
}

/* 获取当前电流幅值百分比，调试用 */
unsigned char StepperPulse57_8412_GetAmplitude(void)
{
	return SP_57_8412_Amplitude;
}

/* 获取当前方向（1=正转，-1=反转），调试用 */
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
 * 持续运行（速度模式）
 *   rpm_x10      — 转速 ×10，正=正转，负=反转，0=停止
 *   amplitude    — 电流幅值 0~100
 *   accelTime_ms — 加速时间ms（>0时启用S形加速，0=立即启动）
 *   startRpm_x10 — 起步转速 ×10（S形加速起始速度，0=使用默认300RPM）
 */
void StepperPulse57_8412_RunAtSpeed(signed int rpm_x10, unsigned char amplitude, unsigned int accelTime_ms, unsigned int startRpm_x10)
{
	signed char direction;
	unsigned int startRPM;

	if (rpm_x10 == 0)
	{
		StepperPulse57_8412_Stop();
		return;
	}

	direction = (rpm_x10 > 0) ? 1 : -1;
	if (rpm_x10 < 0)
		rpm_x10 = -rpm_x10;

	/* 起步转速：用户指定 > 0 用用户值，否则用默认 */
	startRPM = (startRpm_x10 > 0) ? startRpm_x10 : SP_57_8412_START_RPM_X10;

	SP_57_8412_TargetAmplitude = amplitude;

	/* 设置了加速时间 → S形加速 */
	if (accelTime_ms > 0)
	{
		/* 如果电机未运行，从起步速度开始加速 */
		if (!StepperPulse57_8412_Enable || SP_57_8412_Direction != direction)
		{
			if ((unsigned int)rpm_x10 <= startRPM)
			{
				/* 目标速度 ≤ 起步速度，直接以目标速度启动 */
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
			/* 电机已在运行，从当前速度继续加速 */
			if ((unsigned int)rpm_x10 <= SP_57_8412_CurrentRPM)
			{
				/* 目标速度 ≤ 当前速度，直接设置 */
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
			StepperPulse57_8412_Enable = 1;
		}
	}
	else
	{
		/* 无加速时间 → 直接启动 */
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
			StepperPulse57_8412_Enable = 1;
		}
	}
}

/* 简化版持续运行（无加速，直接启动） */
void StepperPulse57_8412_Run(signed int rpm_x10, unsigned char amplitude)
{
	StepperPulse57_8412_RunAtSpeed(rpm_x10, amplitude, 0, 0);
}

/*
 * S曲线加速更新 — 在 task10ms 中调用
 * 每 10ms 更新一次当前 RPM
 *
 * S曲线公式（smoothstep）：s = 3p² - 2p³
 *   p = elapsed / total（0→1）
 *   Q16定点运算，64位中间变量（主循环中安全，非ISR）
 */
void StepperPulse57_8412_RampUpdate(void)
{
	unsigned int curRPM;

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

	/* S曲线（smoothstep）：s = 3p² - 2p³, Q16定点, 64位中间运算 */
	{
		uint64_t x = ((uint64_t)SP_57_8412_RampTime * 65536UL) / SP_57_8412_RampTotal;  /* 0~65536, Q16 */
		uint64_t x2 = x * x;                                        /* Q32 */
		uint64_t x3 = x2 * x;                                       /* Q48 */
		uint64_t s = (3UL * x2 - 2UL * x3 / 65536UL) / 65536UL;   /* 0~65536, Q16 */

		curRPM = SP_57_8412_RampStartRPM + (unsigned int)((uint64_t)(SP_57_8412_RampTargetRPM - SP_57_8412_RampStartRPM) * s / 65536UL);
	}

	SP_57_8412_CurrentRPM = curRPM;
	SP_57_8412_ApplyRPM(curRPM, SP_57_8412_Direction);
}

/*
 * 转到指定角度（位置模式）
 *   angle     — 目标角度，正=正转，负=反转
 *   rpm_x10   — 转速 ×10
 *   amplitude — 电流幅值 0~100
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
	SP_57_8412_Amplitude   = amplitude;
	SP_57_8412_ApplyRPM(rpm_x10, (diff > 0) ? 1 : -1);
	SP_57_8412_Index       = 0;
	SP_57_8412_Divider     = 0;
	SP_57_8412_StepAccum   = 0;
	SP_57_8412_StepCount   = 0;
	SP_57_8412_Mode        = SP_57_8412_MODE_STEPS;
	StepperPulse57_8412_Enable = 1;
}
