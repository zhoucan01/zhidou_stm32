#include "StepperPulse57_8701.h"

/*
 * 4路步进电机微步进控制算法（DRV8701E PH/EN 驱动）
 *
 * 适配 zhidou-f103c8t6 工程：
 *   系统时钟: HSI/2×PLL16 = 64MHz，APB1定时器时钟=64MHz
 *   TIM2: Prescaler=0, Period=1599, UP mode, 40000 Hz
 *   正弦表 0~3199，中心值 1600
 *
 * DRV8701E EN=PWM, PH=方向 模式：
 *   PA0(PH1) = GPIO 方向    = sign(sin(θ))
 *   PA1(EN1, CH2) = PWM电流 = |sin(θ)| × amp
 *   PA2(PH2) = GPIO 方向    = sign(cos(θ))
 *   PA3(EN3, CH4) = PWM电流 = |cos(θ)| × amp
 *   PA4(nSLEEP) = GPIO 高   = 唤醒 DRV8701
 *
 * 命名规范：
 *   外部函数: StepperPulse57_8701_XXX (在 .h 中声明)
 *   内部函数/变量: SP_57_8701_XXX (static, 仅本文件可见)
 */

/* 机械换算 */
#define SP_57_8701_STEPS_MICRO  12800   /* 微步：200×64 */
#define SP_57_8701_STEPS_HALF   400     /* 半步：8×50 */
#define SP_57_8701_MORPH_RPM    99999   /* 禁用自动切换，全程微步 */
#define SP_57_8701_START_RPM    2000    /* 启动速度 200RPM (×10) */

/* PWM 参数 */
#define SP_57_8701_PWM_CENTER     800    /* 50%占空比 = 零电流 */
#define SP_57_8701_PWM_RANGE      1600   /* 0~1599 */
#define SP_57_8701_SINE_POINTS    256

/* 运行模式 */
#define SP_57_8701_MODE_IDLE     0
#define SP_57_8701_MODE_STEPS    1   /* 走指定步数后停 */
#define SP_57_8701_MODE_RUN      2   /* 持续运行 */

/* 半步相位表(8步)：bit3=PH1, bit2=EN1, bit1=PH2, bit0=EN2
 * 与正弦波方向一致，交替单/双相通电减少振动
 */
static const uint8_t SP_57_8701_StepPhase8[8] = {
	0b1111, 0b1100, 0b1010, 0b0001,
	0b0101, 0b0110, 0b0111, 0b1011
};

/* 256点正弦表，值域 0~3199
 * 公式: 1600 + 1599 * sin(2π * i / 256)
 */
static const uint16_t SP_57_8701_SineTable256[256] = {
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

/* 运行状态 */
volatile unsigned char StepperPulse57_8701_Enable      = 0;
static unsigned char  SP_57_8701_Amplitude    = 60;
static unsigned short SP_57_8701_Index        = 0;
static unsigned short SP_57_8701_Divider      = 0;
static unsigned int   SP_57_8701_StepCount    = 0;
static unsigned int   SP_57_8701_TargetSteps  = 0;
static signed char    SP_57_8701_Direction    = 1;
static unsigned char  SP_57_8701_Mode         = SP_57_8701_MODE_IDLE;
static signed long    SP_57_8701_Position     = 0;
static unsigned short SP_57_8701_SpeedDivider = 6;
static unsigned short SP_57_8701_StepRate     = 64;
static unsigned short SP_57_8701_StepAccum    = 0;
static unsigned char  SP_57_8701_HalfStepMode = 0;  /* 0=微步 1=半步 */
static unsigned char  SP_57_8701_StepPhase    = 0;  /* 半步相位 0-7 */
static unsigned int   SP_57_8701_StepsPerRev  = SP_57_8701_STEPS_MICRO;
static unsigned char  SP_57_8701_PhaseAdvance = 0;  /* 相位超前 0-64 (0°-90°) */

/* S曲线加速参数 */
static unsigned char  SP_57_8701_RampActive   = 0;   /* 加速中标志 */
static unsigned int   SP_57_8701_RampTime     = 0;   /* 已用时间 ms */
static unsigned int   SP_57_8701_RampTotal    = 0;   /* 总加速时间 ms */
static unsigned int   SP_57_8701_RampStartRPM = 0;   /* 起始 rpm_x10 */
static unsigned int   SP_57_8701_RampTargetRPM= 0;   /* 目标 rpm_x10 */
static unsigned int   SP_57_8701_CurrentRPM   = 0;   /* 当前 rpm_x10 */

/*
 * 根据目标 RPM 计算 SpeedDivider 和 StepRate
 *
 * 通用公式（适用于任意 STEPS_PER_REV）：
 *   RPM = (TIM频率 / Divider) × (StepRate/64) / STEPS_PER_REV × 60
 *   TIM频率 = 40000Hz
 *   RPM×10 = (375000 × StepRate) / (StepsPerRev × Divider)
 *
 *   idealDiv = (375000 × StepRate) / (StepsPerRev × rpm_x10)
 */
static void SP_57_8701_CalcRPM(unsigned int rpm_x10, unsigned short *divider, unsigned short *stepRate)
{
	unsigned short bestSR = 64;
	unsigned short bestDiv = 65535;
	unsigned long bestErr = 0xFFFFFFFF;
	unsigned short i;

	if (rpm_x10 == 0)
	{
		*divider = 65535;
		*stepRate = 64;
		return;
	}

	for (i = 64; i <= 256; i++)
	{
		unsigned long idealDiv = (375000UL * i) / (SP_57_8701_StepsPerRev * rpm_x10);
		unsigned long actualRPMx10, err;
		unsigned long divClamped = idealDiv;
		if (divClamped < 1) divClamped = 1;
		if (divClamped > 65535) divClamped = 65535;

		actualRPMx10 = (375000UL * i) / (SP_57_8701_StepsPerRev * divClamped);
		err = (actualRPMx10 > rpm_x10) ? (actualRPMx10 - rpm_x10) : (rpm_x10 - actualRPMx10);

		if (err < bestErr)
		{
			bestErr = err;
			bestSR = i;
			bestDiv = (unsigned short)divClamped;
		}
	}

	*divider = bestDiv;
	*stepRate = bestSR;
}

static void SP_57_8701_ApplyRPM(unsigned int rpm_x10, signed char direction)
{
	unsigned short div;
	unsigned short sr;
	SP_57_8701_Direction = direction;
	SP_57_8701_CalcRPM(rpm_x10, &div, &sr);
	SP_57_8701_SpeedDivider = div;
	SP_57_8701_StepRate = sr;
	SP_57_8701_StepAccum = 0;
}

/* 初始化步进电机算法（在 MX_TIM2_Init 之后调用）
 * DRV8701E EN=PWM, PH=方向 模式：
 *   PA0(PH1) = GPIO 方向输出（正/反）
 *   PA1(EN1, TIM2_CH2) = PWM 电流控制 |sin(θ)|
 *   PA2(PH2) = GPIO 方向输出（正/反）
 *   PA3(EN2, TIM2_CH4) = PWM 电流控制 |cos(θ)|
 *   PA4(nSLEEP) = GPIO 拉高唤醒 DRV8701
 */
void StepperPulse57_8701_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* PA4 改为 GPIO 输出并拉高（DRV8701 nSLEEP 唤醒） */
	GPIO_InitStruct.Pin = GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);  /* nSLEEP = 高，退出睡眠 */

	/* 启动 EN1(CH2) 和 EN2(CH4) 的 PWM 输出 */
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

	/* 初始：零电流（EN PWM = 0） */
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);

	/* PH1(PA0) 和 PH2(PA2) 初始低电平（正方向），由 gpio.c 已配置 */
	/* 使能 TIM2 Update 中断并启动定时器 */
	__HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE);
	__HAL_TIM_ENABLE(&htim2);

	StepperPulse57_8701_Enable = 0;
	SP_57_8701_Mode             = SP_57_8701_MODE_IDLE;
}

/*
 * TIM2 中断回调 — 在 HAL_TIM_PeriodElapsedCallback 的 TIM2 分支中调用
 *
 * DRV8701E EN=PWM, PH=方向 模式微步进：
 *   EN1(PA1, CH2) = |sin(θ)| × amp/100  → PWM 占空比控制电流大小
 *   PH1(PA0)       = sign(sin(θ))        → GPIO 控制电流方向
 *   EN2(PA3, CH4) = |cos(θ)| × amp/100
 *   PH2(PA2)       = sign(cos(θ))
 *
 *   sin > 0 → PH=高(正转), EN=PWM(|sin|)
 *   sin < 0 → PH=低(反转), EN=PWM(|sin|)
 *   sin = 0 → EN=0(零电流), 过零切换方向
 */
void StepperPulse57_8701_Tick(void)
{
	int16_t diffA, diffB;
	uint16_t enA, enB, idx;
	uint8_t phase;

	if (!StepperPulse57_8701_Enable)
		return;

	/* 分频：每 SpeedDivider 次中断才推进一步 */
	if (++SP_57_8701_Divider < SP_57_8701_SpeedDivider)
	{
		/* 微步模式：未推进一步也刷新 CCR 保持电流连续 */
		if (!SP_57_8701_HalfStepMode)
		{
			idx = (SP_57_8701_Index + SP_57_8701_PhaseAdvance) & 0xFF;
			diffA = (int16_t)SP_57_8701_SineTable256[idx] - 1600;
			diffB = (int16_t)SP_57_8701_SineTable256[(idx + SP_57_8701_SINE_POINTS / 4) % SP_57_8701_SINE_POINTS] - 1600;
			enA = (uint16_t)((diffA >= 0 ? diffA : -diffA) * SP_57_8701_Amplitude / 100);
			enB = (uint16_t)((diffB >= 0 ? diffB : -diffB) * SP_57_8701_Amplitude / 100);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, enA);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, enB);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, diffA >= 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, diffB >= 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
		}
		/* 半步模式：PWM 固定，无需刷新 */
		return;
	}
	SP_57_8701_Divider = 0;

	/* 分数步进 */
	SP_57_8701_StepAccum += SP_57_8701_StepRate;
	while (SP_57_8701_StepAccum >= 64)
	{
		SP_57_8701_StepAccum -= 64;
		if (SP_57_8701_HalfStepMode)
		{
			if (SP_57_8701_Direction >= 0)
				SP_57_8701_StepPhase = (SP_57_8701_StepPhase + 1) & 7;
			else
				SP_57_8701_StepPhase = (SP_57_8701_StepPhase - 1) & 7;
		}
		else
		{
			if (SP_57_8701_Direction >= 0)
				SP_57_8701_Index = (SP_57_8701_Index + 1) & 0xFF;
			else
				SP_57_8701_Index = (SP_57_8701_Index - 1) & 0xFF;
		}
	}

	/* 输出 */
	if (SP_57_8701_HalfStepMode)
	{
		/* 半步：8步循环，单/双相交替，独立控制每相 EN */
		phase = SP_57_8701_StepPhase8[SP_57_8701_StepPhase];
		enA = (uint16_t)(SP_57_8701_Amplitude * 1599 / 100);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, (phase & 0b0100) ? enA : 0);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, (phase & 0b0001) ? enA : 0);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, (phase & 0b1000) ? GPIO_PIN_SET : GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, (phase & 0b0010) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	}
	else
	{
		/* 微步：正弦波（加相位超前） */
		idx = (SP_57_8701_Index + SP_57_8701_PhaseAdvance) & 0xFF;
		diffA = (int16_t)SP_57_8701_SineTable256[idx] - 1600;
		diffB = (int16_t)SP_57_8701_SineTable256[(idx + SP_57_8701_SINE_POINTS / 4) % SP_57_8701_SINE_POINTS] - 1600;
		enA = (uint16_t)((diffA >= 0 ? diffA : -diffA) * SP_57_8701_Amplitude / 100);
		enB = (uint16_t)((diffB >= 0 ? diffB : -diffB) * SP_57_8701_Amplitude / 100);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, enA);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, enB);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, diffA >= 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, diffB >= 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
	}

	/* 累计步数和绝对位置 */
	SP_57_8701_StepCount++;
	if (SP_57_8701_Direction >= 0)
		SP_57_8701_Position++;
	else
		SP_57_8701_Position--;

	/* 步数模式：走到目标步数则自动停止 */
	if (SP_57_8701_Mode == SP_57_8701_MODE_STEPS &&
		SP_57_8701_TargetSteps != 0 &&
		SP_57_8701_StepCount >= SP_57_8701_TargetSteps)
	{
		StepperPulse57_8701_Enable = 0;
		SP_57_8701_Mode             = SP_57_8701_MODE_IDLE;
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
	}
}

/* 停止，零电流 */
void StepperPulse57_8701_Stop(void)
{
	StepperPulse57_8701_Enable = 0;
	SP_57_8701_Mode             = SP_57_8701_MODE_IDLE;
	SP_57_8701_RampActive       = 0;
	SP_57_8701_HalfStepMode     = 0;
	SP_57_8701_StepsPerRev      = SP_57_8701_STEPS_MICRO;
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
}

/* 电机自锁（保持当前位置，持续通电产生保持力矩）
 *   holdDuty — 保持电流占空比 0~100（如 30 = 30%电流）
 * 停止步进推进，根据当前微步位置计算两相保持电流
 * PH 保持方向电平，EN 输出固定 PWM，防止外力推动
 */
void StepperPulse57_8701_Lock(unsigned char holdDuty)
{
	uint16_t enA, enB, idx;
	int16_t diffA, diffB;

	/* 停止步进推进 */
	StepperPulse57_8701_Enable = 0;
	SP_57_8701_Mode             = SP_57_8701_MODE_IDLE;
	SP_57_8701_RampActive       = 0;

	/* 用当前 Index 位置计算两相保持电流 */
	idx = (SP_57_8701_Index + SP_57_8701_PhaseAdvance) & 0xFF;
	diffA = (int16_t)SP_57_8701_SineTable256[idx] - 1600;
	diffB = (int16_t)SP_57_8701_SineTable256[(idx + SP_57_8701_SINE_POINTS / 4) % SP_57_8701_SINE_POINTS] - 1600;

	enA = (uint16_t)((diffA >= 0 ? diffA : -diffA) * holdDuty / 100);
	enB = (uint16_t)((diffB >= 0 ? diffB : -diffB) * holdDuty / 100);

	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, enA);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, enB);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, diffA >= 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, diffB >= 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* 释放自锁，零电流 */
void StepperPulse57_8701_Unlock(void)
{
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
}

/* 运行中调整电流幅值 */
void StepperPulse57_8701_SetAmplitude(unsigned char amplitude)
{
	SP_57_8701_Amplitude = amplitude;
}

/* 获取绝对位置（微步） */
signed long StepperPulse57_8701_GetPosition(void)
{
	return SP_57_8701_Position;
}

/* 获取当前转速 ×10 */
unsigned int StepperPulse57_8701_GetCurrentRPM(void)
{
	return SP_57_8701_CurrentRPM;
}

/* 持续运行（速度模式） */
void StepperPulse57_8701_RunAtSpeed(signed int rpm_x10, unsigned char amplitude)
{
	signed char direction;

	if (rpm_x10 == 0)
	{
		StepperPulse57_8701_Stop();
		return;
	}

	direction = (rpm_x10 > 0) ? 1 : -1;
	if (rpm_x10 < 0)
		rpm_x10 = -rpm_x10;

	SP_57_8701_Amplitude    = amplitude;
	SP_57_8701_ApplyRPM((unsigned int)rpm_x10, direction);
	SP_57_8701_Mode          = SP_57_8701_MODE_RUN;
	SP_57_8701_TargetSteps   = 0;

	if (!StepperPulse57_8701_Enable)
	{
		SP_57_8701_Index     = 0;
		SP_57_8701_Divider   = 0;
		SP_57_8701_StepAccum = 0;
		SP_57_8701_StepCount = 0;
		StepperPulse57_8701_Enable = 1;
	}
}

/* 转到指定角度（位置模式） */
void StepperPulse57_8701_MoveToAngle(signed int angle, unsigned int rpm_x10, unsigned char amplitude)
{
	signed long targetPos;
	long diff;

	if (rpm_x10 == 0)
		return;

	targetPos = (signed long)angle * SP_57_8701_StepsPerRev / 360;
	diff = targetPos - SP_57_8701_Position;

	if (diff == 0)
		return;

	SP_57_8701_TargetSteps = (unsigned int)(diff > 0 ? diff : -diff);
	SP_57_8701_Amplitude   = amplitude;
	SP_57_8701_ApplyRPM(rpm_x10, (diff > 0) ? 1 : -1);
	SP_57_8701_Index       = 0;
	SP_57_8701_Divider     = 0;
	SP_57_8701_StepAccum   = 0;
	SP_57_8701_StepCount   = 0;
	SP_57_8701_Mode        = SP_57_8701_MODE_STEPS;
	StepperPulse57_8701_Enable = 1;
}

/*
 * S曲线加速启动
 *   rpm_x10   — 目标转速 ×10，正=正转，负=反转
 *   amplitude — 电流幅值 0~100
 *   accel_ms  — 加速时间（毫秒），如 1000 = 1秒加速到目标
 *
 * S曲线公式（smoothstep）：
 *   progress = t / T（0→1）
 *   s = 3p² - 2p³
 *   当前RPM = 起始RPM + (目标RPM - 起始RPM) × s
 *
 *   特点：起始和结束加速度为0，中间最大，无冲击
 */
void StepperPulse57_8701_RunAtSpeedS(signed int rpm_x10, unsigned char amplitude, unsigned int accel_ms)
{
	signed char direction;

	if (rpm_x10 == 0)
	{
		StepperPulse57_8701_Stop();
		return;
	}

	direction = (rpm_x10 > 0) ? 1 : -1;
	if (rpm_x10 < 0)
		rpm_x10 = -rpm_x10;

	SP_57_8701_Amplitude      = amplitude;
	SP_57_8701_RampTargetRPM  = (unsigned int)rpm_x10;
	SP_57_8701_RampTotal       = accel_ms;
	SP_57_8701_RampTime        = 0;
	SP_57_8701_RampActive      = 1;

	/* 如果电机未运行，从低速开始加速 */
	if (!StepperPulse57_8701_Enable)
	{
		if (SP_57_8701_RampTargetRPM <= SP_57_8701_START_RPM)
		{
			/* 目标速度 ≤ 启动速度，直接以目标速度启动 */
			SP_57_8701_RampStartRPM = SP_57_8701_RampTargetRPM;
			SP_57_8701_CurrentRPM   = SP_57_8701_RampTargetRPM;
			SP_57_8701_RampActive   = 0;
		}
		else
		{
			SP_57_8701_RampStartRPM = SP_57_8701_START_RPM;
			SP_57_8701_CurrentRPM   = SP_57_8701_START_RPM;
		}
		SP_57_8701_Index        = 0;
		SP_57_8701_Divider      = 0;
		SP_57_8701_StepAccum    = 0;
		SP_57_8701_StepCount    = 0;
		SP_57_8701_Mode         = SP_57_8701_MODE_RUN;
		SP_57_8701_TargetSteps  = 0;
		SP_57_8701_ApplyRPM(SP_57_8701_CurrentRPM, direction);
		StepperPulse57_8701_Enable = 1;
	}
	else
	{
		/* 电机已在运行，从当前速度继续加速 */
		if (SP_57_8701_RampTargetRPM <= SP_57_8701_CurrentRPM)
		{
			/* 目标速度 ≤ 当前速度，直接设置目标速度 */
			SP_57_8701_ApplyRPM(SP_57_8701_RampTargetRPM, direction);
			SP_57_8701_CurrentRPM  = SP_57_8701_RampTargetRPM;
			SP_57_8701_RampActive  = 0;
		}
		else
		{
			SP_57_8701_RampStartRPM = SP_57_8701_CurrentRPM;
		}
		SP_57_8701_Mode = SP_57_8701_MODE_RUN;
	}
}

/*
 * S曲线加速更新 — 在 task10ms 中调用
 * 每 10ms 更新一次当前 RPM
 */
void StepperPulse57_8701_RampUpdate(void)
{
	if (!SP_57_8701_RampActive)
		return;

	SP_57_8701_RampTime += 10;
	if (SP_57_8701_RampTime >= SP_57_8701_RampTotal)
	{
		/* 加速完成，到达目标速度 */
		SP_57_8701_ApplyRPM(SP_57_8701_RampTargetRPM, SP_57_8701_Direction);
		SP_57_8701_CurrentRPM  = SP_57_8701_RampTargetRPM;
		SP_57_8701_RampActive  = 0;
		return;
	}

	/* S曲线（smoothstep）：s = 3p² - 2p³ */
	{
		float p = (float)SP_57_8701_RampTime / (float)SP_57_8701_RampTotal;
		float s = 3.0f * p * p - 2.0f * p * p * p;
		unsigned int curRPM = SP_57_8701_RampStartRPM +
			(unsigned int)((float)(SP_57_8701_RampTargetRPM - SP_57_8701_RampStartRPM) * s);
		SP_57_8701_CurrentRPM = curRPM;

		/* 相位超前：400~750RPM 线性增加 0→64 (0°~90°)
		 * 补偿高速反电动势导致的电流滞后 */
		if (curRPM > 4000)
		{
			unsigned int diff = curRPM - 4000;
			SP_57_8701_PhaseAdvance = (diff > 3500) ? 64 :
				(unsigned char)(diff * 64 / 3500);
		}
		else
			SP_57_8701_PhaseAdvance = 0;

		/* 模式切换 */
		if (curRPM >= SP_57_8701_MORPH_RPM && !SP_57_8701_HalfStepMode)
		{
			/* 微步→半步：从正弦索引推算半步相位（256/8=32） */
			SP_57_8701_StepPhase = (SP_57_8701_Index / 32) & 7;
			SP_57_8701_HalfStepMode = 1;
			SP_57_8701_StepsPerRev = SP_57_8701_STEPS_HALF;
		}
		else if (curRPM < SP_57_8701_MORPH_RPM && SP_57_8701_HalfStepMode)
		{
			/* 半步→微步：从半步相位推算正弦索引 */
			SP_57_8701_Index = SP_57_8701_StepPhase * 32;
			SP_57_8701_HalfStepMode = 0;
			SP_57_8701_StepsPerRev = SP_57_8701_STEPS_MICRO;
		}

		SP_57_8701_ApplyRPM(curRPM, SP_57_8701_Direction);
	}
}
