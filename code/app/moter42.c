#include "moter42.h"

#include "stdbool.h"

/* 双 42 步进电机驱动：支持受限位保护的抬升及同向/反向联动模式。 */
#include "enc.h"

/* 机械换算：1圈 = 256微步 × 50减速比 = 12800步（电机1视角，电机2自动反向） */
#define STEPS_PER_REV  12800

/* PWM 参数 */
#define SP_42_PWM_CENTER     1875   /* 50%占空比 = 零电流 */
#define SP_42_PWM_RANGE      3750   /* 0~3750 */
#define SP_42_SINE_POINTS    256

/* 运行模式 */
#define SP_42_MODE_IDLE     0
#define SP_42_MODE_STEPS    1   /* 走指定步数后停 */
#define SP_42_MODE_RUN      2   /* 持续运行 */

/* S形加速参数 */
#define SP_42_START_RPM_X10   3000    /* 启动速度 300RPM(×10)，从静止可可靠启动 */
#define SP_42_PWM_UPDATE_HZ   40000UL
#define SP_42_RAMP_PERIOD_MS  10UL
#define SP_42_RAMP_TICKS      (SP_42_PWM_UPDATE_HZ * SP_42_RAMP_PERIOD_MS / 1000UL)

/* 抬升电机的默认速度与电流幅值；如需更快/更慢可在此统一调整。 */
#define RISE_MOTER_RPM_X10     600
/* 抬升电流档（低/高）：
 *   低档 — 普通上/下行：上位机手动点动、开机自检（main.c：下行 8s → 上行到高位 → 下行到最低位）；
 *   高档 — 带载动作：压制期间的抬升（-1000/-601 段动作、归位下行、保压后回高位上行）。
 * 档位由保持寄存器 0x0004 设置（0=低档，非 0=高档），默认低档、粘滞保存；
 * 段动作一定带载，会强制切高档，直到显式改档或重新上电。 */
#define RISE_MOTER_LEFT_AMPLITUDE   15U
#define RISE_MOTER_RIGHT_AMPLITUDE  15U
#define RISE_MOTER_LEFT_AMPLITUDE_LOADED   20U
#define RISE_MOTER_RIGHT_AMPLITUDE_LOADED  20U
/* 兼容启动接口的单一幅值参数，启动后立即恢复左右独立幅值。 */
#define RISE_MOTER_AMPLITUDE       RISE_MOTER_LEFT_AMPLITUDE
#define RISE_MOTER_MAX_RPM_X10 6000U

/* 抬升编码器分辨率：1000 脉冲/圈 */
#define RISE_MOTER_ENC_PPR       1000U
/* 左右同步死区（单位：编码器脉冲）。抬升机构要求左右绝对脉冲严格一致：
 * 任一侧领先 1 脉冲即冻结该侧，仅让落后侧补步。 */
#define RISE_MOTER_SYNC_DEADBAND 20U
/* 下行到位容差：编码器可能因量化误差或机械堵转停在目标前 1~2 个脉冲，
 * 给予小容差使到位判定可靠触发，避免电机堵转在目标前一刻。 */
#define RISE_MOTER_TARGET_TOLERANCE  2U
/* TMR4 40 kHz：连续 3 秒内任一侧位移不超过 200 脉冲即停止。 */
#define RISE_MOTER_STALL_TICKS      120000UL
#define RISE_MOTER_STALL_PULSES     200U

/* 抬升"下降一段"固定段长度（编码器计数，RISE_MOTER_ENC_PPR = 1000 计数/圈）。
 * 由保持寄存器 0x0003 写 -1000（主站 MODBUS_RISE_MOTOR_TIMED_DOWN_HOLD）触发，
 * 段长度固定写在下位机，主站不传距离、不读状态；现场可用 NB+rise=seg,<计数> 试值。 */
#define RISE_MOTER_SEGMENT_COUNTS  14409U

/* 抬升"上行一段"固定段长度（编码器计数，RISE_MOTER_ENC_PPR = 1000 计数/圈）。
 * 压制流程不再抬到最高位：从最低位向上走本距离后停住等排水，由保持寄存器
 * 0x0003 写 -601（主站 MODBUS_RISE_MOTOR_TIMED_UP_HOLD）触发，段长写在下位机。
 * 28000 = 抬升左 +28000 / 抬升右 -28000，比全行程 riseMoterHighEncNum(31200)
 * 低 3200，天然停在到顶之前，不依赖高位限位开关（排水位 30301 → 29500 → 29000 → 28000）。
 * 上一条 RISE_MOTER_SEGMENT_COUNTS = 14409 是现场压制时抬升编码器读数
 * （从上行停点量到定型位），本流程照旧沿用；排水位相对 30301 已累计下调 2301 计数，
 * 定型位会跟着下降同样的计数，若要保持定型位不动，把它加到 16710（= 14409 + 2301）。 */
#define RISE_MOTER_UP_SEGMENT_COUNTS  28000U

/* 转速-电流幅值查表（与 moter57 保持一致）
 * 按转速区间(rpm_x10)分段，每段对应一个电流幅值
 * 电机转速落在 [rpm, 下一行rpm) 区间内时使用对应幅值
 * 最后一行为最高转速上限，超过则用用户设定值
 * 修改表项可自由插入新的转速电流区间
 */
typedef struct {
	unsigned int  rpm_x10;   /* 区间下限转速 ×10 */
	unsigned char amplitude;  /* 该区间电流幅值 */
} SP_42_AmpTableEntry;

/* 左抬升电机自动电流表（TMR4 CH1/CH2）。
 * 可按左侧电机的实际负载、温升单独修正此表。 */
static const SP_42_AmpTableEntry SP_42_LeftAmpTable[] = {
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

/* 右抬升电机自动电流表（TMR5 CH1/CH2）。
 * 初始值与左侧一致；右侧电机电流需要单独标定时，只改本表即可。 */
static const SP_42_AmpTableEntry SP_42_RightAmpTable[] = {
	{ 0,    20 }, { 250,  22 }, { 500,  25 }, { 750,  28 },
	{ 1000, 30 }, { 1250, 33 }, { 1500, 35 }, { 1750, 38 },
	{ 2000, 42 }, { 2250, 45 }, { 2375, 48 }, { 2500, 52 },
	{ 2625, 55 }, { 2750, 58 }, { 2875, 62 }, { 3000, 67 },
	{ 3125, 78 }, { 3250, 79 }, { 3375, 80 }, { 3500, 99 },
	{ 3750, 99 }, { 4000, 99 }, { 4250, 99 }, { 4500, 99 },
	{ 4750, 99 }, { 5000, 99 }, { 5250, 99 }, { 5500, 99 },
	{ 5750, 99 }, { 6000, 0 }
};
#define SP_42_LEFT_AMP_TABLE_SIZE  \
	(sizeof(SP_42_LeftAmpTable) / sizeof(SP_42_LeftAmpTable[0]))
#define SP_42_RIGHT_AMP_TABLE_SIZE \
	(sizeof(SP_42_RightAmpTable) / sizeof(SP_42_RightAmpTable[0]))

/* 256点正弦表，值域 0~3750（与 moter57 保持一致）
 * 1875 = 50%占空比 = 零电流
 * 3750 = 100%占空比 = 正向满电流
 * 0    = 0%占空比   = 反向满电流
 *
 * 公式: 1875 + 1875 * sin(2π * i / 256)
 */
static const uint16_t SP_42_SineTable256[256] = {
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
 * 微步进状态（内部变量）
 * SP_42_Index 为双电机共用电相位。第二台电机的绕组接线反向，
 * 因此使用相同电相位时，两台的实际机械旋转方向相反。
 */
volatile unsigned char StepperPulse42_Enable    = 0;
/* 两台 42 电机各自的电流幅值：左侧=TMR4(CH1/CH2)，右侧=TMR5(CH1/CH2)。 */
static unsigned char  SP_42_LeftAmplitude       = 60;
static unsigned char  SP_42_RightAmplitude      = 60;
static unsigned char  SP_42_TargetAmplitude     = 60;  /* 左侧目标幅值 */
static unsigned char  SP_42_TargetRightAmplitude = 60; /* 右侧目标幅值 */
static unsigned short SP_42_Index               = 0;
/* 第二台电机必须拥有独立电相位，才能在编码器不同步时单独补步。 */
static unsigned short SP_42_Motor2Index         = 0;
static unsigned char  SP_42_Motor2PhaseReverse  = 0;
static unsigned char  SP_42_Divider             = 0;
static unsigned int   SP_42_StepCount           = 0;
static unsigned int   SP_42_TargetSteps         = 0;
static signed char    SP_42_Direction           = 1;
static unsigned char  SP_42_Mode                = SP_42_MODE_IDLE;
static signed long    SP_42_Position            = 0;   /* 电机1位置，电机2 = -Position */
static unsigned char  SP_42_SpeedDivider        = 6;
static unsigned short SP_42_StepRate            = 64;
static unsigned short SP_42_StepAccum           = 0;

/* S形加减速状态（由 TMR4 中断每 10ms 更新）。 */
static volatile unsigned char  SP_42_RampActive          = 0;
static volatile unsigned int   SP_42_RampTime            = 0;
static volatile unsigned int   SP_42_RampTotal           = 0;
static volatile unsigned int   SP_42_RampStartRPM        = 0;
static volatile unsigned int   SP_42_RampTargetRPM       = 0;
static volatile unsigned int   SP_42_CurrentRPM          = 0;
static unsigned short          SP_42_RampTickCounter     = 0;

/* Modbus 抬升高/低位状态：由限位或下行目标脉冲到达时更新。 */
 volatile bool riseMoterLowPosition=false;		//抬升低位标志位
 volatile bool riseMoterHighPosition=false;		//抬升高位标志位

static volatile bool riseMoterRunning = false;	//抬升电机是否运行标志位,与42电机左区分，一个是被位置控制，一个不受位置控制
static volatile int8_t riseMoterDirection = 1;	//抬升电机运行方向标志（1=上行, -1=下行）
/* 本轮上行到位锁存，避免限位抖动使已到位的一侧再次运动。 */
static volatile bool riseMoterLeftHighReached = false;
static volatile bool riseMoterRightHighReached = false;
/* 抬升双电机机械方向模式：0=两台反向（默认），1=两台同向。 */
/* 默认两台电机机械方向相反。 */
static volatile uint8_t riseMoterSameDirection = 0U;
/* 抬升专用左右电流幅值；上行和下行均沿用该配置。 */
static volatile uint8_t riseMoterLeftAmplitude = RISE_MOTER_LEFT_AMPLITUDE;
static volatile uint8_t riseMoterRightAmplitude = RISE_MOTER_RIGHT_AMPLITUDE;
/* 当前抬升电流档：0=低档（RISE_MOTER_*_AMPLITUDE），1=高档（*_LOADED）。 */
static volatile uint8_t riseMoterLoadGear = 0U;
/* 当前动作开始时的编码器基准；同步和低位判断均使用本次动作增量。 */
static volatile int32_t riseMoterLeftStartPosition;
static volatile int32_t riseMoterRightStartPosition;
static volatile uint32_t riseMoterStallTicks;
static volatile int32_t riseMoterStallLeftPosition;
static volatile int32_t riseMoterStallRightPosition;

volatile const uint32_t riseMoterHighEncNum = 31200;//两边同时旋转的脉冲数


/* 抬升"下降一段"：从当前停点（高位或"上行一段"停点）向下走固定编码器距离后停止。
 * 距离单位为编码器计数（RISE_MOTER_ENC_PPR = 1000 计数/圈）。
 * 语义参照上一代固件的 riseMoterDownFor()：
 *   - 只能从高位启动，否则拒绝并置故障位；
 *   - RS485 重传不重启动作（执行中再调用直接返回成功）；
 *   - 途中触及低位判定为故障，说明段长度不小于全行程。
 * 段结束时为零电流停止，位置靠丝杆机械自锁保持。 */
static volatile uint8_t  riseMoterSegActive = 0U;	/* 1=正在执行"下降一段" */
static volatile uint8_t  riseMoterSegFault  = 0U;	/* 1=上一次"下降一段"失败 */
static volatile uint32_t riseMoterSegCounts = 0U;	/* 本段长度（编码器计数） */


/* 返回本次动作以来的绝对位移，兼容编码器正负方向和计数递减。 */
static uint32_t riseMoterTravel(int32_t current, int32_t start)
{
	int64_t delta = (int64_t)current - (int64_t)start;
	return (uint32_t)(delta < 0 ? -delta : delta);
}

/* 当前下行目标（编码器计数）："下降一段"期间为段长度，否则为全行程。 */
static uint32_t riseMoterMotionTarget(void)
{
	if(riseMoterSegActive != 0U && riseMoterSegCounts != 0U)
	{
		return riseMoterSegCounts;
	}
	return riseMoterHighEncNum;
}

/* "上行一段"/"下降一段"是否已走够：左右两侧都必须达到段长度，不使用容差。 */
static uint8_t riseMoterSegmentReached(void)
{
	int32_t left = encRiseMoterLEncPositionGet();
	int32_t right = encRiseMoterREncPositionGet();

	return (riseMoterTravel(left, riseMoterLeftStartPosition) >= riseMoterSegCounts &&
	        riseMoterTravel(right, riseMoterRightStartPosition) >= riseMoterSegCounts) ? 1U : 0U;
}

//检测电机编码器的脉冲数是否达到指定脉冲
static uint8_t riseMoterTargetReached(void)
{
	int32_t left = encRiseMoterLEncPositionGet();
	int32_t right = encRiseMoterREncPositionGet();
	uint32_t leftAbs = riseMoterTravel(left, riseMoterLeftStartPosition);
	uint32_t rightAbs = riseMoterTravel(right, riseMoterRightStartPosition);
	uint32_t threshold = (riseMoterHighEncNum > RISE_MOTER_TARGET_TOLERANCE) ?
	                     (riseMoterHighEncNum - RISE_MOTER_TARGET_TOLERANCE) : 0U;

	/* 两侧编码器都达到（目标-容差）才允许判定为下行到位。
	 * 容差用于应对编码器量化误差或电机堵转在目标前 1~2 脉冲的情况。 */
	return (leftAbs >= threshold && rightAbs >= threshold) ? 1U : 0U;
}

/*
 * 决定本个微步周期应推进哪一台 42 电机。
 * 左侧对应 TMR4（SP_42_Index），右侧对应 TMR5（SP_42_Motor2Index）。
 * 两边差值不超过 RISE_MOTER_SYNC_DEADBAND 时同步推进；否则停止领先侧的相位，
 * 只让落后侧补步，直到两边重新同步。下行时也不会让先到目标的一侧超程。
 */
static void riseMoterStepMask(uint8_t *stepLeft, uint8_t *stepRight)
{
	int32_t left = encRiseMoterLEncPositionGet();
	int32_t right = encRiseMoterREncPositionGet();
	uint32_t leftAbs = riseMoterTravel(left, riseMoterLeftStartPosition);
	uint32_t rightAbs = riseMoterTravel(right, riseMoterRightStartPosition);
	uint32_t target;

	*stepLeft = 1U;
	*stepRight = 1U;

	if(riseMoterRunning == false)
	{
		return;
	}

	/* 一侧高位到达后冻结该侧相位，保持励磁；另一侧独立寻找高位。
	 * 此时跳过编码器同步，否则未到位侧可能因位移领先而无法继续。 */
	if(riseMoterDirection > 0 &&
	   (riseMoterLeftHighReached || riseMoterRightHighReached))
	{
		*stepLeft = riseMoterLeftHighReached ? 0U : 1U;
		*stepRight = riseMoterRightHighReached ? 0U : 1U;
		return;
	}

	/* 动作目标：段动作（上行段/下降段）用段长度，普通下降用全行程，都不允许超程。 */
	target = riseMoterMotionTarget();
	if(riseMoterDirection < 0 && target != 0U)
	{
		if(leftAbs >= target && rightAbs < target)
		{
			*stepLeft = 0U;
		}
		else if(rightAbs >= target && leftAbs < target)
		{
			*stepRight = 0U;
		}
	}

	if(leftAbs > rightAbs + RISE_MOTER_SYNC_DEADBAND)
	{
		*stepLeft = 0U;
	}
	else if(rightAbs > leftAbs + RISE_MOTER_SYNC_DEADBAND)
	{
		*stepRight = 0U;
	}
}

//位置检测
static void risePosDec(void)
{
	bool leftHigh = (gpio_input_data_bit_read(riseMoterLPosL_GPIO_PORT,
	                                        riseMoterLPosL_PIN) == RESET);
	bool rightHigh = (gpio_input_data_bit_read(riseMoterRPosL_GPIO_PORT,
	                                         riseMoterRPosL_PIN) == RESET);
	/* 低位状态需要锁存：下行到位后编码器会清零，不能在这里再次清零低位标志。 */
	riseMoterHighPosition = false;	

	if(riseMoterRunning != false && riseMoterDirection > 0)
	{
		if(leftHigh) riseMoterLeftHighReached = true;
		if(rightHigh) riseMoterRightHighReached = true;
		/* 两侧先后到位也算完成，已到位侧不会因开关释放而重新启动。 */
		riseMoterHighPosition = riseMoterLeftHighReached && riseMoterRightHighReached;
	}
	else if(leftHigh && rightHigh)
	{
		riseMoterHighPosition = true;//抬升高位置1
	}
	/* 只有正在下行时才用编码器目标判断低位。 */
	if(riseMoterRunning != false && riseMoterDirection < 0 &&
	   riseMoterTargetReached() != 0U)
	{
		riseMoterLowPosition = true;//抬升低位置1
	}
}

//抬升机构是否到位检测
static void riseMoterMonitor(void)
{
	risePosDec();//单独检测位置
	if(riseMoterRunning == false)
	{
		return;
	}
	/* 上行只允许高位限位停止。高位开关是机械零点，到位后清零两侧
	 * 编码器，供后续下行按编码器脉冲计程。 */
	if(riseMoterDirection > 0 && riseMoterHighPosition == true)
	{
		StepperPulse42_Stop();
		riseMoterRunning = false;
		encRiseMoterLEncPositionReset();
		encRiseMoterLEncBPulseReset();
		encRiseMoterREncPositionReset();
		encRiseMoterREncBPulseReset();
		return;
	}
	
	/* "上行一段"：走够段长度即正常结束（零电流停，位置靠丝杆自锁保持）。
	 * 紧跟"高位停"之后判断：本流程按段长停，不依赖高位限位。 */
	if(riseMoterDirection > 0 && riseMoterSegActive != 0U &&
	   riseMoterSegmentReached() != 0U)
	{
		riseMoterSegActive = 0U;
		StepperPulse42_Stop();
		riseMoterRunning = false;
		return;
	}

	/* "下降一段"：走够段长度即正常结束。零电流停止，位置靠丝杆自锁保持。 */
	if(riseMoterDirection < 0 && riseMoterSegActive != 0U &&
	   riseMoterSegmentReached() != 0U)
	{
		riseMoterSegActive = 0U;
		StepperPulse42_Stop();
		riseMoterRunning = false;
		return;
	}

	/* 下行只允许编码器目标停止。riseMoterTargetReached() 已要求左右
	 * 两侧的绝对脉冲数都达到（riseMoterHighEncNum - 容差）；其中一侧
	 * 先到时，riseMoterStepMask() 会冻结该侧、仅让落后侧补步。 */
	if(riseMoterDirection < 0 && riseMoterLowPosition == true)
	{
		/* 段下降途中触到低位：段长度不小于全行程，按故障处理。 */
		if(riseMoterSegActive != 0U)
		{
			riseMoterSegActive = 0U;
			riseMoterSegFault  = 1U;
		}
		StepperPulse42_Stop();
		riseMoterRunning = false;
		/* 先保留低位标志，再清零编码器，防止堵塞空跑影响下一次上行。 */
		encRiseMoterLEncPositionReset();
		encRiseMoterLEncBPulseReset();
		encRiseMoterREncPositionReset();
		encRiseMoterREncBPulseReset();
		return;
	}
	/* 按完整的 3 秒窗口检查两侧实际位移，重复下行命令不重置计时。
	 * 保护停机不清零编码器；低位锁存直到新一轮上行。 */
	if(riseMoterDirection < 0 && ++riseMoterStallTicks >= RISE_MOTER_STALL_TICKS)
	{
		int32_t left = encRiseMoterLEncPositionGet();
		int32_t right = encRiseMoterREncPositionGet();
		if(riseMoterTravel(left, riseMoterStallLeftPosition) <= RISE_MOTER_STALL_PULSES ||
		   riseMoterTravel(right, riseMoterStallRightPosition) <= RISE_MOTER_STALL_PULSES)
		{
			riseMoterLowPosition = true;
			StepperPulse42_Stop();
			riseMoterRunning = false;
			return;
		}
		riseMoterStallTicks = 0U;
		riseMoterStallLeftPosition = left;
		riseMoterStallRightPosition = right;
	}
}

/*
 * 根据目标 RPM 计算 SpeedDivider 和 StepRate（与 moter57 相同）
 *
 * rpm_x10 为 ×10 定点转速（如 5=0.5RPM, 1100=110.0RPM）
 * RPM = 60000 × StepRate / (2048 × Divider)
 *   StepRate 范围 64~512（即 1.0~8.0 步）
 *   SpeedDivider 范围 1~255
 *   最大 RPM = 60000 × 512 / 2048 = 15000(×10) = 1500RPM
 */
static void SP_42_CalcRPM(unsigned int rpm_x10, unsigned char *divider, unsigned short *stepRate)
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

	/* Divider = 60000 × StepRate / (2048 × rpm_x10)
	 * 实际RPM×10 = 60000 × StepRate / (2048 × Divider) */
	for (i = 64; i <= 512; i++)
	{
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

/* 根据当前转速查表获取电流幅值（与 moter57 相同）
 * 遍历区间表，找到转速所在区间返回对应幅值
 * 末行幅值为0表示用用户设定值
 */
static unsigned char SP_42_CalcAmplitude(const SP_42_AmpTableEntry *table,
									 unsigned char tableSize,
									 unsigned int rpm_x10,
									 unsigned char fallbackAmplitude)
{
	unsigned char i;
	for (i = 0; i < tableSize; i++)
	{
		if (rpm_x10 < table[i].rpm_x10)
		{
			/* 低于第一行下限，用第一行幅值 */
			return (i == 0) ? table[0].amplitude : table[i - 1U].amplitude;
		}
		if (table[i].amplitude == 0U)
		{
			/* 标记行：用用户设定值 */
			return fallbackAmplitude;
		}
	}
	/* 超过所有区间，用用户设定值 */
	return fallbackAmplitude;
}

static void SP_42_ApplyRPM(unsigned int rpm_x10, signed char direction)
{
	unsigned char div;
	unsigned short sr;
	SP_42_Direction = direction;
	SP_42_CalcRPM(rpm_x10, &div, &sr);
	SP_42_SpeedDivider = div;
	SP_42_StepRate = sr;
	SP_42_StepAccum = 0;
	SP_42_CurrentRPM = rpm_x10;
	/* 0..100 means an explicit current request. Values above 100 retain the
	 * optional legacy automatic current table for compatibility/testing. */
	if (SP_42_TargetAmplitude <= 100U)
		SP_42_LeftAmplitude = SP_42_TargetAmplitude;
	else
		SP_42_LeftAmplitude = SP_42_CalcAmplitude(SP_42_LeftAmpTable,
				(unsigned char)SP_42_LEFT_AMP_TABLE_SIZE, rpm_x10,
				SP_42_TargetAmplitude);
	if (SP_42_TargetRightAmplitude <= 100U)
		SP_42_RightAmplitude = SP_42_TargetRightAmplitude;
	else
		SP_42_RightAmplitude = SP_42_CalcAmplitude(SP_42_RightAmpTable,
				(unsigned char)SP_42_RIGHT_AMP_TABLE_SIZE, rpm_x10,
				SP_42_TargetRightAmplitude);
}

/*
 * 双电机 PWM 输出 —— 同一个函数同时驱动两个电机
 *   电机1: TMR4 CH1/CH2，相位 = idx1
 *   电机2: TMR5 CH1/CH2，相位 = -idx1（与电机1方向相反）
 * 两相电流相差 90°（相位偏移 64 个采样点 = 256/4）
 */
static void SP_42_OutputPWM(unsigned short idx1, unsigned short idx2Phase)
{
	unsigned short idx2;
	uint16_t valA, valB;

	/* 电机1：绕组A = sin(θ)，绕组B = sin(θ+90°) */
	valA = SP_42_SineTable256[idx1];
	valB = SP_42_SineTable256[(idx1 + SP_42_SINE_POINTS / 4) % SP_42_SINE_POINTS];
	/* 按幅值缩放 */
	valA = SP_42_PWM_CENTER + (uint16_t)((int16_t)(valA - SP_42_PWM_CENTER) * SP_42_LeftAmplitude / 100);
	valB = SP_42_PWM_CENTER + (uint16_t)((int16_t)(valB - SP_42_PWM_CENTER) * SP_42_LeftAmplitude / 100);
	tmr_channel_value_set(TMR4, TMR_SELECT_CHANNEL_1, valA);
	tmr_channel_value_set(TMR4, TMR_SELECT_CHANNEL_2, valB);

	/* 电机2 绕组与电机1反接：同相输出才是相反的机械转向。 */
	idx2 = (SP_42_Motor2PhaseReverse != 0U) ?
	       (unsigned short)((0U - idx2Phase) & 0xFFU) : idx2Phase;
	valA = SP_42_SineTable256[idx2];
	valB = SP_42_SineTable256[(idx2 + SP_42_SINE_POINTS / 4) % SP_42_SINE_POINTS];
	valA = SP_42_PWM_CENTER + (uint16_t)((int16_t)(valA - SP_42_PWM_CENTER) * SP_42_RightAmplitude / 100);
	valB = SP_42_PWM_CENTER + (uint16_t)((int16_t)(valB - SP_42_PWM_CENTER) * SP_42_RightAmplitude / 100);
	tmr_channel_value_set(TMR5, TMR_SELECT_CHANNEL_1, valA);
	tmr_channel_value_set(TMR5, TMR_SELECT_CHANNEL_2, valB);
}

/* 双电机输出零电流 */
static void SP_42_OutputIdle(void)
{
	tmr_channel_value_set(TMR4, TMR_SELECT_CHANNEL_1, SP_42_PWM_CENTER);
	tmr_channel_value_set(TMR4, TMR_SELECT_CHANNEL_2, SP_42_PWM_CENTER);
	tmr_channel_value_set(TMR5, TMR_SELECT_CHANNEL_1, SP_42_PWM_CENTER);
	tmr_channel_value_set(TMR5, TMR_SELECT_CHANNEL_2, SP_42_PWM_CENTER);
}

/*
 * 初始化双电机步进算法
 * 在 wk_tmr4_init()/wk_tmr5_init() 之后调用
 * 配置双电机为零电流
 */
void StepperPulse42_Init(void)
{
	/* 先启动双电机 PWM 通道输出 */
	tmr_channel_enable(TMR4, TMR_SELECT_CHANNEL_1, TRUE);
	tmr_channel_enable(TMR4, TMR_SELECT_CHANNEL_2, TRUE);
	tmr_channel_enable(TMR5, TMR_SELECT_CHANNEL_1, TRUE);
	tmr_channel_enable(TMR5, TMR_SELECT_CHANNEL_2, TRUE);

	/* 初始：零电流 */
	SP_42_OutputIdle();

	StepperPulse42_Enable = 0;
	SP_42_Mode            = SP_42_MODE_IDLE;
}

/*
 * 40kHz 中断回调 — 在 TMR4_GLOBAL_IRQHandler 的 OVF 分支中调用
 *
 * 单一函数同时推进双电机：电机1（TMR4 CH1/CH2）与电机2（TMR5 CH1/CH2）
 * 共用同一相位累加器 SP_42_Index；根据硬件绕组接线，两台会机械反向，
 * 无需分别调用。
 * 低速时也持续刷新 CCR，保持电流波形连续。
 * S形加减速由本函数内的 10ms 节拍自动更新。
 */
void StepperPulse42_Tick(void)
{
	uint8_t stepLeft;
	uint8_t stepRight;

	riseMoterMonitor();

	if (!StepperPulse42_Enable)
		return;

	/* TMR4 runs at 40 kHz. Update the S curve at 10 ms intervals. */
	if (++SP_42_RampTickCounter >= SP_42_RAMP_TICKS)
	{
		SP_42_RampTickCounter = 0;
		StepperPulse42_RampUpdate();
	}

	/* 分频：每 SpeedDivider 次中断才推进一步 */
	if (++SP_42_Divider < SP_42_SpeedDivider)
	{
		/* 即使未推进一步，也刷新 CCR 保持电流连续 */
		SP_42_OutputPWM(SP_42_Index, SP_42_Motor2Index);
		return;
	}
	SP_42_Divider = 0;

	/* 分数步进：StepRate 为 ×64 定点值 */
	SP_42_StepAccum += SP_42_StepRate;
	while (SP_42_StepAccum >= 64)
	{
		SP_42_StepAccum -= 64;
		riseMoterStepMask(&stepLeft, &stepRight);
		if (SP_42_Direction >= 0)
		{
			if(stepLeft != 0U) SP_42_Index = (SP_42_Index + 1U) & 0xFFU;
			if(stepRight != 0U) SP_42_Motor2Index = (SP_42_Motor2Index + 1U) & 0xFFU;
		}
		else
		{
			if(stepLeft != 0U) SP_42_Index = (SP_42_Index - 1U) & 0xFFU;
			if(stepRight != 0U) SP_42_Motor2Index = (SP_42_Motor2Index - 1U) & 0xFFU;
		}

		/* SP_42_Position/StepCount are defined from left motor's perspective. */
		if(stepLeft != 0U)
		{
			SP_42_StepCount++;
			if (SP_42_Direction >= 0) SP_42_Position++;
			else SP_42_Position--;
		}
	}

	/* 双电机输出（硬件绕组使电机2机械反向） */
	SP_42_OutputPWM(SP_42_Index, SP_42_Motor2Index);

	/* 步数模式：走到目标步数则自动停止 */
	if (SP_42_Mode == SP_42_MODE_STEPS &&
		SP_42_TargetSteps != 0 &&
		SP_42_StepCount >= SP_42_TargetSteps)
	{
		StepperPulse42_Enable = 0;
		SP_42_Mode            = SP_42_MODE_IDLE;
		SP_42_OutputIdle();
	}
}

/* 停止双电机，零电流 */
void StepperPulse42_Stop(void)
{
	SP_42_RampActive      = 0;
	SP_42_CurrentRPM      = 0;
	SP_42_RampTickCounter = 0;
	StepperPulse42_Enable = 0;
	SP_42_Mode            = SP_42_MODE_IDLE;
	SP_42_OutputIdle();
}


/* 抬升机构带保护运行：方向由接口决定，速度为正数 rpm×10。
 * 速度为 0 时等效停止；上限限制为 3000，防止上位机误设超速。 */
static uint16_t riseMoterSpeedLimit(uint16_t rpm_x10)
{
	return (rpm_x10 > RISE_MOTER_MAX_RPM_X10) ? RISE_MOTER_MAX_RPM_X10 : rpm_x10;
}

/* 按当前抬升方向模式启动双 42 电机。两种模式均使用同一套抬升
 * 限位、编码器同步和目标脉冲保护；仅电机2的相位方向不同。 */
static void riseMoterRun(int16_t signedRpm_x10)
{
	if(riseMoterSameDirection != 0U)
	{
		StepperPulse42_RunSameDirection(signedRpm_x10, RISE_MOTER_AMPLITUDE);
	}
	else
	{
		StepperPulse42_Run(signedRpm_x10, RISE_MOTER_AMPLITUDE);
	}
	/* 通用启动接口会写入默认幅值，随后恢复抬升专用的左右电流设定。 */
	StepperPulse42_SetAmplitudeLeftRight(riseMoterLeftAmplitude, riseMoterRightAmplitude);
}

void riseMoterSetSameDirection(uint8_t sameDirection)
{
	/* 仅影响下一次 riseMoterUp/Down；运行中切换不重置相位，避免突变。 */
	riseMoterSameDirection = (sameDirection != 0U) ? 1U : 0U;
}

uint8_t riseMoterGetSameDirection(void)
{
	return riseMoterSameDirection;
}

void riseMoterSetAmplitudeLeftRight(uint8_t leftAmplitude, uint8_t rightAmplitude)
{
	if(leftAmplitude > 100U) leftAmplitude = 100U;
	if(rightAmplitude > 100U) rightAmplitude = 100U;
	riseMoterLeftAmplitude = leftAmplitude;
	riseMoterRightAmplitude = rightAmplitude;
	/* 运行中立即生效；不会改变速度、方向和限位保护。 */
	if(riseMoterRunning != false)
	{
		StepperPulse42_SetAmplitudeLeftRight(leftAmplitude, rightAmplitude);
	}
}

void riseMoterSetAmplitude(uint8_t amplitude)
{
	riseMoterSetAmplitudeLeftRight(amplitude, amplitude);
}

uint8_t riseMoterGetLeftAmplitude(void)
{
	return riseMoterLeftAmplitude;
}

uint8_t riseMoterGetRightAmplitude(void)
{
	return riseMoterRightAmplitude;
}

/* 按当前档位套用左右电流幅值；运行中调用立即生效。 */
static void riseMoterApplyLoadGear(void)
{
	if(riseMoterLoadGear != 0U)
	{
		riseMoterSetAmplitudeLeftRight(RISE_MOTER_LEFT_AMPLITUDE_LOADED,
		                               RISE_MOTER_RIGHT_AMPLITUDE_LOADED);
	}
	else
	{
		riseMoterSetAmplitudeLeftRight(RISE_MOTER_LEFT_AMPLITUDE,
		                               RISE_MOTER_RIGHT_AMPLITUDE);
	}
}

/* 切换抬升电流档：0=低档，非 0=高档。立即生效，不改变速度/方向/限位保护。 */
void riseMoterSetLoadGear(uint8_t gear)
{
	if(gear != 0U)
	{
		riseMoterLoadGear = 1U;
	}
	else
	{
		riseMoterLoadGear = 0U;
	}
	riseMoterApplyLoadGear();
}

uint8_t riseMoterGetLoadGear(void)
{
	return riseMoterLoadGear;
}

void riseMoterUpAtSpeed(uint16_t rpm_x10)
{
	uint32_t irqMask;
	/* 按当前档位起步：普通上/下行默认低档，压制流程已切高档时保持高档。 */
	riseMoterApplyLoadGear();
	rpm_x10 = riseMoterSpeedLimit(rpm_x10);
	if(rpm_x10 == 0U)
	{
		riseMoterStop();
		return;
	}
	/* 状态与驱动启动必须原子更新，防止中断停止后又被启动函数开启。 */
	irqMask = __get_PRIMASK();
	__disable_irq();
	if(riseMoterRunning != false && riseMoterDirection > 0)
	{
		riseMoterRun((int16_t)rpm_x10);
		__set_PRIMASK(irqMask);
		return;
	}
	/* 新一轮上行开始，清除上一轮下行到位状态。 */
	riseMoterLowPosition = false;
	riseMoterLeftHighReached = false;
	riseMoterRightHighReached = false;
	riseMoterSegActive = 0U;
	riseMoterLeftStartPosition = encRiseMoterLEncPositionGet();
	riseMoterRightStartPosition = encRiseMoterREncPositionGet();
	riseMoterDirection = 1;
	riseMoterRunning = true;
	riseMoterRun((int16_t)rpm_x10);
	__set_PRIMASK(irqMask);
}

void riseMoterDownAtSpeed(uint16_t rpm_x10)
{
	uint32_t irqMask;
	/* 按当前档位起步：普通上/下行默认低档，压制流程已切高档时保持高档。 */
	riseMoterApplyLoadGear();
	rpm_x10 = riseMoterSpeedLimit(rpm_x10);
	if(rpm_x10 == 0U)
	{
		riseMoterStop();
		return;
	}
	irqMask = __get_PRIMASK();
	__disable_irq();
	/* 已锁存低位时拒绝再次下行；与中断中的到位停止互斥。 */
	if(riseMoterLowPosition == true)
	{
		riseMoterStop();
		__set_PRIMASK(irqMask);
		return;
	}
	/* 重复下行命令只更新速度，不重新建立计程起点。 */
	if(riseMoterRunning != false && riseMoterDirection < 0)
	{
		riseMoterRun(-(int16_t)rpm_x10);
		__set_PRIMASK(irqMask);
		return;
	}
	/* 普通下行不是段下降，清除可能残留的段状态。 */
	riseMoterSegActive = 0U;
	riseMoterLeftStartPosition = encRiseMoterLEncPositionGet();
	riseMoterRightStartPosition = encRiseMoterREncPositionGet();
	riseMoterDirection = -1;
	riseMoterStallTicks = 0U;
	riseMoterStallLeftPosition = riseMoterLeftStartPosition;
	riseMoterStallRightPosition = riseMoterRightStartPosition;
	riseMoterRunning = true;
	riseMoterRun(-(int16_t)rpm_x10);
	__set_PRIMASK(irqMask);
}

//抬升电机上升：默认速度
void riseMoterUp(void)
{
	riseMoterUpAtSpeed(RISE_MOTER_RPM_X10);
}

//抬升电机下降：默认速度
void riseMoterDown(void)
{
	riseMoterDownAtSpeed(RISE_MOTER_RPM_X10);
}

//抬升电机停止
void riseMoterStop(void)
{
	uint32_t irqMask = __get_PRIMASK();
	__disable_irq();
	StepperPulse42_Stop();
	riseMoterRunning = false;
	riseMoterSegActive = 0U;
	__set_PRIMASK(irqMask);
}



uint8_t riseMoterLowPositionGet(void)
{
	return riseMoterLowPosition;
}

uint8_t riseMoterHighPositionGet(void)
{
	return riseMoterHighPosition;
}

/*
 * 抬升"下降一段"：从当前停点向下走 counts 个编码器脉冲后停止。
 *   counts — 段长度（编码器计数，1000 计数/圈），必须小于全行程 riseMoterHighEncNum
 * 返回 1 = 动作已启动，或已在执行中（重传幂等）；
 *      0 = 被拒绝：已在最低位、长度为 0、或不小于全行程，此时故障位为 1。
 * 段结束时零电流停止，位置靠丝杆机械自锁保持。
 */
uint8_t riseMoterDownSegment(uint32_t counts)
{
	/* 幂等：已在执行中直接返回成功，避免 RS485 重传重启动作。 */
	if(riseMoterSegActive != 0U)
	{
		return 1U;
	}

	/* 前提：不能在最低位（低位标志为 1 时拒绝），且长度合法；
	 * 起算点为本段动作启动时的编码器位置。 */
	if(counts == 0U || riseMoterLowPosition == true ||
	   counts >= riseMoterHighEncNum)
	{
		riseMoterSegFault = 1U;
		return 0U;
	}

	riseMoterSegFault    = 0U;
	riseMoterSegCounts   = counts;
	riseMoterLowPosition = false;
	/* 段动作一定带载：先切高档，之后压制流程里的抬升动作也保持高档。 */
	riseMoterSetLoadGear(1U);
	riseMoterDownAtSpeed(RISE_MOTER_RPM_X10);
	/* 启动后再置位：riseMoterDownAtSpeed() 会清除本标志。 */
	riseMoterSegActive = 1U;

	return 1U;
}

/*
 * 抬升"下降一段"默认入口：段长度固定写在下位机（RISE_MOTER_SEGMENT_COUNTS），
 * 供保持寄存器 0x0003 写 -1000 时调用。语义与 riseMoterDownSegment() 完全一致。
 */
uint8_t riseMoterDownSegmentDefault(void)
{
	return riseMoterDownSegment(RISE_MOTER_SEGMENT_COUNTS);
}

/*
 * 抬升"上行一段"：从最低位向上走 counts 个编码器脉冲后停住，供压制流程
 * 排水定位使用（不再抬到最高位，不依赖高位限位开关）。
 *   counts — 段长度（编码器计数，1000 计数/圈），必须小于全行程 riseMoterHighEncNum
 * 返回 1 = 动作已启动，或已在执行中（重传幂等）；
 *      0 = 被拒绝：不在最低位、长度为 0、或不小于全行程，此时故障位为 1。
 * 段结束时零电流停止，位置靠丝杆机械自锁保持。
 */
uint8_t riseMoterUpSegment(uint32_t counts)
{
	/* 幂等：已在执行中直接返回成功，避免 RS485 重传重启动作。 */
	if(riseMoterSegActive != 0U)
	{
		return 1U;
	}

	/* 前提：必须停在最低位（整段位移从最低位起算），且长度合法。 */
	if(counts == 0U || riseMoterLowPosition == false ||
	   counts >= riseMoterHighEncNum)
	{
		riseMoterSegFault = 1U;
		return 0U;
	}

	riseMoterSegFault  = 0U;
	riseMoterSegCounts = counts;
	/* 段动作一定带载：先切高档，之后压制流程里的抬升动作也保持高档。 */
	riseMoterSetLoadGear(1U);
	riseMoterUpAtSpeed(RISE_MOTER_RPM_X10);
	/* 启动后再置位：riseMoterUpAtSpeed() 会清除本标志。 */
	riseMoterSegActive = 1U;

	return 1U;
}

/*
 * 抬升"上行一段"默认入口：段长度固定写在下位机（RISE_MOTER_UP_SEGMENT_COUNTS），
 * 供保持寄存器 0x0003 写 -601 时调用。语义与 riseMoterUpSegment() 完全一致。
 */
uint8_t riseMoterUpSegmentDefault(void)
{
	return riseMoterUpSegment(RISE_MOTER_UP_SEGMENT_COUNTS);
}

uint8_t riseMoterSegmentActiveGet(void)
{
	return riseMoterSegActive;
}

uint8_t riseMoterSegmentFaultGet(void)
{
	return riseMoterSegFault;
}

uint32_t riseMoterSegmentCountsGet(void)
{
	return riseMoterSegCounts;
}

/* 兼容旧接口：一次设置两侧相同的电流幅值。 */
void StepperPulse42_SetAmplitude(unsigned char amplitude)
{
	SP_42_TargetAmplitude = amplitude;
	SP_42_TargetRightAmplitude = amplitude;
	SP_42_LeftAmplitude = amplitude;
	SP_42_RightAmplitude = amplitude;
}

/* 独立设置两侧 42 电机的电流幅值（0~100%）。
 * 左侧为 TMR4 CH1/CH2，右侧为 TMR5 CH1/CH2。 */
void StepperPulse42_SetAmplitudeLeftRight(unsigned char leftAmplitude,
                                          unsigned char rightAmplitude)
{
	SP_42_TargetAmplitude = leftAmplitude;
	SP_42_TargetRightAmplitude = rightAmplitude;
	SP_42_LeftAmplitude = leftAmplitude;
	SP_42_RightAmplitude = rightAmplitude;
}

/* 获取电机1绝对位置（微步；电机2位置 = -返回值） */
signed long StepperPulse42_GetPosition(void)
{
	return SP_42_Position;
}

/* 获取电机1当前累积角度（电机2角度 = -返回值） */
signed int StepperPulse42_GetCurrentAngle(void)
{
	return (signed int)(SP_42_Position * 360 / STEPS_PER_REV);
}

/* 获取当前转速（rpm_x10，电机1视角），调试用 */
unsigned int StepperPulse42_GetCurrentRPM(void)
{
	return SP_42_CurrentRPM;
}

/* 获取当前电流幅值百分比，调试用 */
unsigned char StepperPulse42_GetAmplitude(void)
{
	return SP_42_LeftAmplitude;
}

unsigned char StepperPulse42_GetLeftAmplitude(void)
{
	return SP_42_LeftAmplitude;
}

unsigned char StepperPulse42_GetRightAmplitude(void)
{
	return SP_42_RightAmplitude;
}

/* 获取电机1当前方向（1=正转，-1=反转；电机2方向相反），调试用 */
signed char StepperPulse42_GetDirection(void)
{
	return SP_42_Direction;
}

/* 获取S曲线加速进行中标志，调试用 */
unsigned char StepperPulse42_IsRamping(void)
{
	return SP_42_RampActive;
}

/*
 * 持续运行（速度模式）— 单一接口同时控制双电机
 *   rpm_x10      — 转速 ×10，正=电机1正转(电机2反转)，负=电机1反转(电机2正转)，0=停止
 *                  方向由速度符号自动辨识，无需单独输入方向
 *   amplitude    — 电流幅值 0~100
 *   accelTime_ms — 加速时间ms（>0时启用S形加速，0=立即启动）
 *                  S曲线公式: s = 3x² - 2x³，加速度两端缓、中间快
 *                  由 TMR4 中断每 10ms 自动更新，无需在主循环调用。
 *   startRpm_x10 — 起步转速 ×10（S形加速起始速度，0=使用默认300RPM）
 */
static void SP_42_RunAtSpeed(signed int rpm_x10, unsigned char amplitude, unsigned int accelTime_ms, unsigned int startRpm_x10)
{
	signed char direction;
	unsigned int startRPM;

	if (rpm_x10 == 0)
	{
		StepperPulse42_Stop();
		return;
	}

	direction = (rpm_x10 > 0) ? 1 : -1;
	if (rpm_x10 < 0)
		rpm_x10 = -rpm_x10;

	/* 起步转速：用户指定 > 0 用用户值，否则用默认 */
	startRPM = (startRpm_x10 > 0) ? startRpm_x10 : SP_42_START_RPM_X10;

	/* 原有 Run 接口语义：启动时左右两侧使用同一幅值。独立设置请在
	 * 启动后调用 StepperPulse42_SetAmplitudeLeftRight()。 */
	SP_42_TargetAmplitude = amplitude;
	SP_42_TargetRightAmplitude = amplitude;

	/* 设置了加速时间 → S形加速 */
	if (accelTime_ms > 0)
	{
		/* 如果电机未运行，从起步速度开始加速 */
		if (!StepperPulse42_Enable || SP_42_Direction != direction)
		{
			if ((unsigned int)rpm_x10 <= startRPM)
			{
				/* 目标速度 ≤ 起步速度，直接以目标速度启动 */
				SP_42_RampStartRPM  = (unsigned int)rpm_x10;
				SP_42_CurrentRPM    = (unsigned int)rpm_x10;
				SP_42_RampActive    = 0;
			}
			else
			{
				SP_42_RampStartRPM  = startRPM;
				SP_42_CurrentRPM    = startRPM;
				SP_42_RampActive    = 1;
			}
		}
		else
		{
			/* 电机已在运行，从当前速度继续加速 */
			if ((unsigned int)rpm_x10 == SP_42_CurrentRPM)
			{
				/* 目标速度等于当前速度，无需更新曲线。 */
				SP_42_RampActive = 0;
			}
			else
			{
				SP_42_RampStartRPM = SP_42_CurrentRPM;
				SP_42_RampActive   = 1;
			}
		}

		SP_42_RampTargetRPM = (unsigned int)rpm_x10;
		SP_42_RampTotal     = accelTime_ms;
		SP_42_RampTime      = 0;
		SP_42_RampTickCounter = 0;

		/* 设置初始速度 */
		SP_42_ApplyRPM(SP_42_CurrentRPM, direction);

		SP_42_Mode        = SP_42_MODE_RUN;
		SP_42_TargetSteps = 0;

		if (!StepperPulse42_Enable)
		{
			SP_42_Index     = 0;
			SP_42_Motor2Index = 0;
			SP_42_Divider   = 0;
			SP_42_StepAccum = 0;
			SP_42_StepCount = 0;
			StepperPulse42_Enable = 1;
		}
	}
	else
	{
		/* 无加速时间 → 直接启动 */
		SP_42_RampActive = 0;
		SP_42_ApplyRPM((unsigned int)rpm_x10, direction);
		SP_42_Mode        = SP_42_MODE_RUN;
		SP_42_TargetSteps = 0;

		if (!StepperPulse42_Enable)
		{
			SP_42_Index     = 0;
			SP_42_Motor2Index = 0;
			SP_42_Divider   = 0;
			SP_42_StepAccum = 0;
			SP_42_StepCount = 0;
			StepperPulse42_Enable = 1;
		}
	}
}

/* 简化版持续运行（无加速，直接启动） */
void StepperPulse42_RunAtSpeed(signed int rpm_x10, unsigned char amplitude, unsigned int accelTime_ms, unsigned int startRpm_x10)
{
	SP_42_Motor2PhaseReverse = 0U;
	SP_42_RunAtSpeed(rpm_x10, amplitude, accelTime_ms, startRpm_x10);
}

void StepperPulse42_RunAtSpeedSameDirection(signed int rpm_x10, unsigned char amplitude, unsigned int accelTime_ms, unsigned int startRpm_x10)
{
	SP_42_Motor2PhaseReverse = 1U;
	SP_42_RunAtSpeed(rpm_x10, amplitude, accelTime_ms, startRpm_x10);
}

void StepperPulse42_Run(signed int rpm_x10, unsigned char amplitude)
{
	StepperPulse42_RunAtSpeed(rpm_x10, amplitude, 5000, 300);
}

void StepperPulse42_RunSameDirection(signed int rpm_x10, unsigned char amplitude)
{
	StepperPulse42_RunAtSpeedSameDirection(rpm_x10, amplitude, 0, 0);
}

/*
 * S曲线加速更新 — 由 StepperPulse42_Tick 每 10ms 调用
 *
 * S曲线公式（smoothstep）：s = 3p² - 2p³
 *   p = elapsed / total（0→1）
 *   Q16定点运算，64位中间变量
 */
void StepperPulse42_RampUpdate(void)
{
	unsigned int curRPM;
	signed long rpmDelta;

	if (!SP_42_RampActive)
		return;

	SP_42_RampTime += 10;
	if (SP_42_RampTime >= SP_42_RampTotal)
	{
		/* 加速完成，到达目标速度 */
		SP_42_ApplyRPM(SP_42_RampTargetRPM, SP_42_Direction);
		SP_42_CurrentRPM = SP_42_RampTargetRPM;
		SP_42_RampActive = 0;
		return;
	}

	/* S曲线（smoothstep）：s = 3p² - 2p³, Q16定点, 64位中间运算 */
	{
		uint64_t x = ((uint64_t)SP_42_RampTime * 65536UL) / SP_42_RampTotal;   /* 0~65536, Q16 */
		uint64_t x2 = x * x;                                        /* Q32 */
		uint64_t x3 = x2 * x;                                       /* Q48 */
		uint64_t s = (3UL * x2 - 2UL * x3 / 65536UL) / 65536UL;   /* 0~65536, Q16 */

		rpmDelta = (signed long)SP_42_RampTargetRPM - (signed long)SP_42_RampStartRPM;
		curRPM = (unsigned int)((signed long)SP_42_RampStartRPM + rpmDelta * (signed long)s / 65536L);
	}

	SP_42_CurrentRPM = curRPM;
	SP_42_ApplyRPM(curRPM, SP_42_Direction);
}

/*
 * 转到指定角度（位置模式，电机1视角，电机2自动反向，走完自动停）
 *   angle     — 目标角度（电机1视角），正=电机1正转(电机2反转)，支持多圈
 *   rpm_x10   — 转速 ×10
 *   amplitude — 电流幅值 0~100
 */
void StepperPulse42_MoveToAngle(signed int angle, unsigned int rpm_x10, unsigned char amplitude)
{
	signed long targetPos;
	long diff;

	if (rpm_x10 == 0)
		return;

	targetPos = (signed long)angle * STEPS_PER_REV / 360;
	diff = targetPos - SP_42_Position;

	if (diff == 0)
		return;

	SP_42_TargetSteps    = (unsigned int)(diff > 0 ? diff : -diff);
	SP_42_TargetAmplitude = amplitude;
	SP_42_TargetRightAmplitude = amplitude;
	SP_42_LeftAmplitude   = amplitude;
	SP_42_RightAmplitude  = amplitude;
	SP_42_ApplyRPM(rpm_x10, (diff > 0) ? 1 : -1);
	SP_42_Index       = 0;
	SP_42_Motor2Index = 0;
	SP_42_Divider     = 0;
	SP_42_StepAccum   = 0;
	SP_42_StepCount   = 0;
	SP_42_Mode        = SP_42_MODE_STEPS;
	StepperPulse42_Enable = 1;
}
