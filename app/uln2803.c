#include "uln2803.h"

#include "main.h"
#include "tim.h"
#include "stdbool.h"
#include "tmux1308.h"

#include "servo.h"

const float lfvServoRunTime = 1.0f;

extern TIM_HandleTypeDef htim2;
extern int moter28Dir;
static uint16_t sinTable[64]={
    0,80,159,239,319,398,477,556,
    634,712,789,866,943,1019,1094,1169,
    1243,1316,1388,1459,1530,1599,1668,1736,
    1802,1867,1932,1995,2056,2117,2176,2234,
    2290,2345,2399,2451,2501,2550,2597,2643,
    2687,2730,2770,2809,2847,2882,2916,2948,
    2978,3006,3032,3057,3079,3100,3119,3136,
    3150,3163,3174,3183,3190,3195,3198,3199
};
volatile uint8_t moter28Index = 0;  // 当前步进位置 0-63
// 步进电机走一步
// dir: 0正转, 1反转 
int8_t uln2803Index=0;
void moter28step(uint8_t dir) {
  
    switch(uln2803Index)
		{
			case 0://00->10
				TIM2->CCR1=sinTable[moter28Index];
				TIM2->CCR2=0;
				break;
			case 1://10->11
				TIM2->CCR1=3199;
				TIM2->CCR2=sinTable[moter28Index];
				break;
			case 2://11->01
				TIM2->CCR1=sinTable[63-moter28Index];
				TIM2->CCR2=3199;
			  break;				
			case 3://01->00
				TIM2->CCR1=0;
				TIM2->CCR2=sinTable[63-moter28Index];	
			  break;			
		}

    
    if (dir == 0) {
        moter28Index = (moter28Index + 1) % 64;
				if((moter28Index%64)==0)
				{
					uln2803Index++;
				}
				
				if(uln2803Index>3)
				{
					uln2803Index=0;
				}
    } else {
        moter28Index = (moter28Index == 0) ? 63 : moter28Index - 1;
				if((moter28Index%64)==0)
				{
					uln2803Index--;
				}
				if(uln2803Index<0)
				{
					uln2803Index=3;
				}
    }
}

void moter28Start(void)
{
    HAL_TIM_Base_Start_IT(&htim2);
}

bool isDectect;

void valveOn(void)
{
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
	moter28Dir=0;
	isDectect=true;
}

void valveOff	(void)
{
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
	moter28Dir=1;
	isDectect=true;
}

void valveDect(void)
{
	if(isDectect)
	{
		switch (moter28Dir)
		{
			case 0: 
				if(getTmux1308Io0Data(valveOpenPos))
					{		
						HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_1);
						HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_2);
						isDectect=false;
					}
				break;
			
			case 1: 
				if(getTmux1308Io0Data(valveClosePos))
					{		
						HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_1);
						HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_2);
						isDectect=false;
					}
				break;
		}
	}
}


#define ServoRise 0
#if ServoRise==1

void riseMoterUp(void)
{
	servoUp(lfvServoRunTime);
}

void riseMoterDown(void)
{
	servoDown(lfvServoRunTime);
}

#else
//定义42步进运行节拍，放入定时器中断2，每100us一次，最大值250 溢出置0
static uint8_t riseTickSymbol=0;

static const int antiMotorRunHighCur=10;//反转大电流 1.1ms
static const int antiMotorRunLowCur=11; //反转小电流 1.2ms
static const int MoterStop   =13;//停止1.4ms
static const int MoterLatch=15;  //自锁1.6ms
static const int MotorRunLowCur=17;		 //正转小电流1.8ms
static const int MotorRunHighCur=18;	//正转大电流1.9ms


 int moterRunFlag=0;			//用于判断值

static int  risedir=0;					//方向位
static bool riseFlag=false;			//是否检测位

void riseRunDec(void)
{
	if(riseFlag)
	{
		switch (risedir)
		{
			case 0: 
				if(1==getTmux1308Io0Data(riseHighPos))
					{		
						moterRunFlag = MoterLatch;
						riseFlag=false;
					}
				break;
			
			case 1: 
				if(1==getTmux1308Io0Data(riseLowPos))
					{		
						moterRunFlag = MoterLatch;
						riseFlag=false;
					}
				break;
		}
	}	
}

void riseMoterUp(void)
{
	risedir=0;
	riseFlag = true;
	moterRunFlag = MotorRunHighCur;
}

void riseMoterDown(void)
{
	risedir=1;
	riseFlag = true;
	moterRunFlag = antiMotorRunHighCur;
}

void riseMoterStop(void)
{
	riseFlag = false;
	moterRunFlag = MoterStop;
}

void riseTick(void)
{
	riseTickSymbol++;
	if(riseTickSymbol>250)
	{
		riseTickSymbol=0;
	}
	if(riseTickSymbol<=moterRunFlag)
	{
		HAL_GPIO_WritePin(Servo_GPIO_Port,Servo_Pin,GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(Servo_GPIO_Port,Servo_Pin,GPIO_PIN_RESET);
	}
}


#endif


