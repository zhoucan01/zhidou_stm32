#include "AT8236.h"
#include "gpio.h"

#include "tim.h"

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
//抽点卤剂泵开
void brinePumpOn(void)
{
    HAL_GPIO_WritePin(drinePumpA_GPIO_Port,drinePumpA_Pin,GPIO_PIN_SET);
    HAL_GPIO_WritePin(drinePumpB_GPIO_Port,drinePumpB_Pin,GPIO_PIN_RESET);
}
//抽点卤剂泵关
void brinePumpOff(void)
{
    HAL_GPIO_WritePin(drinePumpA_GPIO_Port,drinePumpA_Pin,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(drinePumpB_GPIO_Port,drinePumpB_Pin,GPIO_PIN_RESET);
}
//抽豆浆泵开
int hosePumpOnSpeed=80;
int hosePumpFlag=0;
void hosePumpOn(uint8_t ppPumpSpeed)
{
	  hosePumpFlag=1;
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    HAL_GPIO_WritePin(hosePumpIo_GPIO_Port,hosePumpIo_Pin,GPIO_PIN_RESET);
    TIM4->CCR3=80;
		//hosePumpOnSpeed=ppPumpSpeed;
}
//抽豆浆泵关
void hosePumpOff(void)
{
		TIM4->CCR3=0;
    //HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
    HAL_GPIO_WritePin(hosePumpIo_GPIO_Port,hosePumpIo_Pin,GPIO_PIN_RESET);
}
extern int clearWaterPumpPulseNum;
//抽清水泵开
void waterPumpOn(int ISPumpSpeed)
{
		clearWaterPumpPulseNum=0;
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_GPIO_WritePin(waterPumpIo_GPIO_Port,waterPumpIo_Pin,GPIO_PIN_RESET);
    TIM3->CCR3=3150;
}
//抽清水泵关

void waterPumpOff(void)
{	
		clearWaterPumpPulseNum=0;
    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
    HAL_GPIO_WritePin(waterPumpIo_GPIO_Port,waterPumpIo_Pin,GPIO_PIN_RESET);  
}


