#include "PulseTurns.h"

#include "tim.h"
#include "gpio.h"


volatile int64_t pulseCount=0;


void pulseTurnsInit(void)
{
    HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_1);
}


float pulseTurnsGet(void)
{
    return (float)pulseCount/1000.0f;
}


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance==TIM1)
    {
        if(htim->Channel==HAL_TIM_ACTIVE_CHANNEL_1)
        {
            if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_11)==GPIO_PIN_RESET)
            {
                pulseCount++;
            }
            else
            {
                pulseCount--;
            }
        }
    }
}