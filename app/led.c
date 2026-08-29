#include "led.h"
#include "stdbool.h"

void ledControl(GPIO_PinState PinState)
{
	HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,PinState);
}

