#ifndef LED__H
#define LED__H

#include "main.h"
#include "stm32f1xx_hal.h"
static inline void ledOn(void)
{
	GPIOB->BSRR = (1 << 2);
}
static inline void ledOff(void) {
    GPIOB->BSRR = (1 << (2 + 16));  // 把第12位移到高16位，即 BR12
}
#endif
