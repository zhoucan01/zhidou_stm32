#include "LED.h"

/* 板载 LED 的逻辑开关控制，GPIO 有效电平封装在本模块中。 */

#include "at32f415.h"
#include "at32f415_wk_config.h"


void LEDOff(void)
{
    gpio_bits_reset(LED1_GPIO_PORT, LED1_PIN);
}

void LEDOn(void)
{
    gpio_bits_set(LED1_GPIO_PORT, LED1_PIN);
}

