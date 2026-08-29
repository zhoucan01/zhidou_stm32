#include "ft.h"

#include "stdint.h"
#include "main.h"
#include "gpio.h"
#include "AT8236.h"
#include "memoryPool.h"
volatile int clearWaterPumpPulseNum = 0;
volatile uint16_t drinePumpPulseNum = 0;
const int	clearWaterPumpPulseNumTar = 14500;//1300
const uint16_t	drinePumpPulseNumTar = 2100;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // 判断是哪个引脚触发的中断
    if(GPIO_Pin == GPIO_PIN_13)
    {
        clearWaterPumpPulseNum++;  // 流量计脉冲 +1
				if(clearWaterPumpPulseNum>=clearWaterPumpPulseNumTar)
				{
					clearWaterPumpPulseNum=0;
					waterPumpOff();
          uint8_t *pState = getmoterStateDate();
          pState[clearWaterPump] = 0;
				}
    }
    if(GPIO_Pin == GPIO_PIN_3)
    {
        drinePumpPulseNum++;  // 流量计脉冲 +1
				if(drinePumpPulseNum>=drinePumpPulseNumTar)
				{
					drinePumpPulseNum=0;
					waterPumpOff();
          uint8_t *pState = getmoterStateDate();
          pState[drinePump] = 0;
				}
    }
    else if(GPIO_Pin == GPIO_PIN_15)
    {
        // PIN_15 对应其他功能，做别的事
				//poweroff
    }

}

