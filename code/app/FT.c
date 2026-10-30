#include "FT.h"

/* 流量计边沿计数：分别累计清水和卤水流量传感器的脉冲数。 */

#include "stdint.h"
#include "at32f415_wk_config.h"
#include "wk_gpio.h"
#include "AT8236.h"

static volatile uint32_t brineFTNum = 0U; /* 卤水流量计脉冲数 */
static volatile uint32_t waterFTNum = 0U; /* 清水流量计脉冲数 */

const int waterFtMaxNum = 13000;
const int brineFtMaxNum = 200;

//点卤泵计数
void brineFTtick(void)
{
    brineFTNum++;
    if(brineFTNum>brineFtMaxNum)
    {
//        brinePumpOff();
        brineFTNum = 0;
    }
}

//水泵计数
void waterFTtick(void)
{
    waterFTNum++;
    if(waterFTNum>waterFtMaxNum)
    {
        waterPumpOff();
        waterFTNum = 0;
    }
}

uint32_t waterFTPulseGet(void)
{
    return waterFTNum;
}

uint32_t brineFTPulseGet(void)
{
    return brineFTNum;
}

void clearWaterFT(void)
{
		waterFTNum=0;
}
