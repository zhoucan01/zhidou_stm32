#include "totolCTL.h"

/* 公共电机复位/使能输入与输出控制。 */

#include "wk_tmr.h"
#include "wk_gpio.h"

#include "at32f415.h"
#include "at32f415_wk_config.h"
#include "AT8236.h"
//文件说明 由于很多外设共用了一个引脚会造成干扰，所以在总控中对共用引脚外设进行统一管理，让相关外设不会改变原有的功能

static bool istotolRSTHigh=false;//总控复位信号，true表示高电平，false表示低电平

bool getTotolRstLevel(void)
{
    return istotolRSTHigh;
}

void moterEN(void)
{
    gpio_bits_set(moterRST_GPIO_PORT,moterRST_PIN);
    istotolRSTHigh=true;
    pumpSharedInputSync();
}

void moterDIS(void)
{
    gpio_bits_reset(moterRST_GPIO_PORT,moterRST_PIN);
    istotolRSTHigh=false;
    pumpSharedInputSync();
}

/* 按参数设置 PC10（moterRST）电平，电平未变化时不做任何操作。
 * 返回设置后的电平。供上位机命令使用，保证状态变量与实际引脚一致。 */
bool moterRstSet(bool high)
{
    if(high != istotolRSTHigh)
    {
        if(high)
        {
            moterEN();
        }
        else
        {
            moterDIS();
        }
    }
    return istotolRSTHigh;
}
