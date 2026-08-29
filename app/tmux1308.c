#include "tmux1308.h"

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "gpio.h"
#include "adc.h"


extern ADC_HandleTypeDef hadc1;

/*
 * 板载ADC
 */
uint16_t currentAdc = 0U;

/*
 * TMUX数据
 */
bool tmux1308Io0Data[tmux1308ChannelCount] = {false};

bool tmux1308Io1Data[tmux1308ChannelCount] = {false};

uint16_t tmux1308AdcData[tmux1308ChannelCount] = {0U};

/*
 * 选择TMUX1308通道
 *
 * A0最低位
 */
static void selectTmux1308Channel(uint8_t channel)
{
    if(channel >= tmux1308ChannelCount)
    {
        return;
    }


    HAL_GPIO_WritePin(tmux1308A0_GPIO_Port,tmux1308A0_Pin,
				(channel & 0x01U)
        ? GPIO_PIN_SET
        : GPIO_PIN_RESET
    );


    HAL_GPIO_WritePin(
        tmux1308A1_GPIO_Port,
        tmux1308A1_Pin,
        (channel & 0x02U)
        ? GPIO_PIN_SET
        : GPIO_PIN_RESET
    );


    HAL_GPIO_WritePin(
        tmux1308A2_GPIO_Port,
        tmux1308A2_Pin,
        (channel & 0x04U)
        ? GPIO_PIN_SET
        : GPIO_PIN_RESET
    );
}



/*
 * TMUX切换等待
 */
static void waitTmuxStable(void)
{
    HAL_Delay(1);
}


/*
 * 读取板载ADC
 *
 * Regular:
 * ADC_CHANNEL_4
 */
static HAL_StatusTypeDef readBoardAdc(
        uint16_t *value)
{
    HAL_StatusTypeDef status;


    if(value == NULL)
    {
        return HAL_ERROR;
    }


    status = HAL_ADC_Start(&hadc1);


    if(status != HAL_OK)
    {
        return status;
    }



    status = HAL_ADC_PollForConversion(
                &hadc1,
                10
             );


    if(status != HAL_OK)
    {
        HAL_ADC_Stop(&hadc1);
        return status;
    }



    *value =
        (uint16_t)HAL_ADC_GetValue(&hadc1);



    HAL_ADC_Stop(&hadc1);



    return HAL_OK;
}



/*
 * 读取TMUX1308 ADC
 *
 * Injected:
 * ADC_CHANNEL_5
 */
static HAL_StatusTypeDef readTmuxAdc(
        uint16_t *value)
{
    HAL_StatusTypeDef status;


    if(value == NULL)
    {
        return HAL_ERROR;
    }



    status =
    HAL_ADCEx_InjectedStart(&hadc1);


    if(status != HAL_OK)
    {
        return status;
    }



    status =
    HAL_ADCEx_InjectedPollForConversion(
        &hadc1,
        10
    );


    if(status != HAL_OK)
    {
        HAL_ADCEx_InjectedStop(&hadc1);

        return status;
    }



    *value =
    (uint16_t)
    HAL_ADCEx_InjectedGetValue(
        &hadc1,
        ADC_INJECTED_RANK_1
    );



    HAL_ADCEx_InjectedStop(&hadc1);



    return HAL_OK;
}



/*
 * 扫描全部8路TMUX1308
 */
bool getTmux1308Data(void)
{
    uint8_t channel;


    uint32_t boardSum = 0U;


    uint16_t boardValue = 0U;

    uint16_t tmuxValue = 0U;



    for(channel = 0U;
        channel < tmux1308ChannelCount;
        channel++)
    {


        /*
         * 切换TMUX
         */
        selectTmux1308Channel(channel);



        /*
         * 等待模拟稳定
         */
        waitTmuxStable();
					/*
         * 读取TMUX ADC
         */
        if(readTmuxAdc(&tmuxValue)
           != HAL_OK)
        {
            return false;
        }

        /*
         * 读取板载ADC
         */
        if(readBoardAdc(&boardValue)
           != HAL_OK)
        {
            return false;
        }
  
        /*
         * 保存
         */
        boardSum += boardValue;
				currentAdc=boardValue;
        tmux1308AdcData[channel]
            = tmuxValue;
        /*
         * D0
         */
        tmux1308Io0Data[channel]
        =
        (
            HAL_GPIO_ReadPin(
                tmux1308D0_GPIO_Port,
                tmux1308D0_Pin
            )
            == GPIO_PIN_SET
        );
        /*
         * D1
         */
        tmux1308Io1Data[channel]
        =
        (
            HAL_GPIO_ReadPin(
                tmux1308D1_GPIO_Port,
                tmux1308D1_Pin
            )
            == GPIO_PIN_SET
        );
	
    }



    /*
     * 板载ADC平均
     */
//    currentAdc =
//        boardSum /
//        tmux1308ChannelCount;



    return true;
}




uint16_t getCurrentAdc(void)
{
    return currentAdc;
}




uint16_t getTmux1308AdcData(uint8_t channel)
{
    if(channel >= tmux1308ChannelCount)
    {
        return 0U;
    }


    return tmux1308AdcData[channel];
}




bool getTmux1308Io0Data(uint8_t channel)
{
    if(channel >= tmux1308ChannelCount)
    {
        return false;
    }


    return tmux1308Io0Data[channel];
}




bool getTmux1308Io1Data(uint8_t channel)
{
    if(channel >= tmux1308ChannelCount)
    {
        return false;
    }


    return tmux1308Io1Data[channel];
}