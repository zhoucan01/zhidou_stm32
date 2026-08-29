#include "ws2812.h"
#include "spi.h"
#include <string.h>
static void Ws2812EnterSpiMode(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* 关闭EXTI中断 */
    HAL_NVIC_DisableIRQ(EXTI3_IRQn);

    /* PB3切换为SPI SCK */
    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

static void Ws2812ExitSpiMode(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* PB3恢复GPIO中断输入 */
    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;      // 根据你的实际需要
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* 清掉可能残留的挂起标志 */
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_3);
		HAL_NVIC_ClearPendingIRQ(EXTI3_IRQn);
    /* 重新打开EXTI */
    HAL_NVIC_EnableIRQ(EXTI3_IRQn);
}

/*
 * 每个WS2812数据位编码成3个SPI位：
 *
 * 0 -> 100
 * 1 -> 110
 *
 * 一颗灯24位：
 * 24 × 3 = 72个SPI位 = 9字节
 */
#define WS2812_BUFFER_SIZE \
    (WS2812_LED_COUNT * WS2812_BYTES_PER_LED + WS2812_RESET_BYTES)

static uint8_t ws2812_buffer[WS2812_BUFFER_SIZE];
static volatile uint8_t ws2812_busy = 0;


/**
 * @brief 将一个WS2812数据字节编码为3个SPI字节
 */
static void WS2812_EncodeByte(uint8_t data, uint8_t *output)
{
    uint32_t encoded = 0;

    for (uint8_t i = 0; i < 8; i++)
    {
        encoded <<= 3;

        if ((data & 0x80U) != 0U)
        {
            /* WS2812逻辑1 -> SPI 110 */
            encoded |= 0x06U;
        }
        else
        {
            /* WS2812逻辑0 -> SPI 100 */
            encoded |= 0x04U;
        }

        data <<= 1;
    }

    output[0] = (uint8_t)(encoded >> 16);
    output[1] = (uint8_t)(encoded >> 8);
    output[2] = (uint8_t)encoded;
}


void WS2812_Init(void)
{
    memset(ws2812_buffer,0,sizeof(ws2812_buffer));

    Ws2812EnterSpiMode();

    HAL_SPI_Transmit(&hspi1,ws2812_buffer,
                     sizeof(ws2812_buffer),
                     HAL_MAX_DELAY);

    while(__HAL_SPI_GET_FLAG(&hspi1,SPI_FLAG_BSY)!=RESET)
    {
    }

    HAL_Delay(1);
		HAL_NVIC_SetPriority(EXTI3_IRQn,5,0);
    Ws2812ExitSpiMode();
		
}


void WS2812_SetColor(uint16_t index,
                     uint8_t red,
                     uint8_t green,
                     uint8_t blue)
{
    uint32_t offset;

    if (index >= WS2812_LED_COUNT)
    {
        return;
    }

    /*
     * DMA发送时不要修改缓冲区。
     */
    if (ws2812_busy != 0U)
    {
        return;
    }

    offset = index * WS2812_BYTES_PER_LED;

    /*
     * WS2812通常按照GRB顺序发送。
     */
    WS2812_EncodeByte(green, &ws2812_buffer[offset + 0U]);
    WS2812_EncodeByte(red,   &ws2812_buffer[offset + 3U]);
    WS2812_EncodeByte(blue,  &ws2812_buffer[offset + 6U]);
}


HAL_StatusTypeDef WS2812_Show(void)
{
    HAL_StatusTypeDef status;

    uint32_t led_data_size =
        WS2812_LED_COUNT * WS2812_BYTES_PER_LED;

    /* 尾部全部清零，产生Reset低电平 */
    memset(&ws2812_buffer[led_data_size],
           0,
           WS2812_RESET_BYTES);

    Ws2812EnterSpiMode();

		status = HAL_SPI_Transmit(&hspi1,
															ws2812_buffer,
															sizeof(ws2812_buffer),
															HAL_MAX_DELAY);

		while (__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_BSY) != RESET)
		{
		}

		HAL_Delay(1);

		Ws2812ExitSpiMode();

    return status;
}


HAL_StatusTypeDef WS2812_Show_DMA(void)
{
    HAL_StatusTypeDef status;
    uint32_t led_data_size =
        WS2812_LED_COUNT * WS2812_BYTES_PER_LED;

    if (ws2812_busy != 0U)
    {
        return HAL_BUSY;
    }

    memset(&ws2812_buffer[led_data_size],
           0,
           WS2812_RESET_BYTES);

    ws2812_busy = 1U;

    status = HAL_SPI_Transmit_DMA(&hspi1,
                                  ws2812_buffer,
                                  sizeof(ws2812_buffer));

    if (status != HAL_OK)
    {
        ws2812_busy = 0U;
    }

    return status;
}


uint8_t WS2812_IsBusy(void)
{
    return ws2812_busy;
}


/**
 * @brief SPI DMA发送完成回调
 */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1)
    {
        ws2812_busy = 0U;
    }
}


/**
 * @brief SPI错误回调
 */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1)
    {
        ws2812_busy = 0U;
    }
}

