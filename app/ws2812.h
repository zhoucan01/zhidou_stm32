#ifndef WS2812__H
#define WS2812__H

#include "main.h"

#define WS2812_LED_COUNT       5U
#define WS2812_BYTES_PER_LED   9U
#define WS2812_RESET_BYTES     100U

void WS2812_Init(void);
void WS2812_SetColor(uint16_t index,
                     uint8_t red,
                     uint8_t green,
                     uint8_t blue);

HAL_StatusTypeDef WS2812_Show(void);
HAL_StatusTypeDef WS2812_Show_DMA(void);
uint8_t WS2812_IsBusy(void);

#endif


