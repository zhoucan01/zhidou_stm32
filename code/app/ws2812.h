#ifndef WS2812_H
#define WS2812_H
/* WS2812 RGB pixel data type and strip update API. */

#include <stdint.h>

/* 使用 SPI 波形编码方式驱动 WS2812 灯珠。 */

typedef struct
{
  uint8_t red;
  uint8_t green;
  uint8_t blue;
} ws2812_rgb_type;

#define WS2812_PIXEL_COUNT  4U

/* SPI1_MOSI remap 到 PB5；在外设时钟初始化后调用一次。 */
/* 配置灯带输出所使用的 SPI 外设。 */
void ws2812Init(void);
/* 同步发送 GRB 数据，count 可以为 0（只输出复位低电平）。 */
/* 发送 RGB 像素数据，随后输出 WS2812 复位低电平。 */
void ws2812Show(const ws2812_rgb_type *pixels, uint16_t count);
/* 将灯带中的全部 WS2812 设置为同一 RGB 颜色。 */
void ws2812SetRgb(uint8_t red, uint8_t green, uint8_t blue);

#endif
