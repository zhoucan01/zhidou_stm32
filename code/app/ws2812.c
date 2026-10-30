#include "ws2812.h"

/* WS2812：将 GRB 颜色编码为 SPI 时序，并发送整帧多像素数据。 */
#include "at32f415_wk_config.h"
#include "at32f415_spi.h"

/*
 * 将 WS2812 位编码为 3 个 SPI 位。
 *
 * 系统 SPI1 时钟为 75 MHz，使用 /32 后 SPI 时钟为 2.34375 MHz：
 *   一个 WS2812 位 = 3 / 2.34375 MHz = 1.28 us。
 *   0 -> 100：高 0.427 us，低 0.853 us。
 *   1 -> 110：高 0.853 us，低 0.427 us。
 *
 * 原先的 /16 + 5 位编码中，0 的高电平只有约 0.213 us，低于 WS2812
 * 可可靠识别的时间范围，因此可能出现完全不亮或颜色不稳定。
 */
#define WS2812_SPI_ZERO       0x04U
#define WS2812_SPI_ONE        0x06U
#define WS2812_SYMBOL_BITS    3U
#define WS2812_RESET_BYTES    20U /* 20 * 8 / 2.34375 MHz = 68.3 us > 50 us */

static void ws2812SpiWrite(uint8_t data)
{
  while(spi_i2s_flag_get(SPI1, SPI_I2S_TDBE_FLAG) == RESET)
  {
  }
  spi_i2s_data_transmit(SPI1, data);
}

static void ws2812WriteBit(uint8_t bit, uint8_t *packed, uint8_t *bitCount)
{
  uint8_t symbol = (bit != 0U) ? WS2812_SPI_ONE : WS2812_SPI_ZERO;
  uint8_t index;

  for(index = 0U; index < WS2812_SYMBOL_BITS; index++)
  {
    *packed = (uint8_t)((*packed << 1) |
                        ((symbol >> (WS2812_SYMBOL_BITS - 1U - index)) & 1U));
    (*bitCount)++;
    if(*bitCount == 8U)
    {
      ws2812SpiWrite(*packed);
      *packed = 0U;
      *bitCount = 0U;
    }
  }
}

static void ws2812WriteByte(uint8_t value, uint8_t *packed, uint8_t *bitCount)
{
  uint8_t index;

  for(index = 0U; index < 8U; index++)
    ws2812WriteBit((uint8_t)(value & (0x80U >> index)), packed, bitCount);
}

void ws2812Init(void)
{
  gpio_init_type gpio_init_struct;
  spi_init_type spi_init_struct;

  /* SPI1_MOSI 重映射到 PB5，WS2812 数据线仅使用该引脚。 */
  crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_SPI1_PERIPH_CLOCK, TRUE);
  gpio_pin_remap_config(SPI1_MUX_01, TRUE);
  gpio_default_para_init(&gpio_init_struct);
  gpio_init_struct.gpio_pins = GPIO_PINS_5;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init(GPIOB, &gpio_init_struct);

  spi_i2s_reset(SPI1);
  spi_default_para_init(&spi_init_struct);
  spi_init_struct.transmission_mode = SPI_TRANSMIT_HALF_DUPLEX_TX;
  spi_init_struct.master_slave_mode = SPI_MODE_MASTER;
  spi_init_struct.mclk_freq_division = SPI_MCLK_DIV_32;
  spi_init_struct.first_bit_transmission = SPI_FIRST_BIT_MSB;
  spi_init_struct.frame_bit_num = SPI_FRAME_8BIT;
  spi_init_struct.clock_polarity = SPI_CLOCK_POLARITY_LOW;
  spi_init_struct.clock_phase = SPI_CLOCK_PHASE_1EDGE;
  spi_init_struct.cs_mode_selection = SPI_CS_SOFTWARE_MODE;
  spi_init(SPI1, &spi_init_struct);
  spi_enable(SPI1, TRUE);
}

void ws2812Show(const ws2812_rgb_type *pixels, uint16_t count)
{
  uint16_t index;
  uint8_t packed = 0U;
  uint8_t bitCount = 0U;

  for(index = 0U; index < count; index++)
  {
    /* WS2812 的字节顺序为 G、R、B。 */
    ws2812WriteByte(pixels[index].green, &packed, &bitCount);
    ws2812WriteByte(pixels[index].red, &packed, &bitCount);
    ws2812WriteByte(pixels[index].blue, &packed, &bitCount);
  }
  if(bitCount != 0U)
    ws2812SpiWrite((uint8_t)(packed << (8U - bitCount)));
  while(spi_i2s_flag_get(SPI1, SPI_I2S_BF_FLAG) != RESET)
  {
  }
  for(index = 0U; index < WS2812_RESET_BYTES; index++)
    ws2812SpiWrite(0U);
  while(spi_i2s_flag_get(SPI1, SPI_I2S_BF_FLAG) != RESET)
  {
  }
}

void ws2812SetRgb(uint8_t red, uint8_t green, uint8_t blue)
{
  ws2812_rgb_type pixels[WS2812_PIXEL_COUNT];
  uint16_t index;

  for(index = 0U; index < WS2812_PIXEL_COUNT; index++)
  {
    pixels[index].red = red;
    pixels[index].green = green;
    pixels[index].blue = blue;
  }

  ws2812Show(pixels, WS2812_PIXEL_COUNT);
}

