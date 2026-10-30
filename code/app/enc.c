#include "enc.h"

/* 编码器脉冲与有符号位置计数；外部中断回调为当前实际计数入口。 */
#include "at32f415_wk_config.h"
#include "at32f415_gpio.h"
#include "at32f415_misc.h"
#include "at32f415_tmr.h"

/* 抬升电机左编码器：PA11(A) / PA10(B, TMR1_CH3)。 */
/* 抬升电机左侧编码器的有符号累计位置（单位：计数脉冲）。
 * 在配置的计数边沿读取另一相判定方向；正负号只表示软件约定方向。 */
static volatile int32_t riseMoterLEncPosition;
/* 抬升电机左侧编码器 B 相累计边沿数，只增不减，用于统计总脉冲。 */
static volatile uint32_t riseMoterLEncBPulses;

/* 抬升电机右编码器：PA9(A) / PA8(B, TMR1_CH1)。 */
/* 抬升电机右侧编码器的有符号累计位置（单位：计数脉冲）。 */
static volatile int32_t riseMoterREncPosition;
/* 抬升电机右侧编码器 B 相累计边沿数，只增不减。 */
static volatile uint32_t riseMoterREncBPulses;

/* 压制电机编码器：PC9(A) / PC8(B, TMR3_CH3)。 */
/* 压制电机编码器的有符号累计位置（单位：计数脉冲）。 */
static volatile int32_t pressMoterREncPosition;
/* 压制电机编码器累计边沿数，只增不减。 */
static volatile uint32_t pressMoterREncBPulses;

void encInit(void)
{
  gpio_init_type gpio_init_struct;

  riseMoterLEncPosition = 0;
  riseMoterLEncBPulses = 0U;
  riseMoterREncPosition = 0;
  riseMoterREncBPulses = 0U;
  pressMoterREncPosition = 0;
  pressMoterREncBPulses = 0U;

  /* 给全部编码器输入配置上拉，避免断线、开漏或噪声导致误触发计数。 */
  gpio_default_para_init(&gpio_init_struct);
  gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
  gpio_init_struct.gpio_pull = GPIO_PULL_UP;
  gpio_init_struct.gpio_pins = riseMoterLEncA_PIN | riseMoterLEncB_PIN |
                               riseMoterREncA_PIN | riseMoterREncB_PIN;
  gpio_init(GPIOA, &gpio_init_struct);
  gpio_init_struct.gpio_pins = pressMoterEncA_PIN | pressMoterEncB_PIN;
  gpio_init(GPIOC, &gpio_init_struct);

  /* 编码器仅由配置工具生成的 EXINT8/EXINT10/EXINT9 中断计数，
   * 不能再启用定时器输入捕获中断，以免重复计数。 */
}

uint8_t encRiseMoterLEncARead(void)
{
  return gpio_input_data_bit_read(riseMoterLEncA_GPIO_PORT,
                                  riseMoterLEncA_PIN) != RESET ? 1U : 0U;
}

int32_t encRiseMoterLEncPositionGet(void) { return riseMoterLEncPosition; }
void encRiseMoterLEncPositionReset(void) { riseMoterLEncPosition = 0; }
uint32_t encRiseMoterLEncBPulseGet(void) { return riseMoterLEncBPulses; }
void encRiseMoterLEncBPulseReset(void) { riseMoterLEncBPulses = 0U; }

uint8_t encRiseMoterREncARead(void)
{
  return gpio_input_data_bit_read(riseMoterREncA_GPIO_PORT,
                                  riseMoterREncA_PIN) != RESET ? 1U : 0U;
}

int32_t encRiseMoterREncPositionGet(void) { return riseMoterREncPosition; }
void encRiseMoterREncPositionReset(void) { riseMoterREncPosition = 0; }
uint32_t encRiseMoterREncBPulseGet(void) { return riseMoterREncBPulses; }
void encRiseMoterREncBPulseReset(void) { riseMoterREncBPulses = 0U; }

uint8_t encPressMoterREncARead(void)
{
  return gpio_input_data_bit_read(pressMoterEncA_GPIO_PORT,
                                  pressMoterEncA_PIN) != RESET ? 1U : 0U;
}

int32_t encPressMoterREncPositionGet(void) { return pressMoterREncPosition; }
void encPressMoterREncPositionReset(void) { pressMoterREncPosition = 0; }
uint32_t encPressMoterREncBPulseGet(void) { return pressMoterREncBPulses; }
void encPressMoterREncBPulseReset(void) { pressMoterREncBPulses = 0U; }

/* B 相上升沿采样 A 相。若实际正反方向与约定相反，交换 ++/-- 即可。 */
void encRiseMoterLEncTmr1Ch1Irq(void)
{
  if(encRiseMoterLEncARead() != 0U)
    riseMoterLEncPosition++;
  else
    riseMoterLEncPosition--;
  riseMoterLEncBPulses++;
}

void encRiseMoterREncTmr1Ch3Irq(void)
{
  if(encRiseMoterREncARead() != 0U)
    riseMoterREncPosition++;
  else
    riseMoterREncPosition--;
  riseMoterREncBPulses++;
}

void encPressMoterREncTmr3Ch3Irq(void)
{
  if(encPressMoterREncARead() != 0U)
    pressMoterREncPosition++;
  else
    pressMoterREncPosition--;
  pressMoterREncBPulses++;
}

void encRiseMoterLEncExintIrq(void)
{
  encRiseMoterLEncTmr1Ch1Irq();
}

void encRiseMoterREncExintIrq(void)
{
  encRiseMoterREncTmr1Ch3Irq();
}

void encPressMoterREncExintIrq(void)
{
  /* EXINT9 接在 A 相，因此读取 B 相电平判断压制编码器方向。 */
  if(gpio_input_data_bit_read(pressMoterEncB_GPIO_PORT, pressMoterEncB_PIN) != RESET)
    pressMoterREncPosition++;
  else
    pressMoterREncPosition--;
  pressMoterREncBPulses++;
}

int getPressMoterTurns(void)
{
	return pressMoterREncPosition;
}
