#include "tmux1308.h"

/* TMUX1308 通道选择、稳定等待，以及 D0/D1/D2 缓存刷新。 */
#include "stdint.h"
#include "at32f415_wk_config.h"
#include "wk_adc.h"
#include "wk_gpio.h"
#include "FreeRTOS.h"
#include "task.h"

/* The analog mux and its source network require a long settling interval.
 * Keep this at 1 ms to prioritize stable ADC readings over scan throughput. */
#define TMUX1308_SETTLE_US 1000U

/* D0 各通道的数字电平缓存，下标与 tmux1308Data0 枚举值对应。 */
uint8_t tmux1308Data0Values[TMUX1308_CHANNEL_COUNT];
/* D1 各通道的 ADC 原始值缓存，下标与 tmux1308Data1 枚举值对应。 */
uint16_t tmux1308Data1Values[TMUX1308_CHANNEL_COUNT];
/* D2 各通道的 ADC 原始值缓存，下标与 tmux1308Data2 枚举值对应。 */
uint16_t tmux1308Data2Values[TMUX1308_CHANNEL_COUNT];

void tmux1308Init(void)
{
    uint8_t channel;
    /* 先清零，确保任何未完成的通道在初始化期间都有确定值。 */
    for (channel = 0U; channel < TMUX1308_CHANNEL_COUNT; ++channel) {
        tmux1308Data0Values[channel] = 0U;
        tmux1308Data1Values[channel] = 0U;
        tmux1308Data2Values[channel] = 0U;
    }
    /* 完成一次全通道扫描，为上层提供上电后的首组有效数据。 */
    getTMUX1308Data();
}

/* DWT 使用内核周期计数器，不依赖 FreeRTOS 占用的 SysTick。 */
static void tmux1308DelayUs(uint32_t us)
{
    const uint32_t cycles = (system_core_clock / 1000000U) * us;
    uint32_t start;

    if ((CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) == 0U) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    }
    if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0U) {
        DWT->CYCCNT = 0U;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }

    start = DWT->CYCCNT;
    while ((uint32_t)(DWT->CYCCNT - start) < cycles) {
    }
}

/* Address selection and the following ADC conversion form one transaction.
 * Do not use vTaskDelay() here: yielding during the 1 ms settling interval
 * lets another task select a different shared TMUX channel, so the LED
 * current read may intermittently contain brake/valve-channel data. */
static void tmux1308SettleDelay(void)
{
    tmux1308DelayUs(TMUX1308_SETTLE_US);
}

/* 三颗 TMUX1308 共用 A0/A1/A2，通道号的 bit0/1/2 分别控制对应地址线。 */
void tmux1308SetAddress(uint8_t channel)
{
    gpio_bits_write(tmux1308A0_GPIO_PORT, tmux1308A0_PIN,
                    (channel & 0x01U) ? TRUE : FALSE);
    gpio_bits_write(tmux1308A1_GPIO_PORT, tmux1308A1_PIN,
                    (channel & 0x02U) ? TRUE : FALSE);
    gpio_bits_write(tmux1308A2_GPIO_PORT, tmux1308A2_PIN,
                    (channel & 0x04U) ? TRUE : FALSE);
    /* 从最后一根地址线写完开始计稳定时间。 */
    tmux1308SettleDelay();
}

/* 地址线稳定后，先丢弃首次 ADC 转换，再返回下一次转换结果。 */
static uint16_t tmux1308ReadAdc(adc_channel_select_type adc_channel)
{
    (void)adc_read_channel(adc_channel);
    return adc_read_channel(adc_channel);
}

/* 从 PB13 读取 TMUX1308 D0 的数字电平。 */
static uint8_t tmux1308ReadIo(void)
{
    return gpio_input_data_bit_read(tmux1308IOData0_GPIO_PORT,
                                    tmux1308IOData0_PIN) == SET ? 1U : 0U;
}

/* 共用地址线逐通道扫描，同一通道下采集三颗 TMUX1308 的输出。 */
void getTMUX1308Data(void) 
{
    uint8_t channel;
    for (channel = 0U; channel < TMUX1308_CHANNEL_COUNT; ++channel) {
        tmux1308SetAddress(channel);
        tmux1308Data0Values[channel] = tmux1308ReadIo();
        tmux1308Data1Values[channel] = tmux1308ReadAdc(ADC_CHANNEL_13);
        tmux1308Data2Values[channel] = tmux1308ReadAdc(ADC_CHANNEL_14);
    }
}

/* D0 为数字输入：返回 0/1；超出 S0～S7 的枚举值返回 0。 */
uint8_t tmux1308ReadData0(tmux1308Data0 channel)
{
    if ((unsigned int)channel >= TMUX1308_CHANNEL_COUNT) {
        return 0U;
    }
    tmux1308SetAddress((uint8_t)channel);
    tmux1308Data0Values[channel] = tmux1308ReadIo();
    return tmux1308Data0Values[channel];
}

/* D1 为 ADC1_IN13：返回 ADC 原始值；无效通道返回 0。 */
uint16_t tmux1308ReadData1(tmux1308Data1 channel)
{
    if ((unsigned int)channel >= TMUX1308_CHANNEL_COUNT) {
        return 0U;
    }
    tmux1308SetAddress((uint8_t)channel);
    tmux1308Data1Values[channel] = tmux1308ReadAdc(ADC_CHANNEL_13);
    return tmux1308Data1Values[channel];
}

/* D2 为 ADC1_IN14：返回 ADC 原始值；无效通道返回 0。 */
uint16_t tmux1308ReadData2(tmux1308Data2 channel)
{
    if ((unsigned int)channel >= TMUX1308_CHANNEL_COUNT) {
        return 0U;
    }
    tmux1308SetAddress((uint8_t)channel);
    tmux1308Data2Values[channel] = tmux1308ReadAdc(ADC_CHANNEL_14);
    return tmux1308Data2Values[channel];
}


