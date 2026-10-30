#include "getBusVol.h"

/* 母线电压：完成 ADC 采样、滤波、比例换算及共享缓存更新。 */
#include "stdint.h"
#include "at32f415_wk_config.h"
#include "wk_adc.h"
#include "at32f415_adc.h"

/* 采集母线电压 ADC 输入，并将结果换算为伏特。 */
float busVolADC;  /* 母线电压 ADC 的最新原始采样值。 */
float busVol;     /* 最近一次换算得到的母线电压，单位为伏特。 */

#define BUS_VOLTAGE_AVERAGE_SAMPLES  8U

/* 软件触发单次转换并读取指定 ADC1 通道的采样值。
 * wk_adc1_init 已将普通通道长度配置为 1 并选择软件触发，
 * 这里把目标通道放入序列 1，触发一次转换并等待完成后取数。 */


/* 读取一次 ADC：原始值保存到 busVolADC，并按 3.3 V 基准、11 倍分压换算。 */
float getBusVoltage(void)
{
  uint32_t sum = 0U;
  uint8_t sample;

  /* PA5 前一刻可能被其他 ADC 通道切换过来，首个转换不参与结果。 */
  (void)adc_read_channel(ADC_CHANNEL_5);
  for(sample = 0U; sample < BUS_VOLTAGE_AVERAGE_SAMPLES; sample++)
  {
    sum += adc_read_channel(ADC_CHANNEL_5);
  }

  busVolADC = (float)sum / (float)BUS_VOLTAGE_AVERAGE_SAMPLES;
  busVol = busVolADC / 4095.0f * 3.3f * 11.0f;
  return busVol;
}
