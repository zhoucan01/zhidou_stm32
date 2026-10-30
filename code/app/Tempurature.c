#include "Tempurature.h"

/* 使用本板标定参数，将 NTC ADC 原始值换算为摄氏温度。 */
#include "tmux1308.h"
#include <math.h>
#include <stdint.h>

/* Keep the existing board parameters: 10 kohm pull-up, 100 kohm NTC, B3950. */
#define NTC_ADC_FULL_SCALE 4095U
#define NTC_PULLUP_OHMS    10000.0f
#define NTC_R25_OHMS       100000.0f
#define NTC_BETA           3950.0f
#define NTC_T25_KELVIN     298.15f

float tempADCVol;

float getTemp(uint16_t adc_value)
{
    float resistance;

    tempADCVol = (float)adc_value * 3.3f / (float)NTC_ADC_FULL_SCALE;

    /* Rail readings are invalid; avoid division by zero and log(0). */
    if(adc_value == 0U || adc_value >= NTC_ADC_FULL_SCALE)
    {
        return TEMP_INVALID_C;
    }

    resistance = NTC_PULLUP_OHMS * (float)adc_value /
                 (float)(NTC_ADC_FULL_SCALE - adc_value);
    return (float)(1.0 / (log(resistance / NTC_R25_OHMS) / NTC_BETA +
                         1.0 / NTC_T25_KELVIN) - 273.15);
}

/* Legacy callers and Modbus must use the same NTC conversion. */
float calTemp(uint16_t adc_value)
{
    return getTemp(adc_value);
}
