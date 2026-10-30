#ifndef TEMPURATURE__H
#define TEMPURATURE__H
/* NTC temperature conversion API. */

/* Invalid ADC rail reading, in degrees Celsius. */
#define TEMP_INVALID_C (-100.0f)

#include "stdint.h"

float getTemp(uint16_t adc_value);
float calTemp(uint16_t adc_value);
#endif
