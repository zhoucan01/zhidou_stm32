#include "Tempurature.h"
#include "stdint.h"
#include "math.h"
#include "tmux1308.h"
#include <stdint.h>

typedef struct
{
    uint16_t adc;
    float temp;
}TempTable_t;
/*
 * 600W功率测试数据
 * 三组ADC平均
 *
 * 温度:25~98℃
 * 步进:1℃
 *
 * ADC由大到小
 */
static const TempTable_t temp_table[] =
{

    {3381,25},

    {3417,26},
    {3390,27},
    {3327,28},
    {3300,29},
    {3280,30},

    {3255,31},
    {3235,32},
    {3215,33},
    {3194,34},
    {3171,35},

    {3158,36},
    {3128,37},
    {3154,38},
    {3078,39},
    {3063,40},

    {3042,41},
    {3000,42},
    {2976,43},
    {2955,44},
    {2930,45},

    {2895,46},
    {2871,47},
    {2850,48},
    {2821,49},
    {2786,50},

    {2767,51},
    {2728,52},
    {2703,53},
    {2674,54},
    {2647,55},

    {2619,56},
    {2575,57},
    {2553,58},
    {2534,59},
    {2499,60},

    {2459,61},
    {2427,62},
    {2407,63},
    {2376,64},
    {2350,65},

    {2298,66},
    {2276,67},
    {2245,68},
    {2204,69},
    {2200,70},

    {2162,71},
    {2122,72},
    {2090,73},
    {2060,74},
    {2032,75},

    {2004,76},
    {1975,77},
    {1934,78},
    {1910,79},
    {1880,80},

    {1839,81},
    {1805,82},
    {1791,83},
    {1750,84},
    {1734,85},

    {1704,86},
    {1671,87},
    {1652,88},
    {1637,89},
    {1616,90},

    {1600,91},
    {1593,92},
    {1578,93},
    {1564,94},
    {1557,95},

    {1547,96},
    {1540,97},
    {1535,98},

};

#define TEMP_TABLE_SIZE \
(sizeof(temp_table)/sizeof(temp_table[0]))
/*
 * ADC查表转温度
 * 线性插值
 */
static float adc_to_temperature(uint16_t adc)
{
    uint16_t i;
    /*
     ADC超过最高点
     */
    if(adc >= temp_table[0].adc)
    {
        return temp_table[0].temp;
    }
    /*
     ADC低于最低点
     */
    if(adc <= temp_table[TEMP_TABLE_SIZE-1].adc)
    {
        return temp_table[TEMP_TABLE_SIZE-1].temp;
    }
    for(i=0;i<TEMP_TABLE_SIZE-1;i++)
    {

        /*
         * ADC落在两个温度点之间
         */
        if((adc <= temp_table[i].adc) &&
           (adc >= temp_table[i+1].adc))
        {
            float temp;
            temp =temp_table[i].temp +
            ((float)(adc-temp_table[i].adc) *
            (temp_table[i+1].temp-temp_table[i].temp))
            /
            ((float)(temp_table[i+1].adc-temp_table[i].adc));
            return temp;
        }
    }
    return -100.0f;
}
/*
 * 温度读取
 *
 * 原函数名保持
 */
float getTemp(void)
{
    uint16_t adc_value;
    adc_value = getTmux1308AdcData(ntcOut);
    return adc_to_temperature(adc_value);
}
float tempADCVol;
float calTemp(void)
{
	
	tempADCVol=getTmux1308AdcData(ntcOut)/4095.0*3.3f;
//  float temp_V = 3.325 - tempADCVol;
//  float resistance = (10000 * temp_V) / (3.325 - temp_V);	
	float resistance=(10000*tempADCVol)/(3.3-tempADCVol);
	float ntc_temp = 1.0 / (log(resistance / 100000) / 3950.0f + 1 / (273.15 + 25)) - 273.15;
	return ntc_temp;
}

