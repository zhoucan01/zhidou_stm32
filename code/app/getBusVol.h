#ifndef GETBUSVOL__H
#define GETBUSVOL__H
/* Bus-voltage acquisition API and shared voltage value declaration. */

/* 母线电压 ADC 的最新原始采样值（0~4095）。 */
extern float busVolADC;

/* 最近一次换算得到的母线电压，单位为伏特。 */
extern float busVol;

/* 读取 ADC1_IN5，并按 3.3 V 基准和 11 倍分压比返回母线电压。 */
float getBusVoltage(void);

#endif
