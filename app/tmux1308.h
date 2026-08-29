#ifndef TMUX1308_H
#define TMUX1308_H

#include <stdint.h>
#include <stdbool.h>

/*
 * TMUX1308通道数量
 */
#define tmux1308ChannelCount 8U

/*
 * D0输入定义
 */
typedef enum
{
    s19 = 0,
    riseHighPos,//升降高位
    valveOpenPos,//阀开位
    valveClosePos,//阀关位
    s16,
    s13,
    s4,//压制头
    riseLowPos//升降低位
} Io0Data;

/*
 * D1输入定义
 */
typedef enum
{
    s15 = 0,
    brineAvail,//卤水箱在位
    pressSwitch1,//压制高位
    s11,
    pressSwitch2,//压制低位
    waterBoxHighPos,//水箱高位
    wasteBoxAvail,//水箱在位
    waterBoxLowPos//水箱低位
} Io1Data;

/*
 * ADC模拟量定义
 *
 * TMUX1308:
 *
 * AIN0~AIN7
 */
typedef enum
{
    busVol = 0,//母线电压adc
    lid1,//1
    adcNone,//2
    waterBoxAvail,//3
    ntcIn,//4
    tempBack,//5
    ntcOut,//6
    lid2//7

} AdcData;

/**
 * @brief 扫描TMUX1308全部8路数据
 *
 * 流程:
 *
 * 1. 设置A0/A1/A2选择通道
 * 2. 等待模拟稳定
 * 3. Regular读取板载ADC
 * 4. Injected读取TMUX ADC
 * 5. 保存D0/D1状态
 *
 *
 * @return
 * true 读取成功
 * false ADC失败
 */
bool getTmux1308Data(void);

/**
 * @brief 获取板载ADC平均值
 *
 * 当前:
 * ADC_CHANNEL_4
 *
 * @return ADC值
 */
uint16_t getCurrentAdc(void);


/**
 * @brief 获取TMUX1308 ADC数据
 *
 * @param channel
 * 0~7
 *
 * @return ADC值
 */
uint16_t getTmux1308AdcData(uint8_t channel);





/**
 * @brief 获取TMUX1308 D0状态
 *
 * @param channel
 * 0~7
 *
 * @return
 * true  高电平
 * false 低电平
 */
bool getTmux1308Io0Data(uint8_t channel);





/**
 * @brief 获取TMUX1308 D1状态
 *
 * @param channel
 * 0~7
 *
 * @return
 * true  高电平
 * false 低电平
 */
bool getTmux1308Io1Data(uint8_t channel);



#endif