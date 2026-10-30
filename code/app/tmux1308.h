#ifndef TMUX1308_H
#define TMUX1308_H
/* TMUX1308 channel identifiers, cache arrays, and acquisition API. */

#include <stdint.h>

/* 八通道 TMUX1308 多路复用器接口及扫描数据缓存。 */

#define TMUX1308_CHANNEL_COUNT 8U

/* D2：模拟输出，接 PC4 / ADC1_IN14；枚举值对应 TMUX1308 的 S0～S7。 */
typedef enum {
    NTC2=0,//筒壁NTC2 ain8
    NTC3,//筒壁NTC3 ain7
    NTCBacK,//预留NTC ain6
    NTC1,//筒壁NTC1 ain4
    waterPos,//水位 AIN9
    Lid1,//盖子1    AIN2
    overFlow,//溢流 AIN3
    Lid2//盖子2    AIN5
}tmux1308Data2;

/* D1：模拟输出，接 PC3 / ADC1_IN13；枚举值对应 TMUX1308 的 S0～S7。 */
typedef enum {
    pressMoterFault=0,//压制电机故障 Fault-M4
    riseMoterRCurrB,//右抬升电机线圈B电流 AD-M12
    riseMoterRCurrA,//右抬升电机线圈A电流 AD-M11
    modeAvail,       //成型通道在位 S13
    riseMoterRFaultR,//右抬升电机故障     Fault-M1
    riseMoterRFaultL,//左抬升电机故障     Fault-M2
    riseMoterLCurrB,//左抬升电机线圈B电流 AD-M22
    riseMoterLCurrA//左抬升电机线圈A电流  AD-M21
}tmux1308Data1;

/* D0：数字输出，接 PB13；枚举值对应 TMUX1308 的 S0～S7。 */
typedef enum {
    pressaAvail=0,//压制在位 S17
    brineLowPos,//点卤盒低水位 s6
    brineAvail,//卤水盒在位 s15
    wasteBoxAvail,//黄水盒在位 s8
    valveFault,//阀故障  Fault-M7
    waterHighPos,//水位高 S14
    lidAvail,//盖子在位 S16
    waterTankAvail//水箱在位 S2
}tmux1308Data0;

/* 数组下标直接使用对应枚举值：S0～S7。上电后首次扫描前元素为 0。 */
/* 三颗 TMUX1308 的全通道最新采样缓存。 */
extern uint8_t tmux1308Data0Values[TMUX1308_CHANNEL_COUNT];
extern uint16_t tmux1308Data1Values[TMUX1308_CHANNEL_COUNT];
extern uint16_t tmux1308Data2Values[TMUX1308_CHANNEL_COUNT];

/* 清空缓存并执行一次全通道扫描。 */
void tmux1308Init(void);

/* 扫描 S0～S7，每个通道同时更新 D0、D1、D2 的数组元素。 */
/* 扫描全部通道并更新 tmux1308Data0、tmux1308Data1 和 tmux1308Data2。 */
void getTMUX1308Data(void);
/* 设置三颗 TMUX1308 的共用地址线，并等待通道稳定；0～7 对应 S0～S7。 */
/* 选择 0～7 范围内的一个有效通道。 */
void tmux1308SetAddress(uint8_t channel);
/* 选择 D0 的指定通道，更新数组并返回 PB13 电平（0 或 1）。 */
uint8_t tmux1308ReadData0(tmux1308Data0 channel);
/* 选择 D1 的指定通道，更新数组并返回 12 位 ADC 原始值。 */
uint16_t tmux1308ReadData1(tmux1308Data1 channel);
/* 选择 D2 的指定通道，更新数组并返回 12 位 ADC 原始值。 */
uint16_t tmux1308ReadData2(tmux1308Data2 channel);

#endif
