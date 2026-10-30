#ifndef CURRENT_H
#define CURRENT_H
/* Shared current measurement cache and current-channel access APIs. */

#include <stdint.h>

/* 电流监测数据以及 ADC 原始值到安培值的换算接口。 */

/*
 * 电流采样结果：*_raw 是 0～4095 的 ADC 原始值，*_ampere 的单位为 A。
 * 各字段按采样顺序更新，不是同一时刻的同步快照。
 */
typedef struct {
    uint16_t busRaw;             /* PA4 / ADC1_IN4：母线电流 */
    float busAmpere;             /* busRaw × 0.000806 A */
    uint16_t ledRaw;             /* PC0 / ADC1_IN10：LED 电流 */
    float ledAmpere;             /* ledRaw × 0.000745 A */
    uint16_t brakeRaw;           /* PC0 / ADC1_IN10：电机抱闸电流 */
    float brakeAmpere;           /* brakeRaw × 0.000745 A */
    uint16_t strainGageRaw;     /* PB0 / ADC1_IN8：应变片电流原始值 */
    float strainGageAmpere;     /* strainGageRaw × 0.005 A */
    uint16_t riseRightBRaw;    /* TMUX1308 D1：AD-M12 */
    float riseRightBAmpere;    /* riseRightBRaw × 0.0008 A */
    uint16_t riseRightARaw;    /* TMUX1308 D1：AD-M11 */
    float riseRightAAmpere;    /* riseRightARaw × 0.0008 A */
    uint16_t riseLeftBRaw;     /* TMUX1308 D1：AD-M22 */
    float riseLeftBAmpere;     /* riseLeftBRaw × 0.0008 A */
    uint16_t riseLeftARaw;     /* TMUX1308 D1：AD-M21 */
    float riseLeftAAmpere;     /* riseLeftARaw × 0.0008 A */
} current_data_t;

/* 抬升两台电机相电流幅值测量结果（单位 A，均为正值）。
 * left/right 取该电机 A/B 两相中较大的一相幅值。 */
typedef struct {
    float leftA;    /* 左电机 A 相电流幅值 */
    float leftB;    /* 左电机 B 相电流幅值 */
    float rightA;   /* 右电机 A 相电流幅值 */
    float rightB;   /* 右电机 B 相电流幅值 */
    float left;     /* 左电机相电流幅值 = max(leftA, leftB) */
    float right;    /* 右电机相电流幅值 = max(rightA, rightB) */
} rise_current_peak_t;

/* 抬升电流峰值捕获窗口的默认值/上下限（毫秒）。 */
#define RISE_CURRENT_PEAK_DEFAULT_MS  300U
#define RISE_CURRENT_PEAK_MIN_MS       20U
#define RISE_CURRENT_PEAK_MAX_MS     2000U

/* 抬升单通道高速突发采样（12 位原始 ADC 计数），用于看相电流真实波形。
 * 仅供调试口按需调用：进入时独占 TMUX1308 共用地址线并等待 1 ms 稳定，
 * 采样期间不再切地址、不 yield。调用者必须在优先级高于 getState 的任务里
 * 执行（getState 每 20 ms 会全通道扫描并改地址），否则采样点会串通道。 */
typedef struct {
    uint8_t  ch;         /* 0=左A 1=左B 2=右A 3=右B */
    uint16_t zero;       /* 该通道零基线原始计数：I(A)=(raw-zero)*0.0008 */
    uint32_t elapsedUs;  /* 整段实际耗时(us)，用于算真实采样率 */
} rise_wave_info_t;

/* 单次突发采样的最大点数（缓冲区由调用者提供，建议从 FreeRTOS 堆申请）。 */
#define RISE_WAVE_MAX_SAMPLES  2048U

/* 在 buf 里连续采 n 个 12 位原始 ADC 点，返回实际点数。
 * intervalUs=0 表示不加间隔、尽量快；否则按该间隔节拍采样。
 * 参数非法或 buf 为空返回 0；info 可为 NULL。 */
uint16_t currentCaptureRiseWave(uint8_t idx, uint16_t *buf, uint16_t n,
                                uint16_t intervalUs, rise_wave_info_t *info);

/* 全局最近一次采样结果；首次调用采样函数前各字段为 0。 */
/* 全部电流通道的最新测量结果。 */
extern current_data_t currentData;

/* 清空所有电流采样缓存。 */
void currentInit(void);

/* ADC1_IN4：读取母线电流，更新 busRaw/busAmpere，返回安培值。 */
float currentReadBus(void);
/* A0=0，读取 ADC1_IN10 的 LED 电流，更新原始值并返回安培值。 */
float currentReadLed(void);
/* A0=1，读取 ADC1_IN10 的抱闸电流，更新原始值并返回安培值。 */
float currentReadBrake(void);
/* PB0 / ADC1_IN8：读取应变片电流，更新原始值并返回安培值。 */
float currentReadStrainGage(void);
/* 以下四路从 TMUX1308 D1 / ADC1_IN13 读取，更新原始值并返回安培值。 */
/* 换算时减去零电流基线，返回有符号绕组电流（可正可负）。 */
/* 右抬升电机线圈 B：枚举 riseMoterRCurrB，输入 AD-M12。 */
float currentReadRiseRightB(void);
/* 右抬升电机线圈 A：枚举 riseMoterRCurrA，输入 AD-M11。 */
float currentReadRiseRightA(void);
/* 左抬升电机线圈 B：枚举 riseMoterLCurrB，输入 AD-M22。 */
float currentReadRiseLeftB(void);
/* 左抬升电机线圈 A：枚举 riseMoterLCurrA，输入 AD-M21。 */
float currentReadRiseLeftA(void);

/* 标定抬升四路电流的零电流基线：电机静止（零电流）时调用，每路平均多次。 */
void currentCalibrateRiseZero(void);
/* 读取抬升四路电流的零电流基线 ADC 原始值，顺序：左A,左B,右A,右B。 */
void currentRiseZeroGetArray(uint16_t zeros[4]);
/* 在 windowMs 毫秒窗口内高速循环采样抬升四路电流，返回各相峰值。
 * 返回 1 成功；0 参数非法（out 为空）。窗口会钳到
 * [RISE_CURRENT_PEAK_MIN_MS, RISE_CURRENT_PEAK_MAX_MS]。
 * 注意：本函数独占 TMUX1308 地址线并阻塞调用者整个窗口（几十~几百 ms），
 * 应在电机静止标定或运行观测时通过调试口按需调用，不要放进高频周期任务。 */
uint8_t currentCaptureRisePeak(uint16_t windowMs, rise_current_peak_t *out);

/*
 * 依次更新全部七路电流数据。
 * 函数会切换 TMUX1308 共用 A0/A1/A2；返回时地址线停在最后采样的通道。
 */
/* 刷新 currentData 中的全部电流测量值。 */
void currentUpdateAll(void);

#endif
