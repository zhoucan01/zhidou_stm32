#include "current.h"

/* 电流采集：刷新共享 currentData 中各通道的换算电流值。 */
#include "wk_adc.h"
#include "tmux1308.h"
#include <string.h>

/* wk_timebase_get() 工程里没有公共头声明，按 usrDebug.c 的做法就地补一个。 */
extern uint32_t wk_timebase_get(void);

/* 将 ADC 原始采样值换算成电流，并保存到 currentData。 */
/* 各路 ADC 计数到安培的换算系数。 */
#define BUS_CURRENT_AMP_PER_COUNT 0.000806f
#define A0_CURRENT_AMP_PER_COUNT 0.000745f
#define RISE_CURRENT_AMP_PER_COUNT 0.0008f
/* PB0 / ADC1_IN8 电流换算系数，单位为 A/ADC计数。 */
#define STRAIN_GAGE_AMP_PER_COUNT 1.0f

/* 所有电流通道的最新原始值和换算值，上电时自动清零。 */
current_data_t currentData;

/* 抬升四路电流的零电流基线（ADC 原始值），换算前先减去它得到有符号电流。 */
/* 数组下标约定：0=左A 1=左B 2=右A 3=右B。 */
#define RISE_ZERO_LEFT_A   0U
#define RISE_ZERO_LEFT_B   1U
#define RISE_ZERO_RIGHT_A  2U
#define RISE_ZERO_RIGHT_B  3U
#define RISE_ZERO_SAMPLES  20U
/* 峰值捕获时每个通道连续读取的点数（地址不变，省去逐点稳定等待）。 */
#define RISE_CAPTURE_CH_READS  8U

static uint16_t riseCurrentZero[4];

/* 抬升四路电流对应的 TMUX1308 D1 通道号，顺序同 riseCurrentZero[]。 */
static const uint8_t riseCurrentMuxCh[4] = {
    riseMoterLCurrA, riseMoterLCurrB, riseMoterRCurrA, riseMoterRCurrB
};

/* 减去零电流基线，把 ADC 原始值换算成有符号绕组电流（A）。 */
static float riseCurrentFromRaw(uint16_t raw, uint8_t idx)
{
    int32_t signedCounts = (int32_t)raw - (int32_t)riseCurrentZero[idx];
    return (float)signedCounts * RISE_CURRENT_AMP_PER_COUNT;
}

void currentInit(void)
{
    /* 只清空软件缓存，不在初始化阶段阻塞执行多通道 ADC 转换。 */
    memset(&currentData, 0, sizeof(currentData));
}

/* 切换 ADC 通道后丢弃首次转换，降低前一通道残余电荷的影响。 */
static uint16_t currentReadAdc(adc_channel_select_type adc_channel)
{
    (void)adc_read_channel(adc_channel);
    return adc_read_channel(adc_channel);
}

/* 母线电流：PA4 / ADC1_IN4；原始读数与安培值一起更新。 */
float currentReadBus(void)
{
    currentData.busRaw = currentReadAdc(ADC_CHANNEL_4);
    currentData.busAmpere = currentData.busRaw * BUS_CURRENT_AMP_PER_COUNT;
    return currentData.busAmpere;
}

/* LED 电流：A0=0，A1=A2=0；等待地址线稳定后读 PC0 / ADC1_IN10。 */
float currentReadLed(void)
{
    tmux1308SetAddress(0U);
    currentData.ledRaw = currentReadAdc(ADC_CHANNEL_10);
    currentData.ledAmpere = currentData.ledRaw * A0_CURRENT_AMP_PER_COUNT;
    return currentData.ledAmpere;
}

/* 电机抱闸电流：A0=1，A1=A2=0；与 LED 共用 ADC1_IN10。 */
float currentReadBrake(void)
{
    tmux1308SetAddress(1U);
    currentData.brakeRaw = currentReadAdc(ADC_CHANNEL_10);
    currentData.brakeAmpere = currentData.brakeRaw * A0_CURRENT_AMP_PER_COUNT;
    return currentData.brakeAmpere;
}

/* 应变片电流：PB0 / ADC1_IN8。 */
float currentReadStrainGage(void)
{
    currentData.strainGageRaw = currentReadAdc(ADC_CHANNEL_8);
    currentData.strainGageAmpere =
        currentData.strainGageRaw * STRAIN_GAGE_AMP_PER_COUNT;
    return currentData.strainGageAmpere;
}

/* 抬升电机四路电流使用 tmux1308Data1 枚举选择 D1 输入。 */
float currentReadRiseRightB(void)
{
    currentData.riseRightBRaw = tmux1308Data1Values[riseMoterRCurrB];
    currentData.riseRightBAmpere =
        riseCurrentFromRaw(currentData.riseRightBRaw, RISE_ZERO_RIGHT_B);
    return currentData.riseRightBAmpere;
}

float currentReadRiseRightA(void)
{
    currentData.riseRightARaw = tmux1308Data1Values[riseMoterRCurrA];
    currentData.riseRightAAmpere =
        riseCurrentFromRaw(currentData.riseRightARaw, RISE_ZERO_RIGHT_A);
    return currentData.riseRightAAmpere;
}

float currentReadRiseLeftB(void)
{
    currentData.riseLeftBRaw = tmux1308Data1Values[riseMoterLCurrB];
    currentData.riseLeftBAmpere =
        riseCurrentFromRaw(currentData.riseLeftBRaw, RISE_ZERO_LEFT_B);
    return currentData.riseLeftBAmpere;
}

float currentReadRiseLeftA(void)
{
    currentData.riseLeftARaw = tmux1308Data1Values[riseMoterLCurrA];
    currentData.riseLeftAAmpere =
        riseCurrentFromRaw(currentData.riseLeftARaw, RISE_ZERO_LEFT_A);
    return currentData.riseLeftAAmpere;
}

/* 标定抬升四路电流的零电流基线：电机静止（零电流）时调用，每路平均多次。 */
void currentCalibrateRiseZero(void)
{
    uint8_t idx;
    uint8_t n;

    for (idx = 0U; idx < 4U; idx++)
    {
        uint32_t sum = 0U;

        for (n = 0U; n < RISE_ZERO_SAMPLES; n++)
        {
            tmux1308SetAddress(riseCurrentMuxCh[idx]);
            (void)adc_read_channel(ADC_CHANNEL_13);   /* 丢弃切换后的首次转换 */
            sum += adc_read_channel(ADC_CHANNEL_13);
        }
        riseCurrentZero[idx] = (uint16_t)(sum / RISE_ZERO_SAMPLES);
    }
}

void currentRiseZeroGetArray(uint16_t zeros[4])
{
    uint8_t idx;

    if (zeros == NULL)
    {
        return;
    }
    for (idx = 0U; idx < 4U; idx++)
    {
        zeros[idx] = riseCurrentZero[idx];
    }
}

/* 在 windowMs 窗口内高速循环采样抬升四路电流，返回各相峰值。
 * 每个通道：切换地址并等待稳定后连续读 RISE_CAPTURE_CH_READS 次；地址不变
 * 期间免去逐点稳定等待。整段窗口覆盖多个电周期，因此能抓到正弦峰值。 */
uint8_t currentCaptureRisePeak(uint16_t windowMs, rise_current_peak_t *out)
{
    uint32_t peakCounts[4];
    uint32_t startMs;
    uint8_t idx;
    uint8_t n;

    if (out == NULL)
    {
        return 0U;
    }
    if (windowMs < RISE_CURRENT_PEAK_MIN_MS)
    {
        windowMs = RISE_CURRENT_PEAK_MIN_MS;
    }
    if (windowMs > RISE_CURRENT_PEAK_MAX_MS)
    {
        windowMs = RISE_CURRENT_PEAK_MAX_MS;
    }

    for (idx = 0U; idx < 4U; idx++)
    {
        peakCounts[idx] = 0U;
    }

    startMs = wk_timebase_get();
    do
    {
        for (idx = 0U; idx < 4U; idx++)
        {
            uint32_t best = peakCounts[idx];

            tmux1308SetAddress(riseCurrentMuxCh[idx]);
            (void)adc_read_channel(ADC_CHANNEL_13);   /* 丢弃切换后的首次转换 */
            for (n = 0U; n < RISE_CAPTURE_CH_READS; n++)
            {
                uint16_t raw = adc_read_channel(ADC_CHANNEL_13);
                int32_t counts = (int32_t)raw - (int32_t)riseCurrentZero[idx];

                if (counts < 0)
                {
                    counts = -counts;
                }
                if ((uint32_t)counts > best)
                {
                    best = (uint32_t)counts;
                }
            }
            peakCounts[idx] = best;
        }
    } while ((wk_timebase_get() - startMs) < (uint32_t)windowMs);

    out->leftA  = (float)peakCounts[RISE_ZERO_LEFT_A]  * RISE_CURRENT_AMP_PER_COUNT;
    out->leftB  = (float)peakCounts[RISE_ZERO_LEFT_B]  * RISE_CURRENT_AMP_PER_COUNT;
    out->rightA = (float)peakCounts[RISE_ZERO_RIGHT_A] * RISE_CURRENT_AMP_PER_COUNT;
    out->rightB = (float)peakCounts[RISE_ZERO_RIGHT_B] * RISE_CURRENT_AMP_PER_COUNT;
    out->left   = (out->leftA  > out->leftB)  ? out->leftA  : out->leftB;
    out->right  = (out->rightA > out->rightB) ? out->rightA : out->rightB;
    return 1U;
}

/* DWT 周期计数：给突发采样做微秒级计时（与 tmux1308DelayUs 同一套）。 */
static void riseWaveEnableDwt(void)
{
    if ((CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) == 0U)
    {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    }
    if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0U)
    {
        DWT->CYCCNT = 0U;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }
}

static uint32_t riseWaveNowUs(void)
{
    return (uint32_t)(DWT->CYCCNT / (system_core_clock / 1000000U));
}

/* 抬升单通道高速突发采样：地址固定在一个通道上，背靠背读 ADC，不做通道轮转。
 * 调用者必须在优先级高于 getState 的任务里（本工程是 usrDebug 任务），
 * 这样 20 ms 的全通道扫描不会插进来改共用地址线。全程不 yield，不回显。 */
uint16_t currentCaptureRiseWave(uint8_t idx, uint16_t *buf, uint16_t n,
                                uint16_t intervalUs, rise_wave_info_t *info)
{
    uint16_t i;
    uint32_t t0;
    uint32_t ts;

    if (buf == NULL || idx >= 4U || n == 0U)
    {
        return 0U;
    }
    if (n > RISE_WAVE_MAX_SAMPLES)
    {
        n = RISE_WAVE_MAX_SAMPLES;
    }

    riseWaveEnableDwt();
    /* 内含 1 ms 稳定等待；之后地址保持不变，靠 RC 已经稳定。 */
    tmux1308SetAddress(riseCurrentMuxCh[idx]);

    t0 = riseWaveNowUs();
    for (i = 0U; i < n; i++)
    {
        ts = riseWaveNowUs();
        buf[i] = adc_read_channel(ADC_CHANNEL_13);
        if (intervalUs != 0U)
        {
            while ((uint32_t)(riseWaveNowUs() - ts) < (uint32_t)intervalUs)
            {
            }
        }
    }

    if (info != NULL)
    {
        info->ch = idx;
        info->zero = riseCurrentZero[idx];
        info->elapsedUs = riseWaveNowUs() - t0;
    }
    return n;
}

/* 更新顺序：母线、LED、抱闸、右 B、右 A、左 B、左 A。 */
void currentUpdateAll(void)
{
    (void)currentReadBus();
    (void)currentReadLed();
    (void)currentReadBrake();
    (void)currentReadStrainGage();
    (void)currentReadRiseRightB();
    (void)currentReadRiseRightA();
    (void)currentReadRiseLeftB();
    (void)currentReadRiseLeftA();
}
