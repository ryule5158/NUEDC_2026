#ifndef ADC_APP_H
#define ADC_APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

#define ADC_APP_MAX_SAMPLE_COUNT     512U      /* 每通道最大采样点数。 */
#define ADC_APP_CHANNEL_COUNT        2U        /* 同步采样的ADC通道数。 */
#define ADC_APP_MIN_SAMPLE_RATE_HZ   1000.0f   /* 定时器支持的最低采样率。 */
#define ADC_APP_MAX_SAMPLE_RATE_HZ   1000000.0f /* 当前配置允许的最高采样率。 */

/* ADC通道编号。 */
typedef enum {
    ADC_APP_CHANNEL_VOLTAGE = 0, /* ADC0/PB25电压通道。 */
    ADC_APP_CHANNEL_CURRENT = 1, /* ADC1/PB17电流通道。 */
} ADC_AppChannel_t;

/* ADC采集模式。 */
typedef enum {
    ADC_MODE_ONESHOT = 0,   /* 采满一帧后停止。 */
    ADC_MODE_CONTINUOUS = 1, /* DMA循环双半区连续采集。 */
} ADC_SampleMode_t;

/* ADC电压换算质量模式。 */
typedef enum {
    ADC_QUALITY_RAW = 0,     /* 保留原始采样，适合高速波形和FFT。 */
    ADC_QUALITY_MEDIAN3 = 1, /* 三点中值抑制MSPM0 ADC偶发跳码。 */
} ADC_QualityMode_t;

/* 连续采集半区状态。 */
typedef enum {
    ADC_HALF_NONE = 0,  /* 没有半区就绪。 */
    ADC_HALF_FIRST = 1, /* 前半区就绪。 */
    ADC_HALF_SECOND = 2, /* 后半区就绪。 */
} ADC_HalfReady_t;

/* ADC运行参数。 */
typedef struct {
    float measured_vref_v;          /* 实测参考电压，仅用于码值换算。 */
    float sample_rate_hz;           /* 每通道目标采样率。 */
    uint16_t sample_count;          /* 每通道一帧采样点数。 */
    ADC_AppChannel_t active_channel; /* 默认分析和打印通道。 */
    ADC_SampleMode_t mode;          /* 单帧或连续采集模式。 */
    ADC_QualityMode_t quality;      /* 原始或三点中值质量模式。 */
    uint8_t print_raw;              /* 非零时由轮询入口打印原始码。 */
} ADC_SampleConfig_t;

/* 获取适合波形采集的默认配置。 */
void ADC_Sample_GetDefaultConfig(ADC_SampleConfig_t *config);

/* 初始化ADC采样库。 */
HAL_StatusTypeDef ADC_Sample_Init(const ADC_SampleConfig_t *config);

/* 启动一次或连续ADC采集。 */
HAL_StatusTypeDef ADC_Sample_Start(void);

/* 停止ADC和DMA。 */
void ADC_Sample_Stop(void);

/* 在主循环中换算已就绪数据并按配置打印。 */
void ADC_Sample_Process(void);

/* 配置连续模式的覆盖检测。 */
void ADC_Sample_ConfigSync(uint8_t enable);

/* 设置默认分析通道。 */
HAL_StatusTypeDef ADC_Sample_SetActiveChannel(uint8_t channel);

/* 获取默认分析通道。 */
uint8_t ADC_Sample_GetActiveChannel(void);

/* 获取定时器实际采样率。 */
float ADC_Sample_GetSampleRateHz(void);

/* 获取当前每通道采样点数。 */
uint16_t ADC_Sample_GetSampleCount(void);

/* 查询ADC是否正在采集。 */
uint8_t ADC_Sample_IsRunning(void);

/* 查询当前模式是否有数据就绪。 */
uint8_t ADC_Sample_DataReady(void);

/* 清除单帧数据就绪标志。 */
void ADC_Sample_Acknowledge(void);

/* 取出连续模式下一个就绪半区。 */
ADC_HalfReady_t ADC_Sample_GetReadyHalf(void);

/* 获取最近取出半区的起始位置。 */
uint16_t ADC_Sample_GetReadyOffset(void);

/* 获取最近取出半区的每通道长度。 */
uint16_t ADC_Sample_GetReadyLength(void);

/* 获取已完成半区或帧的序号。 */
uint32_t ADC_Sample_GetFrameSequence(void);

/* 锁定当前连续采集半区供消费者读取。 */
void ADC_Sample_BufferLock(void);

/* 解除连续采集半区锁定。 */
void ADC_Sample_BufferUnlock(void);

/* 返回并清除连续采集覆盖次数。 */
uint8_t ADC_Sample_IsOverrun(void);

/* 查询DMA是否出现地址或数据错误。 */
uint8_t ADC_Sample_HasError(void);

/* 获取默认通道原始码缓冲区。 */
const uint16_t *ADC_Sample_GetRawData(void);

/* 获取指定通道原始码缓冲区，参数非法时返回空指针。 */
const uint16_t *ADC_Sample_GetRawDataByChannel(uint8_t channel);

/* 获取指定通道最近就绪半区的原始码。 */
const uint16_t *ADC_Sample_GetReadyRawDataByChannel(uint8_t channel);

/* 获取默认通道电压缓冲区。 */
const float *ADC_Sample_GetVoltageData(void);

/* 获取指定通道电压缓冲区，参数非法时返回空指针。 */
const float *ADC_Sample_GetVoltageDataByChannel(uint8_t channel);

/* 将指定连续采集半区换算为电压。 */
void ADC_Sample_ConvertHalf(ADC_HalfReady_t half);

/* 打印默认通道原始ADC码。 */
void ADC_Sample_PrintRaw(void);

#ifdef __cplusplus
}
#endif

#endif /* ADC_APP_H */
