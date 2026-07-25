#ifndef __ADC_APP_H
#define __ADC_APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

#define ADC_APP_MAX_SAMPLE_COUNT 4096U /* 每通道最大采样点数。 */
#define ADC_APP_CHANNEL_COUNT       2U /* ADC1规则序列通道数。 */

/*
 * 当前PC4、PC5由同一个ADC1规则序列依次转换，并非同时采样。
 * 若赛题需要双通道相位差，须在CubeMX中改为ADC1+ADC2双ADC同步模式。
 */

/* ADC采集模式。 */
typedef enum {
    ADC_MODE_ONESHOT = 0,    /* 采满一帧后停止。 */
    ADC_MODE_CONTINUOUS = 1, /* DMA循环双半区连续采集。 */
} ADC_SampleMode_t;

/* ADC电压换算质量模式。 */
typedef enum {
    ADC_QUALITY_RAW = 0,     /* 保留原始波形，适合FFT和相位分析。 */
    ADC_QUALITY_MEDIAN3 = 1, /* 三点中值，适合慢变精密测量。 */
} ADC_QualityMode_t;

/* 连续采集半区状态。 */
typedef enum {
    ADC_HALF_NONE = 0,   /* 没有半区就绪。 */
    ADC_HALF_FIRST = 1,  /* 前半区就绪。 */
    ADC_HALF_SECOND = 2, /* 后半区就绪。 */
} ADC_HalfReady_t;

/* ADC错误位。 */
typedef enum {
    ADC_ERROR_NONE = 0U,       /* 无错误。 */
    ADC_ERROR_PERIPHERAL = 1U, /* ADC外设错误。 */
    ADC_ERROR_DMA = 2U,        /* DMA传输错误。 */
} ADC_ErrorFlag_t;

/* ADC运行参数。 */
typedef struct {
    float measured_vref_v;           /* 实测参考电压，仅用于码值换算。 */
    float sample_rate_hz;            /* 每通道目标采样率。 */
    uint16_t sample_count;           /* 每通道一帧采样点数。 */
    uint8_t active_channel;          /* 默认分析通道，0为PC4，1为PC5。 */
    uint8_t sync_enable;             /* 连续模式电压缓冲区保护开关。 */
    ADC_SampleMode_t mode;           /* 单帧或连续采集模式。 */
    ADC_QualityMode_t quality;       /* 原始或三点中值质量模式。 */
} ADC_SampleConfig_t;

/* 获取适合波形采集的默认配置。 */
void ADC_Sample_GetDefaultConfig(ADC_SampleConfig_t *config);

/* 初始化ADC采样库。 */
HAL_StatusTypeDef ADC_Sample_Init(const ADC_SampleConfig_t *config);

/* 启动一次或连续ADC采集。 */
HAL_StatusTypeDef ADC_Sample_Start(void);

/* 停止ADC、DMA和触发定时器。 */
void ADC_Sample_Stop(void);

/* 在主循环中换算一批已就绪数据。 */
__weak void ADC_Sample_Process(void);

/* 将一批已就绪原始码换算到电压缓冲区。 */
HAL_StatusTypeDef ADC_Sample_ConvertReadyData(void);

/* 查询是否至少完成过一次电压换算。 */
uint8_t ADC_Sample_VoltageReady(void);

/* 配置连续模式的电压缓冲区保护。 */
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

/* 返回并清除ADC和DMA错误位。 */
uint32_t ADC_Sample_GetAndClearError(void);

/* 查询单帧采集是否完成。 */
uint8_t ADC_Sample_DataReady(void);

/* 查看连续模式下一个就绪半区，不清除通知。 */
ADC_HalfReady_t ADC_Sample_GetReadyHalf(void);

/* 锁定连续模式电压缓冲区。 */
void ADC_Sample_BufferLock(void);

/* 解除连续模式电压缓冲区锁定。 */
void ADC_Sample_BufferUnlock(void);

/* 返回并清除连续模式覆盖次数。 */
uint8_t ADC_Sample_IsOverrun(void);

/* 获取双通道交错排列的DMA原始码。 */
const uint16_t *ADC_Sample_GetRawData(void);

/* 获取默认通道电压缓冲区。 */
const float *ADC_Sample_GetVoltageData(void);

/* 获取指定通道电压缓冲区，参数非法时返回空指针。 */
const float *ADC_Sample_GetVoltageDataByChannel(uint8_t channel);

/* 打印默认通道原始ADC码。 */
void ADC_Sample_PrintRaw(void);

/* 打印默认通道电压样本。 */
void ADC_Sample_PrintVoltage(void);

/* 打印指定浮点数组。 */
void ADC_Sample_PrintArray(const float *data, uint32_t length);

#ifdef __cplusplus
}
#endif

#endif /* __ADC_APP_H */
