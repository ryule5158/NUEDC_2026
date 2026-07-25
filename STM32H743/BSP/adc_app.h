/**
 * @file    adc_app.h
 * @brief   STM32 片上 ADC+DMA 采样驱动 — 双模式（单次/连续）
 * @note    模式 A (ONESHOT)：  单缓冲区，DMA 采满后自动停，适合"采一帧→分析→再采"
 *          模式 B (CONTINUOUS)：DMA 循环+双缓冲，无间断连续采集，适合实时流处理
 * @date    2026-06-17 V2
 */

#ifndef __ADC_APP_H
#define __ADC_APP_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ---------------------------------------------------------- */
#include "main.h"
#include <stdint.h>

/* 常量 -------------------------------------------------------------- */
#define ADC_APP_SAMPLE_LENGTH            4096U        /* 每个ADC通道的采样点数 */
#define ADC_APP_CHANNEL_COUNT            2U           /* Rank1=PC4/ADC1_INP4，Rank2=PC5/ADC1_INP8 */
#define ADC_APP_PRINT_RAW_OFF            0U           /* print_raw 参数：不打印原始采样点 */
#define ADC_APP_PRINT_RAW_ON             1U           /* print_raw 参数：打印原始采样点 */

/* 模式选择 ---------------------------------------------------------- */
typedef enum {
    ADC_MODE_ONESHOT    = 0,   /* 模式 A：单缓冲，DMA 满后自动停，手动重启 */
    ADC_MODE_CONTINUOUS = 1,   /* 模式 B：DMA 循环+双缓冲，无间断持续采集 */
} ADC_SampleMode_t;

/* 半区就绪标志 (仅模式 B 使用) -------------------------------------- */
typedef enum {
    ADC_HALF_NONE   = 0,       /* 无半区就绪 */
    ADC_HALF_FIRST  = 1,       /* 模式B前半DMA就绪：每通道4096点，可直接FFT */
    ADC_HALF_SECOND = 2,       /* 模式B后半DMA就绪：每通道4096点，可直接FFT */
} ADC_HalfReady_t;

/* ---- 公共 API ---------------------------------------------------- */

/**
 * @brief  初始化 ADC 采样模块（两种模式通用）
 * @param  vref_v              ADC 参考电压 (V)，如 3.3
 * @param  sample_rate_hz      期望采样率 (Hz)，内部自动调整 TIM3 分频
 * @param  max_harmonic_hz     最高谐波统计频率 (Hz)，超出 Fs/2 会自动截断
 * @param  print_raw           是否打印原始采样点 (ADC_APP_PRINT_RAW_ON/OFF)
 * @param  mode                采样模式 (ADC_MODE_ONESHOT / ADC_MODE_CONTINUOUS)
 * @return HAL_OK 成功，其他值失败
 */
HAL_StatusTypeDef ADC_Sample_Init(float             vref_v,
                                  float             sample_rate_hz,
                                  float             max_harmonic_hz,
                                  uint8_t           print_raw,
                                  ADC_SampleMode_t  mode);

/**
 * @brief  启动 DMA 采样（模式 A 启动一轮，模式 B 启动循环）
 * @return HAL_OK 成功
 */
HAL_StatusTypeDef ADC_Sample_Start(void);

/**
 * @brief  停止 DMA 采样
 */
void ADC_Sample_Stop(void);

/**
 * @brief  主循环轮询 — 弱定义，默认只转换 ADC 码值为电压，不做 FFT，不重启 DMA
 * @note   用户可重写此函数以加入自己的处理逻辑。
 *         默认行为：
 *           模式 A：帧完成 → 转换电压 → 清零标志（需用户手动 ADC_Sample_Start() 启动下一轮）
 *           模式 B：半区就绪 → 转换该半区电压 → 清零标志（DMA 循环不停）
 *         FFT 分析、打印等由 DSP/ADC_FFT.* 完成，BSP层只负责采样和电压换算
 */
__weak void ADC_Sample_Process(void);

/**
 * @brief  将已就绪的DMA原始码转换到电压缓冲区。
 * @return 转换新数据返回HAL_OK，无数据返回HAL_BUSY，状态异常返回HAL_ERROR。
 */
HAL_StatusTypeDef ADC_Sample_ConvertReadyData(void);

/** @brief 至少一批原始码已转换为电压时返回1。 */
uint8_t ADC_Sample_VoltageReady(void);

/** @brief 启用/禁用同步保护（模式 B），默认禁用。启用后 Lock/Unlock/IsOverrun 生效 */
void ADC_Sample_ConfigSync(uint8_t enable);

/** @brief 设置默认分析通道，0表示ADC规则Rank1，1表示ADC规则Rank2 */
HAL_StatusTypeDef ADC_Sample_SetActiveChannel(uint8_t channel);

/** @brief 获取当前默认分析通道编号 */
uint8_t ADC_Sample_GetActiveChannel(void);

/** @brief 获取TIM3实际配置出的ADC采样率(Hz) */
float ADC_Sample_GetSampleRateHz(void);

/* ---- 模式 A 专用：查询单帧是否就绪 ------------------------------- */

/**
 * @brief  查询一轮采样是否完成（仅模式 A 有效）
 * @return 0 = 未完成，非 0 = 已完成，数据可读
 */
uint8_t ADC_Sample_DataReady(void);

/* ---- 模式 B 专用：查询双缓冲半区 -------------------------------- */

/**
 * @brief  查询哪个半缓冲区就绪（仅模式 B 有效）
 * @return ADC_HALF_NONE / ADC_HALF_FIRST / ADC_HALF_SECOND
 * @note   调用后自动清零，下次同一半区就绪会再次触发
 */
ADC_HalfReady_t ADC_Sample_GetReadyHalf(void);

/**
 * @brief  消费者锁住缓冲区（需先 ADC_Sample_ConfigSync(1) 启用同步）
 * @note   锁住后 Process() 丢弃新半区通知并累加 overrun。必须与 Unlock 配对。
 */
void ADC_Sample_BufferLock(void);

/** @brief  消费者解锁缓冲区 */
void ADC_Sample_BufferUnlock(void);

/**
 * @brief  查询 overrun 次数（需先启用同步），调用后自动清零
 * @return overrun 次数（同步未启用时始终返回 0）
 */
uint8_t ADC_Sample_IsOverrun(void);

/* ---- 数据访问（两种模式通用） ------------------------------------ */

/** @brief 获取DMA原始ADC缓冲区指针，双通道时数据按Rank1,Rank2,Rank1,Rank2交错排列 */
const uint16_t* ADC_Sample_GetRawData(void);

/** @brief 获取当前默认通道的电压值缓冲区指针（float，长度 ADC_APP_SAMPLE_LENGTH） */
const float*    ADC_Sample_GetVoltageData(void);

/** @brief 获取指定通道的电压值缓冲区指针，channel=0对应Rank1，channel=1对应Rank2 */
const float*    ADC_Sample_GetVoltageDataByChannel(uint8_t channel);

/** @brief printf 打印当前默认通道的电压缓冲区原始采样点 */
void ADC_Sample_PrintRaw(void);

/** @brief 逐行打印当前通道的电压样本。 */
void ADC_Sample_PrintVoltage(void);

/** @brief 逐行打印任意浮点数组。 */
void ADC_Sample_PrintArray(const float *data, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif /* __ADC_APP_H */
