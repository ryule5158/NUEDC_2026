/**
 * @file    adc_app.c
 * @brief   STM32 片上 ADC+DMA 采样驱动 — 双模式实现
 * @note
 *   【模式 A — 单次 (ONESHOT)】
 *     Init → Start → DMA 采满 4096 点 → 回调停止 DMA 置标志
 *     → Process() 检测标志 → 转换电压 → 用户手动 Start 下一轮
 *     DMA 模式: NORMAL, 每轮采完停止
 *
 *   【模式 B — 连续 (CONTINUOUS)】
 *     Init → Start → DMA 循环搬运 4096 点不停
 *     → 到半满 (0~2047)  触发 HalfCpltCallback → 置半区标志
 *     → 到全满 (2048~4095) 触发 ConvCpltCallback → 置半区标志
 *     → Process() 轮询半区 → 转换电压（可选同步保护，需 ConfigSync 启用）
 *
 *   两种模式在 Init 时选定，之后不切换，内部代码路径完全隔离。
 */

#include "adc_app.h"
#include "adc.h"
#include "tim.h"
#include <stddef.h>
#include <stdio.h>
#include <string.h>

/* ================================================================ */
/*  ADC硬件适配宏                                                     */
/* ================================================================ */

/* 当前CubeMX使用ADC1；若以后换ADC外设，只需要同步修改句柄和实例 */
#define ADC_APP_HAL_HANDLE              hadc1

/* 当前CubeMX使用ADC1；HAL回调中用它判断是不是本驱动的ADC */
#define ADC_APP_HAL_INSTANCE            ADC1

/* 当前PC4/PC5为ADC1单端输入；若以后改成差分输入，把这里改为ADC_DIFFERENTIAL_ENDED */
#define ADC_APP_SINGLE_DIFF             ADC_SINGLE_ENDED

/* 当前ADC分辨率为16位，单端换算时65535对应Vref */
#define ADC_APP_FULL_SCALE_CODE         65535.0f

/* 差分换算时32768为零点；当前单端模式不会使用，仅为以后切换保留 */
#define ADC_APP_DIFF_ZERO_CODE          32768.0f

/* DMA实际搬运长度；双通道扫描时DMA数据按Rank1,Rank2交错排列 */
#define ADC_APP_FRAME_DMA_LENGTH        (ADC_APP_SAMPLE_LENGTH * ADC_APP_CHANNEL_COUNT)
#define ADC_APP_MODE_B_DMA_LENGTH       (ADC_APP_FRAME_DMA_LENGTH * 2U)
#define ADC_APP_DMA_LENGTH              ADC_APP_MODE_B_DMA_LENGTH

/* DMA原始缓冲区字节数 */
#define ADC_APP_DMA_BYTES               (ADC_APP_DMA_LENGTH * sizeof(uint16_t))

/* Cortex-M7 DCache缓存行大小，DMA缓冲区按32字节对齐处理 */
#define ADC_APP_CACHE_LINE_SIZE         32U

/* 默认分析通道：0=规则Rank1(PC4/ADC1_INP4)，1=规则Rank2(PC5/ADC1_INP8) */
#define ADC_APP_DEFAULT_CHANNEL         0U

#if defined(__ARMCC_VERSION)
#define ADC_APP_DMA_BUFFER_ATTR         __attribute__((section(".bss.adc_dma_buffer"), aligned(32))) /* DMA缓冲区放入可访问SRAM并32字节对齐。 */
#else
#define ADC_APP_DMA_BUFFER_ATTR         __attribute__((section(".bss.adc_dma_buffer"), aligned(32))) /* DMA缓冲区放入可访问SRAM并32字节对齐。 */
#endif

#define ADC_APP_MODE_B_FIRST_MASK       0x01U       /* 连续采样前半区待处理位。 */
#define ADC_APP_MODE_B_SECOND_MASK      0x02U       /* 连续采样后半区待处理位。 */
#define ADC_APP_DTCM_START              0x20000000UL /* DMA不可访问DTCM的起始地址。 */
#define ADC_APP_DTCM_END                0x20020000UL /* DMA不可访问DTCM的结束地址。 */

static volatile uint8_t s_voltage_ready = 0U;        /* 电压缓冲区已更新标志。 */
static volatile uint8_t s_mode_b_pending_mask = 0U;  /* 连续模式待处理半区位图。 */
static uint32_t s_last_raw_start_idx = 0U;           /* 最近转换的DMA原始数据起始下标。 */

static uint8_t s_adc_calibrated = 0U;  /* ADC校准完成标志：0=未校准，1=已校准，避免反复进入校准流程 */

/* ================================================================ */
/*  静态变量 — 公共                                                   */
/* ================================================================ */

static ADC_SampleMode_t   s_mode;                                      /* 当前采样模式 */

static uint16_t s_raw_buffer[ADC_APP_DMA_LENGTH] ADC_APP_DMA_BUFFER_ATTR; /* DMA原始ADC码值，双通道时按Rank1/Rank2交错 */
static float    s_voltage_buffer[ADC_APP_CHANNEL_COUNT][ADC_APP_SAMPLE_LENGTH]; /* 各通道换算后的电压值 */

static float    s_vref_v            = 3.3f;                            /* ADC 参考电压 (V) */
static float    s_sample_rate_hz    = 20000.0f;                        /* 实际ADC采样率(Hz) */
static uint8_t  s_active_channel    = ADC_APP_DEFAULT_CHANNEL;         /* 当前默认分析通道 */

/* ================================================================ */
/*  静态变量 — 模式 A (单次)                                           */
/* ================================================================ */
static volatile uint8_t s_mode_a_ready = 0U;                           /* 一整帧 DMA 完成标志 */

/* ================================================================ */
/*  静态变量 — 模式 B (连续)                                           */
/* ================================================================ */
static volatile ADC_HalfReady_t s_mode_b_half = ADC_HALF_NONE;         /* 半区就绪标志 */
static volatile uint8_t        s_sync_enabled;                         /* 同步保护开关 (0=关 1=开) */
static volatile uint8_t        s_consumer_locked;                      /* 消费者正在读数据 */
static volatile uint8_t        s_overrun_count;                        /* DMA 冲掉未消费数据的次数 */

/* ================================================================ */
/*  前向声明                                                         */
/* ================================================================ */

static uint32_t          GetTim3ClockHz(void);
static HAL_StatusTypeDef ConfigTimerSampleRate(float sample_rate_hz);
static HAL_StatusTypeDef ConfigDmaMode(uint32_t dma_mode);
static void              ConvertRawToVoltage(uint32_t raw_start_idx,
                                             uint32_t voltage_start_idx,
                                             uint32_t count);
static float             ConvertCodeToVoltage(uint16_t code);
static void              InvalidateRawBufferCache(uint32_t start_idx, uint32_t count);
static void              CleanInvalidateRawBufferCache(uint32_t start_idx, uint32_t count);
static void              PrepareRawBufferForDma(void);
static uint8_t           IsRawBufferDmaAccessible(void);
static uint8_t           IsDCacheEnabled(void);
static void              SetModeBPending(ADC_HalfReady_t half);
static ADC_HalfReady_t   PopModeBPending(void);
static void              ResetModeBPending(void);
static int32_t           VoltageToMillivolt(float voltage);

/* ---- 模式专用前向声明 ---- */
static HAL_StatusTypeDef StartModeA(void);
static HAL_StatusTypeDef StartModeB(void);
static HAL_StatusTypeDef ProcessModeA(void);
static HAL_StatusTypeDef ProcessModeB(void);

/* ================================================================ */
/*  公共 API — 初始化                                                 */
/* ================================================================ */

/**
 * @brief  初始化 ADC 采样模块
 * @param  vref_v             ADC 参考电压 (V)
 * @param  sample_rate_hz     期望采样率 (Hz)
 * @param  max_harmonic_hz    最高谐波频率 (Hz)
 * @param  print_raw          0=不打印原始点, 非0=打印
 * @param  mode               采样模式
 * @return HAL_OK / HAL_ERROR
 */
HAL_StatusTypeDef ADC_Sample_Init(float             vref_v,
                                  float             sample_rate_hz,
                                  float             max_harmonic_hz,
                                  uint8_t           print_raw,
                                  ADC_SampleMode_t  mode)
{
    /* ---- 参数校验 ---- */
    if (vref_v <= 0.0f || sample_rate_hz <= 0.0f || max_harmonic_hz <= 0.0f) {
        printf("[ADC] Init Error: invalid params\r\n");
        return HAL_ERROR;
    }

    /* ---- 保存参数 ---- */
    s_mode            = mode;
    s_vref_v          = vref_v;
    (void)print_raw;  /* 保留参数兼容性，当前默认 Process 不打印 */
    s_mode_a_ready    = 0U;
    s_mode_b_half     = ADC_HALF_NONE;
    ResetModeBPending();
    s_sync_enabled    = 0U;
    s_consumer_locked = 0U;
    s_overrun_count   = 0U;
    s_active_channel  = ADC_APP_DEFAULT_CHANNEL;
    s_voltage_ready   = 0U;
    s_last_raw_start_idx = 0U;

    // memset(s_raw_buffer, 0, sizeof(s_raw_buffer));       // BSS 已归零，不需要
    // memset(s_voltage_buffer, 0, sizeof(s_voltage_buffer)); // BSS 已归零，不需要

    (void)HAL_ADC_Stop_DMA(&ADC_APP_HAL_HANDLE);
    (void)HAL_ADC_Stop(&ADC_APP_HAL_HANDLE);

    /* ---- 停止 TIM3，重新配置采样率 ---- */
    (void)HAL_TIM_Base_Stop(&htim3);

    if (ConfigTimerSampleRate(sample_rate_hz) != HAL_OK) {
        printf("[ADC] Init Error: sample rate config failed\r\n");
        return HAL_ERROR;
    }

    (void)max_harmonic_hz; /* FFT分析上限由DSP/ADC_FFT.*维护，BSP层不依赖DSP。 */

    /* ---- ADC 校准 ---- */
    if (s_adc_calibrated == 0U) {
        if (HAL_ADCEx_Calibration_Start(&ADC_APP_HAL_HANDLE, ADC_CALIB_OFFSET,
                                        ADC_APP_SINGLE_DIFF) != HAL_OK) {
            printf("[ADC] Init Error: ADC calibration failed, adc_err=0x%08lX\r\n",
                   (unsigned long)HAL_ADC_GetError(&ADC_APP_HAL_HANDLE));
            return HAL_ERROR;
        }

        s_adc_calibrated = 1U;
    }

    return HAL_OK;
}

/* ================================================================ */
/*  公共 API — 启动 / 停止                                            */
/* ================================================================ */

/* 按初始化时选定的模式启动ADC和DMA。 */
HAL_StatusTypeDef ADC_Sample_Start(void)
{
    s_mode_a_ready = 0U;
    s_mode_b_half  = ADC_HALF_NONE;
    ResetModeBPending();
    s_voltage_ready = 0U;
    s_last_raw_start_idx = 0U;

    if (s_mode == ADC_MODE_CONTINUOUS) {
        return StartModeB();
    } else {
        return StartModeA();
    }
}

/* 配置连续模式的消费者同步保护。 */
void ADC_Sample_ConfigSync(uint8_t enable)
{
    s_sync_enabled    = (enable != 0U) ? 1U : 0U;
    s_consumer_locked = 0U;
    s_overrun_count   = 0U;
}

/* 停止ADC DMA与TIM3触发并清除就绪标志。 */
void ADC_Sample_Stop(void)
{
    (void)HAL_ADC_Stop_DMA(&ADC_APP_HAL_HANDLE);
    (void)HAL_TIM_Base_Stop(&htim3);
    s_mode_a_ready = 0U;
    s_mode_b_half  = ADC_HALF_NONE;
    ResetModeBPending();
    s_voltage_ready = 0U;
}

/* ================================================================ */
/*  公共 API — 主循环轮询                                             */
/* ================================================================ */

/* 默认弱定义轮询入口，用户可在业务层覆盖。 */
__weak void ADC_Sample_Process(void)
{
    (void)ADC_Sample_ConvertReadyData();
}

/* 将已就绪的DMA原始码转换到电压缓冲区。 */
HAL_StatusTypeDef ADC_Sample_ConvertReadyData(void)
{
    if (s_mode == ADC_MODE_CONTINUOUS) {
        return ProcessModeB();
    }

    return ProcessModeA();
}

/* ================================================================ */
/*  公共 API — 模式 A 查询                                           */
/* ================================================================ */

/* 查询单次模式的整帧完成标志。 */
uint8_t ADC_Sample_DataReady(void)
{
    return (s_mode_a_ready != 0U) ? 1U : 0U;
}

/* ================================================================ */
/*  公共 API — 模式 B 查询                                           */
/* ================================================================ */

/* 取出连续模式下一个待处理半区。 */
ADC_HalfReady_t ADC_Sample_GetReadyHalf(void)
{
    return PopModeBPending();
}

/* 锁定消费者正在读取的电压缓冲区。 */
void ADC_Sample_BufferLock(void)
{
    if (s_sync_enabled) {
        s_consumer_locked = 1U;
    }
}

/* 解除消费者缓冲区锁定。 */
void ADC_Sample_BufferUnlock(void)
{
    s_consumer_locked = 0U;
}

/* 返回并清零连续模式的覆盖计数。 */
uint8_t ADC_Sample_IsOverrun(void)
{
    uint8_t ret = s_overrun_count;
    s_overrun_count = 0U;
    return ret;
}

/* ================================================================ */
/*  公共 API — 数据访问                                              */
/* ================================================================ */

/* 返回DMA原始ADC码缓冲区。 */
const uint16_t* ADC_Sample_GetRawData(void)
{
    return s_raw_buffer;
}

/* 返回当前分析通道的电压缓冲区。 */
const float* ADC_Sample_GetVoltageData(void)
{
    return s_voltage_buffer[s_active_channel];
}

/* 返回指定ADC规则通道的电压缓冲区。 */
const float* ADC_Sample_GetVoltageDataByChannel(uint8_t channel)
{
    if (channel >= ADC_APP_CHANNEL_COUNT) {
        return NULL;
    }

    return s_voltage_buffer[channel];
}

/* 设置默认分析和打印的ADC通道。 */
HAL_StatusTypeDef ADC_Sample_SetActiveChannel(uint8_t channel)
{
    if (channel >= ADC_APP_CHANNEL_COUNT) {
        return HAL_ERROR;
    }

    s_active_channel = channel;
    return HAL_OK;
}

/* 获取当前默认分析通道。 */
uint8_t ADC_Sample_GetActiveChannel(void)
{
    return s_active_channel;
}

/* 获取TIM3实际产生的ADC采样率。 */
float ADC_Sample_GetSampleRateHz(void)
{
    return s_sample_rate_hz;
}

/* 查询是否已完成至少一批电压转换。 */
uint8_t ADC_Sample_VoltageReady(void)
{
    return (s_voltage_ready != 0U) ? 1U : 0U;
}

/* ================================================================ */
/*  HAL 回调 — 模式 A 使用 ADC_ConvCpltCallback                       */
/*           模式 B 使用 HalfCplt + ConvCplt                         */
/* ================================================================ */

/**
 * @brief  DMA 传输完成回调 (两种模式都用到)
 *   - 模式 A: 一轮 4096 点全部采完，停止 DMA，置 ready 标志
 *   - 模式 B: 后半区 (2048~4095) 采完，置 HALF_SECOND 标志，DMA 不停止
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc == NULL || hadc->Instance != ADC_APP_HAL_INSTANCE) {
        return;
    }

    if (s_mode == ADC_MODE_ONESHOT) {
        /* ---- 模式 A: 停止 DMA，通知主循环 ---- */
        (void)HAL_TIM_Base_Stop(&htim3);
        s_mode_a_ready = 1U;
    } else {
        /* ---- 模式 B: 后半区就绪，DMA 继续循环 ---- */
        SetModeBPending(ADC_HALF_SECOND);
    }
}

/**
 * @brief  DMA 半传输完成回调 (仅模式 B 使用)
 *   前半区 (0~2047) 采完，置 HALF_FIRST 标志，DMA 继续。
 *   模式 A 因为 DMA_NORMAL 模式不会触发此回调。
 */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc == NULL || hadc->Instance != ADC_APP_HAL_INSTANCE) {
        return;
    }

    /* 仅模式 B 会进入这里（模式 A 的 DMA_NORMAL 不产生半传输中断） */
    if (s_mode == ADC_MODE_CONTINUOUS) {
        SetModeBPending(ADC_HALF_FIRST);
    }
}

/* 处理本驱动ADC的HAL错误回调。 */
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc == NULL || hadc->Instance != ADC_APP_HAL_INSTANCE) {
        return;
    }

    printf("[ADC] Error callback, adc_err=0x%08lX\r\n",
           (unsigned long)HAL_ADC_GetError(hadc));
}

/* 处理本驱动DMA通道的HAL错误回调。 */
void HAL_DMA_ErrorCallback(DMA_HandleTypeDef *hdma)
{
    if ((hdma == NULL) || (ADC_APP_HAL_HANDLE.DMA_Handle != hdma)) {
        return;
    }

    printf("[ADC] DMA error callback, dma_err=0x%08lX\r\n",
           (unsigned long)HAL_DMA_GetError(hdma));
}

/* ================================================================ */
/*  ========  模式 A 内部实现（单次 DMA） ========                    */
/* ================================================================ */

/**
 * @brief  模式 A 启动: 单次 DMA，采满 4096 点后自动停止
 */
static HAL_StatusTypeDef StartModeA(void)
{
    HAL_StatusTypeDef status;

    (void)HAL_TIM_Base_Stop(&htim3);
    (void)HAL_ADC_Stop_DMA(&ADC_APP_HAL_HANDLE);

    /* 配置 ADC+DMA 为单次模式 */
    ADC_APP_HAL_HANDLE.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_ONESHOT;
    status = HAL_ADC_Init(&ADC_APP_HAL_HANDLE);
    if (status != HAL_OK) {
        printf("[ADC-A] ADC init error, adc_err=0x%08lX\r\n",
               (unsigned long)HAL_ADC_GetError(&ADC_APP_HAL_HANDLE));
        return status;
    }

    status = ConfigDmaMode(DMA_NORMAL);
    if (status != HAL_OK) {
        printf("[ADC-A] DMA mode config error\r\n");
        return status;
    }

    /* 先启动ADC+DMA等待外部触发，再启动TIM3 TRGO，避免漏掉触发 */
    s_mode_a_ready = 0U;
    if (IsRawBufferDmaAccessible() == 0U) {
        printf("[ADC-A] DMA buffer is in DTCM, move it to AXI/D2 SRAM\r\n");
        return HAL_ERROR;
    }
    PrepareRawBufferForDma();
    status = HAL_ADC_Start_DMA(&ADC_APP_HAL_HANDLE,
                               (uint32_t *)s_raw_buffer,
                               ADC_APP_FRAME_DMA_LENGTH);
    if (status != HAL_OK) {
        printf("[ADC-A] DMA start error, adc_err=0x%08lX, dma_err=0x%08lX\r\n",
               (unsigned long)HAL_ADC_GetError(&ADC_APP_HAL_HANDLE),
               (unsigned long)HAL_DMA_GetError(ADC_APP_HAL_HANDLE.DMA_Handle));
        return status;
    }

    __HAL_TIM_SET_COUNTER(&htim3, 0U);
    if (HAL_TIM_Base_Start(&htim3) != HAL_OK) {
        printf("[ADC-A] TIM3 start error\r\n");
        (void)HAL_ADC_Stop_DMA(&ADC_APP_HAL_HANDLE);
        return HAL_ERROR;
    }

    return status;
}

/**
 * @brief  模式 A 主循环处理 (默认弱实现): 等待就绪 → 转换电压
 * @note   不重启 DMA，用户需手动调用 ADC_Sample_Start() 启动下一轮
 */
static HAL_StatusTypeDef ProcessModeA(void)
{
    if (s_mode_a_ready == 0U) {
        return HAL_BUSY;
    }

    s_mode_a_ready = 0U;

    /* 一整帧就绪: 转换 ADC 码值 → 电压 */
    InvalidateRawBufferCache(0U, ADC_APP_SAMPLE_LENGTH);
    s_last_raw_start_idx = 0U;
    ConvertRawToVoltage(0U, 0U, ADC_APP_SAMPLE_LENGTH);
    s_voltage_ready = 1U;

    return HAL_OK;
}

/* ================================================================ */
/*  ========  模式 B 内部实现（循环 DMA + 双缓冲） ========           */
/* ================================================================ */

/* 每个循环DMA半区对应每通道一帧完整分析数据。 */
#define MODE_B_HALF_LENGTH  ADC_APP_SAMPLE_LENGTH

/**
 * @brief  模式 B 启动: 循环 DMA，永不停，双缓冲乒乓
 */
static HAL_StatusTypeDef StartModeB(void)
{
    HAL_StatusTypeDef status;

    (void)HAL_TIM_Base_Stop(&htim3);
    (void)HAL_ADC_Stop_DMA(&ADC_APP_HAL_HANDLE);

    /* 配置 ADC+DMA 为循环模式 */
    ADC_APP_HAL_HANDLE.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
    status = HAL_ADC_Init(&ADC_APP_HAL_HANDLE);
    if (status != HAL_OK) {
        printf("[ADC-B] ADC init error, adc_err=0x%08lX\r\n",
               (unsigned long)HAL_ADC_GetError(&ADC_APP_HAL_HANDLE));
        return status;
    }

    status = ConfigDmaMode(DMA_CIRCULAR);
    if (status != HAL_OK) {
        printf("[ADC-B] DMA mode config error\r\n");
        return status;
    }

    /* 先启动ADC+DMA等待外部触发，再启动TIM3 TRGO，避免漏掉触发 */
    s_mode_b_half  = ADC_HALF_NONE;
    ResetModeBPending();
    if (IsRawBufferDmaAccessible() == 0U) {
        printf("[ADC-B] DMA buffer is in DTCM, move it to AXI/D2 SRAM\r\n");
        return HAL_ERROR;
    }
    PrepareRawBufferForDma();
    status = HAL_ADC_Start_DMA(&ADC_APP_HAL_HANDLE,
                               (uint32_t *)s_raw_buffer,
                               ADC_APP_MODE_B_DMA_LENGTH);
    if (status != HAL_OK) {
        printf("[ADC-B] DMA start error, adc_err=0x%08lX, dma_err=0x%08lX\r\n",
               (unsigned long)HAL_ADC_GetError(&ADC_APP_HAL_HANDLE),
               (unsigned long)HAL_DMA_GetError(ADC_APP_HAL_HANDLE.DMA_Handle));
        return status;
    }

    __HAL_TIM_SET_COUNTER(&htim3, 0U);
    if (HAL_TIM_Base_Start(&htim3) != HAL_OK) {
        printf("[ADC-B] TIM3 start error\r\n");
        (void)HAL_ADC_Stop_DMA(&ADC_APP_HAL_HANDLE);
        return HAL_ERROR;
    }

    return status;
}

/**
 * @brief  模式 B 主循环处理 (默认弱实现): 轮询半区就绪 → 转换电压
 * @note   不执行 FFT 和打印，用户重写 Process() 可加入自己的分析逻辑
 *
 *  前半区 (0~2047) 就绪:  转换该半区电压
 *  后半区 (2048~4095) 就绪: 转换该半区电压（此时整帧完整，用户可自行 FFT）
 */
static HAL_StatusTypeDef ProcessModeB(void)
{
    ADC_HalfReady_t half = PopModeBPending();

    if (half == ADC_HALF_NONE) {
        return HAL_BUSY;
    }

    /* 同步模式：消费者还在读上一批数据 → 丢弃本次通知，记录 overrun */
    if (s_sync_enabled && s_consumer_locked) {
        s_overrun_count++;
        return HAL_BUSY;
    }

    /* 消费标志 */
    if (half == ADC_HALF_FIRST) {
        InvalidateRawBufferCache(0U, MODE_B_HALF_LENGTH);
        s_last_raw_start_idx = 0U;
        ConvertRawToVoltage(0U, 0U, MODE_B_HALF_LENGTH);
    } else {
        InvalidateRawBufferCache(MODE_B_HALF_LENGTH, MODE_B_HALF_LENGTH);
        s_last_raw_start_idx = MODE_B_HALF_LENGTH;
        ConvertRawToVoltage(MODE_B_HALF_LENGTH, 0U, MODE_B_HALF_LENGTH);
    }

    s_voltage_ready = 1U;
    return HAL_OK;
}

/* ================================================================ */
/*  内部工具 — TIM3 时钟频率获取                                      */
/* ================================================================ */

/* 根据APB1分频获取TIM3实际输入时钟。 */
static uint32_t GetTim3ClockHz(void)
{
    RCC_ClkInitTypeDef clk_cfg;
    uint32_t           flash_latency;
    uint32_t           timer_clock_hz;

    HAL_RCC_GetClockConfig(&clk_cfg, &flash_latency);
    timer_clock_hz = HAL_RCC_GetPCLK1Freq();

    /* APB1 不是 1 分频时，定时器时钟是 PCLK1 × 2 */
    if (clk_cfg.APB1CLKDivider != RCC_APB1_DIV1) {
        timer_clock_hz *= 2U;
    }

    return timer_clock_hz;
}

/* ================================================================ */
/*  内部工具 — 配置 TIM3 触发采样率                                    */
/* ================================================================ */

/* 计算并写入TIM3分频参数以获得目标采样率。 */
static HAL_StatusTypeDef ConfigTimerSampleRate(float sample_rate_hz)
{
    uint32_t timer_clock_hz;
    uint32_t target_ticks;
    uint32_t prescaler;
    uint32_t period_count;

    timer_clock_hz = GetTim3ClockHz();

    if (timer_clock_hz == 0U) {
        return HAL_ERROR;
    }

    /* 采样率不能超过定时器时钟的一半 */
    if (sample_rate_hz > (float)timer_clock_hz / 2.0f) {
        return HAL_ERROR;
    }

    /* 计算分频值: target_ticks = timer_clock / sample_rate */
    target_ticks = (uint32_t)((float)timer_clock_hz / sample_rate_hz + 0.5f);

    if (target_ticks < 2U) {
        return HAL_ERROR;
    }

    /* 两级分频: prescaler × period */
    prescaler = (target_ticks - 1U) / 65536U;
    if (prescaler > 65535U) {
        return HAL_ERROR;
    }

    period_count = target_ticks / (prescaler + 1U);
    if (period_count == 0U || period_count > 65536U) {
        return HAL_ERROR;
    }

    /* 写入 TIM3 寄存器 */
    htim3.Init.Prescaler = prescaler;
    htim3.Init.Period    = period_count - 1U;
    __HAL_TIM_SET_PRESCALER(&htim3, prescaler);
    __HAL_TIM_SET_AUTORELOAD(&htim3, period_count - 1U);
    __HAL_TIM_SET_COUNTER(&htim3, 0U);
    htim3.Instance->EGR = TIM_EGR_UG;

    /* 计算实际采样率，供上层DSP分析模块读取 */
    s_sample_rate_hz = (float)timer_clock_hz /
                       ((float)(prescaler + 1U) * (float)period_count);

    return HAL_OK;
}

/* ================================================================ */
/*  内部工具 — ADC 码值 → 电压值                                       */
/* ================================================================ */
/*  当前CubeMX为16位单端ADC：0对应0V，65535对应Vref；差分模式也预留了换算公式 */

/* 在单次与循环之间配置ADC的DMA运行模式。 */
static HAL_StatusTypeDef ConfigDmaMode(uint32_t dma_mode)
{
    DMA_HandleTypeDef *hdma = ADC_APP_HAL_HANDLE.DMA_Handle;

    if (hdma == NULL) {
        return HAL_ERROR;
    }

    if (hdma->Init.Mode == dma_mode) {
        return HAL_OK;
    }

    hdma->Init.Mode = dma_mode;
    return HAL_DMA_Init(hdma);
}

/* 将指定区间的交错ADC码换算为各通道电压。 */
static void ConvertRawToVoltage(uint32_t raw_start_idx,
                                uint32_t voltage_start_idx,
                                uint32_t count)
{
    for (uint32_t i = 0U; i < count; i++) {
        uint32_t raw_sample_index = raw_start_idx + i;
        uint32_t voltage_index = voltage_start_idx + i;
        uint32_t raw_index = raw_sample_index * ADC_APP_CHANNEL_COUNT;

        for (uint32_t ch = 0U; ch < ADC_APP_CHANNEL_COUNT; ch++) {
            s_voltage_buffer[ch][voltage_index] =
                ConvertCodeToVoltage(s_raw_buffer[raw_index + ch]);
        }
    }
}

/* 按单端或差分模式将一个ADC码换算为电压。 */
static float ConvertCodeToVoltage(uint16_t code)
{
    if (ADC_APP_SINGLE_DIFF == ADC_SINGLE_ENDED) {
        return (float)code * s_vref_v / ADC_APP_FULL_SCALE_CODE;
    }

    return ((float)code - ADC_APP_DIFF_ZERO_CODE) * s_vref_v / ADC_APP_DIFF_ZERO_CODE;
}

/* 采样后使指定DMA缓冲区的DCache失效。 */
static void InvalidateRawBufferCache(uint32_t start_idx, uint32_t count)
{
    uint32_t start_addr = (uint32_t)&s_raw_buffer[start_idx * ADC_APP_CHANNEL_COUNT];
    uint32_t byte_count = count * ADC_APP_CHANNEL_COUNT * sizeof(uint16_t);
    uint32_t aligned_addr = start_addr & ~(ADC_APP_CACHE_LINE_SIZE - 1U);
    uint32_t end_addr = start_addr + byte_count;
    uint32_t aligned_end = (end_addr + ADC_APP_CACHE_LINE_SIZE - 1U) &
                           ~(ADC_APP_CACHE_LINE_SIZE - 1U);

    if (IsDCacheEnabled() == 0U) {
        return;
    }

    SCB_InvalidateDCache_by_Addr((uint32_t *)aligned_addr,
                                 (int32_t)(aligned_end - aligned_addr));
}

/* DMA启动前清理并使指定缓冲区的DCache失效。 */
static void CleanInvalidateRawBufferCache(uint32_t start_idx, uint32_t count)
{
    uint32_t start_addr = (uint32_t)&s_raw_buffer[start_idx * ADC_APP_CHANNEL_COUNT];
    uint32_t byte_count = count * ADC_APP_CHANNEL_COUNT * sizeof(uint16_t);
    uint32_t aligned_addr = start_addr & ~(ADC_APP_CACHE_LINE_SIZE - 1U);
    uint32_t end_addr = start_addr + byte_count;
    uint32_t aligned_end = (end_addr + ADC_APP_CACHE_LINE_SIZE - 1U) &
                           ~(ADC_APP_CACHE_LINE_SIZE - 1U);

    if (IsDCacheEnabled() == 0U) {
        return;
    }

    SCB_CleanInvalidateDCache_by_Addr((uint32_t *)aligned_addr,
                                      (int32_t)(aligned_end - aligned_addr));
}

/* 在DMA接管前同步整个原始采样缓冲区。 */
static void PrepareRawBufferForDma(void)
{
    CleanInvalidateRawBufferCache(0U, ADC_APP_DMA_LENGTH / ADC_APP_CHANNEL_COUNT);
}

/* 检查原始采样缓冲区是否避开DMA不可访问的DTCM。 */
static uint8_t IsRawBufferDmaAccessible(void)
{
    uint32_t start_addr = (uint32_t)s_raw_buffer;
    uint32_t end_addr = start_addr + ADC_APP_DMA_BYTES;

    if ((start_addr < ADC_APP_DTCM_END) && (end_addr > ADC_APP_DTCM_START)) {
        return 0U;
    }

    return 1U;
}

/* 查询Cortex-M7数据缓存是否已启用。 */
static uint8_t IsDCacheEnabled(void)
{
    return ((SCB->CCR & SCB_CCR_DC_Msk) != 0U) ? 1U : 0U;
}

/* 在DMA回调中标记一个待处理半区。 */
static void SetModeBPending(ADC_HalfReady_t half)
{
    uint8_t mask;

    if (half == ADC_HALF_FIRST) {
        mask = ADC_APP_MODE_B_FIRST_MASK;
    } else if (half == ADC_HALF_SECOND) {
        mask = ADC_APP_MODE_B_SECOND_MASK;
    } else {
        return;
    }

    if ((s_mode_b_pending_mask & mask) != 0U) {
        s_overrun_count++;
    }

    s_mode_b_pending_mask |= mask;
    s_mode_b_half = half;
}

/* 在短临界区内取出并清除一个待处理半区。 */
static ADC_HalfReady_t PopModeBPending(void)
{
    ADC_HalfReady_t half = ADC_HALF_NONE;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    if ((s_mode_b_pending_mask & ADC_APP_MODE_B_FIRST_MASK) != 0U) {
        s_mode_b_pending_mask &= (uint8_t)~ADC_APP_MODE_B_FIRST_MASK;
        half = ADC_HALF_FIRST;
    } else if ((s_mode_b_pending_mask & ADC_APP_MODE_B_SECOND_MASK) != 0U) {
        s_mode_b_pending_mask &= (uint8_t)~ADC_APP_MODE_B_SECOND_MASK;
        half = ADC_HALF_SECOND;
    }

    if ((s_mode_b_pending_mask & ADC_APP_MODE_B_FIRST_MASK) != 0U) {
        s_mode_b_half = ADC_HALF_FIRST;
    } else if ((s_mode_b_pending_mask & ADC_APP_MODE_B_SECOND_MASK) != 0U) {
        s_mode_b_half = ADC_HALF_SECOND;
    } else {
        s_mode_b_half = ADC_HALF_NONE;
    }

    if ((primask & 1U) == 0U) {
        __enable_irq();
    }

    return half;
}

/* 原子清除连续模式的所有半区通知。 */
static void ResetModeBPending(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    s_mode_b_pending_mask = 0U;
    s_mode_b_half = ADC_HALF_NONE;
    if ((primask & 1U) == 0U) {
        __enable_irq();
    }
}

/* ================================================================ */
/*  内部工具 — 打印                                                   */
/* ================================================================ */

/* 将浮点伏特值四舍五入为整数毫伏。 */
static int32_t VoltageToMillivolt(float voltage)
{
    if (voltage >= 0.0f) {
        return (int32_t)(voltage * 1000.0f + 0.5f);
    }

    return (int32_t)(voltage * 1000.0f - 0.5f);
}

/* 逐行打印当前通道的原始ADC码。 */
void ADC_Sample_PrintRaw(void)
{
    HAL_StatusTypeDef status = ADC_Sample_ConvertReadyData();

    if ((status == HAL_ERROR) ||
        ((status == HAL_BUSY) && (s_voltage_ready == 0U))) {
        printf("[ADC] raw data not ready\r\n");
        return;
    }

    for (uint32_t i = 0U; i < ADC_APP_SAMPLE_LENGTH; i++) {
        uint32_t raw_index = (s_last_raw_start_idx + i) * ADC_APP_CHANNEL_COUNT + s_active_channel;
        printf("%u\r\n", (unsigned int)s_raw_buffer[raw_index]);
    }
}

/* 逐行打印任意浮点数组。 */
void ADC_Sample_PrintArray(const float *data, uint32_t len)
{
    if (data == NULL) return;
    for (uint32_t i = 0U; i < len; i++) {
        printf("%.3f\r\n", (double)data[i]);
    }
}

/* 逐行打印当前通道的电压样本。 */
void ADC_Sample_PrintVoltage(void)
{
    HAL_StatusTypeDef status = ADC_Sample_ConvertReadyData();

    if ((status == HAL_ERROR) ||
        ((status == HAL_BUSY) && (s_voltage_ready == 0U))) {
        printf("[ADC] voltage data not ready\r\n");
        return;
    }

    for (uint32_t i = 0U; i < ADC_APP_SAMPLE_LENGTH; i++) {
        int32_t millivolt = VoltageToMillivolt(s_voltage_buffer[s_active_channel][i]);
        int32_t whole = millivolt / 1000;
        int32_t frac = millivolt % 1000;

        if (frac < 0) {
            frac = -frac;
        }

        if ((millivolt < 0) && (whole == 0)) {
            printf("-0.%03ld\r\n", (long)frac);
        } else {
            printf("%ld.%03ld\r\n", (long)whole, (long)frac);
        }
    }
}

