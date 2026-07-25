/* STM32H743片上ADC1、TIM3和DMA单帧/连续采样驱动。 */

#include "adc_app.h"
#include "adc.h"
#include "tim.h"
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#define ADC_APP_HAL_HANDLE              hadc1     /* CubeMX生成的ADC句柄。 */
#define ADC_APP_HAL_INSTANCE            ADC1      /* 本驱动使用的ADC实例。 */
#define ADC_APP_SINGLE_DIFF             ADC_SINGLE_ENDED /* 当前单端输入模式。 */
#define ADC_APP_FULL_SCALE_CODE         65535.0f  /* 16位单端最大码值。 */
#define ADC_APP_DIFF_ZERO_CODE          32768.0f  /* 16位差分零点码值。 */
#define ADC_APP_MAX_FRAME_DMA_LENGTH                                \
    (ADC_APP_MAX_SAMPLE_COUNT * ADC_APP_CHANNEL_COUNT) /* 单帧最大DMA长度。 */
#define ADC_APP_MAX_DMA_LENGTH                                      \
    (ADC_APP_MAX_FRAME_DMA_LENGTH * 2U) /* 连续模式最大DMA长度。 */
#define ADC_APP_DMA_BYTES                                           \
    (ADC_APP_MAX_DMA_LENGTH * sizeof(uint16_t)) /* DMA缓冲区字节数。 */
#define ADC_APP_CACHE_LINE_SIZE         32U       /* M7缓存行字节数。 */
#define ADC_APP_DEFAULT_CHANNEL          0U       /* 默认分析PC4通道。 */
#define ADC_APP_DEFAULT_VREF_V           3.3f     /* 默认实测参考电压。 */
#define ADC_APP_DEFAULT_SAMPLE_RATE_HZ 20000.0f   /* 默认每通道采样率。 */
#define ADC_APP_MAX_ADC_CLOCK_HZ    10000000U     /* LQFP176无丢码建议上限。 */
#define ADC_APP_CONVERSION_CYCLES       17.0f     /* 16位加8.5周期采样总周期。 */

#if defined(__ARMCC_VERSION)
#define ADC_APP_DMA_BUFFER_ATTR         __attribute__((section(".bss.adc_dma_buffer"), aligned(32))) /* DMA缓冲区放入可访问SRAM并32字节对齐。 */
#else
#define ADC_APP_DMA_BUFFER_ATTR         __attribute__((section(".bss.adc_dma_buffer"), aligned(32))) /* DMA缓冲区放入可访问SRAM并32字节对齐。 */
#endif

#define ADC_APP_DTCM_START              0x20000000UL /* DMA不可访问DTCM的起始地址。 */
#define ADC_APP_DTCM_END                0x20020000UL /* DMA不可访问DTCM的结束地址。 */

static uint16_t s_raw_buffer[ADC_APP_MAX_DMA_LENGTH] ADC_APP_DMA_BUFFER_ATTR; /* DMA双通道交错原始码。 */
static float s_voltage_buffer[ADC_APP_CHANNEL_COUNT][ADC_APP_MAX_SAMPLE_COUNT]; /* 各通道电压值。 */
static float s_vref_v = ADC_APP_DEFAULT_VREF_V;                 /* 实测参考电压。 */
static float s_sample_rate_hz = ADC_APP_DEFAULT_SAMPLE_RATE_HZ; /* 实际每通道采样率。 */
static uint16_t s_sample_count = ADC_APP_MAX_SAMPLE_COUNT;      /* 每通道采样点数。 */
static uint32_t s_last_raw_start_idx;                           /* 最近处理的原始帧起点。 */
static uint8_t s_active_channel = ADC_APP_DEFAULT_CHANNEL;      /* 默认分析通道。 */
static uint8_t s_adc_calibrated;                               /* ADC校准完成标志。 */
static uint8_t s_initialized;                                  /* 驱动初始化完成标志。 */
static ADC_SampleMode_t s_mode = ADC_MODE_ONESHOT;              /* 当前采集模式。 */
static ADC_QualityMode_t s_quality = ADC_QUALITY_RAW;           /* 当前质量模式。 */
static volatile uint8_t s_running;                              /* ADC运行标志。 */
static volatile uint8_t s_voltage_ready;                        /* 电压数据就绪标志。 */
static volatile uint8_t s_mode_a_ready;                         /* 单帧DMA完成标志。 */
static volatile ADC_HalfReady_t s_mode_b_pending = ADC_HALF_NONE; /* 连续模式最新半区。 */
static volatile uint8_t s_sync_enabled;                         /* 电压缓冲区保护开关。 */
static volatile uint8_t s_consumer_locked;                      /* 电压缓冲区锁定标志。 */
static volatile uint8_t s_overrun_count;                        /* 连续数据覆盖次数。 */
static volatile uint32_t s_error_flags;                         /* ADC和DMA错误位。 */

static uint32_t          GetTim3ClockHz(void);
static uint32_t          GetAdcClockHz(void);
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
static uint16_t          Median3(uint16_t a, uint16_t b, uint16_t c);
static void              SetModeBPending(ADC_HalfReady_t half);
static ADC_HalfReady_t   PeekModeBPending(void);
static ADC_HalfReady_t   PopModeBPending(void);
static void              ResetModeBPending(void);
static int32_t           VoltageToMillivolt(float voltage);

static HAL_StatusTypeDef StartModeA(void);
static HAL_StatusTypeDef StartModeB(void);
static HAL_StatusTypeDef ProcessModeA(void);
static HAL_StatusTypeDef ProcessModeB(void);

/* ================================================================ */
/*  公共 API — 初始化                                                 */
/* ================================================================ */

void ADC_Sample_GetDefaultConfig(ADC_SampleConfig_t *config)
{
    if (config == NULL) {
        return;
    }

    config->measured_vref_v = ADC_APP_DEFAULT_VREF_V;
    config->sample_rate_hz = ADC_APP_DEFAULT_SAMPLE_RATE_HZ;
    config->sample_count = ADC_APP_MAX_SAMPLE_COUNT;
    config->active_channel = ADC_APP_DEFAULT_CHANNEL;
    config->sync_enable = 0U;
    config->mode = ADC_MODE_ONESHOT;
    config->quality = ADC_QUALITY_RAW;
}

HAL_StatusTypeDef ADC_Sample_Init(const ADC_SampleConfig_t *config)
{
    uint32_t adc_clock_hz;
    float max_sample_rate_hz;

    if ((config == NULL) ||
        (config->measured_vref_v <= 0.0f) ||
        (config->sample_rate_hz <= 0.0f) ||
        (config->sample_count == 0U) ||
        (config->sample_count > ADC_APP_MAX_SAMPLE_COUNT) ||
        (config->active_channel >= ADC_APP_CHANNEL_COUNT) ||
        ((config->mode != ADC_MODE_ONESHOT) &&
         (config->mode != ADC_MODE_CONTINUOUS)) ||
        ((config->quality != ADC_QUALITY_RAW) &&
         (config->quality != ADC_QUALITY_MEDIAN3))) {
        return HAL_ERROR;
    }

    s_initialized = 0U;
    adc_clock_hz = GetAdcClockHz();
    max_sample_rate_hz =
        (float)adc_clock_hz /
        (ADC_APP_CONVERSION_CYCLES * (float)ADC_APP_CHANNEL_COUNT);
    if ((adc_clock_hz == 0U) ||
        (adc_clock_hz > ADC_APP_MAX_ADC_CLOCK_HZ) ||
        (config->sample_rate_hz > max_sample_rate_hz)) {
        return HAL_ERROR;
    }

    s_mode = config->mode;
    s_quality = config->quality;
    s_vref_v = config->measured_vref_v;
    s_sample_count = config->sample_count;
    s_active_channel = config->active_channel;
    s_sync_enabled = (config->sync_enable != 0U) ? 1U : 0U;
    s_mode_a_ready = 0U;
    ResetModeBPending();
    s_consumer_locked = 0U;
    s_overrun_count = 0U;
    s_voltage_ready = 0U;
    s_last_raw_start_idx = 0U;
    s_error_flags = ADC_ERROR_NONE;
    s_running = 0U;

    (void)HAL_ADC_Stop_DMA(&ADC_APP_HAL_HANDLE);
    (void)HAL_ADC_Stop(&ADC_APP_HAL_HANDLE);
    (void)HAL_TIM_Base_Stop(&htim3);

    if (ConfigTimerSampleRate(config->sample_rate_hz) != HAL_OK) {
        return HAL_ERROR;
    }

    if (s_adc_calibrated == 0U) {
        if (HAL_ADCEx_Calibration_Start(&ADC_APP_HAL_HANDLE, ADC_CALIB_OFFSET,
                                        ADC_APP_SINGLE_DIFF) != HAL_OK) {
            return HAL_ERROR;
        }
        s_adc_calibrated = 1U;
    }

    s_initialized = 1U;
    return HAL_OK;
}

/* ================================================================ */
/*  公共 API — 启动 / 停止                                            */
/* ================================================================ */

/* 按初始化时选定的模式启动ADC和DMA。 */
HAL_StatusTypeDef ADC_Sample_Start(void)
{
    HAL_StatusTypeDef status;

    if ((s_initialized == 0U) || (s_running != 0U)) {
        return HAL_ERROR;
    }

    s_mode_a_ready = 0U;
    ResetModeBPending();
    s_voltage_ready = 0U;
    s_last_raw_start_idx = 0U;
    s_error_flags = ADC_ERROR_NONE;

    if (s_mode == ADC_MODE_CONTINUOUS) {
        status = StartModeB();
    } else {
        status = StartModeA();
    }
    if (status == HAL_OK) {
        s_running = 1U;
    }
    return status;
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
    ResetModeBPending();
    s_voltage_ready = 0U;
    s_running = 0U;
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

/* 查看连续模式下一个待处理半区，不消费通知。 */
ADC_HalfReady_t ADC_Sample_GetReadyHalf(void)
{
    return PeekModeBPending();
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
    uint8_t ret;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    ret = s_overrun_count;
    s_overrun_count = 0U;
    if ((primask & 1U) == 0U) {
        __enable_irq();
    }
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

/* 获取当前每通道采样点数。 */
uint16_t ADC_Sample_GetSampleCount(void)
{
    return s_sample_count;
}

/* 查询ADC是否正在采集。 */
uint8_t ADC_Sample_IsRunning(void)
{
    return s_running;
}

/* 返回并清除ADC和DMA错误位。 */
uint32_t ADC_Sample_GetAndClearError(void)
{
    uint32_t primask = __get_PRIMASK();
    uint32_t error_flags;

    __disable_irq();
    error_flags = s_error_flags;
    s_error_flags = ADC_ERROR_NONE;
    if ((primask & 1U) == 0U) {
        __enable_irq();
    }
    return error_flags;
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

/* 处理单帧完成或连续模式后半区完成。 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc == NULL || hadc->Instance != ADC_APP_HAL_INSTANCE) {
        return;
    }

    if (s_mode == ADC_MODE_ONESHOT) {
        __HAL_TIM_DISABLE(&htim3);
        s_mode_a_ready = 1U;
        s_running = 0U;
    } else {
        SetModeBPending(ADC_HALF_SECOND);
    }
}

/* 处理DMA前半区完成，单帧模式主动忽略该通知。 */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc == NULL || hadc->Instance != ADC_APP_HAL_INSTANCE) {
        return;
    }

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

    s_error_flags |= ADC_ERROR_PERIPHERAL;
    s_running = 0U;
    __HAL_TIM_DISABLE(&htim3);
}

/* 处理本驱动DMA通道的HAL错误回调。 */
void HAL_DMA_ErrorCallback(DMA_HandleTypeDef *hdma)
{
    if ((hdma == NULL) || (ADC_APP_HAL_HANDLE.DMA_Handle != hdma)) {
        return;
    }

    s_error_flags |= ADC_ERROR_DMA;
    s_running = 0U;
    __HAL_TIM_DISABLE(&htim3);
}

/* ================================================================ */
/*  ========  模式 A 内部实现（单次 DMA） ========                    */
/* ================================================================ */

/* 启动单帧DMA采集。 */
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
                               (uint32_t)s_sample_count *
                                   ADC_APP_CHANNEL_COUNT);
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
    InvalidateRawBufferCache(0U, s_sample_count);
    s_last_raw_start_idx = 0U;
    ConvertRawToVoltage(0U, 0U, s_sample_count);
    s_voltage_ready = 1U;

    return HAL_OK;
}

/* ================================================================ */
/*  ========  模式 B 内部实现（循环 DMA + 双缓冲） ========           */
/* ================================================================ */

/* 启动连续循环DMA双半区采集。 */
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
    ResetModeBPending();
    if (IsRawBufferDmaAccessible() == 0U) {
        printf("[ADC-B] DMA buffer is in DTCM, move it to AXI/D2 SRAM\r\n");
        return HAL_ERROR;
    }
    PrepareRawBufferForDma();
    status = HAL_ADC_Start_DMA(&ADC_APP_HAL_HANDLE,
                               (uint32_t *)s_raw_buffer,
                               (uint32_t)s_sample_count *
                                   ADC_APP_CHANNEL_COUNT * 2U);
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

/* 处理连续模式下一帧原始码。 */
static HAL_StatusTypeDef ProcessModeB(void)
{
    ADC_HalfReady_t half;

    if (PeekModeBPending() == ADC_HALF_NONE) {
        return HAL_BUSY;
    }

    if (s_sync_enabled && s_consumer_locked) {
        return HAL_BUSY;
    }
    half = PopModeBPending();

    if (half == ADC_HALF_FIRST) {
        InvalidateRawBufferCache(0U, s_sample_count);
        s_last_raw_start_idx = 0U;
        ConvertRawToVoltage(0U, 0U, s_sample_count);
    } else {
        InvalidateRawBufferCache(s_sample_count, s_sample_count);
        s_last_raw_start_idx = s_sample_count;
        ConvertRawToVoltage(s_sample_count, 0U, s_sample_count);
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

/* 根据ADC内核时钟和CubeMX分频配置计算实际ADC时钟。 */
static uint32_t GetAdcClockHz(void)
{
    uint32_t kernel_clock_hz;
    uint32_t divisor;

    kernel_clock_hz = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_ADC);
    switch (ADC_APP_HAL_HANDLE.Init.ClockPrescaler) {
        case ADC_CLOCK_ASYNC_DIV1:
            divisor = 1U;
            break;
        case ADC_CLOCK_ASYNC_DIV2:
            divisor = 2U;
            break;
        case ADC_CLOCK_ASYNC_DIV4:
            divisor = 4U;
            break;
        case ADC_CLOCK_ASYNC_DIV6:
            divisor = 6U;
            break;
        case ADC_CLOCK_ASYNC_DIV8:
            divisor = 8U;
            break;
        case ADC_CLOCK_ASYNC_DIV10:
            divisor = 10U;
            break;
        case ADC_CLOCK_ASYNC_DIV12:
            divisor = 12U;
            break;
        case ADC_CLOCK_ASYNC_DIV16:
            divisor = 16U;
            break;
        case ADC_CLOCK_ASYNC_DIV32:
            divisor = 32U;
            break;
        case ADC_CLOCK_ASYNC_DIV64:
            divisor = 64U;
            break;
        case ADC_CLOCK_ASYNC_DIV128:
            divisor = 128U;
            break;
        case ADC_CLOCK_ASYNC_DIV256:
            divisor = 256U;
            break;
        default:
            return 0U;
    }

    return kernel_clock_hz / divisor;
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
            uint16_t code = s_raw_buffer[raw_index + ch];

            if ((s_quality == ADC_QUALITY_MEDIAN3) &&
                (i > 0U) && ((i + 1U) < count)) {
                code = Median3(
                    s_raw_buffer[raw_index + ch - ADC_APP_CHANNEL_COUNT],
                    code,
                    s_raw_buffer[raw_index + ch + ADC_APP_CHANNEL_COUNT]);
            }
            s_voltage_buffer[ch][voltage_index] =
                ConvertCodeToVoltage(code);
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

/* 返回三个ADC码的中值，用于抑制孤立跳码。 */
static uint16_t Median3(uint16_t a, uint16_t b, uint16_t c)
{
    uint16_t temp;

    if (a > b) {
        temp = a;
        a = b;
        b = temp;
    }
    if (b > c) {
        temp = b;
        b = c;
        c = temp;
    }
    if (a > b) {
        b = a;
    }

    return b;
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
    uint32_t sample_count = s_sample_count;

    if (s_mode == ADC_MODE_CONTINUOUS) {
        sample_count *= 2U;
    }
    CleanInvalidateRawBufferCache(0U, sample_count);
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
    if ((half != ADC_HALF_FIRST) && (half != ADC_HALF_SECOND)) {
        return;
    }

    if ((s_mode_b_pending != ADC_HALF_NONE) &&
        (s_overrun_count < UINT8_MAX)) {
        s_overrun_count++;
    }

    /* DMA只有两个半区，旧通知延迟一整周期后已不再安全，只保留最新半区。 */
    s_mode_b_pending = half;
}

/* 在短临界区内查看下一个待处理半区，不清除通知。 */
static ADC_HalfReady_t PeekModeBPending(void)
{
    ADC_HalfReady_t half;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    half = s_mode_b_pending;
    if ((primask & 1U) == 0U) {
        __enable_irq();
    }

    return half;
}

/* 在短临界区内取出并清除一个待处理半区。 */
static ADC_HalfReady_t PopModeBPending(void)
{
    ADC_HalfReady_t half = ADC_HALF_NONE;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    half = s_mode_b_pending;
    s_mode_b_pending = ADC_HALF_NONE;

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
    s_mode_b_pending = ADC_HALF_NONE;
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

    for (uint32_t i = 0U; i < s_sample_count; i++) {
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

    for (uint32_t i = 0U; i < s_sample_count; i++) {
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

