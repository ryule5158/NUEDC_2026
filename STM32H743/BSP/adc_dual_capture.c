/**
 * @file adc_dual_capture.c
 * @brief True simultaneous ADC1/ADC2 capture with TIM6 and circular DMA.
 *
 * ADC1 is the multimode master on PC4/INP4 and ADC2 is the slave on PC5/INP8.
 * A TIM6 update edge starts both regular conversions in hardware. ADC12_CDR
 * then supplies one packed 32-bit pair to DMA, preserving channel alignment.
 */
#include "adc_dual_capture.h"
#include "adc.h"
#include "tim.h"
#include "stm32h7xx_hal_adc_ex.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

#define ADC_DUAL_DEFAULT_RATE_HZ            500000.0f
#define ADC_DUAL_DEFAULT_RATE_ERROR_PPM     1000.0f
#define ADC_DUAL_CACHE_LINE_BYTES           32U
#define ADC_DUAL_DTCM_START                 0x20000000UL
#define ADC_DUAL_DTCM_END                   0x20020000UL
/* STM32H743 16-bit conversion: 8.5 sampling + 8.5 SAR + up to 2.5 trigger
 * latency. This limit applies to each ADC independently; dual simultaneous
 * mode does not double the conversion time. */
#define ADC_DUAL_SAMPLE_CYCLES              8.5f
#define ADC_DUAL_SAR_CYCLES                 8.5f
#define ADC_DUAL_TRIGGER_MARGIN_CYCLES      2.5f

static ADC_DualCapture_Config_t s_config;
static volatile uint8_t s_initialized;
static volatile uint8_t s_running;
static volatile uint8_t s_ready_mask;
static volatile uint8_t s_acquired_mask;
static volatile uint16_t s_active_divider;
static volatile uint16_t s_pending_divider;
static volatile uint16_t s_half_divider[2];
static volatile uint32_t s_half_cycles[2];
static volatile uint32_t s_half_sequence[2];
static volatile uint32_t s_last_boundary_cycle;
static volatile uint8_t s_boundary_cycle_valid;
static volatile uint32_t s_completed_half_count;
static volatile uint32_t s_overrun_count;
static volatile uint32_t s_error_flags;
static uint32_t s_timer_clock_hz;
static uint32_t s_adc_clock_hz;
static float s_requested_rate_hz;
static float s_configured_rate_hz;
static float s_measured_rate_hz;
static float s_maximum_rate_hz;

static uint8_t IsDCacheEnabled(void)
{
    return ((SCB->CCR & SCB_CCR_DC_Msk) != 0U) ? 1U : 0U;
}

static void CacheCleanInvalidate(const void *address, uint32_t bytes)
{
    uint32_t start;
    uint32_t end;
    if ((address == NULL) || (bytes == 0U) || (IsDCacheEnabled() == 0U)) return;
    start = ((uint32_t)address) & ~(ADC_DUAL_CACHE_LINE_BYTES - 1U);
    end = (((uint32_t)address + bytes + ADC_DUAL_CACHE_LINE_BYTES - 1U) &
           ~(ADC_DUAL_CACHE_LINE_BYTES - 1U));
    SCB_CleanInvalidateDCache_by_Addr((uint32_t *)start,
                                      (int32_t)(end - start));
}

static void CacheInvalidate(const void *address, uint32_t bytes)
{
    uint32_t start;
    uint32_t end;
    if ((address == NULL) || (bytes == 0U) || (IsDCacheEnabled() == 0U)) return;
    start = ((uint32_t)address) & ~(ADC_DUAL_CACHE_LINE_BYTES - 1U);
    end = (((uint32_t)address + bytes + ADC_DUAL_CACHE_LINE_BYTES - 1U) &
           ~(ADC_DUAL_CACHE_LINE_BYTES - 1U));
    SCB_InvalidateDCache_by_Addr((uint32_t *)start, (int32_t)(end - start));
}

static uint8_t IsBufferDmaAccessible(const void *buffer, uint32_t bytes)
{
    uint32_t start = (uint32_t)buffer;
    uint32_t end = start + bytes;
    if ((buffer == NULL) || ((start & (ADC_DUAL_CACHE_LINE_BYTES - 1U)) != 0U)) {
        return 0U;
    }
    if ((start < ADC_DUAL_DTCM_END) && (end > ADC_DUAL_DTCM_START)) return 0U;
    return (end > start) ? 1U : 0U;
}

static uint32_t GetTim6ClockHz(void)
{
    RCC_ClkInitTypeDef clocks;
    uint32_t latency;
    uint32_t clock_hz;
    HAL_RCC_GetClockConfig(&clocks, &latency);
    clock_hz = HAL_RCC_GetPCLK1Freq();
    if (clocks.APB1CLKDivider != RCC_APB1_DIV1) clock_hz *= 2U;
    return clock_hz;
}

static uint32_t AdcPrescalerDivisor(uint32_t prescaler)
{
    switch (prescaler) {
        case ADC_CLOCK_ASYNC_DIV1: return 1U;
        case ADC_CLOCK_ASYNC_DIV2: return 2U;
        case ADC_CLOCK_ASYNC_DIV4: return 4U;
        case ADC_CLOCK_ASYNC_DIV6: return 6U;
        case ADC_CLOCK_ASYNC_DIV8: return 8U;
        case ADC_CLOCK_ASYNC_DIV10: return 10U;
        case ADC_CLOCK_ASYNC_DIV12: return 12U;
        case ADC_CLOCK_ASYNC_DIV16: return 16U;
        case ADC_CLOCK_ASYNC_DIV32: return 32U;
        case ADC_CLOCK_ASYNC_DIV64: return 64U;
        case ADC_CLOCK_ASYNC_DIV128: return 128U;
        case ADC_CLOCK_ASYNC_DIV256: return 256U;
        default: return 0U;
    }
}

static uint32_t GetAdcClockHz(void)
{
    uint32_t kernel_hz = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_ADC);
    uint32_t divisor = AdcPrescalerDivisor(hadc1.Init.ClockPrescaler);
    return (divisor != 0U) ? (kernel_hz / divisor) : 0U;
}

float ADC_DualCapture_SampleRateFromDivider(uint16_t divider)
{
    if ((divider == 0U) || (s_timer_clock_hz == 0U)) return 0.0f;
    return (float)s_timer_clock_hz / (float)divider;
}

static HAL_StatusTypeDef ValidateRate(float requested_hz,
                                      uint16_t *divider,
                                      float *actual_hz)
{
    uint32_t rounded_divider;
    float actual;
    float error_ppm;
    if ((requested_hz <= 0.0f) || !isfinite(requested_hz) ||
        (divider == NULL) || (actual_hz == NULL) ||
        (s_timer_clock_hz == 0U) || (requested_hz > s_maximum_rate_hz)) {
        return HAL_ERROR;
    }
    rounded_divider = (uint32_t)((float)s_timer_clock_hz / requested_hz + 0.5f);
    if ((rounded_divider < 2U) || (rounded_divider > 65535U)) return HAL_ERROR;
    actual = (float)s_timer_clock_hz / (float)rounded_divider;
    error_ppm = fabsf(actual - requested_hz) * 1000000.0f / requested_hz;
    if (error_ppm > s_config.maximum_rate_error_ppm) return HAL_ERROR;
    *divider = (uint16_t)rounded_divider;
    *actual_hz = actual;
    return HAL_OK;
}

void ADC_DualCapture_GetDefaultConfig(ADC_DualCapture_Config_t *config)
{
    if (config == NULL) return;
    memset(config, 0, sizeof(*config));
    config->sample_rate_hz = ADC_DUAL_DEFAULT_RATE_HZ;
    config->maximum_rate_error_ppm = ADC_DUAL_DEFAULT_RATE_ERROR_PPM;
}

HAL_StatusTypeDef ADC_DualCapture_ValidateHardware(void)
{
    uint32_t common_ccr;
    uint32_t analog_pin_mask = GPIO_MODER_MODE4_Msk | GPIO_MODER_MODE5_Msk;
    uint32_t pull_pin_mask = GPIO_PUPDR_PUPD4_Msk | GPIO_PUPDR_PUPD5_Msk;

    if ((hadc1.Instance != ADC1) || (hadc2.Instance != ADC2) ||
        (hadc1.Init.Resolution != ADC_RESOLUTION_16B) ||
        (hadc2.Init.Resolution != ADC_RESOLUTION_16B) ||
        (hadc1.Init.NbrOfConversion != 1U) ||
        (hadc2.Init.NbrOfConversion != 1U) ||
        (hadc1.Init.ContinuousConvMode != DISABLE) ||
        (hadc2.Init.ContinuousConvMode != DISABLE) ||
        (hadc1.Init.ClockPrescaler != hadc2.Init.ClockPrescaler) ||
        (hadc1.Init.ExternalTrigConv != ADC_EXTERNALTRIG_T6_TRGO) ||
        (hadc1.Init.ExternalTrigConvEdge != ADC_EXTERNALTRIGCONVEDGE_RISING) ||
        (hadc1.Init.ConversionDataManagement != ADC_CONVERSIONDATA_DMA_CIRCULAR) ||
        (hadc2.Init.ExternalTrigConv != ADC_SOFTWARE_START) ||
        (hadc2.Init.ExternalTrigConvEdge != ADC_EXTERNALTRIGCONVEDGE_NONE) ||
        (hadc2.Init.ConversionDataManagement != ADC_CONVERSIONDATA_DR) ||
        (hdma_adc1.Instance != DMA1_Stream0) ||
        (hdma_adc1.Init.Request != DMA_REQUEST_ADC1) ||
        (hdma_adc1.Init.Direction != DMA_PERIPH_TO_MEMORY) ||
        (hdma_adc1.Init.PeriphInc != DMA_PINC_DISABLE) ||
        (hdma_adc1.Init.MemInc != DMA_MINC_ENABLE) ||
        (hdma_adc1.Init.PeriphDataAlignment != DMA_PDATAALIGN_WORD) ||
        (hdma_adc1.Init.MemDataAlignment != DMA_MDATAALIGN_WORD) ||
        (hdma_adc1.Init.Mode != DMA_CIRCULAR) ||
        (htim6.Instance != TIM6) ||
        (htim6.Init.Prescaler != 0U) ||
        ((TIM6->CR1 & TIM_CR1_ARPE) != 0U) ||
        ((TIM6->CR2 & TIM_CR2_MMS) != TIM_TRGO_UPDATE) ||
        ((ADC1->SQR1 & ADC_SQR1_SQ1_Msk) !=
         (4UL << ADC_SQR1_SQ1_Pos)) ||
        ((ADC2->SQR1 & ADC_SQR1_SQ1_Msk) !=
         (8UL << ADC_SQR1_SQ1_Pos)) ||
        ((GPIOC->MODER & analog_pin_mask) != analog_pin_mask) ||
        ((GPIOC->PUPDR & pull_pin_mask) != 0U)) {
        return HAL_ERROR;
    }
    common_ccr = ADC12_COMMON->CCR;
    if (((common_ccr & ADC_CCR_DUAL) != ADC_DUALMODE_REGSIMULT) ||
        ((common_ccr & ADC_CCR_DAMDF) != ADC_DUALMODEDATAFORMAT_32_10_BITS) ||
        ((s_running != 0U) &&
         (((DMA_Stream_TypeDef *)hdma_adc1.Instance)->PAR !=
          (uint32_t)&ADC12_COMMON->CDR))) {
        return HAL_ERROR;
    }
    return HAL_OK;
}

HAL_StatusTypeDef ADC_DualCapture_Init(const ADC_DualCapture_Config_t *config)
{
    uint32_t bytes;
    uint16_t divider;
    float actual_rate;
    if ((config == NULL) || (config->pair_buffer == NULL) ||
        (config->pairs_per_half == 0U) ||
        (config->pairs_per_half >
         (UINT32_MAX / (2U * sizeof(ADC_DualCapture_Pair_t)))) ||
        (((config->pairs_per_half * sizeof(ADC_DualCapture_Pair_t)) &
          (ADC_DUAL_CACHE_LINE_BYTES - 1U)) != 0U) ||
        (!isfinite(config->maximum_rate_error_ppm)) ||
        (config->maximum_rate_error_ppm < 0.0f)) {
        s_error_flags = ADC_DUAL_CAPTURE_ERROR_CONFIG;
        return HAL_ERROR;
    }

    ADC_DualCapture_Stop();
    s_initialized = 0U;
    s_config = *config;
    bytes = 2U * s_config.pairs_per_half * sizeof(ADC_DualCapture_Pair_t);
    if (IsBufferDmaAccessible(s_config.pair_buffer, bytes) == 0U) {
        s_error_flags = ADC_DUAL_CAPTURE_ERROR_CONFIG;
        return HAL_ERROR;
    }

    s_timer_clock_hz = GetTim6ClockHz();
    s_adc_clock_hz = GetAdcClockHz();
    s_maximum_rate_hz = (s_adc_clock_hz != 0U) ?
        ((float)s_adc_clock_hz /
         (ADC_DUAL_SAMPLE_CYCLES + ADC_DUAL_SAR_CYCLES +
          ADC_DUAL_TRIGGER_MARGIN_CYCLES)) : 0.0f;
    if (ADC_DualCapture_ValidateHardware() != HAL_OK) {
        s_error_flags = ADC_DUAL_CAPTURE_ERROR_CONFIG;
        return HAL_ERROR;
    }
    if (ValidateRate(config->sample_rate_hz, &divider, &actual_rate) != HAL_OK) {
        s_error_flags = ADC_DUAL_CAPTURE_ERROR_RATE;
        return HAL_ERROR;
    }

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    s_requested_rate_hz = config->sample_rate_hz;
    s_configured_rate_hz = actual_rate;
    s_measured_rate_hz = 0.0f;
    s_active_divider = divider;
    s_pending_divider = divider;
    s_half_divider[0] = divider;
    s_half_divider[1] = divider;
    s_half_cycles[0] = 0U;
    s_half_cycles[1] = 0U;
    s_half_sequence[0] = 0U;
    s_half_sequence[1] = 0U;
    s_last_boundary_cycle = 0U;
    s_boundary_cycle_valid = 0U;
    s_ready_mask = 0U;
    s_acquired_mask = 0U;
    s_completed_half_count = 0U;
    s_overrun_count = 0U;
    s_error_flags = ADC_DUAL_CAPTURE_ERROR_NONE;

    (void)HAL_TIM_Base_Stop(&htim6);
    TIM6->ARR = (uint32_t)divider - 1U;
    TIM6->CNT = 0U;
    TIM6->EGR = TIM_EGR_UG;
    __HAL_TIM_CLEAR_FLAG(&htim6, TIM_FLAG_UPDATE);

    if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_CALIB_OFFSET,
                                    ADC_SINGLE_ENDED) != HAL_OK) {
        s_error_flags |= ADC_DUAL_CAPTURE_ERROR_ADC;
        return HAL_ERROR;
    }
    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET,
                                    ADC_SINGLE_ENDED) != HAL_OK) {
        s_error_flags |= ADC_DUAL_CAPTURE_ERROR_ADC;
        return HAL_ERROR;
    }
    s_initialized = 1U;
    return HAL_OK;
}

HAL_StatusTypeDef ADC_DualCapture_Start(void)
{
    HAL_StatusTypeDef status;
    uint32_t total_pairs;
    if ((s_initialized == 0U) || (s_running != 0U)) return HAL_ERROR;
    total_pairs = 2U * s_config.pairs_per_half;
    CacheCleanInvalidate(s_config.pair_buffer,
                         total_pairs * sizeof(ADC_DualCapture_Pair_t));
    s_ready_mask = 0U;
    s_acquired_mask = 0U;
    s_boundary_cycle_valid = 0U;
    s_error_flags = ADC_DUAL_CAPTURE_ERROR_NONE;
    status = HAL_ADCEx_MultiModeStart_DMA(&hadc1, s_config.pair_buffer,
                                          total_pairs);
    if (status != HAL_OK) {
        s_error_flags |= ADC_DUAL_CAPTURE_ERROR_ADC |
                         ADC_DUAL_CAPTURE_ERROR_DMA;
        return status;
    }
    s_running = 1U;
    TIM6->CNT = 0U;
    status = HAL_TIM_Base_Start(&htim6);
    if (status != HAL_OK) {
        s_running = 0U;
        s_error_flags |= ADC_DUAL_CAPTURE_ERROR_TIMER;
        (void)HAL_ADCEx_MultiModeStop_DMA(&hadc1);
        return status;
    }
    return HAL_OK;
}

void ADC_DualCapture_Stop(void)
{
    uint8_t should_stop_adc =
        ((s_initialized != 0U) || (s_running != 0U)) ? 1U : 0U;
    s_running = 0U;
    (void)HAL_TIM_Base_Stop(&htim6);
    if ((should_stop_adc != 0U) && (hadc1.Instance == ADC1)) {
        (void)HAL_ADCEx_MultiModeStop_DMA(&hadc1);
    }
    s_ready_mask = 0U;
    s_acquired_mask = 0U;
}

HAL_StatusTypeDef ADC_DualCapture_RequestDivider(uint16_t divider,
                                                 float *actual_hz)
{
    float actual;
    if ((divider < 2U) || (s_timer_clock_hz == 0U)) return HAL_ERROR;
    actual = ADC_DualCapture_SampleRateFromDivider(divider);
    if ((actual <= 0.0f) || (actual > s_maximum_rate_hz)) {
        s_error_flags |= ADC_DUAL_CAPTURE_ERROR_RATE;
        return HAL_ERROR;
    }
    s_requested_rate_hz = actual;
    s_pending_divider = divider;
    if (s_running == 0U) {
        TIM6->ARR = (uint32_t)divider - 1U;
        TIM6->CNT = 0U;
        TIM6->EGR = TIM_EGR_UG;
        __HAL_TIM_CLEAR_FLAG(&htim6, TIM_FLAG_UPDATE);
        s_active_divider = divider;
        s_configured_rate_hz = actual;
    }
    if (actual_hz != NULL) *actual_hz = actual;
    return HAL_OK;
}

HAL_StatusTypeDef ADC_DualCapture_RequestSampleRate(float requested_hz,
                                                    float *actual_hz)
{
    uint16_t divider;
    float actual;
    if (ValidateRate(requested_hz, &divider, &actual) != HAL_OK) {
        s_error_flags |= ADC_DUAL_CAPTURE_ERROR_RATE;
        return HAL_ERROR;
    }
    s_requested_rate_hz = requested_hz;
    if (ADC_DualCapture_RequestDivider(divider, NULL) != HAL_OK) return HAL_ERROR;
    s_requested_rate_hz = requested_hz;
    if (actual_hz != NULL) *actual_hz = actual;
    return HAL_OK;
}

static void BoundaryIsr(uint8_t half_index)
{
    uint32_t now_cycles = DWT->CYCCNT;
    uint8_t bit = (uint8_t)(1U << half_index);
    uint16_t requested;
    if (s_boundary_cycle_valid != 0U) {
        s_half_cycles[half_index] = now_cycles - s_last_boundary_cycle;
    } else {
        s_half_cycles[half_index] = 0U;
        s_boundary_cycle_valid = 1U;
    }
    s_last_boundary_cycle = now_cycles;
    s_half_divider[half_index] = s_active_divider;

    requested = s_pending_divider;
    if ((requested >= 2U) && (requested != s_active_divider)) {
        TIM6->ARR = (uint32_t)requested - 1U;
        s_active_divider = requested;
        s_configured_rate_hz = ADC_DualCapture_SampleRateFromDivider(requested);
    }
    if (((s_ready_mask | s_acquired_mask) & bit) != 0U) {
        s_overrun_count++;
        s_error_flags |= ADC_DUAL_CAPTURE_ERROR_OVERRUN;
    }
    s_completed_half_count++;
    s_half_sequence[half_index] = s_completed_half_count;
    s_ready_mask |= bit;
}

uint8_t ADC_DualCapture_OnHalfCompleteIsr(ADC_HandleTypeDef *hadc)
{
    if ((s_running == 0U) || (hadc == NULL) || (hadc->Instance != ADC1)) {
        return 0U;
    }
    BoundaryIsr(0U);
    return 1U;
}

uint8_t ADC_DualCapture_OnFullCompleteIsr(ADC_HandleTypeDef *hadc)
{
    if ((s_running == 0U) || (hadc == NULL) || (hadc->Instance != ADC1)) {
        return 0U;
    }
    BoundaryIsr(1U);
    return 1U;
}

uint8_t ADC_DualCapture_OnAdcErrorIsr(ADC_HandleTypeDef *hadc)
{
    if ((s_running == 0U) || (hadc == NULL) ||
        ((hadc->Instance != ADC1) && (hadc->Instance != ADC2))) {
        return 0U;
    }
    s_error_flags |= ADC_DUAL_CAPTURE_ERROR_ADC;
    s_running = 0U;
    __HAL_TIM_DISABLE(&htim6);
    return 1U;
}

uint8_t ADC_DualCapture_OnDmaErrorIsr(DMA_HandleTypeDef *hdma)
{
    if ((s_running == 0U) || (hdma == NULL) || (hadc1.DMA_Handle != hdma)) {
        return 0U;
    }
    s_error_flags |= ADC_DUAL_CAPTURE_ERROR_DMA;
    s_running = 0U;
    __HAL_TIM_DISABLE(&htim6);
    return 1U;
}

uint8_t ADC_DualCapture_AcquireFrame(ADC_DualCapture_Frame_t *frame)
{
    uint8_t half_index;
    uint8_t bit;
    uint32_t cycles;
    uint32_t primask;
    if ((frame == NULL) || (s_running == 0U)) return 0U;

    primask = __get_PRIMASK();
    __disable_irq();
    if ((s_ready_mask & 0x03U) == 0x03U) {
        /* If processing was briefly delayed, consume the older completed half
         * first. Signed subtraction preserves order across sequence wrap. */
        half_index = ((int32_t)(s_half_sequence[0] -
                                s_half_sequence[1]) <= 0) ? 0U : 1U;
        bit = (uint8_t)(1U << half_index);
    } else if ((s_ready_mask & 0x01U) != 0U) {
        half_index = 0U;
        bit = 0x01U;
    } else if ((s_ready_mask & 0x02U) != 0U) {
        half_index = 1U;
        bit = 0x02U;
    } else {
        if ((primask & 1U) == 0U) __enable_irq();
        return 0U;
    }
    s_ready_mask &= (uint8_t)~bit;
    s_acquired_mask |= bit;
    frame->timer_divider = s_half_divider[half_index];
    cycles = s_half_cycles[half_index];
    frame->sequence = s_half_sequence[half_index];
    if ((primask & 1U) == 0U) __enable_irq();

    frame->pairs = &s_config.pair_buffer[half_index * s_config.pairs_per_half];
    frame->pair_count = s_config.pairs_per_half;
    frame->half_index = half_index;
    frame->configured_sample_rate_hz =
        ADC_DualCapture_SampleRateFromDivider(frame->timer_divider);
    frame->measured_sample_rate_hz = frame->configured_sample_rate_hz;
    if ((cycles != 0U) && (SystemCoreClock != 0U)) {
        frame->measured_sample_rate_hz =
            ((float)s_config.pairs_per_half * (float)SystemCoreClock) /
            (float)cycles;
    }
    s_measured_rate_hz = frame->measured_sample_rate_hz;
    CacheInvalidate(frame->pairs,
                    frame->pair_count * sizeof(ADC_DualCapture_Pair_t));
    return 1U;
}

void ADC_DualCapture_ReleaseFrame(const ADC_DualCapture_Frame_t *frame)
{
    uint8_t bit;
    uint32_t primask;
    if ((frame == NULL) || (frame->half_index > 1U)) return;
    bit = (uint8_t)(1U << frame->half_index);
    primask = __get_PRIMASK();
    __disable_irq();
    s_acquired_mask &= (uint8_t)~bit;
    if ((primask & 1U) == 0U) __enable_irq();
}

void ADC_DualCapture_GetStatus(ADC_DualCapture_Status_t *status)
{
    uint32_t primask;
    if (status == NULL) return;
    primask = __get_PRIMASK();
    __disable_irq();
    status->completed_half_count = s_completed_half_count;
    status->overrun_count = s_overrun_count;
    status->error_flags = s_error_flags;
    status->active_timer_divider = s_active_divider;
    status->pending_timer_divider = s_pending_divider;
    status->ready_mask = s_ready_mask;
    status->acquired_mask = s_acquired_mask;
    status->initialized = s_initialized;
    status->running = s_running;
    if ((primask & 1U) == 0U) __enable_irq();

    status->requested_sample_rate_hz = s_requested_rate_hz;
    status->configured_sample_rate_hz = s_configured_rate_hz;
    status->measured_sample_rate_hz = s_measured_rate_hz;
    status->maximum_sample_rate_hz = s_maximum_rate_hz;
    status->timer_clock_hz = s_timer_clock_hz;
    status->adc_clock_hz = s_adc_clock_hz;
    status->hardware_simultaneous =
        (ADC_DualCapture_ValidateHardware() == HAL_OK) ? 1U : 0U;
}