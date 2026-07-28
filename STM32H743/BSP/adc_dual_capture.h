#ifndef ADC_DUAL_CAPTURE_H
#define ADC_DUAL_CAPTURE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* Place a caller-owned ping-pong buffer in DMA-accessible SRAM with the cache
 * line alignment required by Cortex-M7. The driver still validates its final
 * address at runtime and rejects DTCM automatically. */
#define ADC_DUAL_CAPTURE_DMA_BUFFER_ATTR \
    __attribute__((section(".bss.adc_dual_dma"), aligned(32)))

/* One DMA word is one simultaneous sample pair: ADC1/PC4 in bits 15:0 and
 * ADC2/PC5 in bits 31:16. No software interleaving or time staggering occurs. */
typedef uint32_t ADC_DualCapture_Pair_t;

typedef enum {
    ADC_DUAL_CAPTURE_HALF_NONE = 0,
    ADC_DUAL_CAPTURE_HALF_FIRST = 1,
    ADC_DUAL_CAPTURE_HALF_SECOND = 2
} ADC_DualCapture_Half_t;

typedef enum {
    ADC_DUAL_CAPTURE_ERROR_NONE = 0U,
    ADC_DUAL_CAPTURE_ERROR_CONFIG = 1U << 0,
    ADC_DUAL_CAPTURE_ERROR_RATE = 1U << 1,
    ADC_DUAL_CAPTURE_ERROR_ADC = 1U << 2,
    ADC_DUAL_CAPTURE_ERROR_DMA = 1U << 3,
    ADC_DUAL_CAPTURE_ERROR_TIMER = 1U << 4,
    ADC_DUAL_CAPTURE_ERROR_OVERRUN = 1U << 5
} ADC_DualCapture_Error_t;

typedef struct {
    ADC_DualCapture_Pair_t *pair_buffer; /* 32-byte aligned DMA buffer. */
    uint32_t pairs_per_half;             /* Simultaneous pairs in each half. */
    float sample_rate_hz;                /* Requested rate for both channels. */
    float maximum_rate_error_ppm;        /* Allowed timer quantization error. */
} ADC_DualCapture_Config_t;

typedef struct {
    const ADC_DualCapture_Pair_t *pairs; /* Stable until ReleaseFrame(). */
    uint32_t pair_count;
    uint32_t sequence;
    uint16_t timer_divider;
    uint8_t half_index;                  /* 0=first half, 1=second half. */
    float configured_sample_rate_hz;
    float measured_sample_rate_hz;
} ADC_DualCapture_Frame_t;

typedef struct {
    float requested_sample_rate_hz;
    float configured_sample_rate_hz;
    float measured_sample_rate_hz;
    float maximum_sample_rate_hz;
    uint32_t timer_clock_hz;
    uint32_t adc_clock_hz;
    uint32_t completed_half_count;
    uint32_t overrun_count;
    uint32_t error_flags;
    uint16_t active_timer_divider;
    uint16_t pending_timer_divider;
    uint8_t ready_mask;
    uint8_t acquired_mask;
    uint8_t initialized;
    uint8_t running;
    uint8_t hardware_simultaneous;
} ADC_DualCapture_Status_t;

/* Minimal use:
 *   static ADC_DualCapture_Pair_t dma[2U * N]
 *       ADC_DUAL_CAPTURE_DMA_BUFFER_ATTR;
 *   ADC_DualCapture_GetDefaultConfig(&cfg);
 *   cfg.pair_buffer = dma; cfg.pairs_per_half = N;
 *   ADC_DualCapture_Init(&cfg); ADC_DualCapture_Start();
 * The main loop then AcquireFrame(), processes cfg.pairs_per_half simultaneous
 * pairs without copying, and calls ReleaseFrame() before that half is reused. */
void ADC_DualCapture_GetDefaultConfig(ADC_DualCapture_Config_t *config);
HAL_StatusTypeDef ADC_DualCapture_Init(const ADC_DualCapture_Config_t *config);
HAL_StatusTypeDef ADC_DualCapture_Start(void);
void ADC_DualCapture_Stop(void);

/* Request a common per-channel rate. The exact achievable rate is returned. */
HAL_StatusTypeDef ADC_DualCapture_RequestSampleRate(float requested_hz,
                                                    float *actual_hz);
/* Exact-divider form used by coherent multi-rate undersampling. */
HAL_StatusTypeDef ADC_DualCapture_RequestDivider(uint16_t divider,
                                                 float *actual_hz);
float ADC_DualCapture_SampleRateFromDivider(uint16_t divider);

/* Zero-copy ping-pong consumer API. Acquire and release one half at a time. */
uint8_t ADC_DualCapture_AcquireFrame(ADC_DualCapture_Frame_t *frame);
void ADC_DualCapture_ReleaseFrame(const ADC_DualCapture_Frame_t *frame);
void ADC_DualCapture_GetStatus(ADC_DualCapture_Status_t *status);

/* Runtime audit of ADC1+ADC2 regular-simultaneous, CDR packing, DMA and TIM6. */
HAL_StatusTypeDef ADC_DualCapture_ValidateHardware(void);

/* Called only by the central HAL callbacks in adc_app.c. */
uint8_t ADC_DualCapture_OnHalfCompleteIsr(ADC_HandleTypeDef *hadc);
uint8_t ADC_DualCapture_OnFullCompleteIsr(ADC_HandleTypeDef *hadc);
uint8_t ADC_DualCapture_OnAdcErrorIsr(ADC_HandleTypeDef *hadc);
uint8_t ADC_DualCapture_OnDmaErrorIsr(DMA_HandleTypeDef *hdma);

static inline uint16_t ADC_DualCapture_GetAdc1Code(ADC_DualCapture_Pair_t pair)
{
    return (uint16_t)(pair & 0xFFFFU);
}

static inline uint16_t ADC_DualCapture_GetAdc2Code(ADC_DualCapture_Pair_t pair)
{
    return (uint16_t)((pair >> 16U) & 0xFFFFU);
}

#ifdef __cplusplus
}
#endif
#endif /* ADC_DUAL_CAPTURE_H */