/**
 * @file pll_test_app.c
 * @brief PC4/PC5 simultaneous ADC -> generic SoftPll -> AD9910 adapter.
 *
 * ADC DMA uses a circular buffer split into two halves, so acquisition and
 * CPU phase processing run concurrently. SoftPll remains hardware-independent:
 * this file alone translates its frequency/phase commands to AD9910 FTW/POW.
 */
#include "pll_test_app.h"
#include "pll_selftest_config.h"
#include "adc.h"
#include "adc_app.h"
#include "ad9910.h"
#include "tim.h"
#include "usart.h"
#include "SoftPll.h"
#include "arm_math.h"
#include "arm_const_structs.h"
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if PLL_SELFTEST_ENABLE
#define PLL_TEST_ADC_TOTAL_SAMPLES  (2U * PLL_TEST_ADC_HALF_SAMPLES)
#define PLL_TEST_UART_LINE_SIZE     96U
#define PLL_TEST_UART_TX_SIZE       320U
#define PLL_TEST_ADC_SCALE          32768.0f
#define PLL_TEST_TWO_PI             6.2831853071795864769f
#define PLL_TEST_POW_SCALE          65536.0
#define PLL_TEST_DMA_ATTR __attribute__((section(".bss.pll_test_dma"), aligned(32)))
#define PLL_TEST_RATE_COUNT         4U
#define PLL_TEST_FFT_SIZE           PLL_TEST_ADC_HALF_SAMPLES
#define PLL_TEST_SWEEP_MIN_RATES    3U
#define PLL_TEST_SWEEP_MAX_RMS_HZ   1500.0f

#if PLL_TEST_ADC_HALF_SAMPLES != 1024U
#error "The RF capture FFT currently requires a 1024-sample half buffer"
#endif

typedef enum {
    PLL_TEST_MODE_ACQUIRE = 0,
    PLL_TEST_MODE_AC_LOCK = 1,
    PLL_TEST_MODE_HOLDOVER = 2
} PllTest_Mode_t;

typedef struct {
    float previous;
    float centre;
    float envelope;
    double last_crossing;
    double period_samples;
    double latest_crossing;
    uint8_t have_previous;
    uint8_t armed;
    uint8_t have_crossing;
} PllTest_Crossing_t;

static uint32_t s_adc_dma[PLL_TEST_ADC_TOTAL_SAMPLES] PLL_TEST_DMA_ATTR;

static SoftPll_t s_pll;
static SoftPll_Config_t s_pll_config;
static PllTest_Crossing_t s_ref_cross;
static PllTest_Crossing_t s_fb_cross;
static volatile uint8_t s_adc_pending;
static volatile uint8_t s_running;
static volatile uint32_t s_adc_overrun;
/* This hardware-frequency test always uses the AD9910 full-scale ASF. */
static const uint16_t s_ad9910_amplitude = PLL_TEST_AD9910_AMPLITUDE;
static double s_ad9910_phase_deg;
static float s_ad9910_frequency_hz = PLL_TEST_DEFAULT_START_FREQ_HZ;
static uint16_t s_ad9910_phase_word;
static uint32_t s_ad9910_update_count;
static uint32_t s_ad9910_error_count;
static uint32_t s_ad9910_last_update_us;
static uint64_t s_sample_counter;
static double s_ref_period_sum;
static uint32_t s_ref_period_count;
static float s_frequency_estimate_hz;
static float s_latest_frequency_hz;
static uint8_t s_frequency_event;
static uint16_t s_frequency_confirm_count;
static PllTest_Mode_t s_mode = PLL_TEST_MODE_ACQUIRE;
static float s_target_phase_deg = PLL_TEST_TARGET_PHASE_DEG;
static float s_fixed_phase_cal_deg = PLL_TEST_FIXED_PHASE_CAL_DEG;
static float s_loop_bandwidth_hz = PLL_TEST_DEFAULT_LOOP_BW_HZ;
static float s_last_ref_mean = 32768.0f;
static float s_last_fb_mean = 32768.0f;
static uint32_t s_last_ac_tick;
static uint32_t s_last_telemetry_tick;
static uint32_t s_telemetry_period_ms = PLL_TEST_TELEMETRY_PERIOD_MS;

/* Four exact timer divisors provide independent alias equations.  The ADC
 * still samples both PC4 and PC5 simultaneously at every selected rate. */
static const uint16_t s_rate_dividers[PLL_TEST_RATE_COUNT] = {
    480U, 487U, 499U, 509U
};
static float s_fft_window[PLL_TEST_FFT_SIZE];
static float s_fft_buffer[2U * PLL_TEST_FFT_SIZE];
static volatile uint16_t s_active_sample_divider;
static volatile uint16_t s_pending_sample_divider;
static volatile uint16_t s_half_sample_divider[2];
static uint16_t s_lock_sample_divider;
static float s_last_block_sample_rate_hz = (float)PLL_TEST_SAMPLE_RATE_HZ;
static float s_last_alias_hz;
static float s_last_alias_snr_db;
static float s_last_reference_iq_amp;
static float s_last_feedback_iq_amp;
static uint8_t s_last_phase_valid;
static float s_alias_hz[PLL_TEST_RATE_COUNT];
static float s_alias_snr_db[PLL_TEST_RATE_COUNT];
static uint8_t s_alias_valid_mask;
static uint8_t s_sweep_active;
static uint8_t s_sweep_rate_index;
static uint8_t s_sweep_block_count;
static uint8_t s_sweep_attempt_count;
static uint16_t s_sweep_first_bin;
static float s_sweep_first_phase;
static float s_sweep_first_alias_hz;
static uint32_t s_sweep_fail_count;
static uint32_t s_lock_start_tick;
static uint32_t s_last_locked_tick;
static uint8_t s_uart_rx_byte;
static char s_uart_rx_line[PLL_TEST_UART_LINE_SIZE];
static char s_uart_command[PLL_TEST_UART_LINE_SIZE];
static volatile uint16_t s_uart_rx_length;
static volatile uint8_t s_uart_command_ready;
static char s_uart_tx_buffer[PLL_TEST_UART_TX_SIZE];
static volatile uint8_t s_uart_tx_busy;

static void CacheInvalidate(const void *address, uint32_t bytes)
{
    uint32_t start;
    uint32_t end;
    if ((SCB->CCR & SCB_CCR_DC_Msk) == 0U) return;
    start = ((uint32_t)address) & ~31UL;
    end = (((uint32_t)address + bytes + 31UL) & ~31UL);
    SCB_InvalidateDCache_by_Addr((uint32_t *)start, (int32_t)(end - start));
}

static void CacheClean(const void *address, uint32_t bytes)
{
    uint32_t start;
    uint32_t end;
    if ((SCB->CCR & SCB_CCR_DC_Msk) == 0U) return;
    start = ((uint32_t)address) & ~31UL;
    end = (((uint32_t)address + bytes + 31UL) & ~31UL);
    SCB_CleanDCache_by_Addr((uint32_t *)start, (int32_t)(end - start));
}

static float ClampFloat(float value, float low, float high)
{
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

static float SampleRateFromDivider(uint16_t divider)
{
    if (divider == 0U) return (float)PLL_TEST_SAMPLE_RATE_HZ;
    return (float)PLL_TEST_TIMER_CLOCK_HZ / (float)divider;
}

static float FoldAliasHz(float frequency_hz, float sample_rate_hz)
{
    float alias = fmodf(frequency_hz, sample_rate_hz);
    if (alias < 0.0f) alias += sample_rate_hz;
    if (alias > (0.5f * sample_rate_hz)) alias = sample_rate_hz - alias;
    return alias;
}

static float WrapPi(float phase_rad)
{
    while (phase_rad > (0.5f * PLL_TEST_TWO_PI)) phase_rad -= PLL_TEST_TWO_PI;
    while (phase_rad <= (-0.5f * PLL_TEST_TWO_PI)) phase_rad += PLL_TEST_TWO_PI;
    return phase_rad;
}

static void RequestSampleDivider(uint16_t divider)
{
    s_pending_sample_divider = divider;
}

/* Called at a DMA half boundary.  ARR is not preloaded, so the completed half
 * keeps its old tag and the next half uses the new exact sample interval. */
static void ApplyPendingSampleDividerIsr(void)
{
    uint16_t requested = s_pending_sample_divider;
    if ((requested != 0U) && (requested != s_active_sample_divider)) {
        TIM6->ARR = (uint32_t)requested - 1U;
        s_active_sample_divider = requested;
    }
}

static void InitFftWindow(void)
{
    uint32_t i;
    for (i = 0U; i < PLL_TEST_FFT_SIZE; ++i) {
        s_fft_window[i] = 0.5f - 0.5f *
            cosf(PLL_TEST_TWO_PI * (float)i /
                 (float)(PLL_TEST_FFT_SIZE - 1U));
    }
}

static void BeginFrequencySweep(void)
{
    memset(s_alias_hz, 0, sizeof(s_alias_hz));
    memset(s_alias_snr_db, 0, sizeof(s_alias_snr_db));
    s_alias_valid_mask = 0U;
    s_sweep_active = 1U;
    s_sweep_rate_index = 0U;
    s_sweep_block_count = 0U;
    s_sweep_attempt_count = 0U;
    s_sweep_first_bin = 0U;
    s_sweep_first_phase = 0.0f;
    s_sweep_first_alias_hz = 0.0f;
    s_pll.ready = 0U;
    s_pll.locked = 0U;
    s_pll.feedback_valid = 0U;
    s_mode = PLL_TEST_MODE_ACQUIRE;
    RequestSampleDivider(s_rate_dividers[0]);
}

static uint8_t EstimateAliasFft(uint32_t start,
                                float reference_mean,
                                float sample_rate_hz,
                                float *alias_hz,
                                uint16_t *peak_bin,
                                float *phase_rad,
                                float *snr_db)
{
    uint32_t i;
    uint32_t k;
    uint32_t first_bin = PLL_TEST_ALIAS_EDGE_BINS;
    uint32_t last_bin = PLL_TEST_FFT_SIZE / 2U -
                        PLL_TEST_ALIAS_EDGE_BINS;
    uint32_t best_bin = first_bin;
    float best_power = 0.0f;
    float noise_power = 0.0f;
    uint32_t noise_bins = 0U;
    float ym;
    float y0;
    float yp;
    float denominator;
    float delta = 0.0f;

    if ((alias_hz == NULL) || (peak_bin == NULL) ||
        (phase_rad == NULL) || (snr_db == NULL) ||
        (sample_rate_hz <= 0.0f)) {
        return 0U;
    }

    for (i = 0U; i < PLL_TEST_FFT_SIZE; ++i) {
        float sample = (float)(s_adc_dma[start + i] & 0xFFFFU) -
                       reference_mean;
        s_fft_buffer[2U * i] = sample * s_fft_window[i];
        s_fft_buffer[2U * i + 1U] = 0.0f;
    }
    arm_cfft_f32(&arm_cfft_sR_f32_len1024, s_fft_buffer, 0U, 1U);

    for (k = first_bin; k <= last_bin; ++k) {
        float re = s_fft_buffer[2U * k];
        float im = s_fft_buffer[2U * k + 1U];
        float power = re * re + im * im;
        if (power > best_power) {
            best_power = power;
            best_bin = k;
        }
    }
    if (best_power <= 1.0f) return 0U;

    for (k = first_bin; k <= last_bin; ++k) {
        if ((k + 2U >= best_bin) && (k <= best_bin + 2U)) continue;
        {
            float re = s_fft_buffer[2U * k];
            float im = s_fft_buffer[2U * k + 1U];
            noise_power += re * re + im * im;
            noise_bins++;
        }
    }
    if (noise_bins == 0U) return 0U;
    noise_power /= (float)noise_bins;
    *snr_db = 10.0f * log10f((best_power + 1.0f) /
                             (noise_power + 1.0f));
    if (*snr_db < PLL_TEST_ALIAS_MIN_SNR_DB) return 0U;

    ym = logf(s_fft_buffer[2U * (best_bin - 1U)] *
              s_fft_buffer[2U * (best_bin - 1U)] +
              s_fft_buffer[2U * (best_bin - 1U) + 1U] *
              s_fft_buffer[2U * (best_bin - 1U) + 1U] + 1.0f);
    y0 = logf(best_power + 1.0f);
    yp = logf(s_fft_buffer[2U * (best_bin + 1U)] *
              s_fft_buffer[2U * (best_bin + 1U)] +
              s_fft_buffer[2U * (best_bin + 1U) + 1U] *
              s_fft_buffer[2U * (best_bin + 1U) + 1U] + 1.0f);
    denominator = ym - 2.0f * y0 + yp;
    if (fabsf(denominator) > 1.0e-12f) {
        delta = 0.5f * (ym - yp) / denominator;
        delta = ClampFloat(delta, -0.5f, 0.5f);
    }

    *peak_bin = (uint16_t)best_bin;
    *phase_rad = atan2f(s_fft_buffer[2U * best_bin + 1U],
                        s_fft_buffer[2U * best_bin]);
    *alias_hz = ((float)best_bin + delta) * sample_rate_hz /
                (float)PLL_TEST_FFT_SIZE;
    return 1U;
}
static void UartSend(const char *format, ...)
{
    va_list args;
    int length;
    uint32_t primask;

    if ((format == NULL) || (s_uart_tx_busy != 0U)) return;
    va_start(args, format);
    length = vsnprintf(s_uart_tx_buffer, sizeof(s_uart_tx_buffer), format, args);
    va_end(args);
    if (length <= 0) return;
    if ((uint32_t)length >= sizeof(s_uart_tx_buffer)) {
        length = (int)sizeof(s_uart_tx_buffer) - 1;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    if (s_uart_tx_busy == 0U) {
        s_uart_tx_busy = 1U;
        if (HAL_UART_Transmit_IT(&huart1, (uint8_t *)s_uart_tx_buffer,
                                 (uint16_t)length) != HAL_OK) {
            s_uart_tx_busy = 0U;
        }
    }
    if ((primask & 1U) == 0U) __enable_irq();
}

static uint16_t PhaseDegToWord(double phase_deg)
{
    while (phase_deg >= 360.0) phase_deg -= 360.0;
    while (phase_deg < 0.0) phase_deg += 360.0;
    return (uint16_t)((phase_deg * PLL_TEST_POW_SCALE / 360.0) + 0.5);
}

/*
 * AD9910 PLL backend. A single 8-byte Profile0 write atomically applies
 * frequency, phase and amplitude. The DDS is initialized only once; no reset
 * or mode rewrite is performed in the control loop, so its RF output remains
 * continuous while ADC DMA continues acquiring the next half-buffer.
 */
static uint8_t ApplyPllOutput(float frequency_hz, float phase_step_deg)
{
    AD9910_Status status;
    uint32_t start_cycles;
    uint32_t elapsed_cycles;

    frequency_hz = ClampFloat(frequency_hz, 0.0f,
                              (float)PLL_TEST_MAX_OUTPUT_HZ);
    s_ad9910_phase_deg += (double)phase_step_deg;
    while (s_ad9910_phase_deg >= 360.0) s_ad9910_phase_deg -= 360.0;
    while (s_ad9910_phase_deg < 0.0) s_ad9910_phase_deg += 360.0;
    s_ad9910_phase_word = PhaseDegToWord(s_ad9910_phase_deg);

    start_cycles = DWT->CYCCNT;
    status = AD9910_UpdateSingleToneFast(frequency_hz,
                                          s_ad9910_phase_word,
                                          s_ad9910_amplitude);
    elapsed_cycles = DWT->CYCCNT - start_cycles;
    if (SystemCoreClock >= 1000000U) {
        s_ad9910_last_update_us =
            elapsed_cycles / (SystemCoreClock / 1000000U);
    }
    if (status != AD9910_OK) {
        s_ad9910_error_count++;
        return 0U;
    }

    s_ad9910_frequency_hz = frequency_hz;
    s_ad9910_update_count++;
    return 1U;
}

static uint8_t CrossingUpdate(PllTest_Crossing_t *state,
                              float sample,
                              double sample_index,
                              float hysteresis,
                              uint8_t reference_channel)
{
    uint8_t accepted = 0U;

    if (state->have_previous == 0U) {
        state->previous = sample;
        state->have_previous = 1U;
        return 0U;
    }
    if (sample < (state->centre - hysteresis)) state->armed = 1U;

    if ((state->armed != 0U) && (state->previous < state->centre) &&
        (sample >= state->centre)) {
        float denominator = sample - state->previous;
        double crossing = sample_index;
        if (fabsf(denominator) > 1.0e-9f) {
            crossing = sample_index - 1.0 +
                       (double)((state->centre - state->previous) / denominator);
        }
        state->latest_crossing = crossing;
        if (state->have_crossing != 0U) {
            double period = crossing - state->last_crossing;
            float candidate = (float)((double)PLL_TEST_SAMPLE_RATE_HZ / period);
            if ((period >= 8.0) &&
                (candidate >= PLL_TEST_MIN_AC_FREQ_HZ) &&
                (candidate <= (float)PLL_TEST_MAX_OUTPUT_HZ)) {
                state->period_samples = period;
                accepted = 1U;
                if (reference_channel != 0U) {
                    s_ref_period_sum += period;
                    s_ref_period_count++;
                }
            }
        }
        state->last_crossing = crossing;
        state->have_crossing = 1U;
        state->armed = 0U;
    }
    state->previous = sample;
    return accepted;
}

static uint8_t EstimateIqPhases(uint32_t start,
                                float reference_mean,
                                float feedback_mean,
                                float frequency_hz,
                                float sample_rate_hz,
                                float *reference_phase,
                                float *feedback_phase,
                                float *reference_amplitude,
                                float *feedback_amplitude)
{
    uint32_t i;
    float signed_alias_hz;
    float angle_step;
    float cos_step;
    float sin_step;
    float oscillator_cos = 1.0f;
    float oscillator_sin = 0.0f;
    float ref_i = 0.0f;
    float ref_q = 0.0f;
    float fb_i = 0.0f;
    float fb_q = 0.0f;

    if ((sample_rate_hz <= 0.0f) || !isfinite(frequency_hz)) return 0U;
    signed_alias_hz = fmodf(frequency_hz, sample_rate_hz);
    if (signed_alias_hz > (0.5f * sample_rate_hz)) {
        signed_alias_hz -= sample_rate_hz;
    } else if (signed_alias_hz <= (-0.5f * sample_rate_hz)) {
        signed_alias_hz += sample_rate_hz;
    }
    if ((fabsf(signed_alias_hz) * (float)PLL_TEST_ADC_HALF_SAMPLES /
         sample_rate_hz) < 1.25f) {
        return 0U;
    }

    /* Reduce the RF frequency to its signed alias before sin/cos.  This avoids
     * loss of float angle precision at 40 MHz while retaining physical phase. */
    angle_step = PLL_TEST_TWO_PI * signed_alias_hz / sample_rate_hz;
    cos_step = cosf(angle_step);
    sin_step = sinf(angle_step);

    for (i = 0U; i < PLL_TEST_ADC_HALF_SAMPLES; ++i) {
        uint32_t packed = s_adc_dma[start + i];
        float ref = (float)(packed & 0xFFFFU) - reference_mean;
        float fb = (float)((packed >> 16U) & 0xFFFFU) - feedback_mean;
        float next_cos;
        float next_sin;

        ref_i += ref * oscillator_cos;
        ref_q -= ref * oscillator_sin;
        fb_i += fb * oscillator_cos;
        fb_q -= fb * oscillator_sin;
        next_cos = oscillator_cos * cos_step - oscillator_sin * sin_step;
        next_sin = oscillator_sin * cos_step + oscillator_cos * sin_step;
        oscillator_cos = next_cos;
        oscillator_sin = next_sin;

        if ((i & 127U) == 127U) {
            float norm = oscillator_cos * oscillator_cos +
                         oscillator_sin * oscillator_sin;
            float correction = 0.5f * (3.0f - norm);
            oscillator_cos *= correction;
            oscillator_sin *= correction;
        }
    }

    *reference_phase = atan2f(ref_q, ref_i);
    *feedback_phase = atan2f(fb_q, fb_i);
    *reference_amplitude =
        2.0f * sqrtf(ref_i * ref_i + ref_q * ref_q) /
        ((float)PLL_TEST_ADC_HALF_SAMPLES * PLL_TEST_ADC_SCALE);
    *feedback_amplitude =
        2.0f * sqrtf(fb_i * fb_i + fb_q * fb_q) /
        ((float)PLL_TEST_ADC_HALF_SAMPLES * PLL_TEST_ADC_SCALE);
    return 1U;
}

static void ConfigurePll(float frequency_hz, float sample_rate_hz)
{
    float update_rate = sample_rate_hz /
                        (float)PLL_TEST_ADC_HALF_SAMPLES;
    float maximum_bw = fmaxf(0.05f, frequency_hz * 0.20f);
    float actual_bw = ClampFloat(s_loop_bandwidth_hz, 0.05f, maximum_bw);
    float branch_guard_hz = fminf(frequency_hz * 0.01f,
                                  sample_rate_hz * 0.20f);

    SoftPll_DefaultConfig(&s_pll_config, frequency_hz);
    s_pll_config.max_freq_offset_hz = fmaxf(10.0f, branch_guard_hz);
    s_pll_config.max_freq_step_hz = fmaxf(
        1.0f, fminf(frequency_hz * 0.001f, sample_rate_hz * 0.02f));
    s_pll_config.min_reference_amp = PLL_TEST_MIN_REFERENCE_IQ_AMP;
    s_pll_config.min_feedback_amp = PLL_TEST_MIN_FEEDBACK_IQ_AMP;
    /* The type-II frequency loop already closes phase error.  Incremental
     * POW kicks add a second phase integrator and reduce capture stability. */
    s_pll_config.phase_kp = 0.0f;
    s_pll_config.phase_ki = 0.0f;
    s_pll_config.freq_kd = 0.0f;
    s_pll_config.lock_rate_hz = fmaxf(0.02f, actual_bw * 0.05f);
    (void)SoftPll_DesignType2(&s_pll_config, actual_bw,
                              PLL_TEST_DEFAULT_DAMPING, update_rate);
    SoftPll_Init(&s_pll, &s_pll_config);
    (void)ApplyPllOutput(frequency_hz, 0.0f);
    s_mode = PLL_TEST_MODE_AC_LOCK;
    s_lock_start_tick = HAL_GetTick();
    s_last_locked_tick = s_lock_start_tick;
}

static uint8_t ResolveSweepFrequency(float *frequency_hz, float *rms_hz)
{
    uint8_t i;
    uint8_t valid_count = 0U;
    uint8_t base_index = 0U;
    uint32_t harmonic;
    uint32_t max_harmonic;
    float best_score = 1.0e30f;
    float second_score = 1.0e30f;
    float best_frequency = 0.0f;
    float base_rate;

    if ((frequency_hz == NULL) || (rms_hz == NULL)) return 0U;
    for (i = 0U; i < PLL_TEST_RATE_COUNT; ++i) {
        if ((s_alias_valid_mask & (1U << i)) != 0U) {
            if (valid_count == 0U) base_index = i;
            valid_count++;
        }
    }
    if (valid_count < PLL_TEST_SWEEP_MIN_RATES) return 0U;

    base_rate = SampleRateFromDivider(s_rate_dividers[base_index]);
    max_harmonic = (uint32_t)((float)PLL_TEST_MAX_OUTPUT_HZ / base_rate) + 2U;
    for (harmonic = 0U; harmonic <= max_harmonic; ++harmonic) {
        int sign_index;
        for (sign_index = -1; sign_index <= 1; sign_index += 2) {
            float seed = (float)harmonic * base_rate +
                         (float)sign_index * s_alias_hz[base_index];
            float estimate_sum = 0.0f;
            uint8_t estimate_count = 0U;
            float refined;
            float error_sum = 0.0f;
            float score;

            if ((seed < PLL_TEST_MIN_AC_FREQ_HZ) ||
                (seed > (float)PLL_TEST_MAX_OUTPUT_HZ)) continue;
            for (i = 0U; i < PLL_TEST_RATE_COUNT; ++i) {
                if ((s_alias_valid_mask & (1U << i)) != 0U) {
                    float rate = SampleRateFromDivider(s_rate_dividers[i]);
                    int32_t k = (int32_t)floorf(seed / rate + 0.5f);
                    float signed_delta = seed - (float)k * rate;
                    estimate_sum += (float)k * rate +
                        ((signed_delta >= 0.0f) ? s_alias_hz[i]
                                                : -s_alias_hz[i]);
                    estimate_count++;
                }
            }
            if (estimate_count < PLL_TEST_SWEEP_MIN_RATES) continue;
            refined = estimate_sum / (float)estimate_count;
            if ((refined < PLL_TEST_MIN_AC_FREQ_HZ) ||
                (refined > (float)PLL_TEST_MAX_OUTPUT_HZ)) continue;

            for (i = 0U; i < PLL_TEST_RATE_COUNT; ++i) {
                if ((s_alias_valid_mask & (1U << i)) != 0U) {
                    float rate = SampleRateFromDivider(s_rate_dividers[i]);
                    float error = FoldAliasHz(refined, rate) - s_alias_hz[i];
                    error_sum += error * error;
                }
            }
            score = sqrtf(error_sum / (float)valid_count);
            if ((score < best_score) &&
                ((best_frequency == 0.0f) ||
                 (fabsf(refined - best_frequency) > 1000.0f))) {
                second_score = best_score;
                best_score = score;
                best_frequency = refined;
            } else if ((score < second_score) &&
                       (fabsf(refined - best_frequency) > 1000.0f)) {
                second_score = score;
            }
        }
    }

    if ((best_frequency <= 0.0f) ||
        (best_score > PLL_TEST_SWEEP_MAX_RMS_HZ)) return 0U;
    /* An ambiguity this close means the analog/FFT data do not justify a
     * 1..40 MHz branch decision; rescan instead of commanding a wrong RF. */
    if ((second_score < 1.0e29f) &&
        ((second_score - best_score) < 250.0f)) return 0U;
    *frequency_hz = best_frequency;
    *rms_hz = best_score;
    return 1U;
}

static uint8_t SelectLockRate(void)
{
    uint8_t i;
    uint8_t best_index = 0U;
    float best_margin = -1.0f;
    for (i = 0U; i < PLL_TEST_RATE_COUNT; ++i) {
        if ((s_alias_valid_mask & (1U << i)) != 0U) {
            float rate = SampleRateFromDivider(s_rate_dividers[i]);
            float alias = s_alias_hz[i];
            float margin = fminf(alias, 0.5f * rate - alias);
            if (margin > best_margin) {
                best_margin = margin;
                best_index = i;
            }
        }
    }
    return best_index;
}

static void FinishFrequencySweep(void)
{
    float resolved_hz;
    float rms_hz;
    if (ResolveSweepFrequency(&resolved_hz, &rms_hz) != 0U) {
        uint8_t lock_rate_index = SelectLockRate();
        float lock_rate_hz =
            SampleRateFromDivider(s_rate_dividers[lock_rate_index]);
        s_lock_sample_divider = s_rate_dividers[lock_rate_index];
        s_frequency_estimate_hz = resolved_hz;
        s_last_alias_hz = FoldAliasHz(resolved_hz, lock_rate_hz);
        ConfigurePll(resolved_hz, lock_rate_hz);
        RequestSampleDivider(s_lock_sample_divider);
        s_sweep_active = 0U;
        /* Print every measured alias.  A single undersampled alias is
         * ambiguous; this vector is the evidence used by the multi-rate
         * resolver to distinguish the actual 0..40 MHz RF frequency. */
        UartSend("[RF_ACQ] fin_Hz=%lu rms_Hz=%lu div=%u mask=0x%02X "
                 "a0=%lu a1=%lu a2=%lu a3=%lu "
                 "s0_dB10=%ld s1_dB10=%ld s2_dB10=%ld s3_dB10=%ld\r\n",
                 (unsigned long)(resolved_hz + 0.5f),
                 (unsigned long)(rms_hz + 0.5f),
                 (unsigned int)s_lock_sample_divider,
                 (unsigned int)s_alias_valid_mask,
                 (unsigned long)(s_alias_hz[0] + 0.5f),
                 (unsigned long)(s_alias_hz[1] + 0.5f),
                 (unsigned long)(s_alias_hz[2] + 0.5f),
                 (unsigned long)(s_alias_hz[3] + 0.5f),
                 (long)(s_alias_snr_db[0] * 10.0f + 0.5f),
                 (long)(s_alias_snr_db[1] * 10.0f + 0.5f),
                 (long)(s_alias_snr_db[2] * 10.0f + 0.5f),
                 (long)(s_alias_snr_db[3] * 10.0f + 0.5f));
    } else {
        s_sweep_fail_count++;
        UartSend("[RF_ACQ] retry=%lu mask=0x%02X "
                 "a0=%lu a1=%lu a2=%lu a3=%lu\r\n",
                 (unsigned long)s_sweep_fail_count,
                 (unsigned int)s_alias_valid_mask,
                 (unsigned long)(s_alias_hz[0] + 0.5f),
                 (unsigned long)(s_alias_hz[1] + 0.5f),
                 (unsigned long)(s_alias_hz[2] + 0.5f),
                 (unsigned long)(s_alias_hz[3] + 0.5f));
        BeginFrequencySweep();
    }
}

static void AdvanceSweepRate(void)
{
    s_sweep_block_count = 0U;
    s_sweep_attempt_count = 0U;
    s_sweep_rate_index++;
    if (s_sweep_rate_index < PLL_TEST_RATE_COUNT) {
        RequestSampleDivider(s_rate_dividers[s_sweep_rate_index]);
    } else {
        FinishFrequencySweep();
    }
}
static void ProcessSweepBlock(uint32_t start,
                              float reference_mean,
                              uint32_t reference_p2p,
                              uint16_t divider)
{
    float sample_rate_hz;
    float alias_hz;
    float phase_rad;
    float snr_db;
    uint16_t peak_bin;
    uint16_t expected_divider;

    if ((s_sweep_active == 0U) ||
        (s_sweep_rate_index >= PLL_TEST_RATE_COUNT)) return;
    expected_divider = s_rate_dividers[s_sweep_rate_index];
    if (divider != expected_divider) {
        RequestSampleDivider(expected_divider);
        return;
    }

    sample_rate_hz = SampleRateFromDivider(divider);
    s_sweep_attempt_count++;
    if (reference_p2p < PLL_TEST_MIN_SIGNAL_P2P_CODE) {
        s_sweep_block_count = 0U;
        if (s_sweep_attempt_count >= 4U) AdvanceSweepRate();
        return;
    }
    if (EstimateAliasFft(start, reference_mean, sample_rate_hz,
                         &alias_hz, &peak_bin, &phase_rad, &snr_db) == 0U) {
        s_sweep_block_count = 0U;
        if (s_sweep_attempt_count >= 4U) AdvanceSweepRate();
        return;
    }

    s_last_alias_hz = alias_hz;
    s_last_alias_snr_db = snr_db;
    if (s_sweep_block_count == 0U) {
        s_sweep_first_alias_hz = alias_hz;
        s_sweep_first_bin = peak_bin;
        s_sweep_first_phase = phase_rad;
        s_sweep_block_count = 1U;
        return;
    }

    if (peak_bin == s_sweep_first_bin) {
        float phase_delta = WrapPi(phase_rad - s_sweep_first_phase);
        float phase_alias =
            ((float)peak_bin + phase_delta / PLL_TEST_TWO_PI) *
            sample_rate_hz / (float)PLL_TEST_FFT_SIZE;
        float bin_hz = sample_rate_hz / (float)PLL_TEST_FFT_SIZE;
        float average_alias = 0.5f * (alias_hz + s_sweep_first_alias_hz);
        if ((phase_alias > bin_hz) &&
            (phase_alias < 0.5f * sample_rate_hz - bin_hz) &&
            (fabsf(phase_alias - average_alias) <= bin_hz)) {
            alias_hz = phase_alias;
        } else {
            alias_hz = average_alias;
        }
    } else {
        alias_hz = 0.5f * (alias_hz + s_sweep_first_alias_hz);
    }

    s_alias_hz[s_sweep_rate_index] = alias_hz;
    s_alias_snr_db[s_sweep_rate_index] = snr_db;
    s_alias_valid_mask |= (uint8_t)(1U << s_sweep_rate_index);
    s_last_alias_hz = alias_hz;
    AdvanceSweepRate();
}
static void ProcessFrequencyEvent(void)
{
    float candidate;
    float relative_change;
    float coarse_error;
    float coarse_threshold;

    if (s_frequency_event == 0U) return;
    s_frequency_event = 0U;
    candidate = s_latest_frequency_hz;
    if (s_frequency_estimate_hz <= 0.0f) {
        s_frequency_estimate_hz = candidate;
        s_frequency_confirm_count = 1U;
    } else {
        relative_change = fabsf(candidate - s_frequency_estimate_hz) /
                          fmaxf(s_frequency_estimate_hz, 1.0f);
        if (relative_change > 0.20f) {
            s_frequency_estimate_hz = candidate;
            s_frequency_confirm_count = 1U;
        } else {
            s_frequency_estimate_hz +=
                0.10f * (candidate - s_frequency_estimate_hz);
            if (s_frequency_confirm_count < UINT16_MAX) {
                s_frequency_confirm_count++;
            }
        }
    }

    if ((s_pll.ready == 0U) || (s_mode != PLL_TEST_MODE_AC_LOCK)) {
        /* A first single-period estimate can be tens of hertz off.  Wait for
         * several whole-buffer multi-cycle estimates before starting the
         * wrapped phase detector, otherwise it can sit outside pull-in range. */
        if (s_frequency_confirm_count >= 6U) {
            ConfigurePll(s_frequency_estimate_hz, s_last_block_sample_rate_hz);
        }
        return;
    }

    coarse_error = fabsf(s_frequency_estimate_hz -
                         s_pll.cfg.nominal_freq_hz);
    coarse_threshold = fmaxf(2.0f, 0.001f * s_pll.cfg.nominal_freq_hz);
    if (coarse_error > fmaxf(2.0f,
                             0.10f * s_pll.cfg.nominal_freq_hz)) {
        ConfigurePll(s_frequency_estimate_hz, s_last_block_sample_rate_hz);
    } else if ((coarse_error > coarse_threshold) &&
               ((s_pll.feedback_valid == 0U) ||
                (fabsf(s_pll.phase_rate_hz) >
                 fmaxf(2.0f, 2.0f * s_loop_bandwidth_hz)))) {
        /* During acquisition the phase detector can be invalid because the
         * DDS and reference are too far apart for coherent IQ correlation.
         * Feed the measured reference frequency directly to the DDS first;
         * the normal type-II phase loop takes over as soon as feedback is
         * valid. This also gives deterministic frequency-only behaviour. */
        if (SoftPll_Retune(&s_pll, s_frequency_estimate_hz, 1U) != 0U) {
            (void)ApplyPllOutput(s_pll.dds_freq_cmd_hz, 0.0f);
        }
    }
}

static void ProcessAdcHalf(uint32_t start, uint8_t half_index)
{
    uint32_t i;
    uint32_t now = HAL_GetTick();
    uint32_t ref_min = 65535U;
    uint32_t ref_max = 0U;
    uint64_t ref_sum = 0U;
    uint64_t fb_sum = 0U;
    uint32_t ref_p2p;
    float reference_mean;
    float feedback_mean;
    uint16_t divider = s_half_sample_divider[half_index & 1U];
    float sample_rate_hz = SampleRateFromDivider(divider);
    SoftPll_PhaseInput_t phase_input;
    float reference_phase;
    float feedback_phase;
    float reference_amplitude;
    float feedback_amplitude;
    uint8_t phase_valid;

    CacheInvalidate(&s_adc_dma[start],
                    PLL_TEST_ADC_HALF_SAMPLES * sizeof(s_adc_dma[0]));
    for (i = 0U; i < PLL_TEST_ADC_HALF_SAMPLES; ++i) {
        uint32_t packed = s_adc_dma[start + i];
        uint32_t ref = packed & 0xFFFFU;
        uint32_t fb = (packed >> 16U) & 0xFFFFU;
        ref_sum += ref;
        fb_sum += fb;
        if (ref < ref_min) ref_min = ref;
        if (ref > ref_max) ref_max = ref;
    }
    reference_mean = (float)ref_sum / (float)PLL_TEST_ADC_HALF_SAMPLES;
    feedback_mean = (float)fb_sum / (float)PLL_TEST_ADC_HALF_SAMPLES;
    ref_p2p = ref_max - ref_min;
    s_last_block_sample_rate_hz = sample_rate_hz;

    if ((ref_p2p >= PLL_TEST_ACTIVITY_P2P_CODE) ||
        (fabsf(reference_mean - s_last_ref_mean) >=
         (0.5f * (float)PLL_TEST_ACTIVITY_P2P_CODE))) {
        s_last_ac_tick = now;
    }

    if (s_sweep_active != 0U) {
        ProcessSweepBlock(start, reference_mean, ref_p2p, divider);
        s_last_ref_mean = reference_mean;
        s_last_fb_mean = feedback_mean;
        return;
    }

    if (divider != s_lock_sample_divider) {
        RequestSampleDivider(s_lock_sample_divider);
        s_last_ref_mean = reference_mean;
        s_last_fb_mean = feedback_mean;
        return;
    }

    if ((now - s_last_ac_tick) >= PLL_TEST_HOLDOVER_CONFIRM_MS) {
        /* Keep the last AD9910 RF command continuous on reference loss. */
        s_mode = PLL_TEST_MODE_HOLDOVER;
        s_pll.locked = 0U;
        s_last_ref_mean = reference_mean;
        s_last_fb_mean = feedback_mean;
        return;
    }

    phase_valid = EstimateIqPhases(start, reference_mean, feedback_mean,
                                   s_ad9910_frequency_hz, sample_rate_hz,
                                   &reference_phase, &feedback_phase,
                                   &reference_amplitude,
                                   &feedback_amplitude);
    s_last_phase_valid = phase_valid;
    if (phase_valid != 0U) {
        s_last_reference_iq_amp = reference_amplitude;
        s_last_feedback_iq_amp = feedback_amplitude;
    } else {
        s_last_reference_iq_amp = 0.0f;
        s_last_feedback_iq_amp = 0.0f;
    }
    if ((phase_valid != 0U) && (s_pll.ready != 0U)) {
        phase_input.reference_phase = reference_phase;
        phase_input.feedback_phase = feedback_phase;
        phase_input.target_offset = SoftPll_DegToRad(s_target_phase_deg);
        phase_input.fixed_cal = SoftPll_DegToRad(s_fixed_phase_cal_deg);
        phase_input.dt_s = (float)PLL_TEST_ADC_HALF_SAMPLES / sample_rate_hz;
        phase_input.reference_amp = reference_amplitude;
        phase_input.feedback_amp = feedback_amplitude;
        if (SoftPll_UpdatePhase(&s_pll, &phase_input,
                                SOFTPLL_UNIT_RAD) != 0U) {
            (void)ApplyPllOutput(s_pll.dds_freq_cmd_hz,
                                 s_pll.phase_step_deg);
        }
    } else if (s_pll.ready != 0U) {
        s_pll.feedback_valid = 0U;
        s_pll.locked = 0U;
    }

    s_last_alias_hz = FoldAliasHz(s_ad9910_frequency_hz, sample_rate_hz);
    if (SoftPll_IsLocked(&s_pll) != 0U) {
        s_last_locked_tick = now;
        s_frequency_estimate_hz +=
            0.02f * (s_pll.dds_freq_cmd_hz - s_frequency_estimate_hz);
    } else if (((now - s_lock_start_tick) >= PLL_TEST_REACQUIRE_MS) &&
               ((now - s_last_locked_tick) >= PLL_TEST_REACQUIRE_MS)) {
        BeginFrequencySweep();
    }

    s_last_ref_mean = reference_mean;
    s_last_fb_mean = feedback_mean;
}
static int32_t ScaleToInt(float value, float scale)
{
    float scaled = value * scale;
    return (int32_t)((scaled >= 0.0f) ? (scaled + 0.5f) : (scaled - 0.5f));
}

static const char *ModeName(void)
{
    if (s_mode == PLL_TEST_MODE_HOLDOVER) return "HOLD";
    if (s_mode == PLL_TEST_MODE_AC_LOCK) return "AC";
    return "ACQ";
}

static void SendTelemetry(void)
{
    uint32_t now = HAL_GetTick();
    if ((now - s_last_telemetry_tick) < s_telemetry_period_ms) return;
    s_last_telemetry_tick = now;

    UartSend("[PLL] mode=%s lock=%u fin_Hz=%lu fout_Hz=%lu "
             "alias_Hz=%lu fs_Hz=%lu scan=%u mask=0x%02X "
             "err_mdeg=%ld drift_mHz=%ld snr_dB10=%ld pv=%u "
             "ref_u=%ld fb_u=%ld ref_mV=%ld fb_mV=%ld "
             "adc_ovr=%lu dds_err=%lu "
             "dds_us=%lu asf=%u\r\n",
             ModeName(), (unsigned int)SoftPll_IsLocked(&s_pll),
             (unsigned long)(fmaxf(0.0f, s_frequency_estimate_hz) + 0.5f),
             (unsigned long)(fmaxf(0.0f, s_ad9910_frequency_hz) + 0.5f),
             (unsigned long)(fmaxf(0.0f, s_last_alias_hz) + 0.5f),
             (unsigned long)(fmaxf(0.0f, s_last_block_sample_rate_hz) + 0.5f),
             (unsigned int)s_sweep_rate_index,
             (unsigned int)s_alias_valid_mask,
             (long)ScaleToInt(s_pll.error_deg, 1000.0f),
             (long)ScaleToInt(s_pll.phase_rate_hz, 1000.0f),
             (long)ScaleToInt(s_last_alias_snr_db, 10.0f),
             (unsigned int)s_last_phase_valid,
             (long)ScaleToInt(s_last_reference_iq_amp, 1000000.0f),
             (long)ScaleToInt(s_last_feedback_iq_amp, 1000000.0f),
             (long)ScaleToInt(s_last_ref_mean * 3.3f / 65.535f, 1.0f),
             (long)ScaleToInt(s_last_fb_mean * 3.3f / 65.535f, 1.0f),
             (unsigned long)s_adc_overrun,
             (unsigned long)s_ad9910_error_count,
             (unsigned long)s_ad9910_last_update_us,
             (unsigned int)s_ad9910_amplitude);
}
static uint8_t ParseValue(const char *line, float *value)
{
    const char *space;
    char *end;
    if ((line == NULL) || (value == NULL)) return 0U;
    space = strchr(line, ' ');
    if (space == NULL) return 0U;
    while (*space == ' ') space++;
    *value = strtof(space, &end);
    return ((end != space) && isfinite(*value)) ? 1U : 0U;
}

static void ProcessCommand(char *line)
{
    float value;
    float update_rate = (float)PLL_TEST_SAMPLE_RATE_HZ /
                        (float)PLL_TEST_ADC_HALF_SAMPLES;

    if ((strcmp(line, "help") == 0) || (strcmp(line, "?") == 0)) {
        UartSend("[CMD] kp <Hz/deg>, ki <Hz/(deg*s)>, kd <ratio>, "
                 "pkp <deg/deg>, bw <Hz>, phase <deg>, cal <deg>, "
                 "telem <ms>, status, scan\r\n");
    } else if (strcmp(line, "status") == 0) {
        s_last_telemetry_tick = HAL_GetTick() - s_telemetry_period_ms;
        SendTelemetry();
    } else if (strcmp(line, "scan") == 0) {
        BeginFrequencySweep();
        UartSend("[CMD] RF rescan started\r\n");
    } else if ((strncmp(line, "kp ", 3U) == 0) && ParseValue(line, &value)) {
        s_pll.cfg.freq_kp_hz_per_deg = value;
        UartSend("[CMD] kp_u=%ld\r\n", (long)ScaleToInt(value, 1000000.0f));
    } else if ((strncmp(line, "ki ", 3U) == 0) && ParseValue(line, &value)) {
        s_pll.cfg.freq_ki_hz_per_deg_s = value;
        UartSend("[CMD] ki_u=%ld\r\n", (long)ScaleToInt(value, 1000000.0f));
    } else if ((strncmp(line, "kd ", 3U) == 0) && ParseValue(line, &value)) {
        s_pll.cfg.freq_kd = value;
        UartSend("[CMD] kd_u=%ld\r\n", (long)ScaleToInt(value, 1000000.0f));
    } else if ((strncmp(line, "pkp ", 4U) == 0) && ParseValue(line, &value)) {
        s_pll.cfg.phase_kp = value;
        UartSend("[CMD] pkp_u=%ld\r\n", (long)ScaleToInt(value, 1000000.0f));
    } else if ((strncmp(line, "bw ", 3U) == 0) && ParseValue(line, &value) &&
               (value >= 0.01f) && (value < 0.20f * update_rate)) {
        s_loop_bandwidth_hz = value;
        if (s_pll.ready != 0U) {
            (void)SoftPll_DesignType2(&s_pll.cfg, value,
                                      PLL_TEST_DEFAULT_DAMPING, update_rate);
        }
        UartSend("[CMD] bw_mHz=%ld kp_u=%ld ki_u=%ld\r\n",
                 (long)ScaleToInt(value, 1000.0f),
                 (long)ScaleToInt(s_pll.cfg.freq_kp_hz_per_deg, 1000000.0f),
                 (long)ScaleToInt(s_pll.cfg.freq_ki_hz_per_deg_s, 1000000.0f));
    } else if ((strncmp(line, "phase ", 6U) == 0) && ParseValue(line, &value)) {
        s_target_phase_deg = SoftPll_Normalize180Deg(value);
        UartSend("[CMD] target_mdeg=%ld\r\n",
                 (long)ScaleToInt(s_target_phase_deg, 1000.0f));
    } else if ((strncmp(line, "cal ", 4U) == 0) && ParseValue(line, &value)) {
        s_fixed_phase_cal_deg = SoftPll_Normalize180Deg(value);
        UartSend("[CMD] cal_mdeg=%ld\r\n",
                 (long)ScaleToInt(s_fixed_phase_cal_deg, 1000.0f));
    } else if ((strncmp(line, "telem ", 6U) == 0) && ParseValue(line, &value) &&
               (value >= 20.0f) && (value <= 10000.0f)) {
        s_telemetry_period_ms = (uint32_t)(value + 0.5f);
        UartSend("[CMD] telem_ms=%lu\r\n",
                 (unsigned long)s_telemetry_period_ms);
    } else {
        UartSend("[CMD] ERROR unknown/invalid: %s\r\n", line);
    }
}

HAL_StatusTypeDef PllTest_Init(void)
{
    HAL_StatusTypeDef status;
    AD9910_Status dds_status;

    memset((void *)s_adc_dma, 0, sizeof(s_adc_dma));
    memset(&s_pll, 0, sizeof(s_pll));
    memset(&s_ref_cross, 0, sizeof(s_ref_cross));
    memset(&s_fb_cross, 0, sizeof(s_fb_cross));
    s_ref_cross.centre = 32768.0f;
    s_fb_cross.centre = 32768.0f;
    s_adc_pending = 0U;
    s_adc_overrun = 0U;
    s_ad9910_phase_deg = 0.0;
    s_ad9910_phase_word = 0U;
    s_ad9910_frequency_hz = PLL_TEST_DEFAULT_START_FREQ_HZ;
    s_ad9910_update_count = 0U;
    s_ad9910_error_count = 0U;
    s_ad9910_last_update_us = 0U;
    s_sample_counter = 0U;
    s_ref_period_sum = 0.0;
    s_ref_period_count = 0U;
    s_frequency_estimate_hz = 0.0f;
    s_frequency_event = 0U;
    s_frequency_confirm_count = 0U;
    s_mode = PLL_TEST_MODE_ACQUIRE;
    s_last_ac_tick = HAL_GetTick();
    s_last_telemetry_tick = HAL_GetTick();
    s_active_sample_divider = s_rate_dividers[0];
    s_pending_sample_divider = s_rate_dividers[0];
    s_half_sample_divider[0] = s_rate_dividers[0];
    s_half_sample_divider[1] = s_rate_dividers[0];
    s_lock_sample_divider = s_rate_dividers[0];
    s_last_block_sample_rate_hz =
        SampleRateFromDivider(s_rate_dividers[0]);
    s_last_alias_hz = 0.0f;
    s_last_alias_snr_db = 0.0f;
    s_last_reference_iq_amp = 0.0f;
    s_last_feedback_iq_amp = 0.0f;
    s_last_phase_valid = 0U;
    s_sweep_fail_count = 0U;
    s_lock_start_tick = HAL_GetTick();
    s_last_locked_tick = s_lock_start_tick;
    InitFftWindow();
    BeginFrequencySweep();
    CacheClean(s_adc_dma, sizeof(s_adc_dma));
    CacheInvalidate(s_adc_dma, sizeof(s_adc_dma));

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    if (HAL_UART_Receive_IT(&huart1, &s_uart_rx_byte, 1U) != HAL_OK) {
        return HAL_ERROR;
    }
    if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_CALIB_OFFSET,
                                    ADC_SINGLE_ENDED) != HAL_OK) {
        return HAL_ERROR;
    }
    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET,
                                    ADC_SINGLE_ENDED) != HAL_OK) {
        return HAL_ERROR;
    }

    /* One power-up initialization; later PLL updates never reset the DDS. */
    dds_status = AD9910_Init(NULL);
    if (dds_status != AD9910_OK) return HAL_ERROR;
    if (ApplyPllOutput(PLL_TEST_DEFAULT_START_FREQ_HZ, 0.0f) == 0U) {
        return HAL_ERROR;
    }

    status = HAL_ADCEx_MultiModeStart_DMA(&hadc1, s_adc_dma,
                                          PLL_TEST_ADC_TOTAL_SAMPLES);
    if (status != HAL_OK) return status;

    s_running = 1U;
    status = HAL_TIM_Base_Start(&htim6);
    if (status != HAL_OK) {
        PllTest_Stop();
        return status;
    }

    UartSend("\r\n[PLL_TEST] READY PC4=ADC1_INP4 PC5=ADC2_INP8 "
             "OUT=AD9910 range=0..%luHz multirate=%u UART1=115200 8N1\r\n",
             (unsigned long)PLL_TEST_MAX_OUTPUT_HZ,
             (unsigned int)PLL_TEST_RATE_COUNT);
    return HAL_OK;
}

void PllTest_Stop(void)
{
    s_running = 0U;
    (void)HAL_TIM_Base_Stop(&htim6);
    (void)HAL_ADCEx_MultiModeStop_DMA(&hadc1);
    /* Deliberately leave AD9910 running continuously at the last command. */
    s_mode = PLL_TEST_MODE_HOLDOVER;
}

void PllTest_Process(void)
{
    uint8_t ready_half = 0U;
    char command[PLL_TEST_UART_LINE_SIZE];
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    if ((s_adc_pending & 0x01U) != 0U) {
        s_adc_pending &= (uint8_t)~0x01U;
        ready_half = 1U;
    } else if ((s_adc_pending & 0x02U) != 0U) {
        s_adc_pending &= (uint8_t)~0x02U;
        ready_half = 2U;
    }
    if (s_uart_command_ready != 0U) {
        memcpy(command, s_uart_command, sizeof(command));
        command[sizeof(command) - 1U] = '\0';
        s_uart_command_ready = 0U;
    } else {
        command[0] = '\0';
    }
    if ((primask & 1U) == 0U) __enable_irq();

    if (ready_half == 1U) {
        ProcessAdcHalf(0U, 0U);
    } else if (ready_half == 2U) {
        ProcessAdcHalf(PLL_TEST_ADC_HALF_SAMPLES, 1U);
    }
    if (command[0] != '\0') ProcessCommand(command);
    SendTelemetry();
}

void ADC_Sample_HalfReadyHook(void)
{
    if (s_running == 0U) return;
    s_half_sample_divider[0] = s_active_sample_divider;
    ApplyPendingSampleDividerIsr();
    if ((s_adc_pending & 0x01U) != 0U) s_adc_overrun++;
    s_adc_pending |= 0x01U;
}

void ADC_Sample_FullReadyHook(void)
{
    if (s_running == 0U) return;
    s_half_sample_divider[1] = s_active_sample_divider;
    ApplyPendingSampleDividerIsr();
    if ((s_adc_pending & 0x02U) != 0U) s_adc_overrun++;
    s_adc_pending |= 0x02U;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if ((huart == NULL) || (huart->Instance != USART1)) return;

    if ((s_uart_rx_byte == '\r') || (s_uart_rx_byte == '\n')) {
        if ((s_uart_rx_length > 0U) && (s_uart_command_ready == 0U)) {
            memcpy(s_uart_command, s_uart_rx_line, s_uart_rx_length);
            s_uart_command[s_uart_rx_length] = '\0';
            s_uart_command_ready = 1U;
        }
        /* Always discard the completed line, even if the prior command
         * has not yet been consumed by the main loop. */
        s_uart_rx_length = 0U;
    } else if (s_uart_rx_length < (PLL_TEST_UART_LINE_SIZE - 1U)) {
        s_uart_rx_line[s_uart_rx_length++] = (char)s_uart_rx_byte;
    } else {
        s_uart_rx_length = 0U;
    }
    (void)HAL_UART_Receive_IT(&huart1, &s_uart_rx_byte, 1U);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if ((huart != NULL) && (huart->Instance == USART1)) s_uart_tx_busy = 0U;
}

#else
HAL_StatusTypeDef PllTest_Init(void) { return HAL_ERROR; }
void PllTest_Process(void) { }
void PllTest_Stop(void) { }
#endif /* PLL_SELFTEST_ENABLE */
