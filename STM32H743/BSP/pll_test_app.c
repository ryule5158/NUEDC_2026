/**
 * @file pll_test_app.c
 * @brief PC4/PC5 simultaneous ADC -> generic SoftPll -> FPGA/AD9708 adapter.
 *
 * ADC DMA uses a circular buffer split into two halves, so acquisition and
 * CPU phase processing run concurrently. SoftPll remains hardware-independent:
 * this file translates its frequency/phase commands to the FPGA AD9708 DDS.
 */
#include "pll_test_app.h"
#include "pll_selftest_config.h"
#include "adc_dual_capture.h"
#include "ad9708.h"
#include "fpga_boot_restore.h"
#include "spi.h"
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
#define PLL_TEST_UART_TX_SIZE       640U
#define PLL_TEST_ADC_SCALE          32768.0f
#define PLL_TEST_TWO_PI             6.2831853071795864769f
#define PLL_TEST_PHASE_WORD_SCALE   4294967296.0

#define PLL_TEST_RATE_COUNT         4U
#define PLL_TEST_FFT_SIZE           PLL_TEST_ADC_HALF_SAMPLES
#define PLL_TEST_SWEEP_MIN_RATES    3U
#define PLL_TEST_SWEEP_MAX_RMS_HZ   1500.0f
#define PLL_TEST_SPECTRUM_BINS       (PLL_TEST_FFT_SIZE / 2U + 1U)
#define PLL_TEST_SPECTRUM_PEAKS      8U
#define PLL_TEST_HINT_DEFAULT_TOL_HZ  5000.0f
#define PLL_TEST_HINT_MAX_TOL_HZ      100000.0f
#define PLL_TEST_HINT_MIN_REL_POWER   0.000316228f /* -35 dB */
#define PLL_TEST_HINT_MIN_DOM_RATIO   15.848932f  /* +12 dB */

#if PLL_TEST_ADC_HALF_SAMPLES != 1024U
#error "The RF capture FFT currently requires a 1024-sample half buffer"
#endif

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

typedef struct {
    uint8_t valid;
    PllTest_Waveform_t waveform;
    float modulation_hz;
    float am_depth;
    float fm_deviation_hz;
    float phase_metric;
    float lower_sideband_db;
    float upper_sideband_db;
    float quality;
    uint8_t rate_index;
} PllTest_ModulationEstimate_t;

static ADC_DualCapture_Pair_t s_adc_dma[PLL_TEST_ADC_TOTAL_SAMPLES]
    ADC_DUAL_CAPTURE_DMA_BUFFER_ATTR;

static SoftPll_t s_pll;
static SoftPll_Config_t s_pll_config;
static PllTest_Crossing_t s_ref_cross;
static PllTest_Crossing_t s_fb_cross;
static volatile uint8_t s_running;
/* AD9708 uses its full 8-bit bipolar code span; amplitude 128 is full scale. */
static const uint16_t s_ad9708_amplitude = PLL_TEST_AD9708_AMPLITUDE;
static const uint8_t s_ad9708_offset = PLL_TEST_AD9708_OFFSET;
static double s_ad9708_phase_deg;
static float s_ad9708_frequency_hz = PLL_TEST_DEFAULT_START_FREQ_HZ;
static uint32_t s_ad9708_phase_word;
static uint32_t s_ad9708_update_count;
static uint32_t s_ad9708_error_count;
static uint8_t s_ad9708_consecutive_errors;
static uint8_t s_ad9708_recovery_pending;
static uint8_t s_ad9708_recovery_requested;
static uint32_t s_ad9708_last_update_us;
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
static float s_acquisition_hint_hz;
static float s_acquisition_hint_tolerance_hz = PLL_TEST_HINT_DEFAULT_TOL_HZ;
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
static float s_reference_spectrum[PLL_TEST_RATE_COUNT][PLL_TEST_SPECTRUM_BINS];
static float s_feedback_spectrum[PLL_TEST_RATE_COUNT][PLL_TEST_SPECTRUM_BINS];
static float s_wave_monitor_spectrum[PLL_TEST_SPECTRUM_BINS];
static float s_reference_noise_power[PLL_TEST_RATE_COUNT];
static uint16_t s_sweep_reference[PLL_TEST_RATE_COUNT][PLL_TEST_FFT_SIZE];
static float s_sweep_reference_mean[PLL_TEST_RATE_COUNT];
static uint16_t s_lock_sample_divider;
static float s_last_block_sample_rate_hz = (float)PLL_TEST_SAMPLE_RATE_HZ;
static float s_last_alias_hz;
static float s_last_alias_snr_db;
static float s_last_reference_iq_amp;
static float s_last_feedback_iq_amp;
static uint8_t s_last_phase_valid;
static float s_alias_hz[PLL_TEST_RATE_COUNT];
static float s_alias_snr_db[PLL_TEST_RATE_COUNT];
static float s_alias_peak2_hz[PLL_TEST_RATE_COUNT];
static float s_alias_peak3_hz[PLL_TEST_RATE_COUNT];
static float s_alias_peak2_rel_db[PLL_TEST_RATE_COUNT];
static float s_alias_peak3_rel_db[PLL_TEST_RATE_COUNT];
static float s_alias_peak1_ref_fb_db[PLL_TEST_RATE_COUNT];
static float s_alias_peak2_ref_fb_db[PLL_TEST_RATE_COUNT];
static float s_alias_peak3_ref_fb_db[PLL_TEST_RATE_COUNT];
static float s_measured_sample_rate_hz[PLL_TEST_RATE_COUNT];
static uint8_t s_alias_valid_mask;
static uint8_t s_sweep_active;
static uint8_t s_finish_sweep_pending;
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
static float s_last_measured_sample_rate_hz;
static float s_rate_assist_sum_hz;
static float s_rate_assist_last_hz;
static uint8_t s_rate_assist_stable_count;
static uint8_t s_rate_assist_applied;
static uint32_t s_rate_assist_apply_count;
static PllTest_Waveform_t s_requested_waveform = PLL_TEST_WAVEFORM_DEFAULT;
static PllTest_Waveform_t s_detected_waveform = PLL_TEST_WAVE_UNKNOWN;
static PllTest_Waveform_t s_output_waveform = PLL_TEST_WAVE_SINE;
static PllTest_Waveform_t s_wave_monitor_candidate = PLL_TEST_WAVE_UNKNOWN;
static float s_harmonic2_relative_db = -120.0f;
static float s_harmonic3_relative_db = -120.0f;
static uint8_t s_wave_monitor_confirm_count;
static uint32_t s_last_wave_monitor_tick;
static PllTest_ModulationEstimate_t s_modulation_estimate;
static uint32_t s_last_modulation_phase_tick;
static float s_modulation_phase_error_deg;
static uint8_t s_modulation_phase_valid;
static uint8_t s_modulation_lock_count;

static AD9708_StatusTypeDef ConfigureAd9708Output(void);
static uint8_t ServiceAd9708Recovery(void);
static uint8_t ApplyPllOutput(float frequency_hz, float phase_step_deg);
static uint8_t ApplyOutputWaveform(PllTest_Waveform_t waveform);
static void MonitorReferenceWaveform(const ADC_DualCapture_Pair_t *pairs,
                                     float reference_mean,
                                     float sample_rate_hz,
                                     uint32_t now);
static uint8_t ApplyStableRateAssist(void);
static int32_t ScaleToInt(float value, float scale);
static uint8_t ApplicationLocked(void);

static float ClampFloat(float value, float low, float high)
{
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

const char *PllTest_WaveformName(PllTest_Waveform_t waveform)
{
    switch (waveform) {
    case PLL_TEST_WAVE_AUTO: return "auto";
    case PLL_TEST_WAVE_SINE: return "sine";
    case PLL_TEST_WAVE_SQUARE: return "square";
    case PLL_TEST_WAVE_TRIANGLE: return "triangle";
    case PLL_TEST_WAVE_SAWTOOTH: return "saw";
    case PLL_TEST_WAVE_AM: return "am";
    case PLL_TEST_WAVE_FM: return "fm";
    default: return "unknown";
    }
}

static AD9708_ModeTypeDef WaveformToAd9708Mode(PllTest_Waveform_t waveform)
{
    switch (waveform) {
    case PLL_TEST_WAVE_SQUARE: return AD9708_MODE_SQUARE;
    case PLL_TEST_WAVE_TRIANGLE: return AD9708_MODE_TRIANGLE;
    case PLL_TEST_WAVE_SAWTOOTH: return AD9708_MODE_SAWTOOTH;
    /* AM/FM currently use a sine carrier. Their time-varying level/frequency
     * reconstruction is handled separately from the carrier shape. */
    default: return AD9708_MODE_RAM_DDS;
    }
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

static uint8_t IsKnownBoardSpur(float frequency_hz)
{
#if PLL_TEST_BOARD_SPUR_REJECT_ENABLE
    if (fabsf(frequency_hz - PLL_TEST_BOARD_SPUR_HZ) <=
        PLL_TEST_BOARD_SPUR_TOLERANCE_HZ) {
        return 1U;
    }
    if (fabsf(frequency_hz - PLL_TEST_BOARD_SPUR2_HZ) <=
        PLL_TEST_BOARD_SPUR_TOLERANCE_HZ) {
        return 1U;
    }
#else
    (void)frequency_hz;
#endif
    return 0U;
}

static float WrapPi(float phase_rad)
{
    while (phase_rad > (0.5f * PLL_TEST_TWO_PI)) phase_rad -= PLL_TEST_TWO_PI;
    while (phase_rad <= (-0.5f * PLL_TEST_TWO_PI)) phase_rad += PLL_TEST_TWO_PI;
    return phase_rad;
}

static void RequestSampleDivider(uint16_t divider)
{
    /* The driver applies the new exact divider only at a DMA half boundary.
     * The completed frame therefore keeps the divider that produced it. */
    (void)ADC_DualCapture_RequestDivider(divider, NULL);
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

static float InterpolateFftPeakOffset(uint32_t peak_bin)
{
    float ym;
    float y0;
    float yp;
    float denominator;
    float delta = 0.0f;
    float re;
    float im;

    re = s_fft_buffer[2U * (peak_bin - 1U)];
    im = s_fft_buffer[2U * (peak_bin - 1U) + 1U];
    ym = logf(re * re + im * im + 1.0f);
    re = s_fft_buffer[2U * peak_bin];
    im = s_fft_buffer[2U * peak_bin + 1U];
    y0 = logf(re * re + im * im + 1.0f);
    re = s_fft_buffer[2U * (peak_bin + 1U)];
    im = s_fft_buffer[2U * (peak_bin + 1U) + 1U];
    yp = logf(re * re + im * im + 1.0f);
    denominator = ym - 2.0f * y0 + yp;
    if (fabsf(denominator) > 1.0e-12f) {
        delta = 0.5f * (ym - yp) / denominator;
    }
    return ClampFloat(delta, -0.5f, 0.5f);
}

static void BeginFrequencySweep(void)
{
    /* During acquisition, move the DDS to DC without resetting it. This
     * removes the previous DDS tone from PC4 through board/channel coupling
     * and prevents the PLL from reacquiring its own old output. */
    if (s_running != 0U) {
        (void)ApplyPllOutput(0.0f, 0.0f);
    }
    memset(s_alias_hz, 0, sizeof(s_alias_hz));
    memset(s_alias_snr_db, 0, sizeof(s_alias_snr_db));
    memset(s_alias_peak2_hz, 0, sizeof(s_alias_peak2_hz));
    memset(s_alias_peak3_hz, 0, sizeof(s_alias_peak3_hz));
    memset(s_alias_peak2_rel_db, 0, sizeof(s_alias_peak2_rel_db));
    memset(s_alias_peak3_rel_db, 0, sizeof(s_alias_peak3_rel_db));
    memset(s_alias_peak1_ref_fb_db, 0, sizeof(s_alias_peak1_ref_fb_db));
    memset(s_alias_peak2_ref_fb_db, 0, sizeof(s_alias_peak2_ref_fb_db));
    memset(s_alias_peak3_ref_fb_db, 0, sizeof(s_alias_peak3_ref_fb_db));
    memset(s_measured_sample_rate_hz, 0,
           sizeof(s_measured_sample_rate_hz));
    memset(s_reference_spectrum, 0, sizeof(s_reference_spectrum));
    memset(s_feedback_spectrum, 0, sizeof(s_feedback_spectrum));
    memset(s_reference_noise_power, 0, sizeof(s_reference_noise_power));
    memset(s_sweep_reference, 0, sizeof(s_sweep_reference));
    memset(s_sweep_reference_mean, 0, sizeof(s_sweep_reference_mean));
    memset(&s_modulation_estimate, 0, sizeof(s_modulation_estimate));
    s_modulation_phase_valid = 0U;
    s_modulation_lock_count = 0U;
    s_modulation_phase_error_deg = 0.0f;
    s_alias_valid_mask = 0U;
    s_sweep_active = 1U;
    s_finish_sweep_pending = 0U;
    s_sweep_rate_index = 0U;
    s_sweep_block_count = 0U;
    s_sweep_attempt_count = 0U;
    s_sweep_first_bin = 0U;
    s_sweep_first_phase = 0.0f;
    s_sweep_first_alias_hz = 0.0f;
    s_pll.ready = 0U;
    s_pll.locked = 0U;
    s_pll.feedback_valid = 0U;
    s_rate_assist_sum_hz = 0.0f;
    s_rate_assist_last_hz = 0.0f;
    s_rate_assist_stable_count = 0U;
    s_rate_assist_applied = 0U;
    s_mode = PLL_TEST_MODE_ACQUIRE;
    RequestSampleDivider(s_rate_dividers[0]);
}

static uint8_t EstimateAliasFft(const ADC_DualCapture_Pair_t *pairs,
                                float reference_mean,
                                float feedback_mean,
                                float sample_rate_hz,
                                float *alias_hz,
                                uint16_t *peak_bin,
                                float *phase_rad,
                                float *snr_db,
                                float *peak2_hz,
                                float *peak3_hz,
                                float *peak2_rel_db,
                                float *peak3_rel_db,
                                float *peak1_ref_fb_db,
                                float *peak2_ref_fb_db,
                                float *peak3_ref_fb_db)
{
    uint32_t i;
    uint32_t k;
    uint32_t first_bin = PLL_TEST_ALIAS_EDGE_BINS;
    uint32_t last_bin = PLL_TEST_FFT_SIZE / 2U -
                        PLL_TEST_ALIAS_EDGE_BINS;
    uint32_t best_bin = first_bin;
    uint32_t second_bin = first_bin;
    uint32_t third_bin = first_bin;
    float best_power = 0.0f;
    float second_power = 0.0f;
    float third_power = 0.0f;
    float noise_power = 0.0f;
    uint32_t noise_bins = 0U;
    float ym;
    float y0;
    float yp;
    float denominator;
    float delta = 0.0f;

    if ((alias_hz == NULL) || (peak_bin == NULL) ||
        (phase_rad == NULL) || (snr_db == NULL) ||
        (peak2_hz == NULL) || (peak3_hz == NULL) ||
        (peak2_rel_db == NULL) || (peak3_rel_db == NULL) ||
        (peak1_ref_fb_db == NULL) || (peak2_ref_fb_db == NULL) ||
        (peak3_ref_fb_db == NULL) || (sample_rate_hz <= 0.0f)) {
        return 0U;
    }

    for (i = 0U; i < PLL_TEST_FFT_SIZE; ++i) {
        uint16_t reference_code = (uint16_t)(pairs[i] & 0xFFFFU);
        float sample = (float)reference_code - reference_mean;
        if (s_sweep_rate_index < PLL_TEST_RATE_COUNT) {
            s_sweep_reference[s_sweep_rate_index][i] = reference_code;
        }
        s_fft_buffer[2U * i] = sample * s_fft_window[i];
        s_fft_buffer[2U * i + 1U] = 0.0f;
    }
    if (s_sweep_rate_index < PLL_TEST_RATE_COUNT) {
        s_sweep_reference_mean[s_sweep_rate_index] = reference_mean;
    }
    arm_cfft_f32(&arm_cfft_sR_f32_len1024, s_fft_buffer, 0U, 1U);
    if (s_sweep_rate_index < PLL_TEST_RATE_COUNT) {
        for (k = 0U; k < PLL_TEST_SPECTRUM_BINS; ++k) {
            float re = s_fft_buffer[2U * k];
            float im = s_fft_buffer[2U * k + 1U];
            s_reference_spectrum[s_sweep_rate_index][k] =
                re * re + im * im;
        }
    }

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
    if (s_sweep_rate_index < PLL_TEST_RATE_COUNT) {
        s_reference_noise_power[s_sweep_rate_index] = noise_power;
    }
    *snr_db = 10.0f * log10f((best_power + 1.0f) /
                             (noise_power + 1.0f));
    if (*snr_db < PLL_TEST_ALIAS_MIN_SNR_DB) return 0U;
    /*
     * Hann coherent gain is 0.5, so a bin-centred sinusoid has
     * |FFT|=A*N/4. Reject the measured disconnected-PC4 spur before it can
     * become an acquisition candidate, not merely after the PLL starts.
     */
    if ((4.0f * sqrtf(best_power) /
         ((float)PLL_TEST_FFT_SIZE * PLL_TEST_ADC_SCALE)) <
        PLL_TEST_MIN_REFERENCE_IQ_AMP) return 0U;

    /* Capture independent secondary peaks for diagnosis. Four bins on each
     * side are excluded so the Hann main lobe is not reported as a signal.
     * Telemetry only: the resolver still uses the original strongest peak. */
    for (k = first_bin; k <= last_bin; ++k) {
        float re;
        float im;
        float power;
        if ((k + 4U >= best_bin) && (k <= best_bin + 4U)) continue;
        re = s_fft_buffer[2U * k];
        im = s_fft_buffer[2U * k + 1U];
        power = re * re + im * im;
        if (power > second_power) {
            second_power = power;
            second_bin = k;
        }
    }
    for (k = first_bin; k <= last_bin; ++k) {
        float re;
        float im;
        float power;
        if ((k + 4U >= best_bin) && (k <= best_bin + 4U)) continue;
        if ((k + 4U >= second_bin) && (k <= second_bin + 4U)) continue;
        re = s_fft_buffer[2U * k];
        im = s_fft_buffer[2U * k + 1U];
        power = re * re + im * im;
        if (power > third_power) {
            third_power = power;
            third_bin = k;
        }
    }
    *peak2_hz = (second_power > 0.0f) ?
        (((float)second_bin + InterpolateFftPeakOffset(second_bin)) *
         sample_rate_hz / (float)PLL_TEST_FFT_SIZE) : 0.0f;
    *peak3_hz = (third_power > 0.0f) ?
        (((float)third_bin + InterpolateFftPeakOffset(third_bin)) *
         sample_rate_hz / (float)PLL_TEST_FFT_SIZE) : 0.0f;
    *peak2_rel_db = (second_power > 0.0f) ?
        (10.0f * log10f((second_power + 1.0f) /
                        (best_power + 1.0f))) : -120.0f;
    *peak3_rel_db = (third_power > 0.0f) ?
        (10.0f * log10f((third_power + 1.0f) /
                        (best_power + 1.0f))) : -120.0f;

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

    /* Compare the same spectral lines on PC4 (reference) and PC5 (feedback).
     * During the acquisition guard the DDS is at DC, so a genuine external
     * reference should have positive PC4/PC5 dominance. */
    for (i = 0U; i < PLL_TEST_FFT_SIZE; ++i) {
        float sample = (float)((pairs[i] >> 16U) & 0xFFFFU) -
                       feedback_mean;
        s_fft_buffer[2U * i] = sample * s_fft_window[i];
        s_fft_buffer[2U * i + 1U] = 0.0f;
    }
    arm_cfft_f32(&arm_cfft_sR_f32_len1024, s_fft_buffer, 0U, 1U);
    if (s_sweep_rate_index < PLL_TEST_RATE_COUNT) {
        for (k = 0U; k < PLL_TEST_SPECTRUM_BINS; ++k) {
            float re = s_fft_buffer[2U * k];
            float im = s_fft_buffer[2U * k + 1U];
            s_feedback_spectrum[s_sweep_rate_index][k] =
                re * re + im * im;
        }
    }
    {
        float re1 = s_fft_buffer[2U * best_bin];
        float im1 = s_fft_buffer[2U * best_bin + 1U];
        float re2 = s_fft_buffer[2U * second_bin];
        float im2 = s_fft_buffer[2U * second_bin + 1U];
        float re3 = s_fft_buffer[2U * third_bin];
        float im3 = s_fft_buffer[2U * third_bin + 1U];
        float fb1_power = re1 * re1 + im1 * im1;
        float fb2_power = re2 * re2 + im2 * im2;
        float fb3_power = re3 * re3 + im3 * im3;
        *peak1_ref_fb_db = 10.0f * log10f((best_power + 1.0f) /
                                          (fb1_power + 1.0f));
        *peak2_ref_fb_db = 10.0f * log10f((second_power + 1.0f) /
                                          (fb2_power + 1.0f));
        *peak3_ref_fb_db = 10.0f * log10f((third_power + 1.0f) /
                                          (fb3_power + 1.0f));
    }
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

static float SpectrumPeakOffset(const float *spectrum, uint16_t bin)
{
    float ym = logf(spectrum[bin - 1U] + 1.0f);
    float y0 = logf(spectrum[bin] + 1.0f);
    float yp = logf(spectrum[bin + 1U] + 1.0f);
    float denominator = ym - 2.0f * y0 + yp;
    if (fabsf(denominator) <= 1.0e-12f) return 0.0f;
    return ClampFloat(0.5f * (ym - yp) / denominator, -0.5f, 0.5f);
}

static float SpectrumBandPower(const float *spectrum,
                               float frequency_hz,
                               float sample_rate_hz,
                               uint8_t *valid)
{
    int32_t centre_bin;
    int32_t first_bin;
    int32_t last_bin;
    int32_t bin;
    float power = 0.0f;

    if (valid != NULL) *valid = 0U;
    if ((spectrum == NULL) || (sample_rate_hz <= 0.0f)) return 0.0f;
    frequency_hz = FoldAliasHz(frequency_hz, sample_rate_hz);
    centre_bin = (int32_t)(frequency_hz * (float)PLL_TEST_FFT_SIZE /
                           sample_rate_hz + 0.5f);
    if ((centre_bin < (int32_t)PLL_TEST_WAVE_HARMONIC_GUARD_BINS) ||
        (centre_bin > (int32_t)(PLL_TEST_SPECTRUM_BINS - 1U -
                                PLL_TEST_WAVE_HARMONIC_GUARD_BINS))) {
        return 0.0f;
    }
    first_bin = centre_bin - (int32_t)PLL_TEST_WAVE_HARMONIC_RADIUS_BINS;
    last_bin = centre_bin + (int32_t)PLL_TEST_WAVE_HARMONIC_RADIUS_BINS;
    for (bin = first_bin; bin <= last_bin; ++bin) {
        power += spectrum[bin];
    }
    if (valid != NULL) *valid = 1U;
    return power;
}

static uint8_t SpectrumHarmonicDb(const float *spectrum,
                                  float carrier_hz,
                                  uint8_t harmonic,
                                  float sample_rate_hz,
                                  float *relative_db)
{
    float fundamental_alias;
    float harmonic_alias;
    float peer_alias;
    float bin_hz;
    float fundamental_power;
    float harmonic_power;
    uint8_t fundamental_valid;
    uint8_t harmonic_valid;

    if ((relative_db == NULL) || (harmonic < 2U)) return 0U;
    fundamental_alias = FoldAliasHz(carrier_hz, sample_rate_hz);
    harmonic_alias = FoldAliasHz((float)harmonic * carrier_hz,
                                 sample_rate_hz);
    peer_alias = FoldAliasHz((float)((harmonic == 2U) ? 3U : 2U) *
                             carrier_hz, sample_rate_hz);
    bin_hz = sample_rate_hz / (float)PLL_TEST_FFT_SIZE;
    if (fabsf(harmonic_alias - fundamental_alias) <=
        (float)(2U * PLL_TEST_WAVE_HARMONIC_RADIUS_BINS + 2U) * bin_hz) {
        return 0U;
    }
    if (fabsf(harmonic_alias - peer_alias) <=
        (float)(2U * PLL_TEST_WAVE_HARMONIC_RADIUS_BINS + 2U) * bin_hz) {
        return 0U;
    }
    fundamental_power = SpectrumBandPower(spectrum, fundamental_alias,
                                           sample_rate_hz,
                                           &fundamental_valid);
    harmonic_power = SpectrumBandPower(spectrum, harmonic_alias,
                                        sample_rate_hz,
                                        &harmonic_valid);
    if ((fundamental_valid == 0U) || (harmonic_valid == 0U) ||
        (fundamental_power <= 1.0f)) return 0U;
    *relative_db = 10.0f * log10f((harmonic_power + 1.0f) /
                                   (fundamental_power + 1.0f));
    return 1U;
}

static uint8_t SpectrumPointPower(const float *spectrum,
                                  float physical_hz,
                                  float sample_rate_hz,
                                  float *power,
                                  int32_t *selected_bin)
{
    int32_t centre_bin;
    int32_t bin;
    float best = 0.0f;

    if ((spectrum == NULL) || (power == NULL) ||
        (sample_rate_hz <= 0.0f)) return 0U;
    centre_bin = (int32_t)(FoldAliasHz(physical_hz, sample_rate_hz) *
                           (float)PLL_TEST_FFT_SIZE / sample_rate_hz + 0.5f);
    if ((centre_bin < 2) ||
        (centre_bin > (int32_t)PLL_TEST_SPECTRUM_BINS - 3)) return 0U;
    for (bin = centre_bin - 1; bin <= centre_bin + 1; ++bin) {
        if (spectrum[bin] > best) best = spectrum[bin];
    }
    *power = best;
    if (selected_bin != NULL) *selected_bin = centre_bin;
    return 1U;
}

/*
 * Windowed DFT exactly at a physical RF frequency. For a component that folds
 * through Nyquist, conjugating the measured positive-alias coefficient
 * restores the phase of the original positive-frequency RF component. This
 * makes C+*C-/C0^2 invariant across all four undersampling rates.
 */
static uint8_t MeasureStoredTone(uint8_t rate_index,
                                 float physical_hz,
                                 float *real_out,
                                 float *imag_out)
{
    float sample_rate_hz;
    float remainder_hz;
    float alias_hz;
    float step_rad;
    float cos_step;
    float sin_step;
    float cos_n = 1.0f;
    float sin_n = 0.0f;
    float real_sum = 0.0f;
    float imag_sum = 0.0f;
    uint8_t conjugate;
    uint32_t i;

    if ((rate_index >= PLL_TEST_RATE_COUNT) || (real_out == NULL) ||
        (imag_out == NULL) || !(physical_hz > 0.0f)) return 0U;
    sample_rate_hz = SampleRateFromDivider(s_rate_dividers[rate_index]);
    remainder_hz = fmodf(physical_hz, sample_rate_hz);
    if (remainder_hz < 0.0f) remainder_hz += sample_rate_hz;
    conjugate = (remainder_hz > 0.5f * sample_rate_hz) ? 1U : 0U;
    alias_hz = (conjugate != 0U) ?
               (sample_rate_hz - remainder_hz) : remainder_hz;
    if ((alias_hz <= sample_rate_hz / (float)PLL_TEST_FFT_SIZE) ||
        (alias_hz >= 0.5f * sample_rate_hz -
                    sample_rate_hz / (float)PLL_TEST_FFT_SIZE)) return 0U;

    step_rad = PLL_TEST_TWO_PI * alias_hz / sample_rate_hz;
    cos_step = cosf(step_rad);
    sin_step = sinf(step_rad);
    for (i = 0U; i < PLL_TEST_FFT_SIZE; ++i) {
        float sample =
            ((float)s_sweep_reference[rate_index][i] -
             s_sweep_reference_mean[rate_index]) * s_fft_window[i];
        float next_cos;
        real_sum += sample * cos_n;
        imag_sum -= sample * sin_n;
        next_cos = cos_n * cos_step - sin_n * sin_step;
        sin_n = sin_n * cos_step + cos_n * sin_step;
        cos_n = next_cos;
    }
    if (conjugate != 0U) imag_sum = -imag_sum;
    *real_out = real_sum;
    *imag_out = imag_sum;
    return 1U;
}

/*
 * Remove the exactly measured carrier before evaluating a nearby sideband.
 * Without this step the Hann main lobe of a pure sine can resemble a pair of
 * approximately 1 kHz sidebands in a 1024-sample record.
 */
static uint8_t MeasureStoredResidualTone(
    uint8_t rate_index,
    float carrier_hz,
    float carrier_re,
    float carrier_im,
    float target_hz,
    float *real_out,
    float *imag_out)
{
    float sample_rate_hz;
    float carrier_remainder;
    float carrier_alias;
    float target_remainder;
    float target_alias;
    float carrier_cos_step;
    float carrier_sin_step;
    float target_cos_step;
    float target_sin_step;
    float carrier_cos = 1.0f;
    float carrier_sin = 0.0f;
    float target_cos = 1.0f;
    float target_sin = 0.0f;
    float window_sum = 0.0f;
    float carrier_phasor_re;
    float carrier_phasor_im;
    float real_sum = 0.0f;
    float imag_sum = 0.0f;
    uint8_t carrier_conjugate;
    uint8_t target_conjugate;
    uint32_t i;

    if ((rate_index >= PLL_TEST_RATE_COUNT) || (real_out == NULL) ||
        (imag_out == NULL) || !(carrier_hz > 0.0f) ||
        !(target_hz > 0.0f)) return 0U;
    sample_rate_hz = SampleRateFromDivider(s_rate_dividers[rate_index]);
    carrier_remainder = fmodf(carrier_hz, sample_rate_hz);
    target_remainder = fmodf(target_hz, sample_rate_hz);
    if (carrier_remainder < 0.0f) carrier_remainder += sample_rate_hz;
    if (target_remainder < 0.0f) target_remainder += sample_rate_hz;
    carrier_conjugate =
        (carrier_remainder > 0.5f * sample_rate_hz) ? 1U : 0U;
    target_conjugate =
        (target_remainder > 0.5f * sample_rate_hz) ? 1U : 0U;
    carrier_alias = (carrier_conjugate != 0U) ?
                    (sample_rate_hz - carrier_remainder) :
                    carrier_remainder;
    target_alias = (target_conjugate != 0U) ?
                   (sample_rate_hz - target_remainder) :
                   target_remainder;
    if ((carrier_alias <= 0.0f) || (target_alias <= 0.0f) ||
        (carrier_alias >= 0.5f * sample_rate_hz) ||
        (target_alias >= 0.5f * sample_rate_hz)) return 0U;

    carrier_cos_step =
        cosf(PLL_TEST_TWO_PI * carrier_alias / sample_rate_hz);
    carrier_sin_step =
        sinf(PLL_TEST_TWO_PI * carrier_alias / sample_rate_hz);
    target_cos_step =
        cosf(PLL_TEST_TWO_PI * target_alias / sample_rate_hz);
    target_sin_step =
        sinf(PLL_TEST_TWO_PI * target_alias / sample_rate_hz);
    for (i = 0U; i < PLL_TEST_FFT_SIZE; ++i) {
        window_sum += s_fft_window[i];
    }
    if (window_sum <= 1.0f) return 0U;
    carrier_phasor_re = 2.0f * carrier_re / window_sum;
    carrier_phasor_im = 2.0f * carrier_im / window_sum;
    if (carrier_conjugate != 0U) {
        carrier_phasor_im = -carrier_phasor_im;
    }

    for (i = 0U; i < PLL_TEST_FFT_SIZE; ++i) {
        float raw =
            (float)s_sweep_reference[rate_index][i] -
            s_sweep_reference_mean[rate_index];
        float reconstructed =
            carrier_phasor_re * carrier_cos -
            carrier_phasor_im * carrier_sin;
        float residual = (raw - reconstructed) * s_fft_window[i];
        float next_carrier_cos;
        float next_target_cos;
        real_sum += residual * target_cos;
        imag_sum -= residual * target_sin;
        next_carrier_cos =
            carrier_cos * carrier_cos_step -
            carrier_sin * carrier_sin_step;
        carrier_sin =
            carrier_sin * carrier_cos_step +
            carrier_cos * carrier_sin_step;
        carrier_cos = next_carrier_cos;
        next_target_cos =
            target_cos * target_cos_step -
            target_sin * target_sin_step;
        target_sin =
            target_sin * target_cos_step +
            target_cos * target_sin_step;
        target_cos = next_target_cos;
    }
    if (target_conjugate != 0U) imag_sum = -imag_sum;
    *real_out = real_sum;
    *imag_out = imag_sum;
    return 1U;
}

static uint8_t MeasurePairedTone(const ADC_DualCapture_Pair_t *pairs,
                                 uint8_t channel_shift,
                                 float mean,
                                 float physical_hz,
                                 float sample_rate_hz,
                                 float *real_out,
                                 float *imag_out)
{
    float remainder_hz;
    float alias_hz;
    float step_rad;
    float cos_step;
    float sin_step;
    float cos_n = 1.0f;
    float sin_n = 0.0f;
    float real_sum = 0.0f;
    float imag_sum = 0.0f;
    uint8_t conjugate;
    uint32_t i;

    if ((pairs == NULL) || (real_out == NULL) || (imag_out == NULL) ||
        ((channel_shift != 0U) && (channel_shift != 16U)) ||
        !(physical_hz > 0.0f) || !(sample_rate_hz > 0.0f)) return 0U;
    remainder_hz = fmodf(physical_hz, sample_rate_hz);
    if (remainder_hz < 0.0f) remainder_hz += sample_rate_hz;
    conjugate = (remainder_hz > 0.5f * sample_rate_hz) ? 1U : 0U;
    alias_hz = (conjugate != 0U) ?
               (sample_rate_hz - remainder_hz) : remainder_hz;
    if ((alias_hz <= sample_rate_hz / (float)PLL_TEST_FFT_SIZE) ||
        (alias_hz >= 0.5f * sample_rate_hz -
                    sample_rate_hz / (float)PLL_TEST_FFT_SIZE)) return 0U;
    step_rad = PLL_TEST_TWO_PI * alias_hz / sample_rate_hz;
    cos_step = cosf(step_rad);
    sin_step = sinf(step_rad);
    for (i = 0U; i < PLL_TEST_FFT_SIZE; ++i) {
        float sample =
            ((float)((pairs[i] >> channel_shift) & 0xFFFFU) - mean) *
            s_fft_window[i];
        float next_cos;
        real_sum += sample * cos_n;
        imag_sum -= sample * sin_n;
        next_cos = cos_n * cos_step - sin_n * sin_step;
        sin_n = sin_n * cos_step + cos_n * sin_step;
        cos_n = next_cos;
    }
    if (conjugate != 0U) imag_sum = -imag_sum;
    *real_out = real_sum;
    *imag_out = imag_sum;
    return 1U;
}

static uint8_t MeasurePairedResidualTone(
    const ADC_DualCapture_Pair_t *pairs,
    uint8_t channel_shift,
    float mean,
    float carrier_hz,
    float carrier_re,
    float carrier_im,
    float target_hz,
    float sample_rate_hz,
    float *real_out,
    float *imag_out)
{
    float carrier_remainder;
    float carrier_alias;
    float target_remainder;
    float target_alias;
    float carrier_cos_step;
    float carrier_sin_step;
    float target_cos_step;
    float target_sin_step;
    float carrier_cos = 1.0f;
    float carrier_sin = 0.0f;
    float target_cos = 1.0f;
    float target_sin = 0.0f;
    float window_sum = 0.0f;
    float carrier_phasor_re;
    float carrier_phasor_im;
    float real_sum = 0.0f;
    float imag_sum = 0.0f;
    uint8_t carrier_conjugate;
    uint8_t target_conjugate;
    uint32_t i;

    if ((pairs == NULL) || (real_out == NULL) || (imag_out == NULL) ||
        ((channel_shift != 0U) && (channel_shift != 16U)) ||
        !(carrier_hz > 0.0f) || !(target_hz > 0.0f) ||
        !(sample_rate_hz > 0.0f)) return 0U;
    carrier_remainder = fmodf(carrier_hz, sample_rate_hz);
    target_remainder = fmodf(target_hz, sample_rate_hz);
    if (carrier_remainder < 0.0f) carrier_remainder += sample_rate_hz;
    if (target_remainder < 0.0f) target_remainder += sample_rate_hz;
    carrier_conjugate =
        (carrier_remainder > 0.5f * sample_rate_hz) ? 1U : 0U;
    target_conjugate =
        (target_remainder > 0.5f * sample_rate_hz) ? 1U : 0U;
    carrier_alias = (carrier_conjugate != 0U) ?
                    (sample_rate_hz - carrier_remainder) :
                    carrier_remainder;
    target_alias = (target_conjugate != 0U) ?
                   (sample_rate_hz - target_remainder) :
                   target_remainder;
    if ((carrier_alias <= 0.0f) || (target_alias <= 0.0f) ||
        (carrier_alias >= 0.5f * sample_rate_hz) ||
        (target_alias >= 0.5f * sample_rate_hz)) return 0U;
    carrier_cos_step =
        cosf(PLL_TEST_TWO_PI * carrier_alias / sample_rate_hz);
    carrier_sin_step =
        sinf(PLL_TEST_TWO_PI * carrier_alias / sample_rate_hz);
    target_cos_step =
        cosf(PLL_TEST_TWO_PI * target_alias / sample_rate_hz);
    target_sin_step =
        sinf(PLL_TEST_TWO_PI * target_alias / sample_rate_hz);
    for (i = 0U; i < PLL_TEST_FFT_SIZE; ++i) {
        window_sum += s_fft_window[i];
    }
    if (window_sum <= 1.0f) return 0U;
    carrier_phasor_re = 2.0f * carrier_re / window_sum;
    carrier_phasor_im = 2.0f * carrier_im / window_sum;
    if (carrier_conjugate != 0U) {
        carrier_phasor_im = -carrier_phasor_im;
    }
    for (i = 0U; i < PLL_TEST_FFT_SIZE; ++i) {
        float raw =
            (float)((pairs[i] >> channel_shift) & 0xFFFFU) - mean;
        float reconstructed =
            carrier_phasor_re * carrier_cos -
            carrier_phasor_im * carrier_sin;
        float residual = (raw - reconstructed) * s_fft_window[i];
        float next_carrier_cos;
        float next_target_cos;
        real_sum += residual * target_cos;
        imag_sum -= residual * target_sin;
        next_carrier_cos =
            carrier_cos * carrier_cos_step -
            carrier_sin * carrier_sin_step;
        carrier_sin =
            carrier_sin * carrier_cos_step +
            carrier_cos * carrier_sin_step;
        carrier_cos = next_carrier_cos;
        next_target_cos =
            target_cos * target_cos_step -
            target_sin * target_sin_step;
        target_sin =
            target_sin * target_cos_step +
            target_cos * target_sin_step;
        target_cos = next_target_cos;
    }
    if (target_conjugate != 0U) imag_sum = -imag_sum;
    *real_out = real_sum;
    *imag_out = imag_sum;
    return 1U;
}

static void BesselJ0J1(float value, float *j0, float *j1)
{
    float x2_over_4 = 0.25f * value * value;
    float term0 = 1.0f;
    float term1 = 0.5f * value;
    float sum0 = term0;
    float sum1 = term1;
    uint8_t k;

    for (k = 1U; k <= 10U; ++k) {
        term0 *= -x2_over_4 / ((float)k * (float)k);
        term1 *= -x2_over_4 / ((float)k * (float)(k + 1U));
        sum0 += term0;
        sum1 += term1;
    }
    if (j0 != NULL) *j0 = sum0;
    if (j1 != NULL) *j1 = sum1;
}

static float FmBetaFromFirstSidebandRatio(float ratio)
{
    float low = 0.0f;
    float high = PLL_TEST_FM_MAX_BETA;
    uint8_t iteration;

    for (iteration = 0U; iteration < 20U; ++iteration) {
        float mid = 0.5f * (low + high);
        float j0;
        float j1;
        BesselJ0J1(mid, &j0, &j1);
        if ((fabsf(j0) <= 1.0e-6f) ||
            ((fabsf(j1) / fabsf(j0)) >= ratio)) {
            high = mid;
        } else {
            low = mid;
        }
    }
    return 0.5f * (low + high);
}

static uint8_t EstimateModulationAtRate(
    uint8_t rate_index,
    float carrier_hz,
    PllTest_ModulationEstimate_t *estimate)
{
    const float *spectrum;
    float sample_rate_hz;
    float bin_hz;
    float carrier_bin_power;
    float best_coarse_score = 0.0f;
    float coarse_modulation_hz = 0.0f;
    float carrier_re;
    float carrier_im;
    float carrier_power;
    int32_t carrier_bin;
    uint16_t first_offset;
    uint16_t last_offset;
    uint16_t offset;
    uint8_t refine;

    if ((estimate == NULL) || (rate_index >= PLL_TEST_RATE_COUNT) ||
        !(carrier_hz > PLL_TEST_MODULATION_MIN_HZ)) return 0U;
    memset(estimate, 0, sizeof(*estimate));
    spectrum = s_reference_spectrum[rate_index];
    sample_rate_hz = SampleRateFromDivider(s_rate_dividers[rate_index]);
    bin_hz = sample_rate_hz / (float)PLL_TEST_FFT_SIZE;
    if (SpectrumPointPower(spectrum, carrier_hz, sample_rate_hz,
                           &carrier_bin_power, &carrier_bin) == 0U) return 0U;
    first_offset = (uint16_t)
        ceilf(PLL_TEST_MODULATION_MIN_HZ / bin_hz);
    if (first_offset < 2U) first_offset = 2U;
    last_offset = (uint16_t)
        floorf(fminf(PLL_TEST_MODULATION_MAX_HZ,
                     0.45f * carrier_hz) / bin_hz);
    if (last_offset <= first_offset) return 0U;

    /* Cheap FFT-bin search; exact-frequency DFT is used only near the winner. */
    for (offset = first_offset; offset <= last_offset; ++offset) {
        float modulation_hz = (float)offset * bin_hz;
        float lower_power;
        float upper_power;
        float score;
        int32_t lower_bin;
        int32_t upper_bin;
        if (SpectrumPointPower(spectrum, carrier_hz - modulation_hz,
                               sample_rate_hz, &lower_power,
                               &lower_bin) == 0U) continue;
        if (SpectrumPointPower(spectrum, carrier_hz + modulation_hz,
                               sample_rate_hz, &upper_power,
                               &upper_bin) == 0U) continue;
        if ((abs(lower_bin - carrier_bin) < 2) ||
            (abs(upper_bin - carrier_bin) < 2) ||
            (abs(upper_bin - lower_bin) < 2)) continue;
        score = sqrtf((lower_power + 1.0f) * (upper_power + 1.0f)) /
                (carrier_bin_power + 1.0f);
        if (score > best_coarse_score) {
            best_coarse_score = score;
            coarse_modulation_hz = modulation_hz;
        }
    }
    if (!(coarse_modulation_hz > 0.0f) ||
        (MeasureStoredTone(rate_index, carrier_hz,
                           &carrier_re, &carrier_im) == 0U)) return 0U;
    carrier_power = carrier_re * carrier_re + carrier_im * carrier_im;
    if (carrier_power <= 1.0f) return 0U;

    /* Refine over +/- one FFT bin in one-eighth-bin steps. */
    for (refine = 0U; refine <= 16U; ++refine) {
        float modulation_hz =
            coarse_modulation_hz + ((float)refine - 8.0f) * bin_hz / 8.0f;
        float lower_re;
        float lower_im;
        float upper_re;
        float upper_im;
        float lower_power;
        float upper_power;
        float lower_db;
        float upper_db;
        float symmetry_db;
        float side_re;
        float side_im;
        float carrier_sq_re;
        float carrier_sq_im;
        float invariant_real;
        float invariant_norm;
        float phase_metric;
        float amplitude_ratio;
        float quality;
        PllTest_Waveform_t waveform;

        if ((modulation_hz < PLL_TEST_MODULATION_MIN_HZ) ||
            (modulation_hz > PLL_TEST_MODULATION_MAX_HZ) ||
            (modulation_hz >= carrier_hz)) continue;
        if ((MeasureStoredResidualTone(
                 rate_index, carrier_hz, carrier_re, carrier_im,
                 carrier_hz - modulation_hz,
                 &lower_re, &lower_im) == 0U) ||
            (MeasureStoredResidualTone(
                 rate_index, carrier_hz, carrier_re, carrier_im,
                 carrier_hz + modulation_hz,
                 &upper_re, &upper_im) == 0U)) continue;
        lower_power = lower_re * lower_re + lower_im * lower_im;
        upper_power = upper_re * upper_re + upper_im * upper_im;
        lower_db = 10.0f * log10f((lower_power + 1.0f) /
                                  (carrier_power + 1.0f));
        upper_db = 10.0f * log10f((upper_power + 1.0f) /
                                  (carrier_power + 1.0f));
        symmetry_db = fabsf(lower_db - upper_db);
        if ((lower_db < PLL_TEST_MODULATION_MIN_SIDEBAND_DB) ||
            (upper_db < PLL_TEST_MODULATION_MIN_SIDEBAND_DB) ||
            (symmetry_db > PLL_TEST_MODULATION_MAX_SYMMETRY_DB)) continue;

        side_re = upper_re * lower_re - upper_im * lower_im;
        side_im = upper_re * lower_im + upper_im * lower_re;
        carrier_sq_re =
            carrier_re * carrier_re - carrier_im * carrier_im;
        carrier_sq_im = 2.0f * carrier_re * carrier_im;
        invariant_real =
            side_re * carrier_sq_re + side_im * carrier_sq_im;
        invariant_norm =
            sqrtf(lower_power * upper_power) * carrier_power;
        if (invariant_norm <= 1.0f) continue;
        phase_metric = invariant_real / invariant_norm;
        if (fabsf(phase_metric) <
            PLL_TEST_MODULATION_MIN_PHASE_METRIC) continue;
        waveform = (phase_metric >= 0.0f) ?
                   PLL_TEST_WAVE_AM : PLL_TEST_WAVE_FM;
        amplitude_ratio =
            sqrtf(0.5f * (lower_power + upper_power) / carrier_power);
        quality = fminf(lower_power, upper_power) / carrier_power;
        quality *= fabsf(phase_metric) / (1.0f + 0.2f * symmetry_db);
        if ((estimate->valid == 0U) || (quality > estimate->quality)) {
            estimate->valid = 1U;
            estimate->waveform = waveform;
            estimate->modulation_hz = modulation_hz;
            estimate->am_depth =
                (waveform == PLL_TEST_WAVE_AM) ?
                ClampFloat(2.0f * amplitude_ratio, 0.0f, 1.0f) : 0.0f;
            estimate->fm_deviation_hz =
                (waveform == PLL_TEST_WAVE_FM) ?
                FmBetaFromFirstSidebandRatio(amplitude_ratio) *
                modulation_hz : 0.0f;
            estimate->phase_metric = phase_metric;
            estimate->lower_sideband_db = lower_db;
            estimate->upper_sideband_db = upper_db;
            estimate->quality = quality;
            estimate->rate_index = rate_index;
        }
    }
    return estimate->valid;
}

static PllTest_Waveform_t ClassifySweepModulation(float carrier_hz)
{
    PllTest_ModulationEstimate_t candidates[PLL_TEST_RATE_COUNT];
    PllTest_ModulationEstimate_t best;
    uint8_t best_found = 0U;
    uint8_t rate_index;
    uint8_t confirm_count = 0U;
    float frequency_sum = 0.0f;
    float am_depth_sum = 0.0f;
    float fm_deviation_sum = 0.0f;

    memset(candidates, 0, sizeof(candidates));
    memset(&best, 0, sizeof(best));
    for (rate_index = 0U; rate_index < PLL_TEST_RATE_COUNT; ++rate_index) {
        if ((s_alias_valid_mask & (1U << rate_index)) == 0U) continue;
        if (EstimateModulationAtRate(rate_index, carrier_hz,
                                     &candidates[rate_index]) == 0U) continue;
        if ((best_found == 0U) ||
            (candidates[rate_index].quality > best.quality)) {
            best = candidates[rate_index];
            best_found = 1U;
        }
    }
    if (best_found == 0U) return PLL_TEST_WAVE_UNKNOWN;

    for (rate_index = 0U; rate_index < PLL_TEST_RATE_COUNT; ++rate_index) {
        PllTest_ModulationEstimate_t *candidate = &candidates[rate_index];
        float tolerance_hz;
        if ((candidate->valid == 0U) ||
            (candidate->waveform != best.waveform)) continue;
        tolerance_hz = fmaxf(750.0f, 0.15f * best.modulation_hz);
        if (fabsf(candidate->modulation_hz - best.modulation_hz) >
            tolerance_hz) continue;
        confirm_count++;
        frequency_sum += candidate->modulation_hz;
        am_depth_sum += candidate->am_depth;
        fm_deviation_sum += candidate->fm_deviation_hz;
    }
    if ((confirm_count < 2U) &&
        (s_requested_waveform != PLL_TEST_WAVE_AM) &&
        (s_requested_waveform != PLL_TEST_WAVE_FM)) {
        return PLL_TEST_WAVE_UNKNOWN;
    }

    s_modulation_estimate = best;
    s_modulation_estimate.modulation_hz =
        frequency_sum / (float)confirm_count;
    if (best.waveform == PLL_TEST_WAVE_AM) {
        s_modulation_estimate.am_depth =
            am_depth_sum / (float)confirm_count;
    } else {
        s_modulation_estimate.fm_deviation_hz =
            fm_deviation_sum / (float)confirm_count;
    }
    return best.waveform;
}

static PllTest_Waveform_t DecidePeriodicWaveform(float carrier_hz,
                                                  float harmonic2_db,
                                                  uint8_t harmonic2_valid,
                                                  float harmonic3_db,
                                                  uint8_t harmonic3_valid)
{
    float square_h3_min_db = PLL_TEST_WAVE_H3_SQUARE_MIN_DB;
    uint8_t low_shape_range =
        (carrier_hz <= PLL_TEST_WAVE_LOW_SHAPE_MAX_HZ) ? 1U : 0U;

    /* Outside the tolerance-extended low-shape band the required periodic
     * set is sine/square.  The PC4 conditioning path increasingly attenuates
     * the square-wave H3 (measured -18..-21 dB at 3 MHz), so use the
     * RF-calibrated threshold there.  Keep the tighter ideal-ratio thresholds
     * where triangle/saw must be distinguished. */
    if (low_shape_range == 0U) {
        square_h3_min_db = PLL_TEST_WAVE_H3_SQUARE_RF_MIN_DB;
    }
    if ((low_shape_range != 0U) && (harmonic2_valid != 0U) &&
        (harmonic2_db >= PLL_TEST_WAVE_H2_SAW_MIN_DB)) {
        return PLL_TEST_WAVE_SAWTOOTH;
    }
    if ((harmonic3_valid != 0U) &&
        (harmonic3_db >= square_h3_min_db)) {
        return PLL_TEST_WAVE_SQUARE;
    }
    if ((low_shape_range != 0U) && (harmonic3_valid != 0U) &&
        (harmonic3_db >= PLL_TEST_WAVE_H3_TRIANGLE_MIN_DB)) {
        return PLL_TEST_WAVE_TRIANGLE;
    }
    if ((harmonic2_valid != 0U) || (harmonic3_valid != 0U)) {
        return PLL_TEST_WAVE_SINE;
    }
    return PLL_TEST_WAVE_UNKNOWN;
}

static PllTest_Waveform_t ClassifySweepWaveform(float carrier_hz)
{
#if PLL_TEST_EXPERIMENTAL_AMFM_ENABLE
    PllTest_Waveform_t modulation =
        ClassifySweepModulation(carrier_hz);
#endif
    float harmonic2_sum = 0.0f;
    float harmonic3_sum = 0.0f;
    uint8_t harmonic2_count = 0U;
    uint8_t harmonic3_count = 0U;
    uint8_t rate_index;

#if PLL_TEST_EXPERIMENTAL_AMFM_ENABLE
    if (modulation != PLL_TEST_WAVE_UNKNOWN) return modulation;
#endif

    for (rate_index = 0U; rate_index < PLL_TEST_RATE_COUNT; ++rate_index) {
        float value_db;
        float sample_rate_hz;
        if ((s_alias_valid_mask & (1U << rate_index)) == 0U) continue;
        sample_rate_hz = SampleRateFromDivider(s_rate_dividers[rate_index]);
        if (SpectrumHarmonicDb(s_reference_spectrum[rate_index], carrier_hz,
                               2U, sample_rate_hz, &value_db) != 0U) {
            harmonic2_sum += value_db;
            harmonic2_count++;
        }
        if (SpectrumHarmonicDb(s_reference_spectrum[rate_index], carrier_hz,
                               3U, sample_rate_hz, &value_db) != 0U) {
            harmonic3_sum += value_db;
            harmonic3_count++;
        }
    }
    if (harmonic2_count != 0U) {
        s_harmonic2_relative_db = harmonic2_sum / (float)harmonic2_count;
    } else {
        s_harmonic2_relative_db = -120.0f;
    }
    if (harmonic3_count != 0U) {
        s_harmonic3_relative_db = harmonic3_sum / (float)harmonic3_count;
    } else {
        s_harmonic3_relative_db = -120.0f;
    }
    return DecidePeriodicWaveform(
        carrier_hz,
        s_harmonic2_relative_db,
        (harmonic2_count >= PLL_TEST_WAVE_HARMONIC_MIN_RATES) ? 1U : 0U,
        s_harmonic3_relative_db,
        (harmonic3_count >= PLL_TEST_WAVE_HARMONIC_MIN_RATES) ? 1U : 0U);
}

static void PrintSpectrum(uint8_t rate_index)
{
    uint16_t selected[PLL_TEST_SPECTRUM_PEAKS];
    uint8_t selected_count = 0U;
    uint8_t peak;
    uint16_t bin;
    float strongest_power = 0.0f;
    float sample_rate_hz;
    char line[600];
    int used;

    if (rate_index >= PLL_TEST_RATE_COUNT) return;
    memset(selected, 0, sizeof(selected));
    sample_rate_hz = SampleRateFromDivider(s_rate_dividers[rate_index]);
    used = snprintf(line, sizeof(line), "[RF_SPEC] r=%u fs_Hz=%lu",
                    (unsigned int)rate_index,
                    (unsigned long)(sample_rate_hz + 0.5f));
    for (peak = 0U; peak < PLL_TEST_SPECTRUM_PEAKS; ++peak) {
        uint16_t best_bin = PLL_TEST_ALIAS_EDGE_BINS;
        float best_power = 0.0f;
        for (bin = PLL_TEST_ALIAS_EDGE_BINS;
             bin < PLL_TEST_SPECTRUM_BINS - PLL_TEST_ALIAS_EDGE_BINS; ++bin) {
            uint8_t prior;
            uint8_t excluded = 0U;
            float power = s_reference_spectrum[rate_index][bin];
            for (prior = 0U; prior < selected_count; ++prior) {
                uint16_t chosen = selected[prior];
                if (((bin + 4U) >= chosen) && (bin <= (chosen + 4U))) {
                    excluded = 1U;
                    break;
                }
            }
            if ((excluded == 0U) && (power > best_power)) {
                best_power = power;
                best_bin = bin;
            }
        }
        if (best_power <= 1.0f) break;
        if (selected_count == 0U) strongest_power = best_power;
        selected[selected_count++] = best_bin;
        if ((used > 0) && ((uint32_t)used < sizeof(line))) {
            float frequency_hz =
                ((float)best_bin + SpectrumPeakOffset(
                    s_reference_spectrum[rate_index], best_bin)) *
                sample_rate_hz / (float)PLL_TEST_FFT_SIZE;
            float relative_db = 10.0f * log10f((best_power + 1.0f) /
                                                (strongest_power + 1.0f));
            float feedback_power = s_feedback_spectrum[rate_index][best_bin];
            float dominance_db = 10.0f * log10f((best_power + 1.0f) /
                                                 (feedback_power + 1.0f));
            int written = snprintf(&line[used], sizeof(line) - (uint32_t)used,
                                   " p%u=%lu:%ld:%ld",
                                   (unsigned int)(peak + 1U),
                                   (unsigned long)(frequency_hz + 0.5f),
                                   (long)ScaleToInt(relative_db, 10.0f),
                                   (long)ScaleToInt(dominance_db, 10.0f));
            if (written > 0) used += written;
        }
    }
    if ((used > 0) && ((uint32_t)used < (sizeof(line) - 2U))) {
        line[used++] = '\r';
        line[used++] = '\n';
        line[used] = '\0';
        UartSend("%s", line);
    }
}

static uint32_t PhaseDegToWord(double phase_deg)
{
    uint64_t rounded_word;

    while (phase_deg >= 360.0) phase_deg -= 360.0;
    while (phase_deg < 0.0) phase_deg += 360.0;
    rounded_word = (uint64_t)((phase_deg * PLL_TEST_PHASE_WORD_SCALE /
                               360.0) + 0.5);
    return (uint32_t)rounded_word;
}

static AD9708_StatusTypeDef ConfigureAd9708Output(void)
{
    AD9708_StatusTypeDef status;
    const AD9708_DataTypeDef *output_data;

    status = AD9708_Init(&hspi2);
    if (status != AD9708_OK) return status;
    status = AD9708_LoadGeneratedWave(
        AD9708_WAVE_SINE, PLL_TEST_AD9708_WAVE_POINTS);
    if (status != AD9708_OK) return status;
    status = AD9708_SetMode(WaveformToAd9708Mode(s_output_waveform));
    if (status != AD9708_OK) return status;
    status = AD9708_SetLevel((uint8_t)s_ad9708_amplitude,
                             s_ad9708_offset);
    if (status != AD9708_OK) return status;
    if ((s_output_waveform == PLL_TEST_WAVE_AM) &&
        (s_modulation_estimate.valid != 0U)) {
        status = AD9708_ConfigureAm(s_modulation_estimate.modulation_hz,
                                    s_modulation_estimate.am_depth,
                                    0U);
    } else if ((s_output_waveform == PLL_TEST_WAVE_FM) &&
               (s_modulation_estimate.valid != 0U)) {
        status = AD9708_ConfigureFm(
            s_modulation_estimate.modulation_hz,
            s_modulation_estimate.fm_deviation_hz,
            0U);
    } else {
        status = AD9708_DisableModulation();
    }
    if (status != AD9708_OK) return status;
    status = AD9708_UpdateDdsFast(0.0f, s_ad9708_phase_word, 0U);
    if (status == AD9708_OK) {
        output_data = AD9708_GetData();
        s_ad9708_frequency_hz = output_data->output_hz;
    }
    return status;
}

static uint8_t ApplyOutputWaveform(PllTest_Waveform_t waveform)
{
    AD9708_StatusTypeDef status;
    uint8_t waveform_changed;

    if ((waveform == PLL_TEST_WAVE_AUTO) ||
        (waveform == PLL_TEST_WAVE_UNKNOWN)) {
        waveform = PLL_TEST_WAVE_SINE;
    }
    waveform_changed = (waveform != s_output_waveform) ? 1U : 0U;
    if ((waveform == s_output_waveform) &&
        (waveform != PLL_TEST_WAVE_AM) &&
        (waveform != PLL_TEST_WAVE_FM)) return 1U;
    if (s_ad9708_recovery_pending != 0U) return 0U;

    if ((waveform != PLL_TEST_WAVE_AM) &&
        (waveform != PLL_TEST_WAVE_FM)) {
        status = AD9708_DisableModulation();
        if (status == AD9708_OK) {
            status = AD9708_SetMode(WaveformToAd9708Mode(waveform));
        }
    } else if ((s_modulation_estimate.valid == 0U) ||
               (s_modulation_estimate.waveform != waveform)) {
        status = AD9708_ERROR_PARAM;
    } else {
        status = AD9708_SetMode(AD9708_MODE_RAM_DDS);
        if ((status == AD9708_OK) && (waveform == PLL_TEST_WAVE_AM)) {
            status = AD9708_ConfigureAm(
                s_modulation_estimate.modulation_hz,
                s_modulation_estimate.am_depth,
                0U);
        } else if (status == AD9708_OK) {
            status = AD9708_ConfigureFm(
                s_modulation_estimate.modulation_hz,
                s_modulation_estimate.fm_deviation_hz,
                0U);
        }
    }
    if (status != AD9708_OK) {
        s_ad9708_error_count++;
        if (s_ad9708_consecutive_errors < 0xFFU) {
            s_ad9708_consecutive_errors++;
        }
        if (s_ad9708_consecutive_errors >=
            PLL_TEST_AD9708_RECOVERY_ERROR_LIMIT) {
            s_ad9708_recovery_pending = 1U;
        }
        return 0U;
    }
    s_ad9708_consecutive_errors = 0U;
    s_output_waveform = waveform;
    if (waveform_changed != 0U) {
        s_modulation_phase_valid = 0U;
        s_modulation_lock_count = 0U;
        s_modulation_phase_error_deg = 0.0f;
    }
    return 1U;
}

static void BeginAd9708Recovery(void)
{
    if ((s_ad9708_recovery_pending == 0U) ||
        (s_ad9708_recovery_requested != 0U)) {
        return;
    }

    /* Stop the DMA producer before the slow FPGA/FIR/wave-RAM transaction.
     * This preserves true ping-pong ownership instead of accumulating stale
     * half-buffer events while the independently powered FPGA restarts. */
    s_running = 0U;
    ADC_DualCapture_Stop();
    FpgaBootRestore_RequestRecovery();
    s_ad9708_recovery_requested = 1U;
    UartSend("[AD9708] LINK_LOST consecutive=%u out_err=%lu; "
             "automatic FPGA restore armed\r\n",
             (unsigned int)s_ad9708_consecutive_errors,
             (unsigned long)s_ad9708_error_count);
}

static uint8_t ServiceAd9708Recovery(void)
{
    AD9708_StatusTypeDef output_status;
    const FpgaBootRestore_Status *boot_status;

    if (s_ad9708_recovery_pending == 0U) return 1U;
    BeginAd9708Recovery();
    boot_status = FpgaBootRestore_GetStatus();
    if ((boot_status == NULL) || (boot_status->ready == 0U)) return 0U;

    output_status = ConfigureAd9708Output();
    if (output_status != AD9708_OK) {
        /* The ProMax register bank may become visible before the AD9708 bank
         * during a rail/configuration transition. Retry the whole transaction. */
        FpgaBootRestore_RequestRecovery();
        return 0U;
    }
    if (ADC_DualCapture_Start() != HAL_OK) return 0U;

    s_ad9708_consecutive_errors = 0U;
    s_ad9708_recovery_pending = 0U;
    s_ad9708_recovery_requested = 0U;
    s_running = 1U;
    BeginFrequencySweep();
    UartSend("[AD9708] RECOVERED boot_attempt=%lu out_err=%lu\r\n",
             (unsigned long)boot_status->attempt_count,
             (unsigned long)s_ad9708_error_count);
    return 1U;
}
/*
 * FPGA/AD9708 PLL backend. The sine table and output level are configured once
 * during startup; built-in square/triangle/saw modes reuse the same phase
 * accumulator. Runtime updates use one SPI session, keep FPGA ACK checking,
 * omit slow register readback, and skip unchanged phase/enable registers.
 * Normal type-II operation therefore writes only the 32-bit frequency word.
 */
static uint8_t ApplyPllOutput(float frequency_hz, float phase_step_deg)
{
    AD9708_StatusTypeDef status;
    const AD9708_DataTypeDef *output_data;
    uint32_t start_cycles;
    uint32_t elapsed_cycles;
    uint8_t enable;

    if (s_ad9708_recovery_pending != 0U) return 0U;

    frequency_hz = ClampFloat(frequency_hz, 0.0f,
                              (float)PLL_TEST_MAX_OUTPUT_HZ);
    s_ad9708_phase_deg += (double)phase_step_deg;
    while (s_ad9708_phase_deg >= 360.0) s_ad9708_phase_deg -= 360.0;
    while (s_ad9708_phase_deg < 0.0) s_ad9708_phase_deg += 360.0;
    s_ad9708_phase_word = PhaseDegToWord(s_ad9708_phase_deg);
    enable = (frequency_hz > 0.0f) ? 1U : 0U;

    start_cycles = DWT->CYCCNT;
    status = AD9708_UpdateDdsFast(frequency_hz,
                                  s_ad9708_phase_word,
                                  enable);
    elapsed_cycles = DWT->CYCCNT - start_cycles;
    if (SystemCoreClock >= 1000000U) {
        s_ad9708_last_update_us =
            elapsed_cycles / (SystemCoreClock / 1000000U);
    }
    if (status != AD9708_OK) {
        s_ad9708_error_count++;
        if (s_ad9708_consecutive_errors < 0xFFU) {
            s_ad9708_consecutive_errors++;
        }
        if (s_ad9708_consecutive_errors >=
            PLL_TEST_AD9708_RECOVERY_ERROR_LIMIT) {
            s_ad9708_recovery_pending = 1U;
        }
        return 0U;
    }

    s_ad9708_consecutive_errors = 0U;
    output_data = AD9708_GetData();
    s_ad9708_frequency_hz = output_data->output_hz;
    s_ad9708_update_count++;
    return 1U;
}

static uint8_t ApplicationLocked(void)
{
    if (SoftPll_IsLocked(&s_pll) == 0U) return 0U;
    if ((s_output_waveform == PLL_TEST_WAVE_AM) ||
        (s_output_waveform == PLL_TEST_WAVE_FM)) {
        return ((s_modulation_phase_valid != 0U) &&
                (s_modulation_lock_count >=
                 PLL_TEST_MOD_PHASE_LOCK_COUNT)) ? 1U : 0U;
    }
    return 1U;
}

static void UpdateModulationPhaseLock(
    const ADC_DualCapture_Pair_t *pairs,
    float reference_mean,
    float feedback_mean,
    float sample_rate_hz,
    uint32_t now)
{
    float ref_carrier_re;
    float ref_carrier_im;
    float ref_upper_re;
    float ref_upper_im;
    float fb_carrier_re;
    float fb_carrier_im;
    float fb_upper_re;
    float fb_upper_im;
    float ref_ratio_re;
    float ref_ratio_im;
    float fb_ratio_re;
    float fb_ratio_im;
    float error_re;
    float error_im;
    float error_deg;
    float correction_deg;
    uint32_t correction_word;
    uint32_t target_word;
    const AD9708_DataTypeDef *output_data;
    AD9708_StatusTypeDef status;

    if ((pairs == NULL) ||
        ((s_output_waveform != PLL_TEST_WAVE_AM) &&
         (s_output_waveform != PLL_TEST_WAVE_FM)) ||
        (s_modulation_estimate.valid == 0U) ||
        (SoftPll_IsLocked(&s_pll) == 0U) ||
        ((now - s_last_modulation_phase_tick) <
         PLL_TEST_MOD_PHASE_UPDATE_MS)) return;
    s_last_modulation_phase_tick = now;

    if ((MeasurePairedTone(pairs, 0U, reference_mean,
                           s_frequency_estimate_hz, sample_rate_hz,
                           &ref_carrier_re, &ref_carrier_im) == 0U) ||
        (MeasurePairedTone(pairs, 0U, reference_mean,
                           s_frequency_estimate_hz +
                           s_modulation_estimate.modulation_hz,
                           sample_rate_hz,
                           &ref_upper_re, &ref_upper_im) == 0U) ||
        (MeasurePairedTone(pairs, 16U, feedback_mean,
                           s_ad9708_frequency_hz, sample_rate_hz,
                           &fb_carrier_re, &fb_carrier_im) == 0U) ||
        (MeasurePairedTone(pairs, 16U, feedback_mean,
                           s_ad9708_frequency_hz +
                           s_modulation_estimate.modulation_hz,
                           sample_rate_hz,
                           &fb_upper_re, &fb_upper_im) == 0U)) {
        s_modulation_phase_valid = 0U;
        s_modulation_lock_count = 0U;
        return;
    }

    /* Sideband/carrier ratios remove the independently controlled carrier
     * phase. Their phase difference is therefore only modulation phase. */
    ref_ratio_re =
        ref_upper_re * ref_carrier_re + ref_upper_im * ref_carrier_im;
    ref_ratio_im =
        ref_upper_im * ref_carrier_re - ref_upper_re * ref_carrier_im;
    fb_ratio_re =
        fb_upper_re * fb_carrier_re + fb_upper_im * fb_carrier_im;
    fb_ratio_im =
        fb_upper_im * fb_carrier_re - fb_upper_re * fb_carrier_im;
    error_re = ref_ratio_re * fb_ratio_re +
               ref_ratio_im * fb_ratio_im;
    error_im = ref_ratio_im * fb_ratio_re -
               ref_ratio_re * fb_ratio_im;
    if ((error_re * error_re + error_im * error_im) <= 1.0f) {
        s_modulation_phase_valid = 0U;
        s_modulation_lock_count = 0U;
        return;
    }
    error_deg = atan2f(error_im, error_re) *
                (360.0f / PLL_TEST_TWO_PI);
    s_modulation_phase_error_deg = error_deg;
    s_modulation_phase_valid = 1U;
    if (fabsf(error_deg) <= PLL_TEST_MOD_PHASE_LOCK_ERROR_DEG) {
        if (s_modulation_lock_count < UINT8_MAX) {
            s_modulation_lock_count++;
        }
    } else {
        s_modulation_lock_count = 0U;
    }

    correction_deg = ClampFloat(
        PLL_TEST_MOD_PHASE_GAIN * error_deg,
        -PLL_TEST_MOD_PHASE_MAX_STEP_DEG,
        PLL_TEST_MOD_PHASE_MAX_STEP_DEG);
    if (fabsf(correction_deg) < 0.05f) return;
    output_data = AD9708_GetData();
    correction_word = PhaseDegToWord((double)correction_deg);
    target_word = output_data->modulation_phase_word + correction_word;
    status = AD9708_UpdateModulationPhaseFast(target_word);
    if (status != AD9708_OK) {
        s_ad9708_error_count++;
        if (s_ad9708_consecutive_errors < UINT8_MAX) {
            s_ad9708_consecutive_errors++;
        }
        if (s_ad9708_consecutive_errors >=
            PLL_TEST_AD9708_RECOVERY_ERROR_LIMIT) {
            s_ad9708_recovery_pending = 1U;
        }
    } else {
        s_ad9708_consecutive_errors = 0U;
    }
}

static void MonitorReferenceWaveform(const ADC_DualCapture_Pair_t *pairs,
                                     float reference_mean,
                                     float sample_rate_hz,
                                     uint32_t now)
{
    PllTest_Waveform_t candidate;
#if PLL_TEST_EXPERIMENTAL_AMFM_ENABLE
    PllTest_ModulationEstimate_t modulation;
#endif
    float h2_db = -120.0f;
    float h3_db = -120.0f;
    uint8_t h2_valid;
    uint8_t h3_valid;
#if PLL_TEST_EXPERIMENTAL_AMFM_ENABLE
    uint8_t rate_index = PLL_TEST_RATE_COUNT;
#endif
    uint32_t i;

    if ((pairs == NULL) || (sample_rate_hz <= 0.0f) ||
        (s_sweep_active != 0U) || (s_mode != PLL_TEST_MODE_AC_LOCK) ||
        ((now - s_last_wave_monitor_tick) <
         PLL_TEST_WAVE_MONITOR_PERIOD_MS)) return;
    s_last_wave_monitor_tick = now;
    for (i = 0U; i < PLL_TEST_FFT_SIZE; ++i) {
        float sample = (float)(pairs[i] & 0xFFFFU) - reference_mean;
        s_fft_buffer[2U * i] = sample * s_fft_window[i];
        s_fft_buffer[2U * i + 1U] = 0.0f;
    }
    arm_cfft_f32(&arm_cfft_sR_f32_len1024, s_fft_buffer, 0U, 1U);
    for (i = 0U; i < PLL_TEST_SPECTRUM_BINS; ++i) {
        float re = s_fft_buffer[2U * i];
        float im = s_fft_buffer[2U * i + 1U];
        s_wave_monitor_spectrum[i] = re * re + im * im;
    }
#if PLL_TEST_EXPERIMENTAL_AMFM_ENABLE
    for (i = 0U; i < PLL_TEST_RATE_COUNT; ++i) {
        if (s_rate_dividers[i] == s_lock_sample_divider) {
            rate_index = (uint8_t)i;
            break;
        }
    }
    memset(&modulation, 0, sizeof(modulation));
    if (rate_index < PLL_TEST_RATE_COUNT) {
        for (i = 0U; i < PLL_TEST_FFT_SIZE; ++i) {
            s_sweep_reference[rate_index][i] =
                (uint16_t)(pairs[i] & 0xFFFFU);
        }
        s_sweep_reference_mean[rate_index] = reference_mean;
        memcpy(s_reference_spectrum[rate_index],
               s_wave_monitor_spectrum,
               sizeof(s_wave_monitor_spectrum));
    }
#endif
    h2_valid = SpectrumHarmonicDb(s_wave_monitor_spectrum,
        s_frequency_estimate_hz, 2U, sample_rate_hz, &h2_db);
    h3_valid = SpectrumHarmonicDb(s_wave_monitor_spectrum,
        s_frequency_estimate_hz, 3U, sample_rate_hz, &h3_db);
    if (h2_valid != 0U) s_harmonic2_relative_db = h2_db;
    if (h3_valid != 0U) s_harmonic3_relative_db = h3_db;
#if PLL_TEST_EXPERIMENTAL_AMFM_ENABLE
    if ((rate_index < PLL_TEST_RATE_COUNT) &&
        (EstimateModulationAtRate(rate_index, s_frequency_estimate_hz,
                                  &modulation) != 0U)) {
        candidate = modulation.waveform;
    } else {
        candidate = DecidePeriodicWaveform(s_frequency_estimate_hz,
            h2_db, h2_valid, h3_db, h3_valid);
    }
#else
    candidate = DecidePeriodicWaveform(s_frequency_estimate_hz,
        h2_db, h2_valid, h3_db, h3_valid);
#endif
    if (candidate == PLL_TEST_WAVE_UNKNOWN) {
        s_wave_monitor_candidate = candidate;
        s_wave_monitor_confirm_count = 0U;
        return;
    }
    if (candidate != s_wave_monitor_candidate) {
        s_wave_monitor_candidate = candidate;
        s_wave_monitor_confirm_count = 1U;
        return;
    }
    if (s_wave_monitor_confirm_count < UINT8_MAX) {
        s_wave_monitor_confirm_count++;
    }
    if (s_wave_monitor_confirm_count <
        PLL_TEST_WAVE_MONITOR_CONFIRM_COUNT) return;
#if PLL_TEST_EXPERIMENTAL_AMFM_ENABLE
    if ((candidate == PLL_TEST_WAVE_AM) ||
        (candidate == PLL_TEST_WAVE_FM)) {
        s_modulation_estimate = modulation;
    } else {
        memset(&s_modulation_estimate, 0, sizeof(s_modulation_estimate));
    }
#endif
    s_detected_waveform = candidate;
    s_wave_monitor_confirm_count = PLL_TEST_WAVE_MONITOR_CONFIRM_COUNT;
    if ((s_requested_waveform == PLL_TEST_WAVE_AUTO) &&
        ((candidate != s_output_waveform) ||
         (candidate == PLL_TEST_WAVE_AM) ||
         (candidate == PLL_TEST_WAVE_FM)) &&
        (ApplyOutputWaveform(candidate) != 0U)) {
        s_last_wave_monitor_tick = now; /* 100% confirmed. */
    }
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

static uint8_t EstimateIqPhases(const ADC_DualCapture_Pair_t *pairs,
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
        uint32_t packed = pairs[i];
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
    /* The type-II frequency loop closes static phase error. Incremental POW
     * kicks remain disabled. A guarded FLL assist below applies a measured
     * phase slope only after several consistent frames, instead of feeding
     * noisy instantaneous slope into every DDS update. */
    s_pll_config.phase_kp = 0.0f;
    s_pll_config.phase_ki = 0.0f;
    s_pll_config.freq_kd = PLL_TEST_FREQ_RATE_FEEDFORWARD;
    s_pll_config.lock_error_deg = PLL_TEST_LOCK_ERROR_DEG;
    s_pll_config.lock_rate_hz = fmaxf(PLL_TEST_LOCK_RATE_HZ,
                                          actual_bw * 0.05f);
    s_pll_config.lock_count_required = PLL_TEST_LOCK_COUNT;
    s_pll_config.unlock_count_required = PLL_TEST_UNLOCK_COUNT;
    (void)SoftPll_DesignType2(&s_pll_config, actual_bw,
                              PLL_TEST_DEFAULT_DAMPING, update_rate);
    SoftPll_Init(&s_pll, &s_pll_config);
    (void)ApplyPllOutput(frequency_hz, 0.0f);
    s_mode = PLL_TEST_MODE_AC_LOCK;
    s_lock_start_tick = HAL_GetTick();
    s_last_locked_tick = s_lock_start_tick;
    s_rate_assist_sum_hz = 0.0f;
    s_rate_assist_last_hz = 0.0f;
    s_rate_assist_stable_count = 0U;
    s_rate_assist_applied = 0U;
}

static uint8_t ApplyStableRateAssist(void)
{
    float rate_hz = s_pll.phase_rate_hz;
    float maximum_rate_hz = 0.45f * s_last_block_sample_rate_hz /
                            (float)PLL_TEST_ADC_HALF_SAMPLES;
    float tolerance_hz;
    float correction_hz;
    float corrected_nominal_hz;

    if ((s_pll.feedback_valid == 0U) || (s_pll.locked != 0U) ||
        (s_rate_assist_applied != 0U) ||
        (!isfinite(rate_hz)) || (fabsf(rate_hz) < PLL_TEST_RATE_ASSIST_MIN_HZ) ||
        (fabsf(rate_hz) > maximum_rate_hz)) {
        s_rate_assist_sum_hz = 0.0f;
        s_rate_assist_stable_count = 0U;
        return 0U;
    }
    tolerance_hz = fmaxf(PLL_TEST_RATE_ASSIST_ABS_TOL_HZ,
                         PLL_TEST_RATE_ASSIST_REL_TOL *
                         fabsf(s_rate_assist_last_hz));
    if ((s_rate_assist_stable_count == 0U) ||
        (fabsf(rate_hz - s_rate_assist_last_hz) <= tolerance_hz)) {
        s_rate_assist_sum_hz += rate_hz;
        if (s_rate_assist_stable_count < UINT8_MAX) {
            s_rate_assist_stable_count++;
        }
    } else {
        s_rate_assist_sum_hz = rate_hz;
        s_rate_assist_stable_count = 1U;
    }
    s_rate_assist_last_hz = rate_hz;
    if (s_rate_assist_stable_count < PLL_TEST_RATE_ASSIST_STABLE_N) return 0U;

    correction_hz = s_rate_assist_sum_hz /
                    (float)s_rate_assist_stable_count;
    corrected_nominal_hz = s_pll.cfg.nominal_freq_hz + correction_hz;
    s_rate_assist_sum_hz = 0.0f;
    s_rate_assist_stable_count = 0U;
    if ((corrected_nominal_hz < PLL_TEST_MIN_AC_FREQ_HZ) ||
        (corrected_nominal_hz > (float)PLL_TEST_MAX_OUTPUT_HZ) ||
        (SoftPll_Retune(&s_pll, corrected_nominal_hz, 1U) == 0U)) {
        return 0U;
    }
    s_frequency_estimate_hz = corrected_nominal_hz;
    s_lock_start_tick = HAL_GetTick();
    s_last_locked_tick = s_lock_start_tick;
    s_rate_assist_applied = 1U;
    s_rate_assist_apply_count++;
    (void)ApplyPllOutput(s_pll.dds_freq_cmd_hz, 0.0f);
    return 1U;
}

/* Select the minimum number of mutually consistent sample rates.  This makes
 * the resolver robust when one rate places the wanted RF near DC/Nyquist and
 * its FFT therefore reports an unrelated spur instead of the wanted alias. */
static uint8_t SelectCandidateInliers(float candidate_hz,
                                      uint8_t *inlier_mask,
                                      float *rms_hz)
{
    uint8_t selected_mask = 0U;
    uint8_t selected_count = 0U;
    float error_sum = 0.0f;

    if ((inlier_mask == NULL) || (rms_hz == NULL)) return 0U;
    while (selected_count < PLL_TEST_SWEEP_MIN_RATES) {
        uint8_t i;
        uint8_t best_index = PLL_TEST_RATE_COUNT;
        float best_error = 1.0e30f;
        for (i = 0U; i < PLL_TEST_RATE_COUNT; ++i) {
            uint8_t bit = (uint8_t)(1U << i);
            if (((s_alias_valid_mask & bit) != 0U) &&
                ((selected_mask & bit) == 0U)) {
                float rate = SampleRateFromDivider(s_rate_dividers[i]);
                float error = fabsf(FoldAliasHz(candidate_hz, rate) -
                                     s_alias_hz[i]);
                if (error < best_error) {
                    best_error = error;
                    best_index = i;
                }
            }
        }
        if (best_index >= PLL_TEST_RATE_COUNT) return 0U;
        selected_mask |= (uint8_t)(1U << best_index);
        error_sum += best_error * best_error;
        selected_count++;
    }
    *inlier_mask = selected_mask;
    *rms_hz = sqrtf(error_sum / (float)selected_count);
    return 1U;
}

/* Project a rough RF candidate onto the nearest k*Fs +/- alias branch for
 * each selected rate, then average the independent RF estimates. */
static float RefineCandidateFromMask(float seed_hz, uint8_t inlier_mask)
{
    uint8_t i;
    uint8_t count = 0U;
    float estimate_sum = 0.0f;
    for (i = 0U; i < PLL_TEST_RATE_COUNT; ++i) {
        if ((inlier_mask & (1U << i)) != 0U) {
            float rate = SampleRateFromDivider(s_rate_dividers[i]);
            int32_t centre = (int32_t)floorf(seed_hz / rate + 0.5f);
            int32_t offset;
            float nearest = seed_hz;
            float nearest_distance = 1.0e30f;
            for (offset = -1; offset <= 1; ++offset) {
                float multiple = (float)(centre + offset) * rate;
                float plus = multiple + s_alias_hz[i];
                float minus = multiple - s_alias_hz[i];
                float plus_distance = fabsf(plus - seed_hz);
                float minus_distance = fabsf(minus - seed_hz);
                if (plus_distance < nearest_distance) {
                    nearest = plus;
                    nearest_distance = plus_distance;
                }
                if (minus_distance < nearest_distance) {
                    nearest = minus;
                    nearest_distance = minus_distance;
                }
            }
            estimate_sum += nearest;
            count++;
        }
    }
    return (count != 0U) ? (estimate_sum / (float)count) : seed_hz;
}

static uint8_t SecondaryPeakEligibleMask(void)
{
    uint8_t i;
    uint8_t mask = 0U;
    for (i = 0U; i < PLL_TEST_RATE_COUNT; ++i) {
        uint8_t bit = (uint8_t)(1U << i);
        if (((s_alias_valid_mask & bit) != 0U) &&
            (s_alias_peak2_rel_db[i] >= PLL_TEST_ALT_PEAK_MIN_REL_DB) &&
            (s_alias_peak2_ref_fb_db[i] >=
             PLL_TEST_ALT_PEAK_MIN_REF_FB_DB)) {
            mask |= bit;
        }
    }
    return mask;
}

static uint8_t SelectSecondaryCandidateInliers(float candidate_hz,
                                                uint8_t eligible_mask,
                                                uint8_t *inlier_mask,
                                                float *rms_hz)
{
    uint8_t selected_mask = 0U;
    uint8_t selected_count = 0U;
    float error_sum = 0.0f;

    if ((inlier_mask == NULL) || (rms_hz == NULL)) return 0U;
    while (selected_count < PLL_TEST_SWEEP_MIN_RATES) {
        uint8_t i;
        uint8_t best_index = PLL_TEST_RATE_COUNT;
        float best_error = 1.0e30f;
        for (i = 0U; i < PLL_TEST_RATE_COUNT; ++i) {
            uint8_t bit = (uint8_t)(1U << i);
            if (((eligible_mask & bit) != 0U) &&
                ((selected_mask & bit) == 0U)) {
                float rate = SampleRateFromDivider(s_rate_dividers[i]);
                float error = fabsf(FoldAliasHz(candidate_hz, rate) -
                                     s_alias_peak2_hz[i]);
                if (error < best_error) {
                    best_error = error;
                    best_index = i;
                }
            }
        }
        if (best_index >= PLL_TEST_RATE_COUNT) return 0U;
        selected_mask |= (uint8_t)(1U << best_index);
        error_sum += best_error * best_error;
        selected_count++;
    }
    *inlier_mask = selected_mask;
    *rms_hz = sqrtf(error_sum / (float)selected_count);
    return 1U;
}

static float RefineSecondaryCandidate(float seed_hz, uint8_t inlier_mask)
{
    uint8_t i;
    uint8_t count = 0U;
    float estimate_sum = 0.0f;
    for (i = 0U; i < PLL_TEST_RATE_COUNT; ++i) {
        if ((inlier_mask & (1U << i)) != 0U) {
            float rate = SampleRateFromDivider(s_rate_dividers[i]);
            int32_t centre = (int32_t)floorf(seed_hz / rate + 0.5f);
            int32_t offset;
            float nearest = seed_hz;
            float nearest_distance = 1.0e30f;
            for (offset = -1; offset <= 1; ++offset) {
                float multiple = (float)(centre + offset) * rate;
                float plus = multiple + s_alias_peak2_hz[i];
                float minus = multiple - s_alias_peak2_hz[i];
                float plus_distance = fabsf(plus - seed_hz);
                float minus_distance = fabsf(minus - seed_hz);
                if (plus_distance < nearest_distance) {
                    nearest = plus;
                    nearest_distance = plus_distance;
                }
                if (minus_distance < nearest_distance) {
                    nearest = minus;
                    nearest_distance = minus_distance;
                }
            }
            estimate_sum += nearest;
            count++;
        }
    }
    return (count != 0U) ? (estimate_sum / (float)count) : seed_hz;
}

/* Resolve a coherent second-peak vector only when the normal strongest-peak
 * result lands on the measured board spur. This keeps a genuine lone
 * 4.375 MHz input valid while allowing a weaker real RF tone to win. */
static uint8_t ResolveSecondaryFrequency(float *frequency_hz,
                                         float *rms_hz,
                                         uint8_t *resolved_mask)
{
    uint8_t i;
    uint8_t eligible_mask = SecondaryPeakEligibleMask();
    uint8_t eligible_count = 0U;
    uint8_t base_index = PLL_TEST_RATE_COUNT;
    uint8_t best_mask = 0U;
    uint8_t second_mask = 0U;
    float base_quality = -1.0e30f;
    float best_score = 1.0e30f;
    float second_score = 1.0e30f;
    float best_frequency = 0.0f;
    float second_frequency = 0.0f;
    float base_rate;
    uint32_t max_harmonic;
    uint32_t harmonic;

    if ((frequency_hz == NULL) || (rms_hz == NULL) ||
        (resolved_mask == NULL)) return 0U;
    for (i = 0U; i < PLL_TEST_RATE_COUNT; ++i) {
        if ((eligible_mask & (1U << i)) != 0U) {
            float quality = s_alias_snr_db[i] + s_alias_peak2_rel_db[i];
            eligible_count++;
            if (quality > base_quality) {
                base_quality = quality;
                base_index = i;
            }
        }
    }
    if ((eligible_count < PLL_TEST_SWEEP_MIN_RATES) ||
        (base_index >= PLL_TEST_RATE_COUNT)) return 0U;

    base_rate = SampleRateFromDivider(s_rate_dividers[base_index]);
    max_harmonic =
        (uint32_t)((float)PLL_TEST_AUTO_SEARCH_MAX_HZ / base_rate) + 2U;
    for (harmonic = 0U; harmonic <= max_harmonic; ++harmonic) {
        int sign_index;
        for (sign_index = -1; sign_index <= 1; sign_index += 2) {
            float seed = (float)harmonic * base_rate +
                         (float)sign_index * s_alias_peak2_hz[base_index];
            float score;
            uint8_t inlier_mask;
            if ((seed < PLL_TEST_MIN_AC_FREQ_HZ) ||
                (seed > (float)PLL_TEST_AUTO_SEARCH_MAX_HZ) ||
                (SelectSecondaryCandidateInliers(seed, eligible_mask,
                                                 &inlier_mask, &score) == 0U)) {
                continue;
            }
            if ((best_frequency > 0.0f) &&
                (fabsf(seed - best_frequency) <= 1000.0f)) {
                if (score < best_score) {
                    best_score = score;
                    best_frequency = seed;
                    best_mask = inlier_mask;
                }
            } else if ((second_frequency > 0.0f) &&
                       (fabsf(seed - second_frequency) <= 1000.0f)) {
                if (score < second_score) {
                    second_score = score;
                    second_frequency = seed;
                    second_mask = inlier_mask;
                }
            } else if (score < best_score) {
                second_score = best_score;
                second_frequency = best_frequency;
                second_mask = best_mask;
                best_score = score;
                best_frequency = seed;
                best_mask = inlier_mask;
            } else if (score < second_score) {
                second_score = score;
                second_frequency = seed;
                second_mask = inlier_mask;
            }
        }
    }
    if (best_frequency <= 0.0f) return 0U;
    best_frequency = RefineSecondaryCandidate(best_frequency, best_mask);
    if (SelectSecondaryCandidateInliers(best_frequency, eligible_mask,
                                        &best_mask, &best_score) == 0U) return 0U;
    best_frequency = RefineSecondaryCandidate(best_frequency, best_mask);
    if ((best_frequency < PLL_TEST_MIN_AC_FREQ_HZ) ||
        (best_frequency > (float)PLL_TEST_AUTO_SEARCH_MAX_HZ) ||
        (best_score > PLL_TEST_SWEEP_MAX_RMS_HZ)) return 0U;

    if (second_frequency > 0.0f) {
        second_frequency = RefineSecondaryCandidate(second_frequency,
                                                    second_mask);
        (void)SelectSecondaryCandidateInliers(second_frequency,
                                              eligible_mask,
                                              &second_mask, &second_score);
    }
    if ((second_score < 1.0e29f) &&
        ((second_score - best_score) < 250.0f)) return 0U;
    *frequency_hz = best_frequency;
    *rms_hz = best_score;
    *resolved_mask = best_mask;
    return 1U;
}
/* A hint is never accepted by itself. Each usable sample rate must contain a
 * local PC4 peak at the predicted alias, with PC4/PC5 dominance proving that
 * it is the external reference rather than DDS feedback. Three independent
 * rates then reconstruct and average the RF branch. */
static uint8_t ResolveHintFrequency(float *frequency_hz, float *rms_hz)
{
    uint8_t rate_index;
    uint8_t count = 0U;
    uint8_t mask = 0U;
    float estimates[PLL_TEST_RATE_COUNT] = {0.0f};
    float aliases[PLL_TEST_RATE_COUNT] = {0.0f};
    float powers[PLL_TEST_RATE_COUNT] = {0.0f};
    float estimate_sum = 0.0f;
    float estimate_mean;
    float residual_sum = 0.0f;

    if ((frequency_hz == NULL) || (rms_hz == NULL) ||
        (s_acquisition_hint_hz <= 0.0f)) return 0U;

    for (rate_index = 0U; rate_index < PLL_TEST_RATE_COUNT; ++rate_index) {
        float sample_rate_hz = SampleRateFromDivider(s_rate_dividers[rate_index]);
        float bin_hz = sample_rate_hz / (float)PLL_TEST_FFT_SIZE;
        float expected_alias_hz = FoldAliasHz(s_acquisition_hint_hz,
                                               sample_rate_hz);
        float expected_bin_f = expected_alias_hz / bin_hz;
        uint32_t centre_bin;
        uint32_t radius_bins;
        uint32_t low_bin;
        uint32_t high_bin;
        uint32_t bin;
        uint32_t best_bin = 0U;
        float strongest_power = 0.0f;
        float best_power = 0.0f;
        float best_feedback_power = 0.0f;
        float actual_alias_hz;
        int32_t centre_harmonic;
        int32_t harmonic_offset;
        float nearest_rf_hz = 0.0f;
        float nearest_distance = 1.0e30f;

        if ((expected_bin_f < (float)PLL_TEST_ALIAS_EDGE_BINS) ||
            (expected_bin_f >= (float)(PLL_TEST_SPECTRUM_BINS -
                                        PLL_TEST_ALIAS_EDGE_BINS))) continue;
        centre_bin = (uint32_t)(expected_bin_f + 0.5f);
        radius_bins = (uint32_t)(s_acquisition_hint_tolerance_hz / bin_hz) + 2U;
        low_bin = (centre_bin > radius_bins) ? centre_bin - radius_bins :
                  PLL_TEST_ALIAS_EDGE_BINS;
        if (low_bin < PLL_TEST_ALIAS_EDGE_BINS) {
            low_bin = PLL_TEST_ALIAS_EDGE_BINS;
        }
        high_bin = centre_bin + radius_bins;
        if (high_bin >= PLL_TEST_SPECTRUM_BINS - PLL_TEST_ALIAS_EDGE_BINS) {
            high_bin = PLL_TEST_SPECTRUM_BINS - PLL_TEST_ALIAS_EDGE_BINS - 1U;
        }
        for (bin = PLL_TEST_ALIAS_EDGE_BINS;
             bin < PLL_TEST_SPECTRUM_BINS - PLL_TEST_ALIAS_EDGE_BINS; ++bin) {
            float power = s_reference_spectrum[rate_index][bin];
            if (power > strongest_power) strongest_power = power;
        }
        for (bin = low_bin; bin <= high_bin; ++bin) {
            float power = s_reference_spectrum[rate_index][bin];
            if (power > best_power) {
                best_power = power;
                best_feedback_power = s_feedback_spectrum[rate_index][bin];
                best_bin = bin;
            }
        }
        if ((best_bin == 0U) || (strongest_power <= 1.0f) ||
            (best_power < strongest_power * PLL_TEST_HINT_MIN_REL_POWER) ||
            ((best_power + 1.0f) <
             (best_feedback_power + 1.0f) * PLL_TEST_HINT_MIN_DOM_RATIO)) {
            continue;
        }

        actual_alias_hz = ((float)best_bin + SpectrumPeakOffset(
            s_reference_spectrum[rate_index], (uint16_t)best_bin)) * bin_hz;
        centre_harmonic = (int32_t)floorf(s_acquisition_hint_hz /
                                          sample_rate_hz + 0.5f);
        for (harmonic_offset = -1; harmonic_offset <= 1; ++harmonic_offset) {
            int32_t harmonic = centre_harmonic + harmonic_offset;
            float multiple;
            float plus;
            float minus;
            float distance;
            if (harmonic < 0) continue;
            multiple = (float)harmonic * sample_rate_hz;
            plus = multiple + actual_alias_hz;
            minus = multiple - actual_alias_hz;
            distance = fabsf(plus - s_acquisition_hint_hz);
            if (distance < nearest_distance) {
                nearest_distance = distance;
                nearest_rf_hz = plus;
            }
            if (minus >= 0.0f) {
                distance = fabsf(minus - s_acquisition_hint_hz);
                if (distance < nearest_distance) {
                    nearest_distance = distance;
                    nearest_rf_hz = minus;
                }
            }
        }
        if (nearest_distance > s_acquisition_hint_tolerance_hz + 2.0f * bin_hz) {
            continue;
        }
        estimates[rate_index] = nearest_rf_hz;
        aliases[rate_index] = actual_alias_hz;
        powers[rate_index] = best_power;
        estimate_sum += nearest_rf_hz;
        mask |= (uint8_t)(1U << rate_index);
        count++;
    }
    if (count < PLL_TEST_SWEEP_MIN_RATES) return 0U;
    estimate_mean = estimate_sum / (float)count;
    for (rate_index = 0U; rate_index < PLL_TEST_RATE_COUNT; ++rate_index) {
        if ((mask & (1U << rate_index)) != 0U) {
            float residual = estimates[rate_index] - estimate_mean;
            float noise = s_reference_noise_power[rate_index];
            residual_sum += residual * residual;
            s_alias_hz[rate_index] = aliases[rate_index];
            s_alias_snr_db[rate_index] = 10.0f * log10f(
                (powers[rate_index] + 1.0f) / (noise + 1.0f));
        }
    }
    *rms_hz = sqrtf(residual_sum / (float)count);
    if ((*rms_hz > PLL_TEST_SWEEP_MAX_RMS_HZ) ||
        (estimate_mean < PLL_TEST_MIN_AC_FREQ_HZ) ||
        (estimate_mean > (float)PLL_TEST_MAX_OUTPUT_HZ)) return 0U;
    s_alias_valid_mask = mask;
    *frequency_hz = estimate_mean;
    return 1U;
}
static uint8_t ResolveSweepFrequency(float *frequency_hz, float *rms_hz)
{
    uint8_t i;
    uint8_t valid_count = 0U;
    uint8_t base_order[PLL_TEST_RATE_COUNT];
    uint8_t base_order_count = 0U;
    uint8_t bases_to_search;
    uint8_t order_index;
    uint8_t best_mask = 0U;
    uint8_t second_mask = 0U;
    float best_score = 1.0e30f;
    float second_score = 1.0e30f;
    float best_frequency = 0.0f;
    float second_frequency = 0.0f;

    if ((frequency_hz == NULL) || (rms_hz == NULL)) return 0U;
    for (i = 0U; i < PLL_TEST_RATE_COUNT; ++i) {
        if ((s_alias_valid_mask & (1U << i)) != 0U) {
            uint8_t insert = base_order_count;
            /* Highest-SNR rates are most likely to contain the wanted alias
             * rather than a DC/Nyquist spur.  Searching two bases guarantees
             * one good base when no more than one of four rates is corrupt. */
            while ((insert > 0U) &&
                   (s_alias_snr_db[base_order[insert - 1U]] <
                    s_alias_snr_db[i])) {
                base_order[insert] = base_order[insert - 1U];
                insert--;
            }
            base_order[insert] = i;
            base_order_count++;
            valid_count++;
        }
    }
    if (valid_count < PLL_TEST_SWEEP_MIN_RATES) return 0U;
    bases_to_search = (base_order_count < PLL_TEST_SWEEP_BASE_COUNT) ?
                      base_order_count : PLL_TEST_SWEEP_BASE_COUNT;

    /* Score all k*Fs +/- alias seeds from the two strongest rates.  Candidate
     * refinement is intentionally deferred until after this search; refining
     * every seed was correct but could exceed the 2 ms DMA half-buffer time. */
    for (order_index = 0U; order_index < bases_to_search; ++order_index) {
        uint8_t base_index = base_order[order_index];
        float base_rate = SampleRateFromDivider(s_rate_dividers[base_index]);
        uint32_t max_harmonic =
            (uint32_t)((float)PLL_TEST_AUTO_SEARCH_MAX_HZ / base_rate) + 2U;
        uint32_t harmonic;
        for (harmonic = 0U; harmonic <= max_harmonic; ++harmonic) {
            int sign_index;
            for (sign_index = -1; sign_index <= 1; sign_index += 2) {
                float seed = (float)harmonic * base_rate +
                             (float)sign_index * s_alias_hz[base_index];
                float score;
                uint8_t inlier_mask;

                if ((seed < PLL_TEST_MIN_AC_FREQ_HZ) ||
                    (seed > (float)PLL_TEST_AUTO_SEARCH_MAX_HZ) ||
                    (SelectCandidateInliers(seed, &inlier_mask, &score) == 0U)) {
                    continue;
                }
                if ((best_frequency > 0.0f) &&
                    (fabsf(seed - best_frequency) <= 1000.0f)) {
                    if (score < best_score) {
                        best_score = score;
                        best_frequency = seed;
                        best_mask = inlier_mask;
                    }
                } else if ((second_frequency > 0.0f) &&
                           (fabsf(seed - second_frequency) <= 1000.0f)) {
                    if (score < second_score) {
                        second_score = score;
                        second_frequency = seed;
                        second_mask = inlier_mask;
                    }
                } else if (score < best_score) {
                    second_score = best_score;
                    second_frequency = best_frequency;
                    second_mask = best_mask;
                    best_score = score;
                    best_frequency = seed;
                    best_mask = inlier_mask;
                } else if (score < second_score) {
                    second_score = score;
                    second_frequency = seed;
                    second_mask = inlier_mask;
                }
            }
        }
    }

    if (best_frequency <= 0.0f) return 0U;
    best_frequency = RefineCandidateFromMask(best_frequency, best_mask);
    if (SelectCandidateInliers(best_frequency, &best_mask, &best_score) == 0U) {
        return 0U;
    }
    best_frequency = RefineCandidateFromMask(best_frequency, best_mask);
    if (SelectCandidateInliers(best_frequency, &best_mask, &best_score) == 0U) {
        return 0U;
    }
    if ((best_frequency < PLL_TEST_MIN_AC_FREQ_HZ) ||
        (best_frequency > (float)PLL_TEST_AUTO_SEARCH_MAX_HZ) ||
        (best_score > PLL_TEST_SWEEP_MAX_RMS_HZ)) return 0U;

#if PLL_TEST_BOARD_SPUR_REJECT_ENABLE
    if (IsKnownBoardSpur(best_frequency) != 0U) {
        float alternate_frequency;
        float alternate_score;
        uint8_t alternate_mask;
        if ((ResolveSecondaryFrequency(&alternate_frequency,
                                       &alternate_score,
                                       &alternate_mask) != 0U) &&
            (IsKnownBoardSpur(alternate_frequency) == 0U)) {
            best_frequency = alternate_frequency;
            best_score = alternate_score;
            best_mask = alternate_mask;
            second_score = 1.0e30f;
            for (i = 0U; i < PLL_TEST_RATE_COUNT; ++i) {
                if ((alternate_mask & (1U << i)) != 0U) {
                    s_alias_hz[i] = s_alias_peak2_hz[i];
                    s_alias_snr_db[i] += s_alias_peak2_rel_db[i];
                }
            }
        }
    }
#endif

    if (second_frequency > 0.0f) {
        second_frequency = RefineCandidateFromMask(second_frequency,
                                                   second_mask);
        (void)SelectCandidateInliers(second_frequency, &second_mask,
                                     &second_score);
    }
    /* An ambiguity this close means the analog/FFT data do not justify a
     * 1..40 MHz branch decision; rescan instead of commanding a wrong RF. */
    if ((second_score < 1.0e29f) &&
        ((second_score - best_score) < 250.0f)) return 0U;
    *frequency_hz = best_frequency;
    *rms_hz = best_score;
    /* Downstream lock-rate selection must use only the mutually consistent
     * rates; otherwise a rejected DC spur could still be selected for IQ. */
    s_alias_valid_mask = best_mask;
    return 1U;
}

static uint8_t SelectLockRate(float carrier_hz,
                              PllTest_Waveform_t waveform)
{
    uint8_t i;
    uint8_t best_index = 0U;
    float best_margin = -1.0f;
    for (i = 0U; i < PLL_TEST_RATE_COUNT; ++i) {
        if ((s_alias_valid_mask & (1U << i)) != 0U) {
            float rate = SampleRateFromDivider(s_rate_dividers[i]);
            float alias = s_alias_hz[i];
            float margin = fminf(alias, 0.5f * rate - alias);
            uint8_t first_harmonic = 0U;
            uint8_t harmonic_step = 1U;
            uint8_t harmonic;

            /* A non-sinusoidal reference can have a harmonic folded onto the
             * fundamental alias.  In that degenerate case the IQ phase moves
             * with the DMA block boundary instead of representing the true
             * fundamental phase.  Prefer the valid sample rate with maximum
             * separation from every relevant low-order harmonic. */
            if (waveform == PLL_TEST_WAVE_SAWTOOTH) {
                first_harmonic = 2U;
            } else if ((waveform == PLL_TEST_WAVE_SQUARE) ||
                       (waveform == PLL_TEST_WAVE_TRIANGLE)) {
                first_harmonic = 3U;
                harmonic_step = 2U;
            }
            for (harmonic = first_harmonic;
                 (first_harmonic != 0U) &&
                 (harmonic <= PLL_TEST_LOCK_HARMONIC_MAX);
                 harmonic = (uint8_t)(harmonic + harmonic_step)) {
                float harmonic_alias =
                    FoldAliasHz((float)harmonic * carrier_hz, rate);
                margin = fminf(margin, fabsf(harmonic_alias - alias));
            }
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
    uint8_t resolved = (s_acquisition_hint_hz > 0.0f) ?
        ResolveHintFrequency(&resolved_hz, &rms_hz) :
        ResolveSweepFrequency(&resolved_hz, &rms_hz);
    if (resolved != 0U) {
        PllTest_Waveform_t target_waveform;
        uint8_t lock_rate_index;
        float lock_rate_hz;
        s_detected_waveform = ClassifySweepWaveform(resolved_hz);
        target_waveform = (s_requested_waveform == PLL_TEST_WAVE_AUTO) ?
            s_detected_waveform : s_requested_waveform;
        if (target_waveform == PLL_TEST_WAVE_UNKNOWN) {
            target_waveform = PLL_TEST_WAVE_SINE;
        }
        lock_rate_index = SelectLockRate(resolved_hz, target_waveform);
        lock_rate_hz =
            SampleRateFromDivider(s_rate_dividers[lock_rate_index]);
        if (ApplyOutputWaveform(target_waveform) == 0U) return;
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
                 "a=%lu,%lu,%lu,%lu s_dB10=%ld,%ld,%ld,%ld "
                 "fs_m=%lu,%lu,%lu,%lu "
                 "p2=%lu,%lu,%lu,%lu r2_dB10=%ld,%ld,%ld,%ld "
                 "p3=%lu,%lu,%lu,%lu r3_dB10=%ld,%ld,%ld,%ld "
                 "c1_dB10=%ld,%ld,%ld,%ld c2_dB10=%ld,%ld,%ld,%ld "
                 "c3_dB10=%ld,%ld,%ld,%ld\r\n",
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
                 (long)(s_alias_snr_db[3] * 10.0f + 0.5f),
                 (unsigned long)(s_measured_sample_rate_hz[0] + 0.5f),
                 (unsigned long)(s_measured_sample_rate_hz[1] + 0.5f),
                 (unsigned long)(s_measured_sample_rate_hz[2] + 0.5f),
                 (unsigned long)(s_measured_sample_rate_hz[3] + 0.5f),
                 (unsigned long)(s_alias_peak2_hz[0] + 0.5f),
                 (unsigned long)(s_alias_peak2_hz[1] + 0.5f),
                 (unsigned long)(s_alias_peak2_hz[2] + 0.5f),
                 (unsigned long)(s_alias_peak2_hz[3] + 0.5f),
                 (long)(s_alias_peak2_rel_db[0] * 10.0f - 0.5f),
                 (long)(s_alias_peak2_rel_db[1] * 10.0f - 0.5f),
                 (long)(s_alias_peak2_rel_db[2] * 10.0f - 0.5f),
                 (long)(s_alias_peak2_rel_db[3] * 10.0f - 0.5f),
                 (unsigned long)(s_alias_peak3_hz[0] + 0.5f),
                 (unsigned long)(s_alias_peak3_hz[1] + 0.5f),
                 (unsigned long)(s_alias_peak3_hz[2] + 0.5f),
                 (unsigned long)(s_alias_peak3_hz[3] + 0.5f),
                 (long)(s_alias_peak3_rel_db[0] * 10.0f - 0.5f),
                 (long)(s_alias_peak3_rel_db[1] * 10.0f - 0.5f),
                 (long)(s_alias_peak3_rel_db[2] * 10.0f - 0.5f),
                 (long)(s_alias_peak3_rel_db[3] * 10.0f - 0.5f),
                 (long)(s_alias_peak1_ref_fb_db[0] * 10.0f +
                        ((s_alias_peak1_ref_fb_db[0] >= 0.0f) ? 0.5f : -0.5f)),
                 (long)(s_alias_peak1_ref_fb_db[1] * 10.0f +
                        ((s_alias_peak1_ref_fb_db[1] >= 0.0f) ? 0.5f : -0.5f)),
                 (long)(s_alias_peak1_ref_fb_db[2] * 10.0f +
                        ((s_alias_peak1_ref_fb_db[2] >= 0.0f) ? 0.5f : -0.5f)),
                 (long)(s_alias_peak1_ref_fb_db[3] * 10.0f +
                        ((s_alias_peak1_ref_fb_db[3] >= 0.0f) ? 0.5f : -0.5f)),
                 (long)(s_alias_peak2_ref_fb_db[0] * 10.0f +
                        ((s_alias_peak2_ref_fb_db[0] >= 0.0f) ? 0.5f : -0.5f)),
                 (long)(s_alias_peak2_ref_fb_db[1] * 10.0f +
                        ((s_alias_peak2_ref_fb_db[1] >= 0.0f) ? 0.5f : -0.5f)),
                 (long)(s_alias_peak2_ref_fb_db[2] * 10.0f +
                        ((s_alias_peak2_ref_fb_db[2] >= 0.0f) ? 0.5f : -0.5f)),
                 (long)(s_alias_peak2_ref_fb_db[3] * 10.0f +
                        ((s_alias_peak2_ref_fb_db[3] >= 0.0f) ? 0.5f : -0.5f)),
                 (long)(s_alias_peak3_ref_fb_db[0] * 10.0f +
                        ((s_alias_peak3_ref_fb_db[0] >= 0.0f) ? 0.5f : -0.5f)),
                 (long)(s_alias_peak3_ref_fb_db[1] * 10.0f +
                        ((s_alias_peak3_ref_fb_db[1] >= 0.0f) ? 0.5f : -0.5f)),
                 (long)(s_alias_peak3_ref_fb_db[2] * 10.0f +
                        ((s_alias_peak3_ref_fb_db[2] >= 0.0f) ? 0.5f : -0.5f)),
                 (long)(s_alias_peak3_ref_fb_db[3] * 10.0f +
                        ((s_alias_peak3_ref_fb_db[3] >= 0.0f) ? 0.5f : -0.5f)));
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
        /* The resolver no longer needs DMA data. Defer it until the current
         * frame has been released, giving it the full two-half latency budget. */
        s_finish_sweep_pending = 1U;
    }
}
static void ProcessSweepBlock(const ADC_DualCapture_Pair_t *pairs,
                              float reference_mean,
                              float feedback_mean,
                              uint32_t reference_p2p,
                              uint16_t divider)
{
    float sample_rate_hz;
    float alias_hz;
    float phase_rad;
    float snr_db;
    float peak2_hz;
    float peak3_hz;
    float peak2_rel_db;
    float peak3_rel_db;
    float peak1_ref_fb_db;
    float peak2_ref_fb_db;
    float peak3_ref_fb_db;
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
    if (EstimateAliasFft(pairs, reference_mean, feedback_mean, sample_rate_hz,
                         &alias_hz, &peak_bin, &phase_rad, &snr_db,
                         &peak2_hz, &peak3_hz,
                         &peak2_rel_db, &peak3_rel_db,
                         &peak1_ref_fb_db, &peak2_ref_fb_db,
                         &peak3_ref_fb_db) == 0U) {
        s_sweep_block_count = 0U;
        if (s_sweep_attempt_count >= 4U) AdvanceSweepRate();
        return;
    }

    s_alias_peak2_hz[s_sweep_rate_index] = peak2_hz;
    s_alias_peak3_hz[s_sweep_rate_index] = peak3_hz;
    s_alias_peak2_rel_db[s_sweep_rate_index] = peak2_rel_db;
    s_alias_peak3_rel_db[s_sweep_rate_index] = peak3_rel_db;
    s_alias_peak1_ref_fb_db[s_sweep_rate_index] = peak1_ref_fb_db;
    s_alias_peak2_ref_fb_db[s_sweep_rate_index] = peak2_ref_fb_db;
    s_alias_peak3_ref_fb_db[s_sweep_rate_index] = peak3_ref_fb_db;
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

static void ProcessAdcFrame(const ADC_DualCapture_Frame_t *frame)
{
    const ADC_DualCapture_Pair_t *pairs;
    uint32_t i;
    uint32_t now = HAL_GetTick();
    uint32_t ref_min = 65535U;
    uint32_t ref_max = 0U;
    uint64_t ref_sum = 0U;
    uint64_t fb_sum = 0U;
    uint32_t ref_p2p;
    float reference_mean;
    float feedback_mean;
    uint16_t divider;
    float sample_rate_hz;
    float measured_sample_rate_hz;
    SoftPll_PhaseInput_t phase_input;
    float reference_phase;
    float feedback_phase;
    float reference_amplitude;
    float feedback_amplitude;
    uint8_t phase_valid;

    if ((frame == NULL) ||
        (frame->pair_count != PLL_TEST_ADC_HALF_SAMPLES) ||
        (frame->half_index > 1U)) return;
    pairs = frame->pairs;
    divider = frame->timer_divider;
    sample_rate_hz = frame->configured_sample_rate_hz;
    measured_sample_rate_hz = frame->measured_sample_rate_hz;
    for (i = 0U; i < PLL_TEST_ADC_HALF_SAMPLES; ++i) {
        uint32_t packed = pairs[i];
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
    s_last_measured_sample_rate_hz = measured_sample_rate_hz;
    s_last_block_sample_rate_hz = sample_rate_hz;

    if ((ref_p2p >= PLL_TEST_ACTIVITY_P2P_CODE) ||
        (fabsf(reference_mean - s_last_ref_mean) >=
         (0.5f * (float)PLL_TEST_ACTIVITY_P2P_CODE))) {
        s_last_ac_tick = now;
    }

    if (s_sweep_active != 0U) {
        if ((s_sweep_rate_index < PLL_TEST_RATE_COUNT) &&
            (divider == s_rate_dividers[s_sweep_rate_index])) {
            s_measured_sample_rate_hz[s_sweep_rate_index] =
                measured_sample_rate_hz;
        }
        ProcessSweepBlock(pairs, reference_mean, feedback_mean,
                          ref_p2p, divider);
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
        /* Keep the last FPGA/AD9708 command continuous on reference loss. */
        s_mode = PLL_TEST_MODE_HOLDOVER;
        s_pll.locked = 0U;
        s_last_ref_mean = reference_mean;
        s_last_fb_mean = feedback_mean;
        return;
    }

    phase_valid = EstimateIqPhases(pairs, reference_mean, feedback_mean,
                                   s_ad9708_frequency_hz, sample_rate_hz,
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
            if (ApplyStableRateAssist() == 0U) {
                (void)ApplyPllOutput(s_pll.dds_freq_cmd_hz,
                                     s_pll.phase_step_deg);
            }
        }
    } else if (s_pll.ready != 0U) {
        s_pll.feedback_valid = 0U;
        s_pll.locked = 0U;
        s_rate_assist_sum_hz = 0.0f;
        s_rate_assist_stable_count = 0U;
    }

    s_last_alias_hz = FoldAliasHz(s_ad9708_frequency_hz, sample_rate_hz);
    if (SoftPll_IsLocked(&s_pll) != 0U) {
        s_last_locked_tick = now;
        s_frequency_estimate_hz +=
            0.02f * (s_pll.dds_freq_cmd_hz - s_frequency_estimate_hz);
    } else if (((now - s_lock_start_tick) >= PLL_TEST_REACQUIRE_MS) &&
               ((now - s_last_locked_tick) >= PLL_TEST_REACQUIRE_MS)) {
        BeginFrequencySweep();
    }

#if PLL_TEST_EXPERIMENTAL_AMFM_ENABLE
    UpdateModulationPhaseLock(pairs, reference_mean, feedback_mean,
                              sample_rate_hz, now);
#endif
    MonitorReferenceWaveform(pairs, reference_mean, sample_rate_hz, now);

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
    ADC_DualCapture_Status_t adc_status;
    if ((now - s_last_telemetry_tick) < s_telemetry_period_ms) return;
    s_last_telemetry_tick = now;
    ADC_DualCapture_GetStatus(&adc_status);

    UartSend("[PLL] mode=%s lock=%u fin_Hz=%lu fout_Hz=%lu "
             "alias_Hz=%lu fs_Hz=%lu fs_m_Hz=%lu scan=%u mask=0x%02X "
             "err_mdeg=%ld drift_mHz=%ld snr_dB10=%ld pv=%u "
             "ref_u=%ld fb_u=%ld ref_mV=%ld fb_mV=%ld fll=%lu "
              "adc_sync=%u adc_ovr=%lu adc_err=0x%02lX out_err=%lu "
              "out_us=%lu amp=%u wave=%s detect=%s req=%s "
              "h2_dB10=%ld h3_dB10=%ld mod_Hz=%lu am_milli=%ld "
              "fmdev_Hz=%lu mod_metric_milli=%ld moderr_mdeg=%ld "
              "modpv=%u modlc=%u\r\n",
              ModeName(), (unsigned int)ApplicationLocked(),
             (unsigned long)(fmaxf(0.0f, s_frequency_estimate_hz) + 0.5f),
             (unsigned long)(fmaxf(0.0f, s_ad9708_frequency_hz) + 0.5f),
             (unsigned long)(fmaxf(0.0f, s_last_alias_hz) + 0.5f),
             (unsigned long)(fmaxf(0.0f, s_last_block_sample_rate_hz) + 0.5f),
             (unsigned long)(fmaxf(0.0f,
                                   s_last_measured_sample_rate_hz) + 0.5f),
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
             (unsigned long)s_rate_assist_apply_count,
             (unsigned int)adc_status.hardware_simultaneous,
             (unsigned long)adc_status.overrun_count,
             (unsigned long)adc_status.error_flags,
             (unsigned long)s_ad9708_error_count,
             (unsigned long)s_ad9708_last_update_us,
             (unsigned int)s_ad9708_amplitude,
             PllTest_WaveformName(s_output_waveform),
             PllTest_WaveformName(s_detected_waveform),
              PllTest_WaveformName(s_requested_waveform),
              (long)ScaleToInt(s_harmonic2_relative_db, 10.0f),
              (long)ScaleToInt(s_harmonic3_relative_db, 10.0f),
              (unsigned long)(s_modulation_estimate.modulation_hz + 0.5f),
              (long)ScaleToInt(s_modulation_estimate.am_depth, 1000.0f),
              (unsigned long)(s_modulation_estimate.fm_deviation_hz + 0.5f),
              (long)ScaleToInt(s_modulation_estimate.phase_metric, 1000.0f),
              (long)ScaleToInt(s_modulation_phase_error_deg, 1000.0f),
              (unsigned int)s_modulation_phase_valid,
              (unsigned int)s_modulation_lock_count);
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
static uint8_t IsFiniteNonnegative(float value)
{
    return (isfinite(value) && (value >= 0.0f)) ? 1U : 0U;
}

void PllTest_GetDefaultConfig(PllTest_Config_t *config)
{
    if (config == NULL) return;
    config->target_phase_deg = PLL_TEST_TARGET_PHASE_DEG;
    config->fixed_phase_cal_deg = PLL_TEST_FIXED_PHASE_CAL_DEG;
    config->loop_bandwidth_hz = PLL_TEST_DEFAULT_LOOP_BW_HZ;
    config->acquisition_hint_hz = 0.0f;
    config->acquisition_hint_tolerance_hz = PLL_TEST_HINT_DEFAULT_TOL_HZ;
    config->telemetry_period_ms = PLL_TEST_TELEMETRY_PERIOD_MS;
    config->requested_waveform = PLL_TEST_WAVEFORM_DEFAULT;
}

HAL_StatusTypeDef PllTest_SetTargetPhase(float target_phase_deg)
{
    if (!isfinite(target_phase_deg)) return HAL_ERROR;
    s_target_phase_deg = SoftPll_Normalize180Deg(target_phase_deg);
    return HAL_OK;
}

HAL_StatusTypeDef PllTest_SetFixedPhaseCalibration(float calibration_deg)
{
    if (!isfinite(calibration_deg)) return HAL_ERROR;
    s_fixed_phase_cal_deg = SoftPll_Normalize180Deg(calibration_deg);
    return HAL_OK;
}

HAL_StatusTypeDef PllTest_SetLoopBandwidth(float loop_bandwidth_hz)
{
    float update_rate_hz = s_last_block_sample_rate_hz /
                           (float)PLL_TEST_ADC_HALF_SAMPLES;
    if ((!isfinite(loop_bandwidth_hz)) || (loop_bandwidth_hz < 0.01f) ||
        (update_rate_hz <= 0.0f) ||
        (loop_bandwidth_hz >= 0.20f * update_rate_hz)) return HAL_ERROR;
    s_loop_bandwidth_hz = loop_bandwidth_hz;
    if (s_pll.ready != 0U) {
        if (SoftPll_DesignType2(&s_pll.cfg, loop_bandwidth_hz,
                                PLL_TEST_DEFAULT_DAMPING,
                                update_rate_hz) == 0U) return HAL_ERROR;
    }
    return HAL_OK;
}

HAL_StatusTypeDef PllTest_SetLoopGains(float frequency_kp_hz_per_deg,
                                       float frequency_ki_hz_per_deg_s,
                                       float frequency_kd,
                                       float phase_kp)
{
    if ((IsFiniteNonnegative(frequency_kp_hz_per_deg) == 0U) ||
        (IsFiniteNonnegative(frequency_ki_hz_per_deg_s) == 0U) ||
        (IsFiniteNonnegative(frequency_kd) == 0U) ||
        (IsFiniteNonnegative(phase_kp) == 0U)) return HAL_ERROR;
    s_pll.cfg.freq_kp_hz_per_deg = frequency_kp_hz_per_deg;
    s_pll.cfg.freq_ki_hz_per_deg_s = frequency_ki_hz_per_deg_s;
    s_pll.cfg.freq_kd = frequency_kd;
    s_pll.cfg.phase_kp = phase_kp;
    return HAL_OK;
}

HAL_StatusTypeDef PllTest_SetTelemetryPeriod(uint32_t period_ms)
{
    if ((period_ms < 20U) || (period_ms > 10000U)) return HAL_ERROR;
    s_telemetry_period_ms = period_ms;
    return HAL_OK;
}

HAL_StatusTypeDef PllTest_SetAcquisitionHint(float frequency_hz,
                                              float tolerance_hz)
{
    if ((!isfinite(frequency_hz)) || (!isfinite(tolerance_hz)) ||
        (frequency_hz < 0.0f) ||
        (frequency_hz > (float)PLL_TEST_MAX_OUTPUT_HZ) ||
        (tolerance_hz < 100.0f) ||
        (tolerance_hz > PLL_TEST_HINT_MAX_TOL_HZ)) return HAL_ERROR;
    s_acquisition_hint_hz = frequency_hz;
    s_acquisition_hint_tolerance_hz = tolerance_hz;
    return HAL_OK;
}

HAL_StatusTypeDef PllTest_SetWaveform(PllTest_Waveform_t waveform)
{
    if (waveform > PLL_TEST_WAVE_FM) return HAL_ERROR;
#if !PLL_TEST_EXPERIMENTAL_AMFM_ENABLE
    if ((waveform == PLL_TEST_WAVE_AM) ||
        (waveform == PLL_TEST_WAVE_FM)) return HAL_ERROR;
#endif
    s_requested_waveform = waveform;
    s_wave_monitor_candidate = PLL_TEST_WAVE_UNKNOWN;
    s_wave_monitor_confirm_count = 0U;
    if (s_running == 0U) return HAL_OK;
    if (waveform == PLL_TEST_WAVE_AUTO) {
        BeginFrequencySweep();
        return HAL_OK;
    }
    return (ApplyOutputWaveform(waveform) != 0U) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef PllTest_RequestRescan(void)
{
    if (s_running == 0U) return HAL_ERROR;
    BeginFrequencySweep();
    return HAL_OK;
}

void PllTest_GetStatus(PllTest_Status_t *status)
{
    uint32_t primask;
    if (status == NULL) return;
    memset(status, 0, sizeof(*status));
    primask = __get_PRIMASK();
    __disable_irq();
    status->mode = s_mode;
    status->running = s_running;
    status->locked = ApplicationLocked();
    status->phase_valid = s_last_phase_valid;
    status->frequency_sweep_active = s_sweep_active;
    status->input_frequency_hz = s_frequency_estimate_hz;
    status->output_frequency_hz = s_ad9708_frequency_hz;
    status->phase_error_deg = s_pll.error_deg;
    status->phase_rate_hz = s_pll.phase_rate_hz;
    status->target_phase_deg = s_target_phase_deg;
    status->fixed_phase_cal_deg = s_fixed_phase_cal_deg;
    status->loop_bandwidth_hz = s_loop_bandwidth_hz;
    status->acquisition_hint_hz = s_acquisition_hint_hz;
    status->acquisition_hint_tolerance_hz =
        s_acquisition_hint_tolerance_hz;
    status->alias_frequency_hz = s_last_alias_hz;
    status->alias_snr_db = s_last_alias_snr_db;
    status->harmonic2_relative_db = s_harmonic2_relative_db;
    status->harmonic3_relative_db = s_harmonic3_relative_db;
    status->modulation_frequency_hz =
        s_modulation_estimate.modulation_hz;
    status->am_depth = s_modulation_estimate.am_depth;
    status->fm_deviation_hz =
        s_modulation_estimate.fm_deviation_hz;
    status->modulation_phase_metric =
        s_modulation_estimate.phase_metric;
    status->modulation_phase_error_deg =
        s_modulation_phase_error_deg;
    status->modulation_phase_valid =
        s_modulation_phase_valid;
    status->reference_iq_amplitude = s_last_reference_iq_amp;
    status->feedback_iq_amplitude = s_last_feedback_iq_amp;
    status->requested_waveform = s_requested_waveform;
    status->detected_waveform = s_detected_waveform;
    status->output_waveform = s_output_waveform;
    status->dds_update_count = s_ad9708_update_count;
    status->dds_error_count = s_ad9708_error_count;
    status->dds_last_update_us = s_ad9708_last_update_us;
    status->sweep_failure_count = s_sweep_fail_count;
    status->rate_assist_count = s_rate_assist_apply_count;
    status->dds_amplitude_word = s_ad9708_amplitude;
    if ((primask & 1U) == 0U) __enable_irq();
    ADC_DualCapture_GetStatus(&status->adc);
}

static uint8_t ParseWaveformCommand(const char *name,
                                    PllTest_Waveform_t *waveform)
{
    if ((name == NULL) || (waveform == NULL)) return 0U;
    if (strcmp(name, "auto") == 0) *waveform = PLL_TEST_WAVE_AUTO;
    else if (strcmp(name, "sine") == 0) *waveform = PLL_TEST_WAVE_SINE;
    else if (strcmp(name, "square") == 0) *waveform = PLL_TEST_WAVE_SQUARE;
    else if (strcmp(name, "triangle") == 0) {
        *waveform = PLL_TEST_WAVE_TRIANGLE;
    } else if ((strcmp(name, "saw") == 0) ||
               (strcmp(name, "sawtooth") == 0)) {
        *waveform = PLL_TEST_WAVE_SAWTOOTH;
    } else if (strcmp(name, "am") == 0) *waveform = PLL_TEST_WAVE_AM;
    else if (strcmp(name, "fm") == 0) *waveform = PLL_TEST_WAVE_FM;
    else return 0U;
    return 1U;
}

static void ProcessCommand(char *line)
{
    float value;
    PllTest_Waveform_t waveform;

    if ((strcmp(line, "help") == 0) || (strcmp(line, "?") == 0)) {
        UartSend("[CMD] kp <Hz/deg>, ki <Hz/(deg*s)>, kd <ratio>, "
                 "pkp <deg/deg>, bw <Hz>, phase <deg>, cal <deg>, "
                 "telem <ms>, status, scan, spec <0..3>, hint <Hz|0>, "
                 "wave <auto|sine|square|triangle|saw|am|fm>\r\n");
    } else if ((strncmp(line, "wave ", 5U) == 0) &&
               (ParseWaveformCommand(line + 5U, &waveform) != 0U) &&
               (PllTest_SetWaveform(waveform) == HAL_OK)) {
        s_last_telemetry_tick = HAL_GetTick() - s_telemetry_period_ms;
    } else if (strcmp(line, "status") == 0) {
        s_last_telemetry_tick = HAL_GetTick() - s_telemetry_period_ms;
        SendTelemetry();
    } else if ((strcmp(line, "scan") == 0) &&
               (PllTest_RequestRescan() == HAL_OK)) {
        UartSend("[CMD] RF rescan started\r\n");
    } else if ((strncmp(line, "spec ", 5U) == 0) &&
               ParseValue(line, &value) && (value >= 0.0f) &&
               (value < (float)PLL_TEST_RATE_COUNT)) {
        PrintSpectrum((uint8_t)value);
    } else if ((strncmp(line, "hint ", 5U) == 0) &&
               ParseValue(line, &value) &&
               (PllTest_SetAcquisitionHint(value,
                   PLL_TEST_HINT_DEFAULT_TOL_HZ) == HAL_OK)) {
        BeginFrequencySweep();
        UartSend("[CMD] hint_Hz=%lu tol_Hz=%lu rescan started\r\n",
                 (unsigned long)(s_acquisition_hint_hz + 0.5f),
                 (unsigned long)(s_acquisition_hint_tolerance_hz + 0.5f));
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
               (PllTest_SetLoopBandwidth(value) == HAL_OK)) {
        UartSend("[CMD] bw_mHz=%ld kp_u=%ld ki_u=%ld\r\n",
                 (long)ScaleToInt(value, 1000.0f),
                 (long)ScaleToInt(s_pll.cfg.freq_kp_hz_per_deg, 1000000.0f),
                 (long)ScaleToInt(s_pll.cfg.freq_ki_hz_per_deg_s, 1000000.0f));
    } else if ((strncmp(line, "phase ", 6U) == 0) && ParseValue(line, &value) &&
               (PllTest_SetTargetPhase(value) == HAL_OK)) {
        UartSend("[CMD] target_mdeg=%ld\r\n",
                 (long)ScaleToInt(s_target_phase_deg, 1000.0f));
    } else if ((strncmp(line, "cal ", 4U) == 0) && ParseValue(line, &value) &&
               (PllTest_SetFixedPhaseCalibration(value) == HAL_OK)) {
        UartSend("[CMD] cal_mdeg=%ld\r\n",
                 (long)ScaleToInt(s_fixed_phase_cal_deg, 1000.0f));
    } else if ((strncmp(line, "telem ", 6U) == 0) && ParseValue(line, &value) &&
               (PllTest_SetTelemetryPeriod((uint32_t)(value + 0.5f)) == HAL_OK)) {
        UartSend("[CMD] telem_ms=%lu\r\n",
                 (unsigned long)s_telemetry_period_ms);
    } else {
        UartSend("[CMD] ERROR unknown/invalid: %s\r\n", line);
    }
}

HAL_StatusTypeDef PllTest_InitWithConfig(const PllTest_Config_t *config)
{
    HAL_StatusTypeDef status;
    AD9708_StatusTypeDef output_status;
    ADC_DualCapture_Config_t adc_config;
    ADC_DualCapture_Status_t adc_status;
    PllTest_Config_t effective_config;
    const AD9708_DataTypeDef *output_data;
    float initial_update_rate_hz;

    PllTest_GetDefaultConfig(&effective_config);
    if (config != NULL) effective_config = *config;
    initial_update_rate_hz = (float)PLL_TEST_SAMPLE_RATE_HZ /
                             (float)PLL_TEST_ADC_HALF_SAMPLES;
    if ((!isfinite(effective_config.target_phase_deg)) ||
        (!isfinite(effective_config.fixed_phase_cal_deg)) ||
        (!isfinite(effective_config.loop_bandwidth_hz)) ||
        (!isfinite(effective_config.acquisition_hint_hz)) ||
        (!isfinite(effective_config.acquisition_hint_tolerance_hz)) ||
        (effective_config.acquisition_hint_hz < 0.0f) ||
        (effective_config.acquisition_hint_hz >
         (float)PLL_TEST_MAX_OUTPUT_HZ) ||
        (effective_config.acquisition_hint_tolerance_hz < 100.0f) ||
        (effective_config.acquisition_hint_tolerance_hz >
         PLL_TEST_HINT_MAX_TOL_HZ) ||
        (effective_config.requested_waveform > PLL_TEST_WAVE_FM) ||
        (effective_config.loop_bandwidth_hz < 0.01f) ||
        (effective_config.loop_bandwidth_hz >= 0.20f * initial_update_rate_hz) ||
        (effective_config.telemetry_period_ms < 20U) ||
        (effective_config.telemetry_period_ms > 10000U)) return HAL_ERROR;
#if !PLL_TEST_EXPERIMENTAL_AMFM_ENABLE
    if ((effective_config.requested_waveform == PLL_TEST_WAVE_AM) ||
        (effective_config.requested_waveform == PLL_TEST_WAVE_FM)) {
        return HAL_ERROR;
    }
#endif

    memset((void *)s_adc_dma, 0, sizeof(s_adc_dma));
    memset(&s_pll, 0, sizeof(s_pll));
    memset(&s_ref_cross, 0, sizeof(s_ref_cross));
    memset(&s_fb_cross, 0, sizeof(s_fb_cross));
    s_ref_cross.centre = 32768.0f;
    s_fb_cross.centre = 32768.0f;
    s_running = 0U;
    s_target_phase_deg =
        SoftPll_Normalize180Deg(effective_config.target_phase_deg);
    s_fixed_phase_cal_deg =
        SoftPll_Normalize180Deg(effective_config.fixed_phase_cal_deg);
    s_loop_bandwidth_hz = effective_config.loop_bandwidth_hz;
    s_acquisition_hint_hz = effective_config.acquisition_hint_hz;
    s_acquisition_hint_tolerance_hz =
        effective_config.acquisition_hint_tolerance_hz;
    s_requested_waveform = effective_config.requested_waveform;
    s_detected_waveform = PLL_TEST_WAVE_UNKNOWN;
    s_output_waveform = PLL_TEST_WAVE_SINE;
    s_wave_monitor_candidate = PLL_TEST_WAVE_UNKNOWN;
    s_harmonic2_relative_db = -120.0f;
    s_harmonic3_relative_db = -120.0f;
    s_wave_monitor_confirm_count = 0U;
    s_last_wave_monitor_tick = HAL_GetTick();
    memset(&s_modulation_estimate, 0, sizeof(s_modulation_estimate));
    s_last_modulation_phase_tick = HAL_GetTick();
    s_modulation_phase_error_deg = 0.0f;
    s_modulation_phase_valid = 0U;
    s_modulation_lock_count = 0U;
    s_telemetry_period_ms = effective_config.telemetry_period_ms;
    s_ad9708_phase_deg = 0.0;
    s_ad9708_phase_word = 0U;
    s_ad9708_frequency_hz = PLL_TEST_DEFAULT_START_FREQ_HZ;
    s_ad9708_update_count = 0U;
    s_ad9708_error_count = 0U;
    s_ad9708_consecutive_errors = 0U;
    s_ad9708_recovery_pending = 0U;
    s_ad9708_recovery_requested = 0U;
    s_ad9708_last_update_us = 0U;
    s_sample_counter = 0U;
    s_ref_period_sum = 0.0;
    s_ref_period_count = 0U;
    s_frequency_estimate_hz = 0.0f;
    s_frequency_event = 0U;
    s_frequency_confirm_count = 0U;
    s_mode = PLL_TEST_MODE_ACQUIRE;
    s_last_ac_tick = HAL_GetTick();
    s_last_telemetry_tick = HAL_GetTick();
    s_lock_sample_divider = s_rate_dividers[0];
    s_last_block_sample_rate_hz = SampleRateFromDivider(s_rate_dividers[0]);
    s_last_alias_hz = 0.0f;
    s_last_alias_snr_db = 0.0f;
    s_last_reference_iq_amp = 0.0f;
    s_last_feedback_iq_amp = 0.0f;
    s_last_measured_sample_rate_hz = 0.0f;
    s_last_phase_valid = 0U;
    s_sweep_fail_count = 0U;
    s_rate_assist_apply_count = 0U;
    s_rate_assist_sum_hz = 0.0f;
    s_rate_assist_last_hz = 0.0f;
    s_rate_assist_stable_count = 0U;
    s_rate_assist_applied = 0U;
    s_lock_start_tick = HAL_GetTick();
    s_last_locked_tick = s_lock_start_tick;
    InitFftWindow();

    /* ADC1/ADC2, CDR packing, cache and ping-pong ownership live entirely in
     * the capture driver. The PLL consumes immutable paired frames only. */
    ADC_DualCapture_GetDefaultConfig(&adc_config);
    adc_config.pair_buffer = s_adc_dma;
    adc_config.pairs_per_half = PLL_TEST_ADC_HALF_SAMPLES;
    adc_config.sample_rate_hz = SampleRateFromDivider(s_rate_dividers[0]);
    adc_config.maximum_rate_error_ppm = 1000.0f;
    status = ADC_DualCapture_Init(&adc_config);
    if (status != HAL_OK) return status;
    BeginFrequencySweep();

    if (HAL_UART_Receive_IT(&huart1, &s_uart_rx_byte, 1U) != HAL_OK) {
        return HAL_ERROR;
    }

    /*
     * QSPI restores FPGA logic only. Every MCU boot reloads the sine table and
     * AD9708 runtime registers, then the PLL changes only frequency/phase.
     */
    output_status = ConfigureAd9708Output();
    if (output_status != AD9708_OK) return HAL_ERROR;
    output_data = AD9708_GetData();
    status = ADC_DualCapture_Start();
    if (status != HAL_OK) return status;
    s_running = 1U;
    ADC_DualCapture_GetStatus(&adc_status);

    UartSend("[AD9708] READY id=0x%08lX version=0x%08lX dac_Hz=%lu "
             "wave=sine points=%u amp=%u offset=%u\r\n",
             (unsigned long)output_data->device_id,
             (unsigned long)output_data->firmware_version,
             (unsigned long)AD9708_DAC_CLK_HZ,
             (unsigned int)PLL_TEST_AD9708_WAVE_POINTS,
             (unsigned int)s_ad9708_amplitude,
             (unsigned int)s_ad9708_offset);
    UartSend("\r\n[PLL_TEST] READY PC4=ADC1_INP4 PC5=ADC2_INP8 "
             "OUT=FPGA_AD9708 auto=0..%luHz pll_max=%luHz multirate=%u "
             "UART1=115200 8N1\r\n",
             (unsigned long)PLL_TEST_AUTO_SEARCH_MAX_HZ,
             (unsigned long)PLL_TEST_MAX_OUTPUT_HZ,
             (unsigned int)PLL_TEST_RATE_COUNT);
    UartSend("[ADC_DUAL] sync=%u pairs=%lu fs_req_Hz=%lu fs_set_Hz=%lu "
             "fs_max_Hz=%lu tim_Hz=%lu adc_Hz=%lu div=%u\r\n",
             (unsigned int)adc_status.hardware_simultaneous,
             (unsigned long)PLL_TEST_ADC_HALF_SAMPLES,
             (unsigned long)(adc_status.requested_sample_rate_hz + 0.5f),
             (unsigned long)(adc_status.configured_sample_rate_hz + 0.5f),
             (unsigned long)(adc_status.maximum_sample_rate_hz + 0.5f),
             (unsigned long)adc_status.timer_clock_hz,
             (unsigned long)adc_status.adc_clock_hz,
             (unsigned int)adc_status.active_timer_divider);
    return HAL_OK;
}

HAL_StatusTypeDef PllTest_Init(void)
{
    return PllTest_InitWithConfig(NULL);
}

void PllTest_Stop(void)
{
    s_running = 0U;
    ADC_DualCapture_Stop();
    /* Deliberately leave FPGA/AD9708 running continuously at the last command. */
    s_mode = PLL_TEST_MODE_HOLDOVER;
}

void PllTest_Process(void)
{
    ADC_DualCapture_Frame_t frame;
    uint8_t frame_ready;
    char command[PLL_TEST_UART_LINE_SIZE];
    uint32_t primask;

    if (s_ad9708_recovery_pending != 0U) {
        (void)ServiceAd9708Recovery();
        SendTelemetry();
        return;
    }

    frame_ready = ADC_DualCapture_AcquireFrame(&frame);
    primask = __get_PRIMASK();
    __disable_irq();
    if (s_uart_command_ready != 0U) {
        memcpy(command, s_uart_command, sizeof(command));
        command[sizeof(command) - 1U] = '\0';
        s_uart_command_ready = 0U;
    } else {
        command[0] = '\0';
    }
    if ((primask & 1U) == 0U) __enable_irq();

    if (frame_ready != 0U) {
        ProcessAdcFrame(&frame);
        ADC_DualCapture_ReleaseFrame(&frame);
    }
    if (s_finish_sweep_pending != 0U) {
        s_finish_sweep_pending = 0U;
        FinishFrequencySweep();
    }
    if (command[0] != '\0') ProcessCommand(command);
    if (s_ad9708_recovery_pending != 0U) BeginAd9708Recovery();
    SendTelemetry();
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
void PllTest_GetDefaultConfig(PllTest_Config_t *config)
{
    if (config != NULL) memset(config, 0, sizeof(*config));
}
HAL_StatusTypeDef PllTest_InitWithConfig(const PllTest_Config_t *config)
{
    (void)config;
    return HAL_ERROR;
}
HAL_StatusTypeDef PllTest_Init(void) { return HAL_ERROR; }
void PllTest_Process(void) { }
void PllTest_Stop(void) { }
HAL_StatusTypeDef PllTest_SetTargetPhase(float value)
{
    (void)value;
    return HAL_ERROR;
}
HAL_StatusTypeDef PllTest_SetFixedPhaseCalibration(float value)
{
    (void)value;
    return HAL_ERROR;
}
HAL_StatusTypeDef PllTest_SetLoopBandwidth(float value)
{
    (void)value;
    return HAL_ERROR;
}
HAL_StatusTypeDef PllTest_SetLoopGains(float kp, float ki, float kd, float pkp)
{
    (void)kp; (void)ki; (void)kd; (void)pkp;
    return HAL_ERROR;
}
HAL_StatusTypeDef PllTest_SetTelemetryPeriod(uint32_t period_ms)
{
    (void)period_ms;
    return HAL_ERROR;
}
HAL_StatusTypeDef PllTest_SetAcquisitionHint(float frequency_hz,
                                              float tolerance_hz)
{
    (void)frequency_hz; (void)tolerance_hz;
    return HAL_ERROR;
}
HAL_StatusTypeDef PllTest_SetWaveform(PllTest_Waveform_t waveform)
{
    (void)waveform;
    return HAL_ERROR;
}
const char *PllTest_WaveformName(PllTest_Waveform_t waveform)
{
    (void)waveform;
    return "disabled";
}
HAL_StatusTypeDef PllTest_RequestRescan(void) { return HAL_ERROR; }
void PllTest_GetStatus(PllTest_Status_t *status)
{
    if (status != NULL) memset(status, 0, sizeof(*status));
}
#endif /* PLL_SELFTEST_ENABLE */
