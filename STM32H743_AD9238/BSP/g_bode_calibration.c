#include "g_bode_calibration.h"

#include "dual_response_analyzer.h"
#include "signal_adc.h"

#include <math.h>
#include <stdio.h>

#define G_BODE_SAMPLE_COUNT       4096U
#define G_BODE_TIMEOUT_MS         100U
#define G_BODE_MIN_FREQUENCY_HZ   8500.0f
#define G_BODE_MAX_FREQUENCY_HZ   505000.0f
#define G_BODE_OUTPUT_MIN_HZ      9000.0f
#define G_BODE_OUTPUT_MAX_HZ      501000.0f
#define G_BODE_MAX_ERRORS         5U

static float s_ch1_samples[G_BODE_SAMPLE_COUNT];
static float s_ch2_samples[G_BODE_SAMPLE_COUNT];
static uint8_t s_active;
static G_BodeMode s_mode;
static uint32_t s_point_count;
static uint32_t s_consecutive_errors;
static float s_previous_frequency_hz;
static float s_last_output_frequency_hz;
static float s_previous_wrapped_phase_deg;
static float s_unwrapped_phase_deg;
static uint8_t s_phase_started;

const char *G_BodeCalibration_ModeName(G_BodeMode mode)
{
    return (mode == G_BODE_MODE_FILTER) ? "FILTER" : "THRU";
}

void G_BodeCalibration_Init(void)
{
    s_active = 0U;
    s_mode = G_BODE_MODE_THRU;
    s_point_count = 0U;
    s_consecutive_errors = 0U;
    s_previous_frequency_hz = 0.0f;
    s_last_output_frequency_hz = 0.0f;
    s_previous_wrapped_phase_deg = 0.0f;
    s_unwrapped_phase_deg = 0.0f;
    s_phase_started = 0U;
}

static void G_BodeCalibration_PrintHeader(G_BodeMode mode)
{
    printf("[BODE] mode=%s dual_raw_R25=ON N=%u Fs=2000000Hz "
           "band=9000..501000Hz\r\n",
           G_BodeCalibration_ModeName(mode),
           (unsigned)G_BODE_SAMPLE_COUNT);
    printf("BODE_HEADER,mode,frequency_hz,ch1_vpp_mv,ch2_vpp_mv,"
           "gain_linear,gain_db,phase_wrapped_deg,phase_unwrapped_deg,"
           "ch1_residual_pct,ch2_residual_pct,ch1_clip,ch2_clip,quality\r\n");
}

void G_BodeCalibration_Start(G_BodeMode mode)
{
    s_mode = mode;
    s_active = 1U;
    s_point_count = 0U;
    s_consecutive_errors = 0U;
    s_previous_frequency_hz = 0.0f;
    s_last_output_frequency_hz = 0.0f;
    s_previous_wrapped_phase_deg = 0.0f;
    s_unwrapped_phase_deg = 0.0f;
    s_phase_started = 0U;
    G_BodeCalibration_PrintHeader(mode);
    printf("[BODE] START; run a slow ascending sine sweep now\r\n");
}

void G_BodeCalibration_Stop(void)
{
    if (s_active != 0U) {
        printf("[BODE] STOP mode=%s points=%lu\r\n",
               G_BodeCalibration_ModeName(s_mode),
               (unsigned long)s_point_count);
    }
    s_active = 0U;
}

uint8_t G_BodeCalibration_IsActive(void)
{
    return s_active;
}

static float G_BodeCalibration_Unwrap(float frequency_hz,
                                      float wrapped_phase_deg)
{
    float delta;

    if ((s_phase_started == 0U) ||
        (frequency_hz + 200.0f < s_previous_frequency_hz)) {
        s_unwrapped_phase_deg = wrapped_phase_deg;
        s_phase_started = 1U;
    } else {
        delta = wrapped_phase_deg - s_previous_wrapped_phase_deg;
        while (delta >= 180.0f) {
            delta -= 360.0f;
        }
        while (delta < -180.0f) {
            delta += 360.0f;
        }
        s_unwrapped_phase_deg += delta;
    }
    s_previous_wrapped_phase_deg = wrapped_phase_deg;
    s_previous_frequency_hz = frequency_hz;
    return s_unwrapped_phase_deg;
}

static void G_BodeCalibration_AcquireAndPrint(G_BodeMode mode,
                                              uint8_t force_output)
{
    const SignalAdc_Info *adc_info = SignalAdc_GetInfo();
    SignalAdc_FrameInfo ch1_frame;
    SignalAdc_FrameInfo ch2_frame;
    SignalAdc_Status adc_status;
    DualResponse_Result response;
    DualResponse_Status analysis_status;
    float minimum_spacing_hz;
    float unwrapped_phase_deg;
    uint8_t quality;

    if (adc_info == NULL) {
        printf("[BODE] ERROR ADC_INFO\r\n");
        return;
    }
    adc_status = SignalAdc_CaptureDual(s_ch1_samples,
                                      s_ch2_samples,
                                      G_BODE_SAMPLE_COUNT,
                                      G_BODE_SAMPLE_COUNT,
                                      G_BODE_TIMEOUT_MS,
                                      &ch1_frame,
                                      &ch2_frame);
    if (adc_status != SIGNAL_ADC_OK) {
        s_consecutive_errors++;
        printf("[BODE] ERROR capture=%s count=%lu\r\n",
               SignalAdc_StatusName(adc_status),
               (unsigned long)s_consecutive_errors);
        if ((s_active != 0U) &&
            (s_consecutive_errors >= G_BODE_MAX_ERRORS)) {
            printf("[BODE] ABORT too_many_errors\r\n");
            G_BodeCalibration_Stop();
        }
        return;
    }

    analysis_status = DualResponse_Analyze(s_ch1_samples,
                                           s_ch2_samples,
                                           G_BODE_SAMPLE_COUNT,
                                           adc_info->output_sample_rate_hz,
                                           G_BODE_MIN_FREQUENCY_HZ,
                                           G_BODE_MAX_FREQUENCY_HZ,
                                           &response);
    if ((analysis_status != DUAL_RESPONSE_OK) ||
        (response.valid == 0U)) {
        s_consecutive_errors++;
        printf("[BODE] ERROR analysis=%d count=%lu\r\n",
               (int)analysis_status,
               (unsigned long)s_consecutive_errors);
        if ((s_active != 0U) &&
            (s_consecutive_errors >= G_BODE_MAX_ERRORS)) {
            printf("[BODE] ABORT too_many_errors\r\n");
            G_BodeCalibration_Stop();
        }
        return;
    }
    s_consecutive_errors = 0U;

    if ((response.frequency_hz < G_BODE_OUTPUT_MIN_HZ) ||
        (response.frequency_hz > G_BODE_OUTPUT_MAX_HZ)) {
        return;
    }
    /* Track every frame so steep LC phase transitions are unwrapped densely. */
    unwrapped_phase_deg = G_BodeCalibration_Unwrap(
        response.frequency_hz, response.phase_deg);
    minimum_spacing_hz = fmaxf(100.0f, response.frequency_hz * 0.002f);
    if ((force_output == 0U) && (s_last_output_frequency_hz > 0.0f) &&
        (fabsf(response.frequency_hz - s_last_output_frequency_hz) <
         minimum_spacing_hz)) {
        return;
    }

    quality = ((ch1_frame.clipped == 0U) &&
               (ch2_frame.clipped == 0U) &&
               (response.ch1_residual_ratio < 0.20f) &&
               (response.ch2_residual_ratio < 0.20f) &&
               (response.ch2_peak_v > 0.001f)) ? 1U : 0U;
    printf("BODE,%s,%.3f,%.6f,%.6f,%.9f,%.6f,%.6f,%.6f,"
           "%.4f,%.4f,%u,%u,%u\r\n",
           G_BodeCalibration_ModeName(mode),
           (double)response.frequency_hz,
           (double)(2000.0f * response.ch1_peak_v),
           (double)(2000.0f * response.ch2_peak_v),
           (double)response.gain_linear,
           (double)response.gain_db,
           (double)response.phase_deg,
           (double)unwrapped_phase_deg,
           (double)(100.0f * response.ch1_residual_ratio),
           (double)(100.0f * response.ch2_residual_ratio),
           (unsigned)ch1_frame.clipped,
           (unsigned)ch2_frame.clipped,
           (unsigned)quality);
    s_last_output_frequency_hz = response.frequency_hz;
    s_point_count++;
}

void G_BodeCalibration_Process(void)
{
    if (s_active != 0U) {
        G_BodeCalibration_AcquireAndPrint(s_mode, 0U);
    }
}

void G_BodeCalibration_CaptureOnce(G_BodeMode mode)
{
    if (s_active != 0U) {
        printf("[BODE] ERROR stop_continuous_first\r\n");
        return;
    }
    s_mode = mode;
    s_previous_frequency_hz = 0.0f;
    s_last_output_frequency_hz = 0.0f;
    s_phase_started = 0U;
    G_BodeCalibration_PrintHeader(mode);
    G_BodeCalibration_AcquireAndPrint(mode, 1U);
}
