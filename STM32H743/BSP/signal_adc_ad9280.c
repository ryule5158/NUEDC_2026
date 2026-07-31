#include "signal_adc.h"

#include "ad9280_app.h"
#include "signal_adc_ad9280_calibration.h"
#include <math.h>
#include <stddef.h>

/*
 * Current hardware provider: AD9280 -> fixed FPGA CIC3/R16 -> signed Q8.8
 * -> SPI -> STM32.  The adapter publishes the exact CIC transfer function
 * so the analyzer can remove it without an empirical calibration table.
 *
 * These device-specific values intentionally live only in this adapter.
 * The affine code-to-voltage transfer is board-specific DC calibration.
 * It is independent of the disabled empirical frequency-response table.
 */
#define SIGNAL_ADC_AD9280_OUTPUT_RATE_HZ  2000000U
#define SIGNAL_ADC_AD9280_DECIMATION      16U
#define SIGNAL_ADC_AD9280_RESOLUTION_BITS 8U
#define SIGNAL_ADC_AD9280_NOMINAL_PHYSICAL_LSB_V (10.0f / 256.0f)
#define SIGNAL_ADC_AD9280_CLIP_LOW_Q8_8    (-32000)
#define SIGNAL_ADC_AD9280_CLIP_HIGH_Q8_8   31744
#define SIGNAL_ADC_AD9280_RESPONSE_MIN_HZ   0.0f
#define SIGNAL_ADC_AD9280_RESPONSE_MAX_HZ   500000.0f
#define SIGNAL_ADC_AD9280_CIC_STAGES        3U
#define SIGNAL_ADC_AD9280_PI                3.14159265358979323846

static float SignalAdc_CicMagnitude(float frequency_hz);
static float SignalAdc_CicPhaseRad(float frequency_hz);

static const SignalAdc_Info s_info = {
    .name = "AD9280+FPGA-CIC3-R16-S16Q8.8",
    .physical_sample_clock_hz = AD9280_SAMPLE_CLOCK_HZ,
    .output_sample_rate_hz = (float)SIGNAL_ADC_AD9280_OUTPUT_RATE_HZ,
    .max_samples = AD9280_BUFFER_MAX_SAMPLES,
    .resolution_bits = SIGNAL_ADC_AD9280_RESOLUTION_BITS,
    .nominal_lsb_v = SIGNAL_ADC_AD9280_NOMINAL_PHYSICAL_LSB_V,
    .transport_format = "signed-Q8.8",
    .transport_lsb_v = SIGNAL_ADC_AD9280_CAL_SCALE_V_PER_Q8_8,
    .transport_offset_v = SIGNAL_ADC_AD9280_CAL_OFFSET_V,
    .transport_fraction_bits = AD9280_SAMPLE_FRACTION_BITS,
    .transport_counts_per_native_code = 256.0f,
    .voltage_transfer_model = "linear calibrated V=Q*scale+offset",
    .voltage_calibration_source = SIGNAL_ADC_AD9280_CAL_SOURCE,
    .voltage_calibrated = 1U,
    .ac_amplitude_scale =
        SIGNAL_ADC_AD9280_CAL_AC_AMPLITUDE_SCALE,
    .ac_amplitude_calibration_source =
        SIGNAL_ADC_AD9280_CAL_AC_SOURCE,
    .amplitude_path = "fixed CIC3/R16 plus analytic inverse response",
    .digital_response_model = "CIC3/R16 analytic (Fs=32MHz)",
    .digital_response_min_hz = SIGNAL_ADC_AD9280_RESPONSE_MIN_HZ,
    .digital_response_max_hz = SIGNAL_ADC_AD9280_RESPONSE_MAX_HZ,
    .digital_magnitude_response = SignalAdc_CicMagnitude,
    .digital_phase_response_rad = SignalAdc_CicPhaseRad,
    .deterministic_response_valid = 1U,
    .requires_external_antialias = 1U
};

static SignalAdc_Diagnostic s_diagnostic;
static uint8_t s_initialized;

static float SignalAdc_CicMagnitude(float frequency_hz)
{
    double frequency = (double)frequency_hz;
    double x;
    double ratio;
    double magnitude;

    if (frequency < 0.0) {
        frequency = -frequency;
    }
    if (frequency == 0.0) {
        return 1.0f;
    }
    if (frequency > (double)SIGNAL_ADC_AD9280_RESPONSE_MAX_HZ) {
        return 0.0f;
    }

    x = SIGNAL_ADC_AD9280_PI * frequency /
        (double)AD9280_SAMPLE_CLOCK_HZ;
    ratio = sin((double)SIGNAL_ADC_AD9280_DECIMATION * x) /
            ((double)SIGNAL_ADC_AD9280_DECIMATION * sin(x));
    magnitude = ratio * ratio * ratio;
    return (float)magnitude;
}

static float SignalAdc_CicPhaseRad(float frequency_hz)
{
    const double group_delay_input_clocks =
        0.5 * (double)SIGNAL_ADC_AD9280_CIC_STAGES *
        ((double)SIGNAL_ADC_AD9280_DECIMATION - 1.0);
    double frequency = (double)frequency_hz;

    if (frequency < 0.0) {
        frequency = -frequency;
    }
    if (frequency > (double)SIGNAL_ADC_AD9280_RESPONSE_MAX_HZ) {
        return 0.0f;
    }
    return (float)(-2.0 * SIGNAL_ADC_AD9280_PI * frequency *
                   group_delay_input_clocks /
                   (double)AD9280_SAMPLE_CLOCK_HZ);
}

static SignalAdc_Status SignalAdc_MapStatus(AD9280_StatusTypeDef status)
{
    switch (status) {
    case AD9280_OK:              return SIGNAL_ADC_OK;
    case AD9280_ERROR_PARAM:     return SIGNAL_ADC_E_ARGUMENT;
    case AD9280_ERROR_NOT_INIT:  return SIGNAL_ADC_E_NOT_INITIALIZED;
    case AD9280_ERROR_LINK:      return SIGNAL_ADC_E_LINK;
    case AD9280_ERROR_DEVICE:    return SIGNAL_ADC_E_DEVICE;
    case AD9280_ERROR_FORMAT:    return SIGNAL_ADC_E_FORMAT;
    case AD9280_ERROR_NOT_READY: return SIGNAL_ADC_E_NOT_READY;
    case AD9280_ERROR_BUSY:      return SIGNAL_ADC_E_BUSY;
    case AD9280_ERROR_TIMEOUT:   return SIGNAL_ADC_E_TIMEOUT;
    case AD9280_ERROR_NO_DATA:   return SIGNAL_ADC_E_NO_DATA;
    default:                     return SIGNAL_ADC_E_UNKNOWN;
    }
}

static SignalAdc_Stage SignalAdc_MapStage(
    AD9280_AppCaptureStageTypeDef stage)
{
    switch (stage) {
    case AD9280_APP_CAPTURE_IDLE:     return SIGNAL_ADC_STAGE_IDLE;
    case AD9280_APP_CAPTURE_START:    return SIGNAL_ADC_STAGE_START;
    case AD9280_APP_CAPTURE_WAIT:     return SIGNAL_ADC_STAGE_WAIT;
    case AD9280_APP_CAPTURE_READ:     return SIGNAL_ADC_STAGE_READ;
    case AD9280_APP_CAPTURE_STATUS:   return SIGNAL_ADC_STAGE_STATUS;
    case AD9280_APP_CAPTURE_COMPLETE: return SIGNAL_ADC_STAGE_COMPLETE;
    default:                          return SIGNAL_ADC_STAGE_UNKNOWN;
    }
}

static float SignalAdc_Q8_8ToVoltage(AD9280_SampleTypeDef sample)
{
    return (float)sample * SIGNAL_ADC_AD9280_CAL_SCALE_V_PER_Q8_8 +
           SIGNAL_ADC_AD9280_CAL_OFFSET_V;
}

static void SignalAdc_UpdateDiagnostic(AD9280_StatusTypeDef native_status)
{
    const AD9280_AppDataTypeDef *app = AD9280_AppGetData();
    const AD9280_DataTypeDef *driver = AD9280_GetData();

    s_diagnostic.native_status = (uint32_t)native_status;
    if (app != NULL) {
        s_diagnostic.stage = SignalAdc_MapStage(app->capture_stage);
        s_diagnostic.captured_count =
            (uint32_t)app->capture.captured_count;
        s_diagnostic.ready = app->capture.adc_ready;
        s_diagnostic.busy = app->capture.busy;
        s_diagnostic.done = app->capture.done;
    }
    if (driver != NULL) {
        s_diagnostic.device_id = driver->device_id;
        s_diagnostic.firmware_version = driver->firmware_version;
        s_diagnostic.response_profile = driver->response_profile;
        s_diagnostic.read_count = (uint32_t)driver->read_progress;
    }
}

SignalAdc_Status SignalAdc_Init(void)
{
    const AD9280_DataTypeDef *driver;
    AD9280_StatusTypeDef status;

    s_diagnostic = (SignalAdc_Diagnostic){0};
    s_diagnostic.stage = SIGNAL_ADC_STAGE_IDLE;
    s_initialized = 0U;

    if (((AD9280_SAMPLE_CLOCK_HZ %
          SIGNAL_ADC_AD9280_OUTPUT_RATE_HZ) != 0U) ||
        ((AD9280_SAMPLE_CLOCK_HZ /
          SIGNAL_ADC_AD9280_OUTPUT_RATE_HZ) !=
         SIGNAL_ADC_AD9280_DECIMATION) ||
        !(SIGNAL_ADC_AD9280_CAL_SCALE_V_PER_Q8_8 > 0.0f) ||
        !isfinite(SIGNAL_ADC_AD9280_CAL_SCALE_V_PER_Q8_8) ||
        !isfinite(SIGNAL_ADC_AD9280_CAL_OFFSET_V) ||
        !(SIGNAL_ADC_AD9280_CAL_AC_AMPLITUDE_SCALE > 0.0f) ||
        !isfinite(SIGNAL_ADC_AD9280_CAL_AC_AMPLITUDE_SCALE)) {
        return SIGNAL_ADC_E_ARGUMENT;
    }

    status = AD9280_AppInit();
    SignalAdc_UpdateDiagnostic(status);
    if (status != AD9280_OK) {
        return SignalAdc_MapStatus(status);
    }
    driver = AD9280_GetData();
    if ((driver == NULL) ||
        (driver->firmware_version !=
         AD9280_CAPTURE_PROTOCOL_VERSION) ||
        (driver->response_profile !=
         AD9280_RESPONSE_PROFILE_CIC3_R16) ||
        (driver->sample_word_bits != AD9280_SAMPLE_WORD_BITS) ||
        (driver->sample_fraction_bits !=
         AD9280_SAMPLE_FRACTION_BITS)) {
        SignalAdc_UpdateDiagnostic(AD9280_ERROR_FORMAT);
        return SIGNAL_ADC_E_FORMAT;
    }

    s_initialized = 1U;
    return SIGNAL_ADC_OK;
}

SignalAdc_Status SignalAdc_Capture(float *destination_v,
                                  uint32_t capacity,
                                  uint32_t requested_count,
                                  uint32_t timeout_ms,
                                  SignalAdc_FrameInfo *frame)
{
    const AD9280_AppDataTypeDef *app;
    const AD9280_SampleTypeDef *samples;
    AD9280_StatusTypeDef status;
    uint32_t sample_count;

    if (frame != NULL) {
        *frame = (SignalAdc_FrameInfo){0};
    }
    if (s_initialized == 0U) {
        return SIGNAL_ADC_E_NOT_INITIALIZED;
    }
    if ((destination_v == NULL) || (frame == NULL) ||
        (requested_count == 0U) ||
        (requested_count > capacity) ||
        (requested_count > AD9280_BUFFER_MAX_SAMPLES) ||
        (requested_count > 65535U) ||
        (timeout_ms == 0U)) {
        return SIGNAL_ADC_E_ARGUMENT;
    }

    status = AD9280_AppCapture((uint16_t)requested_count,
                               SIGNAL_ADC_AD9280_DECIMATION,
                               timeout_ms);
    SignalAdc_UpdateDiagnostic(status);
    if (status != AD9280_OK) {
        return SignalAdc_MapStatus(status);
    }

    samples = AD9280_AppGetSamples();
    sample_count = (uint32_t)AD9280_AppGetSampleCount();
    app = AD9280_AppGetData();
    if ((samples == NULL) || (app == NULL) ||
        (sample_count != requested_count)) {
        return SIGNAL_ADC_E_NO_DATA;
    }

    for (uint32_t i = 0U; i < sample_count; ++i) {
        destination_v[i] =
            SignalAdc_Q8_8ToVoltage(samples[i]);
    }

    frame->sample_count = sample_count;
    frame->sample_rate_hz = s_info.output_sample_rate_hz;
    frame->min_v =
        SignalAdc_Q8_8ToVoltage(
            app->capture.min_sample_q8_8);
    frame->max_v =
        SignalAdc_Q8_8ToVoltage(
            app->capture.max_sample_q8_8);
    frame->clipped =
        ((app->capture.overrange != 0U) ||
         (app->capture.min_sample_q8_8 <=
          SIGNAL_ADC_AD9280_CLIP_LOW_Q8_8) ||
         (app->capture.max_sample_q8_8 >=
          SIGNAL_ADC_AD9280_CLIP_HIGH_Q8_8))
            ? 1U : 0U;
    return SIGNAL_ADC_OK;
}

SignalAdc_Status SignalAdc_CaptureRawStats(
    uint32_t requested_count,
    uint32_t timeout_ms,
    SignalAdc_RawFrameStats *stats)
{
    const AD9280_AppDataTypeDef *app;
    AD9280_StatusTypeDef status;

    if (stats != NULL) {
        *stats = (SignalAdc_RawFrameStats){0};
    }
    if (s_initialized == 0U) {
        return SIGNAL_ADC_E_NOT_INITIALIZED;
    }
    if ((stats == NULL) || (requested_count == 0U) ||
        (requested_count > AD9280_BUFFER_MAX_SAMPLES) ||
        (requested_count > 65535U) || (timeout_ms == 0U)) {
        return SIGNAL_ADC_E_ARGUMENT;
    }

    status = AD9280_AppCaptureStats((uint16_t)requested_count,
                                    SIGNAL_ADC_AD9280_DECIMATION,
                                    timeout_ms);
    SignalAdc_UpdateDiagnostic(status);
    if (status != AD9280_OK) {
        return SignalAdc_MapStatus(status);
    }

    app = AD9280_AppGetData();
    if ((app == NULL) ||
        ((uint32_t)app->capture.captured_count != requested_count)) {
        return SIGNAL_ADC_E_NO_DATA;
    }

    stats->sample_sum = (int64_t)app->capture.sample_sum_q8_8;
    stats->min_sample = (int32_t)app->capture.min_sample_q8_8;
    stats->max_sample = (int32_t)app->capture.max_sample_q8_8;
    stats->sample_count = (uint32_t)app->capture.captured_count;
    stats->overrange_count = app->capture.overrange_count;
    stats->clipped =
        ((app->capture.overrange != 0U) ||
         (app->capture.min_sample_q8_8 <=
          SIGNAL_ADC_AD9280_CLIP_LOW_Q8_8) ||
         (app->capture.max_sample_q8_8 >=
          SIGNAL_ADC_AD9280_CLIP_HIGH_Q8_8))
            ? 1U : 0U;
    return SIGNAL_ADC_OK;
}

SignalAdc_Status SignalAdc_Abort(void)
{
    AD9280_StatusTypeDef status = AD9280_AbortCapture();
    SignalAdc_UpdateDiagnostic(status);
    return SignalAdc_MapStatus(status);
}

const SignalAdc_Info *SignalAdc_GetInfo(void)
{
    return &s_info;
}

const SignalAdc_Diagnostic *SignalAdc_GetDiagnostic(void)
{
    return &s_diagnostic;
}

const char *SignalAdc_StatusName(SignalAdc_Status status)
{
    switch (status) {
    case SIGNAL_ADC_OK:                return "OK";
    case SIGNAL_ADC_E_ARGUMENT:        return "ARGUMENT";
    case SIGNAL_ADC_E_NOT_INITIALIZED: return "NOT_INITIALIZED";
    case SIGNAL_ADC_E_LINK:            return "LINK";
    case SIGNAL_ADC_E_DEVICE:          return "DEVICE";
    case SIGNAL_ADC_E_FORMAT:          return "SAMPLE_FORMAT";
    case SIGNAL_ADC_E_NOT_READY:       return "NOT_READY";
    case SIGNAL_ADC_E_BUSY:            return "BUSY";
    case SIGNAL_ADC_E_TIMEOUT:         return "TIMEOUT";
    case SIGNAL_ADC_E_NO_DATA:         return "NO_DATA";
    default:                           return "UNKNOWN";
    }
}

const char *SignalAdc_StageName(SignalAdc_Stage stage)
{
    switch (stage) {
    case SIGNAL_ADC_STAGE_IDLE:     return "IDLE";
    case SIGNAL_ADC_STAGE_START:    return "START";
    case SIGNAL_ADC_STAGE_WAIT:     return "WAIT";
    case SIGNAL_ADC_STAGE_READ:     return "READ";
    case SIGNAL_ADC_STAGE_STATUS:   return "STATUS";
    case SIGNAL_ADC_STAGE_COMPLETE: return "COMPLETE";
    default:                        return "UNKNOWN";
    }
}
