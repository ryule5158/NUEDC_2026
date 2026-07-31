#include "signal_adc.h"

#include "ltc2208_app.h"
#include <stddef.h>

/*
 * Current hardware provider: LTC2208 -> FPGA CIC5/R65 + FIR81 -> SPI -> STM32.
 *
 * These device-specific values intentionally live only in this adapter.
 * The voltage transfer below is the module's nominal data-sheet scale.
 * No measured response correction is enabled until an LTC2208 BNC-referred
 * sweep supplies a dedicated calibration table.
 */
#define SIGNAL_ADC_LTC2208_OUTPUT_RATE_HZ  2000000U
#define SIGNAL_ADC_LTC2208_DECIMATION      65U
#define SIGNAL_ADC_LTC2208_RESOLUTION_BITS 16U
#define SIGNAL_ADC_LTC2208_PHYSICAL_LSB_V   (2.25f / 65536.0f)
#define SIGNAL_ADC_LTC2208_Q3_LSB_V          (2.25f / (65536.0f * 8.0f))
#define SIGNAL_ADC_LTC2208_CLIP_LOW_S16      (-32760)
#define SIGNAL_ADC_LTC2208_CLIP_HIGH_S16     32759

static const SignalAdc_Info s_info = {
    "LTC2208+FPGA-CIC5/R65-FIR81-S20Q3",
    LTC2208_SAMPLE_CLOCK_HZ,
    (float)SIGNAL_ADC_LTC2208_OUTPUT_RATE_HZ,
    LTC2208_BUFFER_MAX_SAMPLES,
    SIGNAL_ADC_LTC2208_RESOLUTION_BITS,
    SIGNAL_ADC_LTC2208_PHYSICAL_LSB_V,
    "signed-20-Q3",
    SIGNAL_ADC_LTC2208_Q3_LSB_V
};

static SignalAdc_Diagnostic s_diagnostic;
static uint8_t s_initialized;

static SignalAdc_Status SignalAdc_MapStatus(LTC2208_StatusTypeDef status)
{
    switch (status) {
    case LTC2208_OK:              return SIGNAL_ADC_OK;
    case LTC2208_ERROR_PARAM:     return SIGNAL_ADC_E_ARGUMENT;
    case LTC2208_ERROR_NOT_INIT:  return SIGNAL_ADC_E_NOT_INITIALIZED;
    case LTC2208_ERROR_LINK:      return SIGNAL_ADC_E_LINK;
    case LTC2208_ERROR_DEVICE:    return SIGNAL_ADC_E_DEVICE;
    case LTC2208_ERROR_FORMAT:    return SIGNAL_ADC_E_FORMAT;
    case LTC2208_ERROR_NOT_READY: return SIGNAL_ADC_E_NOT_READY;
    case LTC2208_ERROR_BUSY:      return SIGNAL_ADC_E_BUSY;
    case LTC2208_ERROR_TIMEOUT:   return SIGNAL_ADC_E_TIMEOUT;
    case LTC2208_ERROR_NO_DATA:   return SIGNAL_ADC_E_NO_DATA;
    default:                     return SIGNAL_ADC_E_UNKNOWN;
    }
}

static SignalAdc_Stage SignalAdc_MapStage(
    LTC2208_AppCaptureStageTypeDef stage)
{
    switch (stage) {
    case LTC2208_APP_CAPTURE_IDLE:     return SIGNAL_ADC_STAGE_IDLE;
    case LTC2208_APP_CAPTURE_START:    return SIGNAL_ADC_STAGE_START;
    case LTC2208_APP_CAPTURE_WAIT:     return SIGNAL_ADC_STAGE_WAIT;
    case LTC2208_APP_CAPTURE_READ:     return SIGNAL_ADC_STAGE_READ;
    case LTC2208_APP_CAPTURE_STATUS:   return SIGNAL_ADC_STAGE_STATUS;
    case LTC2208_APP_CAPTURE_COMPLETE: return SIGNAL_ADC_STAGE_COMPLETE;
    default:                          return SIGNAL_ADC_STAGE_UNKNOWN;
    }
}

static float SignalAdc_Q3ToNominalVoltage(LTC2208_SampleTypeDef sample)
{
    return (float)sample * SIGNAL_ADC_LTC2208_Q3_LSB_V;
}

static float SignalAdc_RawS16ToNominalVoltage(int16_t sample)
{
    return (float)sample * SIGNAL_ADC_LTC2208_PHYSICAL_LSB_V;
}

static void SignalAdc_UpdateDiagnostic(LTC2208_StatusTypeDef native_status)
{
    const LTC2208_AppDataTypeDef *app = LTC2208_AppGetData();
    const LTC2208_DataTypeDef *driver = LTC2208_GetData();

    s_diagnostic.native_status = (uint32_t)native_status;
    if (app != NULL) {
        s_diagnostic.stage = SignalAdc_MapStage(app->capture_stage);
        s_diagnostic.captured_count =
            (uint32_t)app->capture.captured_count;
        s_diagnostic.overrange_count =
            app->capture.overrange_count;
        s_diagnostic.sample_sum_s16 =
            app->capture.sample_sum_s16;
        s_diagnostic.min_sample_s16 =
            app->capture.min_sample_s16;
        s_diagnostic.max_sample_s16 =
            app->capture.max_sample_s16;
        s_diagnostic.latest_sample_s16 =
            app->capture.latest_sample_s16;
        s_diagnostic.mean_sample_s16 =
            app->capture.mean_sample_s16;
        s_diagnostic.stored_sample_rate_hz =
            app->capture.stored_sample_rate_hz;
        s_diagnostic.ready = app->capture.adc_ready;
        s_diagnostic.busy = app->capture.busy;
        s_diagnostic.done = app->capture.done;
        s_diagnostic.triggered = app->capture.triggered;
        s_diagnostic.waiting_trigger =
            app->capture.waiting_trigger;
        s_diagnostic.overrange = app->capture.overrange;
    }
    if (driver != NULL) {
        s_diagnostic.device_id = driver->device_id;
        s_diagnostic.firmware_version = driver->firmware_version;
        s_diagnostic.physical_sample_clock_hz =
            driver->sample_clock_hz;
        s_diagnostic.configured_sample_count =
            (uint32_t)driver->config.sample_count;
        s_diagnostic.configured_decimation =
            (uint32_t)driver->config.decimation;
        s_diagnostic.trigger_mode =
            (uint8_t)driver->config.trigger_mode;
        s_diagnostic.trigger_threshold =
            (uint32_t)driver->config.trigger_threshold;
        s_diagnostic.sample_word_bits =
            driver->sample_word_bits;
        s_diagnostic.sample_fraction_bits =
            driver->sample_fraction_bits;
        s_diagnostic.read_count = (uint32_t)driver->read_progress;
    }
}

SignalAdc_Status SignalAdc_Init(void)
{
    const LTC2208_DataTypeDef *driver;
    LTC2208_StatusTypeDef status;

    s_diagnostic = (SignalAdc_Diagnostic){0};
    s_diagnostic.stage = SIGNAL_ADC_STAGE_IDLE;
    s_initialized = 0U;

    if (((LTC2208_SAMPLE_CLOCK_HZ %
          SIGNAL_ADC_LTC2208_OUTPUT_RATE_HZ) != 0U) ||
        ((LTC2208_SAMPLE_CLOCK_HZ /
          SIGNAL_ADC_LTC2208_OUTPUT_RATE_HZ) !=
         SIGNAL_ADC_LTC2208_DECIMATION)) {
        return SIGNAL_ADC_E_ARGUMENT;
    }

    status = LTC2208_AppInit();
    SignalAdc_UpdateDiagnostic(status);
    if (status != LTC2208_OK) {
        return SignalAdc_MapStatus(status);
    }
    driver = LTC2208_GetData();
    if ((driver == NULL) ||
        (driver->firmware_version !=
         LTC2208_CAPTURE_PROTOCOL_Q3_VERSION) ||
        (driver->sample_word_bits != LTC2208_SAMPLE_WORD_BITS) ||
        (driver->sample_fraction_bits !=
         LTC2208_SAMPLE_FRACTION_BITS)) {
        SignalAdc_UpdateDiagnostic(LTC2208_ERROR_FORMAT);
        return SIGNAL_ADC_E_FORMAT;
    }

    status = LTC2208_SetAntiInterferenceFilter(1U);
    SignalAdc_UpdateDiagnostic(status);
    if (status != LTC2208_OK) {
        return SignalAdc_MapStatus(status);
    }

    /*
     * Before the first frame, expose the backend's intended R65/2 MSPS
     * configuration instead of the driver's unused power-on default.
     */
    s_diagnostic.configured_decimation =
        SIGNAL_ADC_LTC2208_DECIMATION;
    s_diagnostic.stored_sample_rate_hz =
        (float)SIGNAL_ADC_LTC2208_OUTPUT_RATE_HZ;
    s_initialized = 1U;
    return SIGNAL_ADC_OK;
}

SignalAdc_Status SignalAdc_Capture(float *destination_v,
                                  uint32_t capacity,
                                  uint32_t requested_count,
                                  uint32_t timeout_ms,
                                  SignalAdc_FrameInfo *frame)
{
    const LTC2208_AppDataTypeDef *app;
    const LTC2208_SampleTypeDef *samples;
    LTC2208_StatusTypeDef status;
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
        (requested_count > LTC2208_BUFFER_MAX_SAMPLES) ||
        (requested_count > 65535U) ||
        (timeout_ms == 0U)) {
        return SIGNAL_ADC_E_ARGUMENT;
    }

    status = LTC2208_AppCapture((uint16_t)requested_count,
                               SIGNAL_ADC_LTC2208_DECIMATION,
                               timeout_ms);
    SignalAdc_UpdateDiagnostic(status);
    if (status != LTC2208_OK) {
        return SignalAdc_MapStatus(status);
    }

    samples = LTC2208_AppGetSamples();
    sample_count = (uint32_t)LTC2208_AppGetSampleCount();
    app = LTC2208_AppGetData();
    if ((samples == NULL) || (app == NULL) ||
        (sample_count != requested_count)) {
        return SIGNAL_ADC_E_NO_DATA;
    }

    for (uint32_t i = 0U; i < sample_count; ++i) {
        destination_v[i] =
            SignalAdc_Q3ToNominalVoltage(samples[i]);
    }

    frame->sample_count = sample_count;
    frame->sample_rate_hz = s_info.output_sample_rate_hz;
    frame->min_v =
        SignalAdc_RawS16ToNominalVoltage(
            app->capture.min_sample_s16);
    frame->max_v =
        SignalAdc_RawS16ToNominalVoltage(
            app->capture.max_sample_s16);
    frame->clipped =
        ((app->capture.overrange != 0U) ||
         (app->capture.min_sample_s16 <=
          SIGNAL_ADC_LTC2208_CLIP_LOW_S16) ||
         (app->capture.max_sample_s16 >=
          SIGNAL_ADC_LTC2208_CLIP_HIGH_S16))
            ? 1U : 0U;
    return SIGNAL_ADC_OK;
}

SignalAdc_Status SignalAdc_Abort(void)
{
    LTC2208_StatusTypeDef status = LTC2208_AbortCapture();
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
