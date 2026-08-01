#include "signal_adc.h"

#include "ad9238_app.h"
#include <stddef.h>

/*
 * Current hardware provider: AD9238 -> FPGA FIR307 + CIC3/R25 Q8.8 -> SPI -> STM32.
 *
 * These device-specific values intentionally live only in this adapter.
 * The voltage transfer below is the module's nominal data-sheet scale.
 * Frequency-limited measured residual correction is applied by the analyzer.
 */
#define SIGNAL_ADC_AD9238_OUTPUT_RATE_HZ  2000000U
#define SIGNAL_ADC_AD9238_DECIMATION      25U
#define SIGNAL_ADC_AD9238_RESOLUTION_BITS 12U
#define SIGNAL_ADC_AD9238_PHYSICAL_LSB_V   (10.0f / 4096.0f)
#define SIGNAL_ADC_AD9238_Q8_8_LSB_V       (10.0f / 65536.0f)
#define SIGNAL_ADC_AD9238_CLIP_LOW_Q8_8    (-32000)
#define SIGNAL_ADC_AD9238_CLIP_HIGH_Q8_8   31744

static const SignalAdc_Info s_info = {
    "AD9238+FPGA-FIR307-CIC3/R25-S16Q8.8",
    AD9238_SAMPLE_CLOCK_HZ,
    (float)SIGNAL_ADC_AD9238_OUTPUT_RATE_HZ,
    AD9238_BUFFER_MAX_SAMPLES,
    SIGNAL_ADC_AD9238_RESOLUTION_BITS,
    SIGNAL_ADC_AD9238_PHYSICAL_LSB_V,
    "signed-Q8.8",
    SIGNAL_ADC_AD9238_Q8_8_LSB_V
};

static SignalAdc_Diagnostic s_diagnostic;
static uint8_t s_initialized;

static SignalAdc_Status SignalAdc_MapStatus(AD9238_StatusTypeDef status)
{
    switch (status) {
    case AD9238_OK:              return SIGNAL_ADC_OK;
    case AD9238_ERROR_PARAM:     return SIGNAL_ADC_E_ARGUMENT;
    case AD9238_ERROR_NOT_INIT:  return SIGNAL_ADC_E_NOT_INITIALIZED;
    case AD9238_ERROR_LINK:      return SIGNAL_ADC_E_LINK;
    case AD9238_ERROR_DEVICE:    return SIGNAL_ADC_E_DEVICE;
    case AD9238_ERROR_FORMAT:    return SIGNAL_ADC_E_FORMAT;
    case AD9238_ERROR_NOT_READY: return SIGNAL_ADC_E_NOT_READY;
    case AD9238_ERROR_BUSY:      return SIGNAL_ADC_E_BUSY;
    case AD9238_ERROR_TIMEOUT:   return SIGNAL_ADC_E_TIMEOUT;
    case AD9238_ERROR_NO_DATA:   return SIGNAL_ADC_E_NO_DATA;
    default:                     return SIGNAL_ADC_E_UNKNOWN;
    }
}

static SignalAdc_Stage SignalAdc_MapStage(
    AD9238_AppCaptureStageTypeDef stage)
{
    switch (stage) {
    case AD9238_APP_CAPTURE_IDLE:     return SIGNAL_ADC_STAGE_IDLE;
    case AD9238_APP_CAPTURE_START:    return SIGNAL_ADC_STAGE_START;
    case AD9238_APP_CAPTURE_WAIT:     return SIGNAL_ADC_STAGE_WAIT;
    case AD9238_APP_CAPTURE_READ:     return SIGNAL_ADC_STAGE_READ;
    case AD9238_APP_CAPTURE_STATUS:   return SIGNAL_ADC_STAGE_STATUS;
    case AD9238_APP_CAPTURE_COMPLETE: return SIGNAL_ADC_STAGE_COMPLETE;
    default:                          return SIGNAL_ADC_STAGE_UNKNOWN;
    }
}

static float SignalAdc_Q8_8ToNominalVoltage(AD9238_SampleTypeDef sample)
{
    return (float)sample * SIGNAL_ADC_AD9238_Q8_8_LSB_V;
}

static void SignalAdc_UpdateDiagnostic(AD9238_StatusTypeDef native_status)
{
    const AD9238_AppDataTypeDef *app = AD9238_AppGetData();
    const AD9238_DataTypeDef *driver = AD9238_GetData();

    s_diagnostic.native_status = (uint32_t)native_status;
    if (app != NULL) {
        s_diagnostic.stage = SignalAdc_MapStage(app->capture_stage);
        s_diagnostic.captured_count =
            (uint32_t)app->capture.captured_count;
        s_diagnostic.overrange_count =
            app->capture.overrange_count;
        s_diagnostic.sample_sum_q8_8 =
            app->capture.sample_sum_q8_8;
        s_diagnostic.min_sample_q8_8 =
            app->capture.min_sample_q8_8;
        s_diagnostic.max_sample_q8_8 =
            app->capture.max_sample_q8_8;
        s_diagnostic.latest_sample_q8_8 =
            app->capture.latest_sample_q8_8;
        s_diagnostic.mean_sample_q8_8 =
            app->capture.mean_sample_q8_8;
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

static void SignalAdc_FillFrame(SignalAdc_FrameInfo *frame,
                                uint32_t sample_count,
                                const AD9238_CaptureInfoTypeDef *capture)
{
    frame->sample_count = sample_count;
    frame->sample_rate_hz = s_info.output_sample_rate_hz;
    frame->min_v = SignalAdc_Q8_8ToNominalVoltage(
        capture->min_sample_q8_8);
    frame->max_v = SignalAdc_Q8_8ToNominalVoltage(
        capture->max_sample_q8_8);
    frame->clipped =
        ((capture->overrange != 0U) ||
         (capture->min_sample_q8_8 <= SIGNAL_ADC_AD9238_CLIP_LOW_Q8_8) ||
         (capture->max_sample_q8_8 >= SIGNAL_ADC_AD9238_CLIP_HIGH_Q8_8))
            ? 1U : 0U;
}

SignalAdc_Status SignalAdc_Init(void)
{
    const AD9238_DataTypeDef *driver;
    AD9238_StatusTypeDef status;

    s_diagnostic = (SignalAdc_Diagnostic){0};
    s_diagnostic.stage = SIGNAL_ADC_STAGE_IDLE;
    s_initialized = 0U;

    if (((AD9238_SAMPLE_CLOCK_HZ %
          SIGNAL_ADC_AD9238_OUTPUT_RATE_HZ) != 0U) ||
        ((AD9238_SAMPLE_CLOCK_HZ /
          SIGNAL_ADC_AD9238_OUTPUT_RATE_HZ) !=
         SIGNAL_ADC_AD9238_DECIMATION)) {
        return SIGNAL_ADC_E_ARGUMENT;
    }

    status = AD9238_AppInit();
    SignalAdc_UpdateDiagnostic(status);
    if (status != AD9238_OK) {
        return SignalAdc_MapStatus(status);
    }
    driver = AD9238_GetData();
    if ((driver == NULL) ||
        (driver->firmware_version !=
         AD9238_CAPTURE_PROTOCOL_Q8_8_VERSION) ||
        (driver->sample_word_bits != AD9238_SAMPLE_WORD_BITS) ||
        (driver->sample_fraction_bits !=
         AD9238_SAMPLE_FRACTION_BITS)) {
        SignalAdc_UpdateDiagnostic(AD9238_ERROR_FORMAT);
        return SIGNAL_ADC_E_FORMAT;
    }

    status = AD9238_SetCic3Filter(1U);
    SignalAdc_UpdateDiagnostic(status);
    if (status != AD9238_OK) {
        return SignalAdc_MapStatus(status);
    }

    /*
     * Before the first frame, expose the backend's intended R25/2 MSPS
     * configuration instead of the driver's unused power-on default.
     */
    s_diagnostic.configured_decimation =
        SIGNAL_ADC_AD9238_DECIMATION;
    s_diagnostic.stored_sample_rate_hz =
        (float)SIGNAL_ADC_AD9238_OUTPUT_RATE_HZ;
    s_initialized = 1U;
    return SIGNAL_ADC_OK;
}

SignalAdc_Status SignalAdc_Capture(float *destination_v,
                                  uint32_t capacity,
                                  uint32_t requested_count,
                                  uint32_t timeout_ms,
                                  SignalAdc_FrameInfo *frame)
{
    const AD9238_AppDataTypeDef *app;
    const AD9238_SampleTypeDef *samples;
    AD9238_StatusTypeDef status;
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
        (requested_count > AD9238_BUFFER_MAX_SAMPLES) ||
        (requested_count > 65535U) ||
        (timeout_ms == 0U)) {
        return SIGNAL_ADC_E_ARGUMENT;
    }

    status = AD9238_AppCapture((uint16_t)requested_count,
                               SIGNAL_ADC_AD9238_DECIMATION,
                               timeout_ms);
    SignalAdc_UpdateDiagnostic(status);
    if (status != AD9238_OK) {
        return SignalAdc_MapStatus(status);
    }

    samples = AD9238_AppGetSamples();
    sample_count = (uint32_t)AD9238_AppGetSampleCount();
    app = AD9238_AppGetData();
    if ((samples == NULL) || (app == NULL) ||
        (sample_count != requested_count)) {
        return SIGNAL_ADC_E_NO_DATA;
    }

    for (uint32_t i = 0U; i < sample_count; ++i) {
        destination_v[i] =
            SignalAdc_Q8_8ToNominalVoltage(samples[i]);
    }

    SignalAdc_FillFrame(frame, sample_count, &app->capture);
    return SIGNAL_ADC_OK;
}

SignalAdc_Status SignalAdc_CaptureDual(float *ch1_destination_v,
                                      float *ch2_destination_v,
                                      uint32_t capacity,
                                      uint32_t requested_count,
                                      uint32_t timeout_ms,
                                      SignalAdc_FrameInfo *ch1_frame,
                                      SignalAdc_FrameInfo *ch2_frame)
{
    const AD9238_AppDataTypeDef *app;
    const AD9238_SampleTypeDef *ch1_samples;
    const AD9238_SampleTypeDef *ch2_samples;
    AD9238_StatusTypeDef status;
    uint32_t ch1_count;
    uint32_t ch2_count;

    if (ch1_frame != NULL) {
        *ch1_frame = (SignalAdc_FrameInfo){0};
    }
    if (ch2_frame != NULL) {
        *ch2_frame = (SignalAdc_FrameInfo){0};
    }
    if (s_initialized == 0U) {
        return SIGNAL_ADC_E_NOT_INITIALIZED;
    }
    if ((ch1_destination_v == NULL) || (ch2_destination_v == NULL) ||
        (ch1_frame == NULL) || (ch2_frame == NULL) ||
        (requested_count == 0U) || (requested_count > capacity) ||
        (requested_count > AD9238_BUFFER_MAX_SAMPLES) ||
        (requested_count > 65535U) || (timeout_ms == 0U)) {
        return SIGNAL_ADC_E_ARGUMENT;
    }

    /* CH1 keeps FIR+CIC; CH2 simultaneously uses its FIR-only R25 path. */
    status = AD9238_AppCaptureDual((uint16_t)requested_count,
                                   SIGNAL_ADC_AD9238_DECIMATION,
                                   timeout_ms);
    SignalAdc_UpdateDiagnostic(status);
    if (status != AD9238_OK) {
        return SignalAdc_MapStatus(status);
    }

    ch1_samples = AD9238_AppGetSamplesChannel(AD9238_CHANNEL_1);
    ch2_samples = AD9238_AppGetSamplesChannel(AD9238_CHANNEL_2);
    ch1_count = (uint32_t)AD9238_AppGetSampleCountChannel(AD9238_CHANNEL_1);
    ch2_count = (uint32_t)AD9238_AppGetSampleCountChannel(AD9238_CHANNEL_2);
    app = AD9238_AppGetData();
    if ((ch1_samples == NULL) || (ch2_samples == NULL) || (app == NULL) ||
        (ch1_count != requested_count) || (ch2_count != requested_count)) {
        return SIGNAL_ADC_E_NO_DATA;
    }

    for (uint32_t i = 0U; i < requested_count; ++i) {
        ch1_destination_v[i] =
            SignalAdc_Q8_8ToNominalVoltage(ch1_samples[i]);
        ch2_destination_v[i] =
            SignalAdc_Q8_8ToNominalVoltage(ch2_samples[i]);
    }
    SignalAdc_FillFrame(ch1_frame, ch1_count, &app->capture);
    SignalAdc_FillFrame(ch2_frame, ch2_count, &app->capture_ch2);
    return SIGNAL_ADC_OK;
}

SignalAdc_Status SignalAdc_Abort(void)
{
    AD9238_StatusTypeDef status = AD9238_AbortCapture();
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
