#include "g_contest_app.h"

#include "ad9280_app.h"
#include "g_contest_config.h"
#include <stdio.h>
#include <string.h>

static float s_input_volts[G_CONTEST_SAMPLE_COUNT];
static PeriodicAnalyzer_Config s_analyzer_config;
static PeriodicAnalyzer_Result s_result;
static uint32_t s_frame_number;
static uint32_t s_last_latency_ms;

/*
 * Combined correction for the six-pole 650 kHz analog filter and the FPGA
 * CIC3/R16 filter. These are theoretical start values; overwrite them with a
 * swept-sine calibration before final judging.
 */
static void GContest_LoadInitialCalibration(void)
{
    static const PeriodicAnalyzer_CalPoint cal[] = {
        { 10000.0f, 1.000108895f,   5.911151f},
        { 50000.0f, 1.002717459f,  29.569450f},
        {100000.0f, 1.010959620f,  59.226000f},
        {200000.0f, 1.045251831f, 119.165300f},
        {300000.0f, 1.107128190f, 180.653500f},
        {400000.0f, 1.205277606f, 244.997800f},
        {500000.0f, 1.374052884f, 314.892400f}
    };

    s_analyzer_config.cal_count =
        (uint8_t)(sizeof(cal) / sizeof(cal[0]));
    memcpy(s_analyzer_config.cal, cal, sizeof(cal));
}

HAL_StatusTypeDef GContest_Init(void)
{
    AD9280_StatusTypeDef adc_status;

    if ((G_CONTEST_ADC_CLOCK_HZ / G_CONTEST_FPGA_DECIMATION) !=
        G_CONTEST_SAMPLE_RATE_HZ) {
        return HAL_ERROR;
    }

    PeriodicAnalyzer_GetDefaultConfig(&s_analyzer_config);
    s_analyzer_config.sample_rate_hz = (float)G_CONTEST_SAMPLE_RATE_HZ;
    s_analyzer_config.min_frequency_hz = G_CONTEST_MIN_FREQ_HZ;
    s_analyzer_config.max_frequency_hz = G_CONTEST_MAX_FREQ_HZ;
    s_analyzer_config.input_volts_per_adc_volt =
        1.0f / G_CONTEST_FRONT_END_GAIN;
    s_analyzer_config.min_component_peak_v =
        G_CONTEST_MIN_COMPONENT_PEAK_V;
    s_analyzer_config.relative_peak_threshold = 0.01f;
    GContest_LoadInitialCalibration();

    memset(&s_result, 0, sizeof(s_result));
    s_frame_number = 0U;
    s_last_latency_ms = 0U;

    adc_status = AD9280_AppInit();
    if (adc_status != AD9280_OK) {
        printf("[G] AD9280 init failed: %d\r\n", (int)adc_status);
        return HAL_ERROR;
    }
    adc_status = AD9280_AppSetVoltageCalibration(
        G_CONTEST_CAL_LOW_CODE,
        G_CONTEST_CAL_LOW_V,
        G_CONTEST_CAL_HIGH_CODE,
        G_CONTEST_CAL_HIGH_V);
    if (adc_status != AD9280_OK) return HAL_ERROR;

    adc_status = AD9280_SetCic3Filter(1U);
    if (adc_status != AD9280_OK) {
        printf("[G] FPGA image lacks CIC3/R16 support (need ADC fw 1.2)\r\n");
        return HAL_ERROR;
    }

    printf("[G] READY AD9280=32MSPS CIC3/R16 Fs=%lu N=%lu df=%.5fHz\r\n",
           (unsigned long)G_CONTEST_SAMPLE_RATE_HZ,
           (unsigned long)G_CONTEST_SAMPLE_COUNT,
           (double)G_CONTEST_BIN_WIDTH_HZ);
    return HAL_OK;
}

void GContest_Process(void)
{
    const uint8_t *codes;
    const AD9280_AppDataTypeDef *adc_data;
    AD9280_StatusTypeDef adc_status;
    PeriodicAnalyzer_Status analyzer_status;
    uint32_t start_ms = HAL_GetTick();

    adc_status = AD9280_AppCapture(G_CONTEST_SAMPLE_COUNT,
                                   G_CONTEST_FPGA_DECIMATION,
                                   G_CONTEST_CAPTURE_TIMEOUT_MS);
    if (adc_status != AD9280_OK) {
        printf("[G] capture failed: %d\r\n", (int)adc_status);
        return;
    }

    codes = AD9280_AppGetSamples();
    adc_data = AD9280_AppGetData();
    if ((codes == NULL) ||
        (AD9280_AppGetSampleCount() != G_CONTEST_SAMPLE_COUNT)) {
        printf("[G] incomplete frame\r\n");
        return;
    }

    for (uint32_t i = 0U; i < G_CONTEST_SAMPLE_COUNT; ++i) {
        s_input_volts[i] = AD9280_AppCodeToVoltage(codes[i]);
    }

    analyzer_status = PeriodicAnalyzer_Analyze(s_input_volts,
                                                G_CONTEST_SAMPLE_COUNT,
                                                &s_analyzer_config,
                                                &s_result);
    s_result.clipped =
        ((adc_data->capture.overrange != 0U) ||
         (adc_data->capture.min_code <= G_CONTEST_CLIP_LOW_CODE) ||
         (adc_data->capture.max_code >= G_CONTEST_CLIP_HIGH_CODE)) ? 1U : 0U;
    s_frame_number++;
    s_last_latency_ms = HAL_GetTick() - start_ms;

    if (analyzer_status == PERIODIC_ANALYZER_OK) {
        printf("[G] #%lu %lums f0=%.2fHz vpp=%.3fmV rms=%.3fmV lines=%u clip=%u\r\n",
               (unsigned long)s_frame_number,
               (unsigned long)s_last_latency_ms,
               (double)s_result.fundamental_hz,
               (double)(1000.0f * s_result.vpp_v),
               (double)(1000.0f * s_result.true_rms_v),
               (unsigned)s_result.component_count,
               (unsigned)s_result.clipped);
        for (uint32_t i = 0U; i < s_result.component_count; ++i) {
            printf("[G] H%u f=%.2fHz Apeak=%.3fmV phase=%.2fdeg\r\n",
                   (unsigned)s_result.component[i].harmonic_order,
                   (double)s_result.component[i].frequency_hz,
                   (double)(1000.0f * s_result.component[i].peak_amplitude_v),
                   (double)(57.2957795f * s_result.component[i].phase_rad));
        }
    } else {
        printf("[G] analyzer status=%d latency=%lums\r\n",
               (int)analyzer_status,
               (unsigned long)s_last_latency_ms);
    }
}

const PeriodicAnalyzer_Result *GContest_GetLastResult(void)
{
    return &s_result;
}

const float *GContest_GetLastSamples(void)
{
    return s_input_volts;
}

uint32_t GContest_GetFrameNumber(void)
{
    return s_frame_number;
}

uint32_t GContest_GetLastLatencyMs(void)
{
    return s_last_latency_ms;
}
