#include "g_contest_app.h"

#include "g_contest_config.h"
#include "signal_adc.h"
#include <float.h>
#include <stdio.h>
#include <string.h>

static float s_input_volts[G_CONTEST_SAMPLE_COUNT];
static PeriodicAnalyzer_Config s_analyzer_config;
static PeriodicAnalyzer_Result s_result;
static uint32_t s_frame_number;
static uint32_t s_last_latency_ms;
static uint8_t s_backend_ready;

static uint8_t GContest_StatusInvalidatesBackend(SignalAdc_Status status)
{
    return ((status == SIGNAL_ADC_E_NOT_INITIALIZED) ||
            (status == SIGNAL_ADC_E_LINK) ||
            (status == SIGNAL_ADC_E_DEVICE) ||
            (status == SIGNAL_ADC_E_FORMAT) ||
            (status == SIGNAL_ADC_E_NOT_READY)) ? 1U : 0U;
}

static uint8_t GContest_DigitalResponseValid(const SignalAdc_Info *info)
{
    const uint32_t check_intervals = 8U;

    if ((info == NULL) ||
        (info->deterministic_response_valid == 0U) ||
        (info->digital_response_model == NULL) ||
        (info->digital_magnitude_response == NULL) ||
        (info->digital_phase_response_rad == NULL) ||
        (G_CONTEST_MIN_FREQ_HZ < info->digital_response_min_hz) ||
        (G_CONTEST_MAX_FREQ_HZ > info->digital_response_max_hz)) {
        return 0U;
    }

    /*
     * Catch a missing, singular, NaN or infinite response before acquiring.
     * Nine points include both G-band edges and exercise the whole interval;
     * the analyzer also validates every actually used component frequency.
     */
    for (uint32_t i = 0U; i <= check_intervals; ++i) {
        const float frequency_hz =
            G_CONTEST_MIN_FREQ_HZ +
            (G_CONTEST_MAX_FREQ_HZ - G_CONTEST_MIN_FREQ_HZ) *
            (float)i / (float)check_intervals;
        const float magnitude =
            info->digital_magnitude_response(frequency_hz);
        const float phase_rad =
            info->digital_phase_response_rad(frequency_hz);

        if (!(magnitude > 0.0f) || !(magnitude <= FLT_MAX) ||
            !(phase_rad >= -FLT_MAX) || !(phase_rad <= FLT_MAX)) {
            return 0U;
        }
    }
    return 1U;
}

static void GContest_PrintCaptureDiagnostic(SignalAdc_Status status)
{
    const SignalAdc_Info *info = SignalAdc_GetInfo();
    const SignalAdc_Diagnostic *diagnostic =
        SignalAdc_GetDiagnostic();

    printf("[G] capture failed code=%d name=%s native=%lu stage=%s\r\n",
           (int)status,
           SignalAdc_StatusName(status),
           (diagnostic != NULL) ?
               (unsigned long)diagnostic->native_status : 0UL,
           (diagnostic != NULL) ?
               SignalAdc_StageName(diagnostic->stage) : "UNKNOWN");
    if ((info != NULL) && (diagnostic != NULL)) {
        printf("[G] adc=%s device=0x%08lX fw=0x%08lX "
               "response_profile=0x%08lX "
               "physical_clk=%luHz ready=%u busy=%u done=%u "
               "captured=%lu read=%lu\r\n",
               info->name,
               (unsigned long)diagnostic->device_id,
               (unsigned long)diagnostic->firmware_version,
               (unsigned long)diagnostic->response_profile,
               (unsigned long)info->physical_sample_clock_hz,
               (unsigned)diagnostic->ready,
               (unsigned)diagnostic->busy,
               (unsigned)diagnostic->done,
               (unsigned long)diagnostic->captured_count,
               (unsigned long)diagnostic->read_count);
    }
    if (status == SIGNAL_ADC_E_LINK) {
        printf("[G] LINK check: inspect active ADC/FPGA transport "
               "and common GND\r\n");
    }
}

static uint8_t GContest_NormalizeWaveformCycles(uint8_t waveform_cycles)
{
    return (waveform_cycles == G_CONTEST_MAX_WAVEFORM_CYCLES) ?
           G_CONTEST_MAX_WAVEFORM_CYCLES : 1U;
}

static void GContest_PrintWaveform(uint8_t waveform_cycles)
{
    const uint8_t cycles =
        GContest_NormalizeWaveformCycles(waveform_cycles);
    const uint32_t total_points =
        (uint32_t)cycles * G_CONTEST_WAVEFORM_POINTS_PER_CYCLE;

    printf("[WAVE] begin cycles=%u points_per_cycle=%lu total_points=%lu "
           "unit=mV\r\n",
           (unsigned)cycles,
           (unsigned long)G_CONTEST_WAVEFORM_POINTS_PER_CYCLE,
           (unsigned long)total_points);

    for (uint8_t cycle = 0U; cycle < cycles; ++cycle) {
        for (uint32_t point = 0U;
             point < G_CONTEST_WAVEFORM_POINTS_PER_CYCLE;
             ++point) {
            const float phase_turns =
                (float)point /
                (float)G_CONTEST_WAVEFORM_POINTS_PER_CYCLE;
            const float voltage_mv = 1000.0f *
                PeriodicAnalyzer_Evaluate(&s_result, phase_turns);

            printf("%.6f\r\n", (double)voltage_mv);
        }
    }
    printf("[WAVE] end\r\n");
}

static void GContest_PrintMeasurementAndSpectrum(void)
{
    printf("[MEAS] Vpp=%.3fmV RMS=%.3fmV F0=%.2fHz "
           "residual_RMS=%.3fmV clip=%u\r\n",
           (double)(1000.0f * s_result.vpp_v),
           (double)(1000.0f * s_result.true_rms_v),
           (double)s_result.fundamental_hz,
           (double)(1000.0f * s_result.residual_rms_v),
           (unsigned)s_result.clipped);
    printf("[REJECT] expected=%.0fHz detected=%u frequency=%.2fHz "
           "post_dsp_peak=%.3fmV post_dsp_rms=%.3fmV\r\n",
           (double)G_CONTEST_INTERFERENCE_FREQ_HZ,
           (unsigned)s_result.interference_detected,
           (double)s_result.interference_frequency_hz,
           (double)(1000.0f *
                    s_result.interference_post_dsp_peak_adc_v),
           (double)(1000.0f *
                    s_result.interference_post_dsp_rms_adc_v));

    printf("[SPEC] begin band=%.0f..%.0fHz count=%u "
           "columns=component,frequency_Hz,peak_mV,vpp_mV "
           "order=frequency_ascending\r\n",
           (double)G_CONTEST_MIN_FREQ_HZ,
           (double)G_CONTEST_MAX_FREQ_HZ,
           (unsigned)s_result.spectrum_component_count);
    for (uint8_t i = 0U;
         i < s_result.spectrum_component_count;
         ++i) {
        if (s_result.spectrum[i].harmonic_order != 0U) {
            printf("SPEC,H%u,%.2f,%.6f,%.6f\r\n",
                   (unsigned)s_result.spectrum[i].harmonic_order,
                   (double)s_result.spectrum[i].frequency_hz,
                   (double)(1000.0f *
                            s_result.spectrum[i].peak_amplitude_v),
                   (double)(2000.0f *
                            s_result.spectrum[i].peak_amplitude_v));
        } else {
            printf("SPEC,C%u,%.2f,%.6f,%.6f\r\n",
                   (unsigned)(i + 1U),
                   (double)s_result.spectrum[i].frequency_hz,
                   (double)(1000.0f *
                            s_result.spectrum[i].peak_amplitude_v),
                   (double)(2000.0f *
                            s_result.spectrum[i].peak_amplitude_v));
        }
    }
    printf("[SPEC] end\r\n");
}

HAL_StatusTypeDef GContest_Init(void)
{
    const SignalAdc_Info *adc_info;
    SignalAdc_Status adc_status;

    s_backend_ready = 0U;
    adc_status = SignalAdc_Init();
    if (adc_status != SIGNAL_ADC_OK) {
        GContest_PrintCaptureDiagnostic(adc_status);
        return HAL_ERROR;
    }
    adc_info = SignalAdc_GetInfo();
    if ((adc_info == NULL) ||
        !(adc_info->output_sample_rate_hz > 0.0f) ||
        !(adc_info->ac_amplitude_scale > 0.0f) ||
        !(adc_info->ac_amplitude_scale <= FLT_MAX) ||
        (adc_info->max_samples < G_CONTEST_SAMPLE_COUNT) ||
        (GContest_DigitalResponseValid(adc_info) == 0U) ||
        (G_CONTEST_MAX_FREQ_HZ >
         0.5f * adc_info->output_sample_rate_hz)) {
        printf("[G] ADC backend has no valid deterministic response "
               "model across the G band\r\n");
        return HAL_ERROR;
    }

    PeriodicAnalyzer_GetDefaultConfig(&s_analyzer_config);
    s_analyzer_config.sample_rate_hz =
        adc_info->output_sample_rate_hz;
    s_analyzer_config.min_frequency_hz = G_CONTEST_MIN_FREQ_HZ;
    s_analyzer_config.max_frequency_hz = G_CONTEST_MAX_FREQ_HZ;
    s_analyzer_config.input_volts_per_adc_volt =
        adc_info->ac_amplitude_scale;
    s_analyzer_config.min_component_peak_v =
        G_CONTEST_MIN_COMPONENT_PEAK_V;
    s_analyzer_config.relative_peak_threshold = 0.01f;
    s_analyzer_config.digital_magnitude_response =
        adc_info->digital_magnitude_response;
    s_analyzer_config.digital_phase_response_rad =
        adc_info->digital_phase_response_rad;
    s_analyzer_config.interference_rejection_enable = 1U;
    s_analyzer_config.interference_expected_frequency_hz =
        G_CONTEST_INTERFERENCE_FREQ_HZ;
    s_analyzer_config.interference_search_half_width_hz =
        G_CONTEST_INTERFERENCE_SEARCH_HZ;
    s_analyzer_config.interference_minimum_rms_adc_v =
        G_CONTEST_INTERFERENCE_MIN_RMS_V;
    /* Empirical calibration stays disabled until the full system is ready. */
    s_analyzer_config.cal_count = 0U;

    memset(&s_result, 0, sizeof(s_result));
    s_frame_number = 0U;
    s_last_latency_ms = 0U;

    printf("[G] READY ADC=%s physicalFs=%luHz Fs=%.0fHz "
           "N=%lu df=%.5fHz\r\n",
           adc_info->name,
           (unsigned long)adc_info->physical_sample_clock_hz,
           (double)adc_info->output_sample_rate_hz,
           (unsigned long)G_CONTEST_SAMPLE_COUNT,
           (double)(adc_info->output_sample_rate_hz /
                    (float)G_CONTEST_SAMPLE_COUNT));
    printf("[G] ADC physical_bits=%u physical_lsb_nominal=%.6fmV "
           "transport=%s transport_scale=%.9fmV/Q "
           "transport_offset=%.6fmV\r\n",
           (unsigned)adc_info->resolution_bits,
           (double)(1000.0f * adc_info->nominal_lsb_v),
           adc_info->transport_format,
           (double)(1000.0f * adc_info->transport_lsb_v),
           (double)(1000.0f * adc_info->transport_offset_v));
    printf("[G] empirical_frequency_calibration=OFF "
           "analytic_response_compensation=ON "
           "voltage_transfer=%s calibrated=%u source=%s\r\n",
           (adc_info->voltage_transfer_model != NULL) ?
               adc_info->voltage_transfer_model : "unspecified",
           (unsigned)adc_info->voltage_calibrated,
           (adc_info->voltage_calibration_source != NULL) ?
               adc_info->voltage_calibration_source : "none");
    printf("[G] ac_amplitude_scale=%.9f source=%s "
           "frequency_table=OFF\r\n",
           (double)adc_info->ac_amplitude_scale,
           (adc_info->ac_amplitude_calibration_source != NULL) ?
               adc_info->ac_amplitude_calibration_source : "none");
    printf("[G] digital_path=%s response_model=%s "
           "model_band=%.0f..%.0fHz "
           "external_antialias=%u\r\n",
           adc_info->amplitude_path,
           adc_info->digital_response_model,
           (double)adc_info->digital_response_min_hz,
           (double)adc_info->digital_response_max_hz,
           (unsigned)adc_info->requires_external_antialias);
    printf("[G] interference_rejection=joint_model expected=%.0fHz "
           "search=+/-%.0fHz min_post_dsp_rms=%.3fmV\r\n",
           (double)G_CONTEST_INTERFERENCE_FREQ_HZ,
           (double)G_CONTEST_INTERFERENCE_SEARCH_HZ,
           (double)(1000.0f * G_CONTEST_INTERFERENCE_MIN_RMS_V));
    s_backend_ready = 1U;
    return HAL_OK;
}

uint8_t GContest_IsReady(void)
{
    return s_backend_ready;
}

void GContest_Process(uint8_t waveform_cycles)
{
    SignalAdc_FrameInfo frame;
    SignalAdc_Status adc_status = SIGNAL_ADC_E_UNKNOWN;
    PeriodicAnalyzer_Status analyzer_status;
    uint32_t start_ms = HAL_GetTick();

    if (s_backend_ready == 0U) {
        printf("[G] measurement rejected: FPGA/ADC backend is not ready\r\n");
        return;
    }

    for (uint32_t attempt = 1U;
         attempt <= G_CONTEST_CAPTURE_MAX_ATTEMPTS;
         ++attempt) {
        adc_status = SignalAdc_Capture(s_input_volts,
                                      G_CONTEST_SAMPLE_COUNT,
                                      G_CONTEST_SAMPLE_COUNT,
                                      G_CONTEST_CAPTURE_TIMEOUT_MS,
                                      &frame);
        if (adc_status == SIGNAL_ADC_OK) break;
        if ((adc_status != SIGNAL_ADC_E_LINK) ||
            (attempt >= G_CONTEST_CAPTURE_MAX_ATTEMPTS)) {
            break;
        }

        {
            const SignalAdc_Diagnostic *diagnostic =
                SignalAdc_GetDiagnostic();
            printf("[G] transient LINK error stage=%s; "
                   "retry %lu/%lu\r\n",
                   (diagnostic != NULL) ?
                       SignalAdc_StageName(diagnostic->stage) : "UNKNOWN",
                   (unsigned long)attempt,
                   (unsigned long)(G_CONTEST_CAPTURE_MAX_ATTEMPTS - 1U));
        }
        (void)SignalAdc_Abort();
        HAL_Delay(G_CONTEST_CAPTURE_RETRY_DELAY_MS);
    }
    if (adc_status != SIGNAL_ADC_OK) {
        if (GContest_StatusInvalidatesBackend(adc_status) != 0U) {
            s_backend_ready = 0U;
        }
        GContest_PrintCaptureDiagnostic(adc_status);
        return;
    }

    if ((frame.sample_count != G_CONTEST_SAMPLE_COUNT) ||
        !(frame.sample_rate_hz > 0.0f) ||
        (G_CONTEST_MAX_FREQ_HZ > 0.5f * frame.sample_rate_hz)) {
        printf("[G] incomplete frame\r\n");
        return;
    }
    s_analyzer_config.sample_rate_hz = frame.sample_rate_hz;

    analyzer_status = PeriodicAnalyzer_Analyze(s_input_volts,
                                                G_CONTEST_SAMPLE_COUNT,
                                                &s_analyzer_config,
                                                &s_result);
    s_result.clipped = frame.clipped;
    s_frame_number++;
    s_last_latency_ms = HAL_GetTick() - start_ms;

    if (analyzer_status == PERIODIC_ANALYZER_OK) {
        printf("[G] frame=%lu capture_analyze=%lums model_lines=%u\r\n",
               (unsigned long)s_frame_number,
               (unsigned long)s_last_latency_ms,
               (unsigned)s_result.component_count);
        GContest_PrintWaveform(waveform_cycles);
        GContest_PrintMeasurementAndSpectrum();
        printf("[DONE] frame=%lu\r\n", (unsigned long)s_frame_number);
    } else {
        printf("[G] analyzer status=%d latency=%lums\r\n",
               (int)analyzer_status,
               (unsigned long)s_last_latency_ms);
    }
}

HAL_StatusTypeDef GContest_CaptureCalibrationRaw(void)
{
    const SignalAdc_Info *info = SignalAdc_GetInfo();
    double frame_means[G_CONTEST_CALIBRATION_FRAMES];
    int64_t total_sum = 0;
    uint32_t total_samples = 0U;
    uint32_t total_overrange = 0U;
    int32_t raw_min = 0;
    int32_t raw_max = 0;
    uint8_t any_clipped = 0U;
    uint32_t start_ms = HAL_GetTick();

    if (s_backend_ready == 0U) {
        printf("[CAL] capture rejected: FPGA/ADC backend is not ready\r\n");
        return HAL_ERROR;
    }

    if ((info == NULL) ||
        (G_CONTEST_CALIBRATION_FRAMES == 0U) ||
        (G_CONTEST_CALIBRATION_SAMPLES == 0U) ||
        (G_CONTEST_CALIBRATION_SAMPLES > info->max_samples)) {
        printf("[CAL] invalid backend calibration contract\r\n");
        return HAL_ERROR;
    }

    printf("[CAL] begin frames=%lu samples_per_frame=%lu "
           "transport=%s fraction_bits=%u counts_per_native_code=%.0f\r\n",
           (unsigned long)G_CONTEST_CALIBRATION_FRAMES,
           (unsigned long)G_CONTEST_CALIBRATION_SAMPLES,
           info->transport_format,
           (unsigned)info->transport_fraction_bits,
           (double)info->transport_counts_per_native_code);

    for (uint32_t frame_index = 0U;
         frame_index < G_CONTEST_CALIBRATION_FRAMES;
         ++frame_index) {
        SignalAdc_RawFrameStats stats;
        SignalAdc_Status status = SIGNAL_ADC_E_UNKNOWN;

        for (uint32_t attempt = 1U;
             attempt <= G_CONTEST_CAPTURE_MAX_ATTEMPTS;
             ++attempt) {
            status = SignalAdc_CaptureRawStats(
                G_CONTEST_CALIBRATION_SAMPLES,
                G_CONTEST_CAPTURE_TIMEOUT_MS,
                &stats);
            if (status == SIGNAL_ADC_OK) {
                break;
            }
            if ((status != SIGNAL_ADC_E_LINK) ||
                (attempt >= G_CONTEST_CAPTURE_MAX_ATTEMPTS)) {
                break;
            }
            (void)SignalAdc_Abort();
            HAL_Delay(G_CONTEST_CAPTURE_RETRY_DELAY_MS);
        }
        if (status != SIGNAL_ADC_OK) {
            if (GContest_StatusInvalidatesBackend(status) != 0U) {
                s_backend_ready = 0U;
            }
            GContest_PrintCaptureDiagnostic(status);
            printf("[CAL] aborted at frame=%lu/%lu\r\n",
                   (unsigned long)(frame_index + 1U),
                   (unsigned long)G_CONTEST_CALIBRATION_FRAMES);
            return HAL_ERROR;
        }
        if (stats.sample_count != G_CONTEST_CALIBRATION_SAMPLES) {
            printf("[CAL] incomplete raw-stat frame=%lu count=%lu\r\n",
                   (unsigned long)(frame_index + 1U),
                   (unsigned long)stats.sample_count);
            return HAL_ERROR;
        }

        frame_means[frame_index] =
            (double)stats.sample_sum / (double)stats.sample_count;
        total_sum += stats.sample_sum;
        total_samples += stats.sample_count;
        total_overrange += stats.overrange_count;
        any_clipped |= stats.clipped;
        if (frame_index == 0U) {
            raw_min = stats.min_sample;
            raw_max = stats.max_sample;
        } else {
            if (stats.min_sample < raw_min) raw_min = stats.min_sample;
            if (stats.max_sample > raw_max) raw_max = stats.max_sample;
        }

        printf("CAL_FRAME,%lu,Qmean=%.6f,Qmin=%ld,Qmax=%ld,"
               "overrange=%lu,clip=%u\r\n",
               (unsigned long)(frame_index + 1U),
               frame_means[frame_index],
               (long)stats.min_sample,
               (long)stats.max_sample,
               (unsigned long)stats.overrange_count,
               (unsigned)stats.clipped);
    }

    for (uint32_t i = 1U; i < G_CONTEST_CALIBRATION_FRAMES; ++i) {
        const double value = frame_means[i];
        uint32_t j = i;
        while ((j > 0U) && (frame_means[j - 1U] > value)) {
            frame_means[j] = frame_means[j - 1U];
            --j;
        }
        frame_means[j] = value;
    }

    {
        const uint32_t middle = G_CONTEST_CALIBRATION_FRAMES / 2U;
        const double median_q =
            ((G_CONTEST_CALIBRATION_FRAMES & 1U) != 0U) ?
                frame_means[middle] :
                0.5 * (frame_means[middle - 1U] +
                       frame_means[middle]);
        const double pooled_mean_q =
            (double)total_sum / (double)total_samples;
        const double frame_span_q =
            frame_means[G_CONTEST_CALIBRATION_FRAMES - 1U] -
            frame_means[0U];

        printf("[CAL] RESULT Q=%.6f method=median_of_frame_means "
               "pooled_Q=%.6f frame_span_Q=%.6f\r\n",
               median_q,
               pooled_mean_q,
               frame_span_q);
        printf("[CAL] raw_min_Q=%ld raw_max_Q=%ld samples=%lu "
               "overrange=%lu clip=%u elapsed=%lums\r\n",
               (long)raw_min,
               (long)raw_max,
               (unsigned long)total_samples,
               (unsigned long)total_overrange,
               (unsigned)any_clipped,
               (unsigned long)(HAL_GetTick() - start_ms));
        printf("[CAL] use the RESULT Q with the voltage measured at the "
               "ADC input; no volts/Q coefficient was applied\r\n");
    }
    return HAL_OK;
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
