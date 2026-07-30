#include "g_contest_app.h"

#include "g_contest_config.h"
#include "signal_adc.h"
#include <stdio.h>
#include <string.h>

static float s_input_volts[G_CONTEST_SAMPLE_COUNT];
static PeriodicAnalyzer_Config s_analyzer_config;
static PeriodicAnalyzer_Result s_result;
static uint32_t s_frame_number;
static uint32_t s_last_latency_ms;

/*
 * Residual whole-chain calibration template after CIC3/R25 theory correction.
 * Replace amplitude=1 and phase=0 with signal-source measurements, then set
 * G_CONTEST_EMPIRICAL_CAL_POINT_COUNT to the number of valid ascending points.
 */
static const PeriodicAnalyzer_CalPoint s_empirical_chain_calibration[
    PERIODIC_ANALYZER_MAX_CAL_POINTS] = {
    { 10000.0f, 1.0f, 0.0f},
    { 50000.0f, 1.0f, 0.0f},
    {100000.0f, 1.0f, 0.0f},
    {200000.0f, 1.0f, 0.0f},
    {300000.0f, 1.0f, 0.0f},
    {400000.0f, 1.0f, 0.0f},
    {450000.0f, 1.0f, 0.0f},
    {500000.0f, 1.0f, 0.0f}
};

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
               "physical_clk=%luHz ready=%u busy=%u done=%u "
               "captured=%lu read=%lu\r\n",
               info->name,
               (unsigned long)diagnostic->device_id,
               (unsigned long)diagnostic->firmware_version,
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
    const SignalAdc_Diagnostic *adc_diagnostic;
    SignalAdc_Status adc_status;
    float max_cic_amplitude_correction;
    float max_cic_phase_correction_rad;

    adc_status = SignalAdc_Init();
    if (adc_status != SIGNAL_ADC_OK) {
        GContest_PrintCaptureDiagnostic(adc_status);
        return HAL_ERROR;
    }
    adc_info = SignalAdc_GetInfo();
    adc_diagnostic = SignalAdc_GetDiagnostic();
    if ((adc_info == NULL) ||
        !(adc_info->output_sample_rate_hz > 0.0f) ||
        (adc_info->max_samples < G_CONTEST_SAMPLE_COUNT) ||
        (G_CONTEST_MAX_FREQ_HZ >
         0.5f * adc_info->output_sample_rate_hz)) {
        printf("[G] ADC backend parameters are incompatible with G test\r\n");
        return HAL_ERROR;
    }

    PeriodicAnalyzer_GetDefaultConfig(&s_analyzer_config);
    s_analyzer_config.sample_rate_hz =
        adc_info->output_sample_rate_hz;
    s_analyzer_config.min_frequency_hz = G_CONTEST_MIN_FREQ_HZ;
    s_analyzer_config.max_frequency_hz = G_CONTEST_MAX_FREQ_HZ;
    s_analyzer_config.input_volts_per_adc_volt = 1.0f;
    s_analyzer_config.min_component_peak_v =
        G_CONTEST_MIN_COMPONENT_PEAK_V;
    s_analyzer_config.relative_peak_threshold = 0.01f;
    PeriodicAnalyzer_EnableCic3R25Compensation(
        &s_analyzer_config,
        (float)adc_info->physical_sample_clock_hz);
    /*
     * Layer 1 is deterministic CIC3/R25 inverse compensation.  Layer 2 stays
     * empty until the complete AD9238 front end, PCB and cable are measured
     * with a calibrated 10..500 kHz source.
     */
    s_analyzer_config.cal_count =
        (uint8_t)G_CONTEST_EMPIRICAL_CAL_POINT_COUNT;
    memcpy(s_analyzer_config.cal,
           s_empirical_chain_calibration,
           (size_t)s_analyzer_config.cal_count *
           sizeof(s_analyzer_config.cal[0]));
    PeriodicAnalyzer_GetCic3R25Correction(
        G_CONTEST_MAX_FREQ_HZ,
        (float)adc_info->physical_sample_clock_hz,
        &max_cic_amplitude_correction,
        &max_cic_phase_correction_rad);

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
    printf("[G] ADC physical_bits=%u physical_lsb=%.6fmV "
           "transport=%s transport_lsb=%.6fmV\r\n",
           (unsigned)adc_info->resolution_bits,
           (double)(1000.0f * adc_info->nominal_lsb_v),
           adc_info->transport_format,
           (double)(1000.0f * adc_info->transport_lsb_v));
    if (adc_diagnostic != NULL) {
        printf("[G] FPGA device=0x%08lX protocol=0x%08lX "
               "sample_word=S%uQ%u ready=%u\r\n",
               (unsigned long)adc_diagnostic->device_id,
               (unsigned long)adc_diagnostic->firmware_version,
               (unsigned)adc_diagnostic->sample_word_bits,
               (unsigned)adc_diagnostic->sample_fraction_bits,
               (unsigned)adc_diagnostic->ready);
    }
    printf("[G] CIC3_R25_theory=ON at %.0fHz amp_x=%.6f "
           "phase_deg=%.3f empirical_chain_cal_points=%u\r\n",
           (double)G_CONTEST_MAX_FREQ_HZ,
           (double)max_cic_amplitude_correction,
           (double)(max_cic_phase_correction_rad *
                    (180.0f / 3.14159265358979323846f)),
           (unsigned)s_analyzer_config.cal_count);
    printf("[G] voltage_transfer=nominal; run 10..500kHz source "
           "calibration before claiming full-chain accuracy\r\n");
    return HAL_OK;
}

void GContest_Process(uint8_t waveform_cycles)
{
    SignalAdc_FrameInfo frame;
    SignalAdc_Status adc_status = SIGNAL_ADC_E_UNKNOWN;
    PeriodicAnalyzer_Status analyzer_status;
    uint32_t start_ms = HAL_GetTick();

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
