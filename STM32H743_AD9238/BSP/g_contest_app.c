#include "g_contest_app.h"

#include "component_alias_guard.h"
#include "g_contest_config.h"
#include "signal_adc.h"
#include <stdio.h>
#include <string.h>

#define G_CONTEST_PI 3.14159265358979323846f

static float s_input_volts[G_CONTEST_SAMPLE_COUNT];
static float s_reference_volts[G_CONTEST_SAMPLE_COUNT];
static PeriodicAnalyzer_Config s_analyzer_config;
static PeriodicAnalyzer_Config s_reference_config;
static PeriodicAnalyzer_Result s_result;
static PeriodicAnalyzer_Result s_reference_result;
static ComponentAliasGuard_Report s_alias_report;
static uint32_t s_frame_number;
static uint32_t s_last_latency_ms;
static uint32_t s_last_capture_ms;
static uint32_t s_last_ch1_analysis_ms;
static uint32_t s_last_ch2_spectrum_ms;
static uint32_t s_last_ch2_full_analysis_ms;
static float s_raw_vpp_v;
static G_CalibrationData s_calibration;
static BSP_StorageStatus s_calibration_storage_status = BSP_STORAGE_ERROR;

static const char *GContest_ResultSourceName(void)
{
    if (s_alias_report.mode == COMPONENT_ALIAS_GUARD_USE_CH2) {
        return "CH2_FILTERED_REFERENCE";
    }
    if (s_alias_report.mode == COMPONENT_ALIAS_GUARD_REJECT) {
        return "REJECTED_ALIAS";
    }
    return "CH1_CALIBRATED";
}

/*
 * Residual whole-chain calibration after the exact FIR307+CIC3/R25 inverse.
 * The eight amplitude factors are a least-squares fit of the A/B acceptance
 * captures after changing the measured front-end gain to 9.55. B02 is omitted
 * because its successful capture used 10.2 kHz instead of the 10.0 kHz target;
 * the C-group interference cases are validation data and are not fitted.
 * Phase remains zero because the measured relative phase errors were already
 * negligible and no independent phase references were supplied.
 */
static const float s_calibration_frequency_hz[
    G_CALIBRATION_POINT_COUNT] = {
     10000.0f, 100000.0f, 125000.0f, 150000.0f,
    200000.0f, 250000.0f, 400000.0f, 500000.0f
};

static const G_CalibrationData s_default_calibration = {
    G_CONTEST_DEFAULT_GAIN_MILLI,
    {
        1001023U, 999632U, 958008U, 959032U,
        962626U, 963263U, 960504U, 963778U
    }
};

typedef char GContestCalibrationPointCountMustMatch[
    (G_CALIBRATION_POINT_COUNT ==
     G_CONTEST_EMPIRICAL_CAL_POINT_COUNT) ? 1 : -1];

uint8_t GContest_IsCalibrationValid(const G_CalibrationData *data)
{
    if ((data == NULL) ||
        (data->gain_milli < G_CONTEST_GAIN_MIN_MILLI) ||
        (data->gain_milli > G_CONTEST_GAIN_MAX_MILLI)) {
        return 0U;
    }
    for (uint32_t i = 0U; i < G_CALIBRATION_POINT_COUNT; ++i) {
        if ((data->coefficient_u1e6[i] <
             G_CONTEST_COEFFICIENT_MIN_U1E6) ||
            (data->coefficient_u1e6[i] >
             G_CONTEST_COEFFICIENT_MAX_U1E6)) {
            return 0U;
        }
    }
    return 1U;
}

static void GContest_ApplyCalibration(void)
{
    s_analyzer_config.input_volts_per_adc_volt =
        1000.0f / (float)s_calibration.gain_milli;
    s_analyzer_config.cal_count = G_CALIBRATION_POINT_COUNT;
    for (uint32_t i = 0U; i < G_CALIBRATION_POINT_COUNT; ++i) {
        s_analyzer_config.cal[i].frequency_hz =
            s_calibration_frequency_hz[i];
        s_analyzer_config.cal[i].amplitude_correction =
            (float)s_calibration.coefficient_u1e6[i] / 1000000.0f;
        s_analyzer_config.cal[i].phase_correction_deg = 0.0f;
    }
    s_analyzer_config.empirical_hold_last_until_hz =
        G_CONTEST_MAX_FREQ_HZ;
}

static const char *GContest_EmpiricalStatusName(
    PeriodicAnalyzer_EmpiricalStatus status)
{
    if (status == PERIODIC_ANALYZER_EMPIRICAL_TABLE) {
        return "TABLE";
    }
    if (status == PERIODIC_ANALYZER_EMPIRICAL_INFERRED_HOLD) {
        return "INFERRED_HOLD";
    }
    return "UNCAL";
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

static float GContest_WrapPhaseRad(float phase_rad)
{
    const float two_pi = 2.0f * G_CONTEST_PI;
    while (phase_rad > G_CONTEST_PI) phase_rad -= two_pi;
    while (phase_rad <= -G_CONTEST_PI) phase_rad += two_pi;
    return phase_rad;
}

static float GContest_ComputeRawVppV(const float *samples,
                                     uint32_t sample_count)
{
    float minimum;
    float maximum;

    if ((samples == NULL) || (sample_count == 0U)) return 0.0f;
    minimum = samples[0];
    maximum = samples[0];
    for (uint32_t i = 1U; i < sample_count; ++i) {
        if (samples[i] < minimum) minimum = samples[i];
        if (samples[i] > maximum) maximum = samples[i];
    }
    return maximum - minimum;
}

static void GContest_PrintPhaseDiagnostics(void)
{
    float fundamental_phase_rad = 0.0f;
    uint8_t fundamental_phase_valid = 0U;

    for (uint8_t i = 0U; i < s_result.component_count; ++i) {
        if (s_result.component[i].harmonic_order == 1U) {
            fundamental_phase_rad = s_result.component[i].phase_rad;
            fundamental_phase_valid = 1U;
            break;
        }
    }

    printf("[PHASE] begin count=%u "
           "columns=component,frequency_Hz,phase_deg,psi_deg "
           "psi=wrap(phi_h-h*phi_1)\r\n",
           (unsigned)s_result.component_count);
    for (uint8_t i = 0U; i < s_result.component_count; ++i) {
        const PeriodicAnalyzer_Component *component =
            &s_result.component[i];
        const float phase_deg =
            component->phase_rad * (180.0f / G_CONTEST_PI);

        if (fundamental_phase_valid != 0U) {
            const float relative_phase_rad = GContest_WrapPhaseRad(
                component->phase_rad -
                (float)component->harmonic_order *
                fundamental_phase_rad);
            const float relative_phase_deg =
                relative_phase_rad * (180.0f / G_CONTEST_PI);
            printf("PHASE,H%u,%.2f,%.6f,%.6f\r\n",
                   (unsigned)component->harmonic_order,
                   (double)component->frequency_hz,
                   (double)phase_deg,
                   (double)relative_phase_deg);
        } else {
            printf("PHASE,H%u,%.2f,%.6f,NA\r\n",
                   (unsigned)component->harmonic_order,
                   (double)component->frequency_hz,
                   (double)phase_deg);
        }
    }
    printf("[PHASE] end\r\n");
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
    const uint8_t using_reference =
        (s_alias_report.mode == COMPONENT_ALIAS_GUARD_USE_CH2) ? 1U : 0U;
    const PeriodicAnalyzer_Config *active_config =
        (using_reference != 0U) ?
        &s_reference_config : &s_analyzer_config;

    printf("[MEAS] Vpp=%.3fmV RMS=%.3fmV F0=%.2fHz "
           "residual_RMS=%.3fmV clip=%u\r\n",
           (double)(1000.0f * s_result.vpp_v),
           (double)(1000.0f * s_result.true_rms_v),
           (double)s_result.fundamental_hz,
           (double)(1000.0f * s_result.residual_rms_v),
           (unsigned)s_result.clipped);
    printf("[VPP] VPP_MODEL=%.6fmV VPP_RAW=%.6fmV "
           "RAW_MINUS_MODEL=%.6fmV "
           "raw_domain=%s\r\n",
           (double)(1000.0f * s_result.vpp_v),
           (double)(1000.0f * s_raw_vpp_v),
           (double)(1000.0f *
                    (s_raw_vpp_v - s_result.vpp_v)),
           (using_reference != 0U) ?
               "post_FIR307_R25_CH2_gain1" :
               "post_FIR307_CIC3_R25_BNC_equivalent");

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
    GContest_PrintPhaseDiagnostics();

    printf("[RESP] begin count=%u "
           "columns=component,frequency_Hz,transport_vpp_mV,"
           "fir_x,cic_x,digital_x,after_digital_vpp_mV,"
           "chain_x,total_x,final_vpp_mV,chain_status\r\n",
           (unsigned)s_result.spectrum_component_count);
    for (uint8_t i = 0U;
         i < s_result.spectrum_component_count;
         ++i) {
        float fir_amplitude_correction;
        float cic_amplitude_correction;
        float empirical_amplitude_correction;
        float total_amplitude_correction;
        float digital_amplitude_correction;
        float transport_vpp_mv;
        float after_digital_vpp_mv;
        float final_vpp_mv;
        PeriodicAnalyzer_EmpiricalStatus empirical_status;

        PeriodicAnalyzer_GetFir307Correction(
            s_result.spectrum[i].frequency_hz,
            active_config->cic_input_sample_rate_hz,
            &fir_amplitude_correction,
            NULL);
        if (using_reference != 0U) {
            cic_amplitude_correction = 1.0f;
        } else {
            PeriodicAnalyzer_GetCic3R25Correction(
                s_result.spectrum[i].frequency_hz,
                active_config->cic_input_sample_rate_hz,
                &cic_amplitude_correction,
                NULL);
        }
        PeriodicAnalyzer_GetAppliedCorrection(
            active_config,
            s_result.spectrum[i].frequency_hz,
            &total_amplitude_correction,
            NULL,
            &empirical_amplitude_correction,
            &empirical_status);
        digital_amplitude_correction =
            fir_amplitude_correction * cic_amplitude_correction;
        final_vpp_mv = 2000.0f *
                       s_result.spectrum[i].peak_amplitude_v;
        transport_vpp_mv =
            (total_amplitude_correction > 0.0f) ?
            (final_vpp_mv / total_amplitude_correction) : 0.0f;
        after_digital_vpp_mv = transport_vpp_mv *
                               digital_amplitude_correction;

        if (s_result.spectrum[i].harmonic_order != 0U) {
            printf("RESP,H%u,%.2f,%.6f,%.9f,%.9f,%.9f,"
                   "%.6f,%.9f,%.9f,%.6f,%s\r\n",
                   (unsigned)s_result.spectrum[i].harmonic_order,
                   (double)s_result.spectrum[i].frequency_hz,
                   (double)transport_vpp_mv,
                   (double)fir_amplitude_correction,
                   (double)cic_amplitude_correction,
                   (double)digital_amplitude_correction,
                   (double)after_digital_vpp_mv,
                   (double)empirical_amplitude_correction,
                   (double)total_amplitude_correction,
                   (double)final_vpp_mv,
                   GContest_EmpiricalStatusName(empirical_status));
        } else {
            printf("RESP,C%u,%.2f,%.6f,%.9f,%.9f,%.9f,"
                   "%.6f,%.9f,%.9f,%.6f,%s\r\n",
                   (unsigned)(i + 1U),
                   (double)s_result.spectrum[i].frequency_hz,
                   (double)transport_vpp_mv,
                   (double)fir_amplitude_correction,
                   (double)cic_amplitude_correction,
                   (double)digital_amplitude_correction,
                   (double)after_digital_vpp_mv,
                   (double)empirical_amplitude_correction,
                   (double)total_amplitude_correction,
                   (double)final_vpp_mv,
                   GContest_EmpiricalStatusName(empirical_status));
        }
    }
    printf("[RESP] end\r\n");
}
HAL_StatusTypeDef GContest_Init(void)
{
    const SignalAdc_Info *adc_info;
    const SignalAdc_Diagnostic *adc_diagnostic;
    SignalAdc_Status adc_status;
    float max_fir_amplitude_correction;
    float max_cic_amplitude_correction;
    float max_digital_amplitude_correction;
    float max_digital_phase_correction_rad;
    float max_empirical_amplitude_correction;
    float max_total_amplitude_correction;
    PeriodicAnalyzer_EmpiricalStatus max_empirical_status;
    G_CalibrationData stored_calibration;
    uint8_t calibration_loaded = 0U;
    const char *calibration_source = "DEFAULT";

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

    s_calibration = s_default_calibration;
    s_calibration_storage_status = G_CalibrationStore_Init(
        &stored_calibration, &calibration_loaded);
    if ((s_calibration_storage_status == BSP_STORAGE_OK) &&
        (calibration_loaded != 0U)) {
        if (GContest_IsCalibrationValid(&stored_calibration) != 0U) {
            s_calibration = stored_calibration;
            calibration_source = "QSPI";
        } else {
            calibration_source = "DEFAULT_INVALID_RECORD";
        }
    }

    PeriodicAnalyzer_GetDefaultConfig(&s_analyzer_config);
    s_analyzer_config.sample_rate_hz =
        adc_info->output_sample_rate_hz;
    s_analyzer_config.min_frequency_hz = G_CONTEST_MIN_FREQ_HZ;
    s_analyzer_config.max_frequency_hz = G_CONTEST_MAX_FREQ_HZ;
    s_analyzer_config.min_component_peak_v =
        G_CONTEST_MIN_COMPONENT_PEAK_V;
    s_analyzer_config.relative_peak_threshold = 0.01f;
    PeriodicAnalyzer_EnableAd9238DigitalCompensation(
        &s_analyzer_config,
        (float)adc_info->physical_sample_clock_hz);
    /*
     * Layer 1 is the exact FIR307 plus CIC3/R25 RTL inverse. Layer 2
     * interpolates the eight measured residual BNC-referred response points.
     */
    GContest_ApplyCalibration();

    /*
     * CH2 is connected after the external 15 MHz low-pass and has no 9.55x
     * amplifier. Scale it 1:1 and compensate only its FPGA FIR307 response;
     * no CH1 empirical calibration is reused.
     */
    PeriodicAnalyzer_GetDefaultConfig(&s_reference_config);
    s_reference_config.sample_rate_hz =
        adc_info->output_sample_rate_hz;
    s_reference_config.min_frequency_hz = G_CONTEST_MIN_FREQ_HZ;
    s_reference_config.max_frequency_hz = G_CONTEST_MAX_FREQ_HZ;
    s_reference_config.input_volts_per_adc_volt = 1.0f;
    s_reference_config.min_component_peak_v =
        G_CONTEST_MIN_COMPONENT_PEAK_V;
    s_reference_config.relative_peak_threshold = 0.01f;
    s_reference_config.cic_input_sample_rate_hz =
        (float)adc_info->physical_sample_clock_hz;
    s_reference_config.fir307_antialias_compensation_enabled = 1U;
    s_reference_config.cic3_r25_compensation_enabled = 0U;
    s_reference_config.cal_count = 0U;
    s_reference_config.empirical_hold_last_until_hz = 0.0f;
    PeriodicAnalyzer_GetFir307Correction(
        G_CONTEST_MAX_FREQ_HZ,
        (float)adc_info->physical_sample_clock_hz,
        &max_fir_amplitude_correction,
        NULL);
    PeriodicAnalyzer_GetCic3R25Correction(
        G_CONTEST_MAX_FREQ_HZ,
        (float)adc_info->physical_sample_clock_hz,
        &max_cic_amplitude_correction,
        NULL);
    PeriodicAnalyzer_GetAd9238DigitalCorrection(
        G_CONTEST_MAX_FREQ_HZ,
        (float)adc_info->physical_sample_clock_hz,
        &max_digital_amplitude_correction,
        &max_digital_phase_correction_rad);
    PeriodicAnalyzer_GetAppliedCorrection(
        &s_analyzer_config,
        G_CONTEST_MAX_FREQ_HZ,
        &max_total_amplitude_correction,
        NULL,
        &max_empirical_amplitude_correction,
        &max_empirical_status);

    memset(&s_result, 0, sizeof(s_result));
    memset(&s_reference_result, 0, sizeof(s_reference_result));
    s_alias_report.mode = COMPONENT_ALIAS_GUARD_KEEP_CH1;
    s_alias_report.removed_spectrum_count = 0U;
    s_frame_number = 0U;
    s_last_latency_ms = 0U;
    s_last_capture_ms = 0U;
    s_last_ch1_analysis_ms = 0U;
    s_last_ch2_spectrum_ms = 0U;
    s_last_ch2_full_analysis_ms = 0U;
    s_raw_vpp_v = 0.0f;

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
    printf("[G] frontend_voltage_gain=%.6f "
           "input_volts_per_adc_volt=%.9f "
           "reference_plane=BNC_before_amplifier\r\n",
           (double)((float)s_calibration.gain_milli / 1000.0f),
           (double)s_analyzer_config.input_volts_per_adc_volt);
    printf("[CAL] storage=W25Q_QSPI status=%s source=%s "
           "gain_milli=%lu points=%u\r\n",
           BSP_Storage_StatusString(s_calibration_storage_status),
           calibration_source,
           (unsigned long)s_calibration.gain_milli,
           (unsigned)G_CALIBRATION_POINT_COUNT);
    if (adc_diagnostic != NULL) {
        printf("[G] FPGA device=0x%08lX protocol=0x%08lX "
               "sample_word=S%uQ%u ready=%u\r\n",
               (unsigned long)adc_diagnostic->device_id,
               (unsigned long)adc_diagnostic->firmware_version,
               (unsigned)adc_diagnostic->sample_word_bits,
               (unsigned)adc_diagnostic->sample_fraction_bits,
               (unsigned)adc_diagnostic->ready);
    }
    printf("[G] FIR307_CIC3_R25_RTL=ON at %.0fHz fir_x=%.9f "
           "cic_x=%.9f digital_x=%.9f phase_deg=%.3f "
           "chain_x=%.9f total_x=%.9f chain_status=%s points=%u\r\n",
           (double)G_CONTEST_MAX_FREQ_HZ,
           (double)max_fir_amplitude_correction,
           (double)max_cic_amplitude_correction,
           (double)max_digital_amplitude_correction,
           (double)(max_digital_phase_correction_rad *
                    (180.0f / 3.14159265358979323846f)),
           (double)max_empirical_amplitude_correction,
           (double)max_total_amplitude_correction,
           GContest_EmpiricalStatusName(max_empirical_status),
           (unsigned)s_analyzer_config.cal_count);
    printf("[G] antialias=FIR307_Q16 passband=0..500000Hz "
           "stopband>=1000000Hz stop_min=52.7dB linear_phase_delay=191clk\r\n");
    printf("[G] alias_guard=CH1_vs_CH2 CH2_gain=1.000000 "
           "CH2_analog_LPF=15MHz match_tolerance=%.0fHz\r\n",
           (double)G_CONTEST_ALIAS_MATCH_TOLERANCE_HZ);
    printf("[G] empirical_response=10000..500000Hz interpolation=ON "
           "extrapolation=OFF points=10/100/125/150/200/250/400/500kHz\r\n");
    return HAL_OK;
}

void GContest_Process(uint8_t waveform_cycles)
{
    SignalAdc_FrameInfo frame;
    SignalAdc_FrameInfo reference_frame;
    SignalAdc_Status adc_status = SIGNAL_ADC_E_UNKNOWN;
    PeriodicAnalyzer_Status analyzer_status;
    PeriodicAnalyzer_Status reference_status =
        PERIODIC_ANALYZER_E_NO_SIGNAL;
    PeriodicAnalyzer_Result ch1_before_guard;
    uint32_t start_ms = HAL_GetTick();
    uint32_t stage_start_ms;

    (void)waveform_cycles;
    s_last_capture_ms = 0U;
    s_last_ch1_analysis_ms = 0U;
    s_last_ch2_spectrum_ms = 0U;
    s_last_ch2_full_analysis_ms = 0U;

    for (uint32_t attempt = 1U;
         attempt <= G_CONTEST_CAPTURE_MAX_ATTEMPTS;
         ++attempt) {
        adc_status = SignalAdc_CaptureDual(
            s_input_volts,
            s_reference_volts,
            G_CONTEST_SAMPLE_COUNT,
            G_CONTEST_SAMPLE_COUNT,
            G_CONTEST_CAPTURE_TIMEOUT_MS,
            &frame,
            &reference_frame);
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
    s_last_capture_ms = HAL_GetTick() - start_ms;

    if ((frame.sample_count != G_CONTEST_SAMPLE_COUNT) ||
        (reference_frame.sample_count != G_CONTEST_SAMPLE_COUNT) ||
        !(frame.sample_rate_hz > 0.0f) ||
        !(reference_frame.sample_rate_hz > 0.0f) ||
        (G_CONTEST_MAX_FREQ_HZ > 0.5f * frame.sample_rate_hz)) {
        printf("[G] incomplete frame\r\n");
        return;
    }
    s_analyzer_config.sample_rate_hz = frame.sample_rate_hz;
    s_reference_config.sample_rate_hz = reference_frame.sample_rate_hz;
    s_raw_vpp_v = GContest_ComputeRawVppV(
        s_input_volts,
        G_CONTEST_SAMPLE_COUNT) *
        s_analyzer_config.input_volts_per_adc_volt;

    stage_start_ms = HAL_GetTick();
    analyzer_status = PeriodicAnalyzer_Analyze(
        s_input_volts,
        G_CONTEST_SAMPLE_COUNT,
        &s_analyzer_config,
        &s_result);
    s_last_ch1_analysis_ms = HAL_GetTick() - stage_start_ms;
    s_result.clipped = frame.clipped;

    if (analyzer_status == PERIODIC_ANALYZER_OK) {
        /*
         * The normal path only needs CH2 peak presence.  Keep a pristine CH1
         * result because the guard may request the uncommon full-CH2 fallback
         * when an unsupported line has entered the harmonic model.
         */
        ch1_before_guard = s_result;
        stage_start_ms = HAL_GetTick();
        reference_status = PeriodicAnalyzer_AnalyzeSpectrumOnly(
            s_reference_volts,
            G_CONTEST_SAMPLE_COUNT,
            &s_reference_config,
            &s_reference_result);
        s_last_ch2_spectrum_ms = HAL_GetTick() - stage_start_ms;
        s_reference_result.clipped = reference_frame.clipped;

        ComponentAliasGuard_Apply(
            &s_result,
            &s_reference_result,
            G_CONTEST_ALIAS_MATCH_TOLERANCE_HZ,
            G_CONTEST_ALIAS_MATERIAL_PEAK_V,
            G_CONTEST_MIN_COMPONENT_PEAK_V,
            G_CONTEST_ALIAS_REFERENCE_MIN_RATIO,
            &s_alias_report);

        if (s_alias_report.mode == COMPONENT_ALIAS_GUARD_USE_CH2) {
            s_result = ch1_before_guard;
            stage_start_ms = HAL_GetTick();
            reference_status = PeriodicAnalyzer_Analyze(
                s_reference_volts,
                G_CONTEST_SAMPLE_COUNT,
                &s_reference_config,
                &s_reference_result);
            s_last_ch2_full_analysis_ms =
                HAL_GetTick() - stage_start_ms;
            s_reference_result.clipped = reference_frame.clipped;
            ComponentAliasGuard_Apply(
                &s_result,
                &s_reference_result,
                G_CONTEST_ALIAS_MATCH_TOLERANCE_HZ,
                G_CONTEST_ALIAS_MATERIAL_PEAK_V,
                G_CONTEST_MIN_COMPONENT_PEAK_V,
                G_CONTEST_ALIAS_REFERENCE_MIN_RATIO,
                &s_alias_report);
        }
    } else {
        /* Preserve the original CH2 fallback when CH1 itself has no model. */
        stage_start_ms = HAL_GetTick();
        reference_status = PeriodicAnalyzer_Analyze(
            s_reference_volts,
            G_CONTEST_SAMPLE_COUNT,
            &s_reference_config,
            &s_reference_result);
        s_last_ch2_full_analysis_ms = HAL_GetTick() - stage_start_ms;
        s_reference_result.clipped = reference_frame.clipped;
        ComponentAliasGuard_Apply(
            &s_result,
            &s_reference_result,
            G_CONTEST_ALIAS_MATCH_TOLERANCE_HZ,
            G_CONTEST_ALIAS_MATERIAL_PEAK_V,
            G_CONTEST_MIN_COMPONENT_PEAK_V,
            G_CONTEST_ALIAS_REFERENCE_MIN_RATIO,
            &s_alias_report);
    }
    analyzer_status = s_result.status;
    if (s_alias_report.mode == COMPONENT_ALIAS_GUARD_USE_CH2) {
        s_raw_vpp_v = GContest_ComputeRawVppV(
            s_reference_volts,
            G_CONTEST_SAMPLE_COUNT);
    }
    s_frame_number++;
    s_last_latency_ms = HAL_GetTick() - start_ms;

    if (analyzer_status != PERIODIC_ANALYZER_OK) {
        printf("[G] analyzer status=%d reference_status=%d "
               "source=%s latency=%lums\r\n",
               (int)analyzer_status,
               (int)reference_status,
               GContest_ResultSourceName(),
               (unsigned long)s_last_latency_ms);
    } else if ((s_alias_report.mode != COMPONENT_ALIAS_GUARD_KEEP_CH1) ||
               (s_alias_report.removed_spectrum_count != 0U)) {
        printf("[ALIAS] source=%s removed_ch1_lines=%u "
               "reference_status=%d\r\n",
               GContest_ResultSourceName(),
               (unsigned)s_alias_report.removed_spectrum_count,
               (int)reference_status);
    }
}

void GContest_PrintLastReport(uint8_t waveform_cycles)
{
    if ((s_result.valid == 0U) ||
        (s_result.status != PERIODIC_ANALYZER_OK)) {
        return;
    }

    printf("[G] frame=%lu capture_analyze=%lums "
           "capture=%lums ch1=%lums ch2_fft=%lums ch2_full=%lums "
           "model_lines=%u source=%s\r\n",
           (unsigned long)s_frame_number,
           (unsigned long)s_last_latency_ms,
           (unsigned long)s_last_capture_ms,
           (unsigned long)s_last_ch1_analysis_ms,
           (unsigned long)s_last_ch2_spectrum_ms,
           (unsigned long)s_last_ch2_full_analysis_ms,
           (unsigned)s_result.component_count,
           GContest_ResultSourceName());
    GContest_PrintWaveform(waveform_cycles);
    GContest_PrintMeasurementAndSpectrum();
    printf("[DONE] frame=%lu\r\n", (unsigned long)s_frame_number);
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

void GContest_GetCalibration(G_CalibrationData *data)
{
    if (data != NULL) *data = s_calibration;
}

HAL_StatusTypeDef GContest_SaveCalibration(const G_CalibrationData *data)
{
    if (GContest_IsCalibrationValid(data) == 0U) return HAL_ERROR;

    s_calibration_storage_status = G_CalibrationStore_Save(data);
    if (s_calibration_storage_status != BSP_STORAGE_OK) {
        printf("[CAL] save failed status=%s\r\n",
               BSP_Storage_StatusString(s_calibration_storage_status));
        return HAL_ERROR;
    }

    s_calibration = *data;
    GContest_ApplyCalibration();
    memset(&s_result, 0, sizeof(s_result));
    memset(&s_reference_result, 0, sizeof(s_reference_result));
    s_alias_report.mode = COMPONENT_ALIAS_GUARD_KEEP_CH1;
    s_alias_report.removed_spectrum_count = 0U;
    s_raw_vpp_v = 0.0f;
    printf("[CAL] saved gain_milli=%lu points=%u\r\n",
           (unsigned long)s_calibration.gain_milli,
           (unsigned)G_CALIBRATION_POINT_COUNT);
    return HAL_OK;
}

HAL_StatusTypeDef GContest_RestoreDefaultCalibration(void)
{
    return GContest_SaveCalibration(&s_default_calibration);
}

uint8_t GContest_IsCalibrationStorageReady(void)
{
    return G_CalibrationStore_IsReady();
}
