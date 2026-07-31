#ifndef PERIODIC_ANALYZER_H
#define PERIODIC_ANALYZER_H

#include <stdint.h>

#define PERIODIC_ANALYZER_MAX_COMPONENTS    3U
#define PERIODIC_ANALYZER_MAX_SPECTRUM_COMPONENTS 12U
#define PERIODIC_ANALYZER_MAX_CAL_POINTS    8U
#define PERIODIC_ANALYZER_MAX_FFT_POINTS    4096U
#define PERIODIC_ANALYZER_CIC3_R25_DECIMATION 25U
/* Nearest float to 274878*15625/2^32, matching the signed-Q4 RTL. */
#define PERIODIC_ANALYZER_CIC3_R25_RTL_GAIN_SCALE 1.0000003576278687f
#define PERIODIC_ANALYZER_CIC3_R25_DELAY_INPUT_SAMPLES 38.0f
#define PERIODIC_ANALYZER_FIR307_DELAY_INPUT_SAMPLES 153.0f
#define PERIODIC_ANALYZER_AD9238_TOTAL_DELAY_INPUT_SAMPLES 191.0f
/* Accept the measured endpoint despite sub-hertz estimator rounding. */
#define PERIODIC_ANALYZER_CAL_EDGE_GUARD_HZ 5.0f

typedef enum {
    PERIODIC_ANALYZER_OK = 0,
    PERIODIC_ANALYZER_E_ARGUMENT = -1,
    PERIODIC_ANALYZER_E_FFT = -2,
    PERIODIC_ANALYZER_E_NO_SIGNAL = -3,
    PERIODIC_ANALYZER_E_MODEL = -4
} PeriodicAnalyzer_Status;

typedef enum {
    PERIODIC_ANALYZER_EMPIRICAL_UNCALIBRATED = 0,
    PERIODIC_ANALYZER_EMPIRICAL_TABLE = 1,
    PERIODIC_ANALYZER_EMPIRICAL_INFERRED_HOLD = 2
} PeriodicAnalyzer_EmpiricalStatus;

typedef struct {
    float frequency_hz;
    /*
     * 实测整链路的剩余幅值修正，叠加在确定性的 CIC3/R25 理论逆补偿之后。
     * 例如理论补偿后仍低 2%，此处填写 1/0.98。
     */
    float amplitude_correction;
    /*
     * 输入相位 = ADC 实测相位 + CIC 理论相位修正
     *            + phase_correction_deg。
     * 此处只填写信号源实测得到的模拟前端、PCB 和连接线剩余误差。
     * 表内相位应保持展开连续，例如 143° 后写 188°，不要写 -172°。
     */
    float phase_correction_deg;
} PeriodicAnalyzer_CalPoint;

typedef struct {
    float sample_rate_hz;
    float min_frequency_hz;
    float max_frequency_hz;
    float input_volts_per_adc_volt;
    float min_component_peak_v;
    float relative_peak_threshold;
    float cic_input_sample_rate_hz;
    float empirical_hold_last_until_hz;
    uint8_t fir307_antialias_compensation_enabled;
    uint8_t cic3_r25_compensation_enabled;
    uint8_t cal_count;
    PeriodicAnalyzer_CalPoint cal[PERIODIC_ANALYZER_MAX_CAL_POINTS];
} PeriodicAnalyzer_Config;

typedef struct {
    uint16_t harmonic_order;
    float frequency_hz;
    float peak_amplitude_v;
    float phase_rad;
} PeriodicAnalyzer_Component;

typedef struct {
    uint16_t harmonic_order;
    float frequency_hz;
    float peak_amplitude_v;
} PeriodicAnalyzer_SpectrumComponent;

typedef struct {
    PeriodicAnalyzer_Status status;
    uint8_t valid;
    uint8_t component_count;
    uint8_t spectrum_component_count;
    uint8_t clipped;
    float fundamental_hz;
    float vpp_v;
    float true_rms_v;
    float residual_rms_v;
    PeriodicAnalyzer_Component component[PERIODIC_ANALYZER_MAX_COMPONENTS];
    PeriodicAnalyzer_SpectrumComponent
        spectrum[PERIODIC_ANALYZER_MAX_SPECTRUM_COMPONENTS];
} PeriodicAnalyzer_Result;

void PeriodicAnalyzer_GetDefaultConfig(PeriodicAnalyzer_Config *config);
void PeriodicAnalyzer_EnableCic3R25Compensation(
    PeriodicAnalyzer_Config *config,
    float input_sample_rate_hz);
void PeriodicAnalyzer_EnableAd9238DigitalCompensation(
    PeriodicAnalyzer_Config *config,
    float input_sample_rate_hz);
void PeriodicAnalyzer_GetFir307Correction(
    float frequency_hz,
    float input_sample_rate_hz,
    float *amplitude_correction,
    float *phase_correction_rad);
void PeriodicAnalyzer_GetCic3R25Correction(
    float frequency_hz,
    float input_sample_rate_hz,
    float *amplitude_correction,
    float *phase_correction_rad);
void PeriodicAnalyzer_GetAd9238DigitalCorrection(
    float frequency_hz,
    float input_sample_rate_hz,
    float *amplitude_correction,
    float *phase_correction_rad);
/*
 * Returns the exact total correction used by Analyze. Empirical calibration
 * is interpolated only inside the measured table span. A caller may explicitly
 * hold the last measured value to empirical_hold_last_until_hz; status then
 * reports INFERRED_HOLD rather than presenting that range as measured.
 */
void PeriodicAnalyzer_GetAppliedCorrection(
    const PeriodicAnalyzer_Config *config,
    float frequency_hz,
    float *amplitude_correction,
    float *phase_correction_rad,
    float *empirical_amplitude_correction,
    PeriodicAnalyzer_EmpiricalStatus *empirical_status);
PeriodicAnalyzer_Status PeriodicAnalyzer_Analyze(
    const float *adc_volts,
    uint32_t length,
    const PeriodicAnalyzer_Config *config,
    PeriodicAnalyzer_Result *result);

/* phase_turns=0..1 表示一个基波周期中的位置，返回输入端重建电压。 */
float PeriodicAnalyzer_Evaluate(const PeriodicAnalyzer_Result *result,
                                float phase_turns);

#endif /* PERIODIC_ANALYZER_H */
