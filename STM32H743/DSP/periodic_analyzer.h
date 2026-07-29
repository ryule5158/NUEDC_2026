#ifndef PERIODIC_ANALYZER_H
#define PERIODIC_ANALYZER_H

#include <stdint.h>

#define PERIODIC_ANALYZER_MAX_COMPONENTS    3U
#define PERIODIC_ANALYZER_MAX_SPECTRUM_COMPONENTS 12U
#define PERIODIC_ANALYZER_MAX_CAL_POINTS    8U
#define PERIODIC_ANALYZER_MAX_FFT_POINTS    4096U

typedef enum {
    PERIODIC_ANALYZER_OK = 0,
    PERIODIC_ANALYZER_E_ARGUMENT = -1,
    PERIODIC_ANALYZER_E_FFT = -2,
    PERIODIC_ANALYZER_E_NO_SIGNAL = -3,
    PERIODIC_ANALYZER_E_MODEL = -4
} PeriodicAnalyzer_Status;

typedef struct {
    float frequency_hz;
    float amplitude_correction;
    /*
     * 输入相位 = ADC 实测相位 + phase_correction_deg。
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
PeriodicAnalyzer_Status PeriodicAnalyzer_Analyze(
    const float *adc_volts,
    uint32_t length,
    const PeriodicAnalyzer_Config *config,
    PeriodicAnalyzer_Result *result);

/* phase_turns=0..1 表示一个基波周期中的位置，返回输入端重建电压。 */
float PeriodicAnalyzer_Evaluate(const PeriodicAnalyzer_Result *result,
                                float phase_turns);

#endif /* PERIODIC_ANALYZER_H */
