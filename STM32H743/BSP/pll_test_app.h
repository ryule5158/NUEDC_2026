#ifndef PLL_TEST_APP_H
#define PLL_TEST_APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "adc_dual_capture.h"
#include "main.h"
#include <stdint.h>

typedef enum {
    PLL_TEST_MODE_ACQUIRE = 0,
    PLL_TEST_MODE_AC_LOCK = 1,
    PLL_TEST_MODE_HOLDOVER = 2
} PllTest_Mode_t;

/* AUTO classifies the reference spectrum and selects the matching FPGA
 * waveform generator. Explicit values retain a deterministic field override. */
typedef enum {
    PLL_TEST_WAVE_AUTO = 0,
    PLL_TEST_WAVE_SINE,
    PLL_TEST_WAVE_SQUARE,
    PLL_TEST_WAVE_TRIANGLE,
    PLL_TEST_WAVE_SAWTOOTH,
    PLL_TEST_WAVE_AM,
    PLL_TEST_WAVE_FM,
    PLL_TEST_WAVE_UNKNOWN = 255
} PllTest_Waveform_t;

/* Application-level tuning. Raw PI/PID gains remain optional runtime knobs;
 * loop_bandwidth_hz is the portable setting normally used by a caller. */
typedef struct {
    float target_phase_deg;
    float fixed_phase_cal_deg;
    float loop_bandwidth_hz;
    float acquisition_hint_hz;       /* 0: automatic full-range search. */
    float acquisition_hint_tolerance_hz;
    uint32_t telemetry_period_ms;
    PllTest_Waveform_t requested_waveform; /* AUTO by default. */
} PllTest_Config_t;

typedef struct {
    PllTest_Mode_t mode;
    uint8_t running;
    uint8_t locked;
    uint8_t phase_valid;
    uint8_t frequency_sweep_active;
    float input_frequency_hz;
    float output_frequency_hz;
    float phase_error_deg;
    float phase_rate_hz;
    float target_phase_deg;
    float fixed_phase_cal_deg;
    float loop_bandwidth_hz;
    float acquisition_hint_hz;
    float acquisition_hint_tolerance_hz;
    float alias_frequency_hz;
    float alias_snr_db;
    float harmonic2_relative_db;
    float harmonic3_relative_db;
    float modulation_frequency_hz;
    float am_depth;
    float fm_deviation_hz;
    float modulation_phase_metric;
    float modulation_phase_error_deg;
    uint8_t modulation_phase_valid;
    float reference_iq_amplitude;
    float feedback_iq_amplitude;
    PllTest_Waveform_t requested_waveform;
    PllTest_Waveform_t detected_waveform;
    PllTest_Waveform_t output_waveform;
    /* Kept as dds_* for source compatibility; backend is FPGA/AD9708. */
    uint32_t dds_update_count;
    uint32_t dds_error_count;
    uint32_t dds_last_update_us;
    uint32_t sweep_failure_count;
    uint32_t rate_assist_count;
    uint16_t dds_amplitude_word; /* AD9708 peak amplitude code: 128 = full scale. */
    ADC_DualCapture_Status_t adc;
} PllTest_Status_t;

void PllTest_GetDefaultConfig(PllTest_Config_t *config);
HAL_StatusTypeDef PllTest_InitWithConfig(const PllTest_Config_t *config);
/* Compatibility wrapper: initializes with the defaults above. */
HAL_StatusTypeDef PllTest_Init(void);
void PllTest_Process(void);
void PllTest_Stop(void);

HAL_StatusTypeDef PllTest_SetTargetPhase(float target_phase_deg);
HAL_StatusTypeDef PllTest_SetFixedPhaseCalibration(float calibration_deg);
HAL_StatusTypeDef PllTest_SetLoopBandwidth(float loop_bandwidth_hz);
HAL_StatusTypeDef PllTest_SetLoopGains(float frequency_kp_hz_per_deg,
                                       float frequency_ki_hz_per_deg_s,
                                       float frequency_kd,
                                       float phase_kp);
HAL_StatusTypeDef PllTest_SetTelemetryPeriod(uint32_t period_ms);
HAL_StatusTypeDef PllTest_SetAcquisitionHint(float frequency_hz,
                                              float tolerance_hz);
HAL_StatusTypeDef PllTest_SetWaveform(PllTest_Waveform_t waveform);
const char *PllTest_WaveformName(PllTest_Waveform_t waveform);
HAL_StatusTypeDef PllTest_RequestRescan(void);
void PllTest_GetStatus(PllTest_Status_t *status);

#ifdef __cplusplus
}
#endif
#endif /* PLL_TEST_APP_H */
