#ifndef DUAL_RESPONSE_ANALYZER_H
#define DUAL_RESPONSE_ANALYZER_H

#include <stdint.h>

typedef enum {
    DUAL_RESPONSE_OK = 0,
    DUAL_RESPONSE_E_ARGUMENT = -1,
    DUAL_RESPONSE_E_NO_SIGNAL = -2,
    DUAL_RESPONSE_E_MODEL = -3
} DualResponse_Status;

typedef struct {
    DualResponse_Status status;
    uint8_t valid;
    float frequency_hz;
    float ch1_peak_v;
    float ch2_peak_v;
    float gain_linear;
    float gain_db;
    float phase_rad;
    float phase_deg;
    float ch1_dc_v;
    float ch2_dc_v;
    float ch1_residual_ratio;
    float ch2_residual_ratio;
} DualResponse_Result;

/*
 * Estimate frequency from CH1, then fit both channels at exactly that
 * frequency.  The result is H(f)=CH2/CH1, with phase wrapped to [-180, 180).
 */
DualResponse_Status DualResponse_Analyze(const float *ch1_v,
                                         const float *ch2_v,
                                         uint32_t length,
                                         float sample_rate_hz,
                                         float min_frequency_hz,
                                         float max_frequency_hz,
                                         DualResponse_Result *result);

#endif /* DUAL_RESPONSE_ANALYZER_H */
