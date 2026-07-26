/**
 * @file    Adaptive.h
 * @brief   Adaptive filtering (LMS, notch), online statistics, and linear calibration.
 */
#ifndef __ADAPTIVE_H
#define __ADAPTIVE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct {
    uint32_t n;
    float mean;
    float m2;
    float min_v;
    float max_v;
} Contest_OnlineStats_t;

typedef struct {
    float gain;
    float offset;
    float r2;
} Contest_LineCal_t;

typedef struct {
    uint32_t taps;
    float mu;
    float leak;
    float *w;
    float *state;
} Contest_LMS_t;

/**
 * @brief  LMS sinusoidal separation structure.
 * @note   Uses 2-weight LMS with sin/cos basis to extract a known-frequency
 *         sinusoid from a multi-tone mixture in real time, sample-by-sample.
 *
 *         After convergence:
 *           - Process() return value  ≈ the target-frequency component A'
 *           - *err_out                 ≈ the residual (B' + noise)
 *           - w_cos, w_sin encode amplitude & phase:
 *                 amplitude = sqrt(w_cos^2 + w_sin^2)
 *                 phase     = atan2(-w_sin, w_cos)   (cos reference convention)
 *
 *         Typical usage for 2023H signal separation:
 *           1) FFT detects fA, fB from mixture C.
 *           2) Init two instances: one @ fA, one @ fB.
 *           3) Feed C to both; after convergence A' ≈ out_A, B' ≈ out_B.
 *           4) Optionally read back amplitude/phase for DDS reconstruction.
 */
typedef struct {
    float fs;           /* 采样率，单位Hz。 */
    float freq;         /* 目标频率，单位Hz。 */
    float mu;           /* LMS步长，典型值0.005~0.05。 */
    float w_cos;        /* 余弦参考权重。 */
    float w_sin;        /* 正弦参考权重。 */
    float delta;        /* 每点相位增量。 */
    float cosd, sind;   /* 相位增量的余弦、正弦。 */
    float c, s;         /* 当前正交参考。 */
    uint32_t sample_count; /* 已处理样本数，用于周期性校正正交基。 */
} Contest_LMS_SineSep_t;

typedef struct {
    float fs;
    float freq;
    float radius;
    float b0, b1, b2;
    float a1, a2;
    float z1, z2;
} Contest_Notch_t;

void Contest_StatsInit(Contest_OnlineStats_t *s);
void Contest_StatsPush(Contest_OnlineStats_t *s, float x);
float Contest_StatsVariance(const Contest_OnlineStats_t *s);
float Contest_StatsStd(const Contest_OnlineStats_t *s);

int Contest_CalibrateLine(const float *raw, const float *ref, uint32_t n,
                          Contest_LineCal_t *cal);
float Contest_CalApply(const Contest_LineCal_t *cal, float raw);

uint32_t Contest_RejectOutliers(const float *in, float *out, uint32_t len,
                                float sigma_limit);

int Contest_LMSInit(Contest_LMS_t *lms, float *w, float *state,
                    uint32_t taps, float mu, float leak);
float Contest_LMSProcess(Contest_LMS_t *lms, float x, float desired,
                         float *err_out);

/**
 * @brief   Initialize LMS sine separation instance.
 * @param   lms    Instance pointer
 * @param   fs     Sample rate (Hz)
 * @param   freq   Target frequency to extract (Hz), 0 < freq < fs/2
 * @param   mu     Step size, typical 0.005~0.05; smaller = slower but cleaner
 * @return  0 on success, -1 on invalid parameters
 */
int Contest_LMS_SineSepInit(Contest_LMS_SineSep_t *lms, float fs,
                            float freq, float mu);

/**
 * @brief   Process one sample: adapt weights and return the extracted component.
 * @param   lms       Instance pointer
 * @param   mixture   One sample of the mixed signal C[n]
 * @param   err_out   [out, optional] error = mixture - output (≈ other components)
 * @return  Extracted target-frequency component (A'[n]).
 * @note    Call repeatedly. Convergence time ≈ several/(mu * fs) samples.
 *          After convergence, w_cos & w_sin encode amplitude and phase.
 */
float Contest_LMS_SineSepProcess(Contest_LMS_SineSep_t *lms, float mixture,
                                 float *err_out);

/**
 * @brief   Read back amplitude & phase from the converged weights.
 * @param   lms         Instance pointer
 * @param   amplitude   [out, optional] sqrt(w_cos^2 + w_sin^2)
 * @param   phase_rad   [out, optional] atan2(-w_sin, w_cos), in radians [-pi, pi]
 */
void Contest_LMS_SineSepGetParams(const Contest_LMS_SineSep_t *lms,
                                  float *amplitude, float *phase_rad);

/**
 * @brief   Reset weights and reference phase to initial state
 *          (e.g. for a new acquisition without re-init).
 */
void Contest_LMS_SineSepReset(Contest_LMS_SineSep_t *lms);

int Contest_NotchInit(Contest_Notch_t *n, float fs, float freq, float radius);
float Contest_NotchProcess(Contest_Notch_t *n, float x);
void Contest_NotchReset(Contest_Notch_t *n);

#ifdef __cplusplus
}
#endif

#endif /* __ADAPTIVE_H */
