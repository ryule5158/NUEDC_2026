/**
 * @file    SoftPll.h
 * @brief   Reusable second-order software PLL controller.
 *
 * The controller is hardware independent: ADC/IQ code supplies phase and
 * amplitude measurements, while the caller applies dds_freq_cmd_hz and the
 * incremental phase_step_deg to an AD9910, an NCO, or a test DAC backend.
 */
#ifndef __SOFT_PLL_H
#define __SOFT_PLL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "IQ.h"

#ifndef SOFTPLL_PI
#define SOFTPLL_PI                              3.14159265358979323846f
#endif

/* Conservative defaults. They are macros so that board tests can override
 * them globally without modifying the controller source. */
#ifndef SOFTPLL_DEFAULT_LOOP_BANDWIDTH_HZ
#define SOFTPLL_DEFAULT_LOOP_BANDWIDTH_HZ       10.0f
#endif
#ifndef SOFTPLL_DEFAULT_DAMPING_RATIO
#define SOFTPLL_DEFAULT_DAMPING_RATIO           0.70710678f
#endif
#ifndef SOFTPLL_DEFAULT_UPDATE_RATE_HZ
#define SOFTPLL_DEFAULT_UPDATE_RATE_HZ          1000.0f
#endif
#ifndef SOFTPLL_DEFAULT_PHASE_KP
#define SOFTPLL_DEFAULT_PHASE_KP                0.12f
#endif
#ifndef SOFTPLL_DEFAULT_PHASE_KI
#define SOFTPLL_DEFAULT_PHASE_KI                0.0f
#endif
#ifndef SOFTPLL_DEFAULT_FREQ_KD
#define SOFTPLL_DEFAULT_FREQ_KD                 0.20f
#endif
#ifndef SOFTPLL_DEFAULT_RATE_ALPHA
#define SOFTPLL_DEFAULT_RATE_ALPHA              0.25f
#endif
#ifndef SOFTPLL_DEFAULT_LOCK_ERROR_DEG
#define SOFTPLL_DEFAULT_LOCK_ERROR_DEG          3.0f
#endif
#ifndef SOFTPLL_DEFAULT_LOCK_RATE_HZ
#define SOFTPLL_DEFAULT_LOCK_RATE_HZ            0.5f
#endif
#ifndef SOFTPLL_DEFAULT_LOCK_COUNT
#define SOFTPLL_DEFAULT_LOCK_COUNT              12U
#endif
#ifndef SOFTPLL_DEFAULT_UNLOCK_COUNT
#define SOFTPLL_DEFAULT_UNLOCK_COUNT            4U
#endif

typedef enum {
    SOFTPLL_UNIT_RAD = 0,
    SOFTPLL_UNIT_DEG = 1
} SoftPll_PhaseUnit_t;

typedef struct {
    float reference_phase;  /**< Input/reference phase, rad or deg. */
    float feedback_phase;   /**< Measured output phase, rad or deg. */
    float dt_s;             /**< Exact interval since the prior update. */
    float reference_amp;    /**< Normalized reference amplitude. */
    float feedback_amp;     /**< Normalized feedback amplitude. */
    float target_offset;    /**< Desired feedback-reference phase offset. */
    float fixed_cal;        /**< Board/frontend fixed phase calibration. */
} SoftPll_PhaseInput_t;

typedef struct {
    float nominal_freq_hz;
    float initial_phase_deg;
    float freq_cal_hz;
    float max_freq_offset_hz;

    float min_reference_amp;
    float min_feedback_amp;

    /* Optional bounded direct phase shifter. This is deliberately separate
     * from the type-II frequency loop so AD9910 POW updates remain optional. */
    float phase_kp;
    float phase_ki;
    float max_phase_step_deg;
    float max_integral_deg;
    float phase_deadband_deg;

    /* Type-II loop:
     *   f_cmd = f_nominal + Kp*phase_error + integral + Kd*phase_rate
     *   d(integral)/dt = Ki*phase_error
     */
    float freq_kp_hz_per_deg;
    float freq_kd;
    float phase_rate_alpha;
    float max_freq_step_hz;
    float freq_step_deadband_hz;
    float hold_error_deg;

    float freq_direction;
    float phase_direction;

    /* Appended fields preserve all existing source-level field names. */
    float freq_ki_hz_per_deg_s;
    float lock_error_deg;
    float lock_rate_hz;
    uint16_t lock_count_required;
    uint16_t unlock_count_required;
} SoftPll_Config_t;

typedef struct {
    SoftPll_Config_t cfg;

    float phase_cmd_deg;
    float freq_cmd_hz;
    float dds_freq_cmd_hz;
    float integral_deg;

    float error_deg;
    float last_error_deg;
    float delta_error_deg;
    float phase_rate_hz;
    float freq_step_hz;
    float phase_step_deg;

    float freq_integral_hz;
    uint16_t lock_count;
    uint16_t unlock_count;
    uint8_t locked;
    uint8_t ready;
    uint8_t last_error_valid;
    uint8_t feedback_valid;
} SoftPll_t;

void SoftPll_DefaultConfig(SoftPll_Config_t *cfg, float nominal_freq_hz);

/**
 * @brief Calculate continuous-time type-II gains from natural frequency.
 * @note  loop_bandwidth_hz is used as fn (wn = 2*pi*fn). The update interval
 *        is still supplied on every SoftPll_UpdatePhase call, so gains remain
 *        correct if block timing has small variations.
 */
uint8_t SoftPll_DesignType2(SoftPll_Config_t *cfg,
                            float loop_bandwidth_hz,
                            float damping_ratio,
                            float update_rate_hz);

void SoftPll_Init(SoftPll_t *pll, const SoftPll_Config_t *cfg);
void SoftPll_Reset(SoftPll_t *pll, float initial_phase_deg, float nominal_freq_hz);

/**
 * @brief Immediately acquire a new centre frequency without changing API users.
 * @param preserve_phase Non-zero keeps the current DDS phase offset.
 */
uint8_t SoftPll_Retune(SoftPll_t *pll,
                       float nominal_freq_hz,
                       uint8_t preserve_phase);

uint8_t SoftPll_UpdatePhase(SoftPll_t *pll,
                            const SoftPll_PhaseInput_t *input,
                            SoftPll_PhaseUnit_t unit);

uint8_t SoftPll_UpdateIq(SoftPll_t *pll,
                         const IQ_Result_t *reference,
                         const IQ_Result_t *feedback,
                         float dt_s,
                         float target_offset_deg,
                         float fixed_cal_deg);

uint8_t SoftPll_IsLocked(const SoftPll_t *pll);
float SoftPll_Wrap360Deg(float phase_deg);
float SoftPll_Normalize180Deg(float phase_deg);
float SoftPll_RadToDeg(float phase_rad);
float SoftPll_DegToRad(float phase_deg);

#ifdef __cplusplus
}
#endif

#endif /* __SOFT_PLL_H */
