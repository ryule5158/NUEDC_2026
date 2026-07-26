/**
 * @file    SoftPll.c
 * @brief   Hardware-independent type-II software PLL implementation.
 */

#include "SoftPll.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

#define SOFTPLL_MIN_POSITIVE                 1.0e-12f
#define SOFTPLL_DEFAULT_MAX_PHASE_STEP_DEG  15.0f
#define SOFTPLL_DEFAULT_MAX_INTEGRAL_DEG     5.0f
#define SOFTPLL_DEFAULT_PHASE_DEADBAND_DEG   0.15f
#define SOFTPLL_DEFAULT_FREQ_DEADBAND_HZ     0.001f
#define SOFTPLL_DEFAULT_HOLD_ERROR_DEG       5.0f
#define SOFTPLL_DEFAULT_MIN_REFERENCE_AMP    0.02f
#define SOFTPLL_DEFAULT_MIN_FEEDBACK_AMP     0.02f

static float SoftPll_Abs(float x)
{
    return (x < 0.0f) ? -x : x;
}

static float SoftPll_Max(float a, float b)
{
    return (a > b) ? a : b;
}

static float SoftPll_Clamp(float x, float lo, float hi)
{
    if (x > hi) return hi;
    if (x < lo) return lo;
    return x;
}

static float SoftPll_LimitSym(float x, float limit)
{
    if (limit <= 0.0f) return x;
    return SoftPll_Clamp(x, -limit, limit);
}

static uint8_t SoftPll_ConfigIsFinite(const SoftPll_Config_t *cfg)
{
    return (cfg != NULL) &&
           isfinite(cfg->nominal_freq_hz) && isfinite(cfg->initial_phase_deg) &&
           isfinite(cfg->freq_cal_hz) && isfinite(cfg->max_freq_offset_hz) &&
           isfinite(cfg->min_reference_amp) && isfinite(cfg->min_feedback_amp) &&
           isfinite(cfg->phase_kp) && isfinite(cfg->phase_ki) &&
           isfinite(cfg->max_phase_step_deg) && isfinite(cfg->max_integral_deg) &&
           isfinite(cfg->phase_deadband_deg) &&
           isfinite(cfg->freq_kp_hz_per_deg) && isfinite(cfg->freq_kd) &&
           isfinite(cfg->phase_rate_alpha) && isfinite(cfg->max_freq_step_hz) &&
           isfinite(cfg->freq_step_deadband_hz) && isfinite(cfg->hold_error_deg) &&
           isfinite(cfg->freq_direction) && isfinite(cfg->phase_direction) &&
           isfinite(cfg->freq_ki_hz_per_deg_s) &&
           isfinite(cfg->lock_error_deg) && isfinite(cfg->lock_rate_hz);
}

static void SoftPll_UpdateLockState(SoftPll_t *pll)
{
    uint8_t in_lock_window;

    in_lock_window =
        ((SoftPll_Abs(pll->error_deg) <= pll->cfg.lock_error_deg) &&
         (SoftPll_Abs(pll->phase_rate_hz) <= pll->cfg.lock_rate_hz)) ? 1U : 0U;

    if (in_lock_window != 0U) {
        pll->unlock_count = 0U;
        if (pll->lock_count < UINT16_MAX) pll->lock_count++;
        if (pll->lock_count >= pll->cfg.lock_count_required) pll->locked = 1U;
    } else {
        pll->lock_count = 0U;
        if (pll->unlock_count < UINT16_MAX) pll->unlock_count++;
        if (pll->unlock_count >= pll->cfg.unlock_count_required) pll->locked = 0U;
    }
}

float SoftPll_Wrap360Deg(float phase_deg)
{
    if (!isfinite(phase_deg)) return 0.0f;
    phase_deg = fmodf(phase_deg, 360.0f);
    if (phase_deg < 0.0f) phase_deg += 360.0f;
    return phase_deg;
}

float SoftPll_Normalize180Deg(float phase_deg)
{
    if (!isfinite(phase_deg)) return 0.0f;
    phase_deg = fmodf(phase_deg, 360.0f);
    if (phase_deg > 180.0f) phase_deg -= 360.0f;
    if (phase_deg <= -180.0f) phase_deg += 360.0f;
    return phase_deg;
}

float SoftPll_RadToDeg(float phase_rad)
{
    return phase_rad * (180.0f / SOFTPLL_PI);
}

float SoftPll_DegToRad(float phase_deg)
{
    return phase_deg * (SOFTPLL_PI / 180.0f);
}

uint8_t SoftPll_DesignType2(SoftPll_Config_t *cfg,
                            float loop_bandwidth_hz,
                            float damping_ratio,
                            float update_rate_hz)
{
    float wn_rad_s;
    float max_step_from_loop;

    if ((cfg == NULL) || !isfinite(loop_bandwidth_hz) ||
        !isfinite(damping_ratio) || !isfinite(update_rate_hz) ||
        (loop_bandwidth_hz <= 0.0f) || (damping_ratio <= 0.0f) ||
        (update_rate_hz <= 0.0f) ||
        (loop_bandwidth_hz >= (0.20f * update_rate_hz))) {
        return 0U;
    }

    /* With phase error in degrees and oscillator command in Hz:
     * e_dot = 360*(f_ref-f_out). Matching the standard second-order
     * characteristic gives the two coefficients below. */
    wn_rad_s = 2.0f * SOFTPLL_PI * loop_bandwidth_hz;
    cfg->freq_kp_hz_per_deg = (2.0f * damping_ratio * wn_rad_s) / 360.0f;
    cfg->freq_ki_hz_per_deg_s = (wn_rad_s * wn_rad_s) / 360.0f;

    /* This slew limit is only a fault guard. */
    max_step_from_loop =
        cfg->freq_ki_hz_per_deg_s * 180.0f / update_rate_hz * 8.0f;
    cfg->max_freq_step_hz =
        SoftPll_Max(cfg->max_freq_step_hz, max_step_from_loop);
    return 1U;
}

void SoftPll_DefaultConfig(SoftPll_Config_t *cfg, float nominal_freq_hz)
{
    float abs_nominal;

    if (cfg == NULL) return;
    memset(cfg, 0, sizeof(*cfg));

    abs_nominal = SoftPll_Abs(nominal_freq_hz);
    cfg->nominal_freq_hz = nominal_freq_hz;
    cfg->initial_phase_deg = 0.0f;
    cfg->freq_cal_hz = 0.0f;
    cfg->max_freq_offset_hz = SoftPll_Max(10.0f, abs_nominal * 0.25f);
    cfg->min_reference_amp = SOFTPLL_DEFAULT_MIN_REFERENCE_AMP;
    cfg->min_feedback_amp = SOFTPLL_DEFAULT_MIN_FEEDBACK_AMP;

    cfg->phase_kp = SOFTPLL_DEFAULT_PHASE_KP;
    cfg->phase_ki = SOFTPLL_DEFAULT_PHASE_KI;
    cfg->max_phase_step_deg = SOFTPLL_DEFAULT_MAX_PHASE_STEP_DEG;
    cfg->max_integral_deg = SOFTPLL_DEFAULT_MAX_INTEGRAL_DEG;
    cfg->phase_deadband_deg = SOFTPLL_DEFAULT_PHASE_DEADBAND_DEG;

    cfg->freq_kd = SOFTPLL_DEFAULT_FREQ_KD;
    cfg->phase_rate_alpha = SOFTPLL_DEFAULT_RATE_ALPHA;
    cfg->max_freq_step_hz = SoftPll_Max(1.0f, abs_nominal * 0.02f);
    cfg->freq_step_deadband_hz = SOFTPLL_DEFAULT_FREQ_DEADBAND_HZ;
    cfg->hold_error_deg = SOFTPLL_DEFAULT_HOLD_ERROR_DEG;
    cfg->freq_direction = 1.0f;
    cfg->phase_direction = 1.0f;

    cfg->lock_error_deg = SOFTPLL_DEFAULT_LOCK_ERROR_DEG;
    cfg->lock_rate_hz = SOFTPLL_DEFAULT_LOCK_RATE_HZ;
    cfg->lock_count_required = SOFTPLL_DEFAULT_LOCK_COUNT;
    cfg->unlock_count_required = SOFTPLL_DEFAULT_UNLOCK_COUNT;

    (void)SoftPll_DesignType2(cfg,
                              SOFTPLL_DEFAULT_LOOP_BANDWIDTH_HZ,
                              SOFTPLL_DEFAULT_DAMPING_RATIO,
                              SOFTPLL_DEFAULT_UPDATE_RATE_HZ);
}

void SoftPll_Init(SoftPll_t *pll, const SoftPll_Config_t *cfg)
{
    if ((pll == NULL) || (cfg == NULL)) return;

    if ((SoftPll_ConfigIsFinite(cfg) == 0U) ||
        (cfg->lock_count_required == 0U) ||
        (cfg->unlock_count_required == 0U)) {
        memset(pll, 0, sizeof(*pll));
        return;
    }

    memset(pll, 0, sizeof(*pll));
    pll->cfg = *cfg;
    pll->cfg.phase_rate_alpha =
        SoftPll_Clamp(pll->cfg.phase_rate_alpha, 0.0f, 1.0f);
    SoftPll_Reset(pll, cfg->initial_phase_deg, cfg->nominal_freq_hz);
}

void SoftPll_Reset(SoftPll_t *pll, float initial_phase_deg, float nominal_freq_hz)
{
    SoftPll_Config_t cfg;

    if (pll == NULL) return;
    cfg = pll->cfg;
    if (!isfinite(initial_phase_deg) || !isfinite(nominal_freq_hz) ||
        !isfinite(cfg.freq_cal_hz)) {
        pll->ready = 0U;
        return;
    }

    memset(pll, 0, sizeof(*pll));
    pll->cfg = cfg;
    pll->cfg.initial_phase_deg = initial_phase_deg;
    pll->cfg.nominal_freq_hz = nominal_freq_hz;
    pll->phase_cmd_deg = SoftPll_Wrap360Deg(initial_phase_deg);
    pll->freq_cmd_hz = nominal_freq_hz;
    pll->dds_freq_cmd_hz = nominal_freq_hz + pll->cfg.freq_cal_hz;
    pll->ready = 1U;
}

uint8_t SoftPll_Retune(SoftPll_t *pll,
                       float nominal_freq_hz,
                       uint8_t preserve_phase)
{
    float phase_deg;

    if ((pll == NULL) || (pll->ready == 0U) ||
        !isfinite(nominal_freq_hz)) {
        return 0U;
    }

    phase_deg = (preserve_phase != 0U) ? pll->phase_cmd_deg
                                      : pll->cfg.initial_phase_deg;
    SoftPll_Reset(pll, phase_deg, nominal_freq_hz);
    return pll->ready;
}

uint8_t SoftPll_UpdatePhase(SoftPll_t *pll,
                            const SoftPll_PhaseInput_t *input,
                            SoftPll_PhaseUnit_t unit)
{
    float reference_deg;
    float feedback_deg;
    float target_offset_deg;
    float fixed_cal_deg;
    float phase_delta_deg;
    float raw_phase_rate_hz;
    float p_term_hz;
    float d_term_hz;
    float desired_freq_hz;
    float freq_min_hz;
    float freq_max_hz;
    float freq_step_hz;
    float phase_step_deg;
    uint8_t saturated = 0U;

    if ((pll == NULL) || (input == NULL) || (pll->ready == 0U) ||
        (input->dt_s <= SOFTPLL_MIN_POSITIVE)) {
        return 0U;
    }

    if (((unit != SOFTPLL_UNIT_RAD) && (unit != SOFTPLL_UNIT_DEG)) ||
        !isfinite(input->reference_phase) || !isfinite(input->feedback_phase) ||
        !isfinite(input->dt_s) || !isfinite(input->reference_amp) ||
        !isfinite(input->feedback_amp) || !isfinite(input->target_offset) ||
        !isfinite(input->fixed_cal) ||
        (SoftPll_ConfigIsFinite(&pll->cfg) == 0U)) {
        pll->feedback_valid = 0U;
        pll->last_error_valid = 0U;
        pll->locked = 0U;
        return 0U;
    }

    pll->freq_step_hz = 0.0f;
    pll->phase_step_deg = 0.0f;
    if ((input->reference_amp < pll->cfg.min_reference_amp) ||
        (input->feedback_amp < pll->cfg.min_feedback_amp)) {
        pll->feedback_valid = 0U;
        pll->last_error_valid = 0U;
        pll->lock_count = 0U;
        pll->locked = 0U;
        return 0U;
    }
    pll->feedback_valid = 1U;

    if (unit == SOFTPLL_UNIT_RAD) {
        reference_deg = SoftPll_RadToDeg(input->reference_phase);
        feedback_deg = SoftPll_RadToDeg(input->feedback_phase);
        target_offset_deg = SoftPll_RadToDeg(input->target_offset);
        fixed_cal_deg = SoftPll_RadToDeg(input->fixed_cal);
    } else {
        reference_deg = input->reference_phase;
        feedback_deg = input->feedback_phase;
        target_offset_deg = input->target_offset;
        fixed_cal_deg = input->fixed_cal;
    }

    pll->error_deg = SoftPll_Normalize180Deg(
        reference_deg + target_offset_deg + fixed_cal_deg - feedback_deg);

    if (pll->last_error_valid != 0U) {
        phase_delta_deg =
            SoftPll_Normalize180Deg(pll->error_deg - pll->last_error_deg);
    } else {
        phase_delta_deg = 0.0f;
        pll->last_error_valid = 1U;
    }
    pll->last_error_deg = pll->error_deg;
    pll->delta_error_deg = phase_delta_deg;

    raw_phase_rate_hz = phase_delta_deg / (360.0f * input->dt_s);
    pll->phase_rate_hz +=
        (raw_phase_rate_hz - pll->phase_rate_hz) *
        pll->cfg.phase_rate_alpha;

    /* Integral is expressed in Hz. dt makes the design independent of the
     * acquisition block length. */
    pll->freq_integral_hz +=
        pll->error_deg * pll->cfg.freq_ki_hz_per_deg_s *
        input->dt_s * pll->cfg.freq_direction;
    pll->freq_integral_hz =
        SoftPll_LimitSym(pll->freq_integral_hz,
                         pll->cfg.max_freq_offset_hz);

    p_term_hz = pll->error_deg * pll->cfg.freq_kp_hz_per_deg *
                pll->cfg.freq_direction;
    d_term_hz = pll->phase_rate_hz * pll->cfg.freq_kd *
                pll->cfg.freq_direction;
    desired_freq_hz = pll->cfg.nominal_freq_hz +
                      pll->freq_integral_hz + p_term_hz + d_term_hz;

    if (pll->cfg.max_freq_offset_hz > 0.0f) {
        freq_min_hz = pll->cfg.nominal_freq_hz -
                      pll->cfg.max_freq_offset_hz;
        freq_max_hz = pll->cfg.nominal_freq_hz +
                      pll->cfg.max_freq_offset_hz;
        if (desired_freq_hz > freq_max_hz) {
            desired_freq_hz = freq_max_hz;
            saturated = 1U;
        } else if (desired_freq_hz < freq_min_hz) {
            desired_freq_hz = freq_min_hz;
            saturated = 1U;
        }
    }

    /* Back-calculate at the output clamp to prevent integrator wind-up. */
    if (saturated != 0U) {
        pll->freq_integral_hz =
            desired_freq_hz - pll->cfg.nominal_freq_hz -
            p_term_hz - d_term_hz;
    }

    freq_step_hz = desired_freq_hz - pll->freq_cmd_hz;
    freq_step_hz =
        SoftPll_LimitSym(freq_step_hz, pll->cfg.max_freq_step_hz);
    if ((SoftPll_Abs(pll->error_deg) < pll->cfg.hold_error_deg) &&
        (SoftPll_Abs(freq_step_hz) < pll->cfg.freq_step_deadband_hz)) {
        freq_step_hz = 0.0f;
    }
    pll->freq_cmd_hz += freq_step_hz;
    pll->freq_step_hz = freq_step_hz;
    pll->dds_freq_cmd_hz = pll->freq_cmd_hz + pll->cfg.freq_cal_hz;

    /* Optional phase kick. Set phase_kp/phase_ki to zero for frequency-only
     * hardware; AD9910 and the test NCO can use phase_step_deg directly. */
    if (SoftPll_Abs(pll->error_deg) >= pll->cfg.phase_deadband_deg) {
        pll->integral_deg += pll->error_deg * pll->cfg.phase_ki;
        pll->integral_deg =
            SoftPll_LimitSym(pll->integral_deg,
                             pll->cfg.max_integral_deg);
        phase_step_deg =
            (pll->error_deg * pll->cfg.phase_kp + pll->integral_deg) *
            pll->cfg.phase_direction;
        phase_step_deg =
            SoftPll_LimitSym(phase_step_deg,
                             pll->cfg.max_phase_step_deg);
        pll->phase_cmd_deg =
            SoftPll_Wrap360Deg(pll->phase_cmd_deg + phase_step_deg);
        pll->phase_step_deg = phase_step_deg;
    }

    SoftPll_UpdateLockState(pll);
    return 1U;
}

uint8_t SoftPll_UpdateIq(SoftPll_t *pll,
                         const IQ_Result_t *reference,
                         const IQ_Result_t *feedback,
                         float dt_s,
                         float target_offset_deg,
                         float fixed_cal_deg)
{
    SoftPll_PhaseInput_t input;

    if ((reference == NULL) || (feedback == NULL)) return 0U;

    input.reference_phase = reference->phase;
    input.feedback_phase = feedback->phase;
    input.dt_s = dt_s;
    input.reference_amp = reference->amplitude;
    input.feedback_amp = feedback->amplitude;
    input.target_offset = SoftPll_DegToRad(target_offset_deg);
    input.fixed_cal = SoftPll_DegToRad(fixed_cal_deg);
    return SoftPll_UpdatePhase(pll, &input, SOFTPLL_UNIT_RAD);
}

uint8_t SoftPll_IsLocked(const SoftPll_t *pll)
{
    if ((pll == NULL) || (pll->ready == 0U)) return 0U;
    return pll->locked;
}
