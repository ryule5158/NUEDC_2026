#ifndef G_CONTEST_CONFIG_H
#define G_CONTEST_CONFIG_H

#define G_CONTEST_SAMPLE_COUNT              4096U
#define G_CONTEST_CAPTURE_TIMEOUT_MS        250U
#define G_CONTEST_CAPTURE_MAX_ATTEMPTS      2U
#define G_CONTEST_CAPTURE_RETRY_DELAY_MS    2U

#define G_CONTEST_MIN_FREQ_HZ               10000.0f
#define G_CONTEST_MAX_FREQ_HZ               500000.0f
#define G_CONTEST_MIN_COMPONENT_PEAK_V      0.0005f
#define G_CONTEST_ALIAS_MATCH_TOLERANCE_HZ  750.0f
/* 2.5 mV peak = 5 mVpp, equal to the required absolute voltage accuracy. */
#define G_CONTEST_ALIAS_MATERIAL_PEAK_V     0.0025f
#define G_CONTEST_ALIAS_REFERENCE_MIN_RATIO 0.20f
#define G_CONTEST_WAVEFORM_POINTS_PER_CYCLE 256U
#define G_CONTEST_MAX_WAVEFORM_CYCLES       3U

/*
 * Default measured voltage gain from the external BNC input to the ADC input.
 * The runtime value is loaded from QSPI and represented as gain * 1000.
 */
#define G_CONTEST_DEFAULT_GAIN_MILLI        9550U
#define G_CONTEST_GAIN_MIN_MILLI            8000U
#define G_CONTEST_GAIN_MAX_MILLI            12000U
#define G_CONTEST_COEFFICIENT_MIN_U1E6      900000U
#define G_CONTEST_COEFFICIENT_MAX_U1E6      1100000U

/*
 * Eight ascending empirical amplitude points are enabled in g_contest_app.c.
 * They correct the residual BNC-to-reported response after the exact
 * FIR307+CIC3/R25 inverse.
 */
#define G_CONTEST_EMPIRICAL_CAL_POINT_COUNT 8U
#if G_CONTEST_EMPIRICAL_CAL_POINT_COUNT > 8U
#error "G contest supports at most eight empirical calibration points"
#endif

#endif /* G_CONTEST_CONFIG_H */
