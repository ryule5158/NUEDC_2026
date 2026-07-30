#ifndef G_CONTEST_CONFIG_H
#define G_CONTEST_CONFIG_H

#define G_CONTEST_SAMPLE_COUNT              4096U
#define G_CONTEST_CAPTURE_TIMEOUT_MS        250U
#define G_CONTEST_CAPTURE_MAX_ATTEMPTS      2U
#define G_CONTEST_CAPTURE_RETRY_DELAY_MS    2U

#define G_CONTEST_MIN_FREQ_HZ               10000.0f
#define G_CONTEST_MAX_FREQ_HZ               500000.0f
#define G_CONTEST_MIN_COMPONENT_PEAK_V      0.0005f
#define G_CONTEST_WAVEFORM_POINTS_PER_CYCLE 256U
#define G_CONTEST_MAX_WAVEFORM_CYCLES       3U

/*
 * Three ascending, independently measured amplitude points are enabled in
 * g_contest_app.c. The last factor is held to 500 kHz only as an explicitly
 * reported inference; FIR307+CIC3/R25 remains a separate first layer.
 */
#define G_CONTEST_EMPIRICAL_CAL_POINT_COUNT 3U
#if G_CONTEST_EMPIRICAL_CAL_POINT_COUNT > 8U
#error "G contest supports at most eight empirical calibration points"
#endif

#endif /* G_CONTEST_CONFIG_H */
