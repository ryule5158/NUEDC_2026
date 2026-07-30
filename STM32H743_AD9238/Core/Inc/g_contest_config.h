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
 * Keep this at zero until the eight identity placeholders in g_contest_app.c
 * are replaced by calibrated residual amplitude/phase data from the complete
 * AD9238 front end, PCB and cable.  CIC3/R25 theoretical correction is
 * independent and remains enabled when this count is zero.
 */
#define G_CONTEST_EMPIRICAL_CAL_POINT_COUNT 0U
#if G_CONTEST_EMPIRICAL_CAL_POINT_COUNT > 8U
#error "G contest supports at most eight empirical calibration points"
#endif

#endif /* G_CONTEST_CONFIG_H */
