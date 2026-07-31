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
 * Configured nominal voltage gain from the external BNC input to the ADC
 * input. All reported amplitudes, RMS values and Vpp values are referred back
 * to the BNC input by multiplying ADC-side volts by its reciprocal. Replace
 * 1.0f only after a BNC-referred physical measurement.
 */
#define G_CONTEST_FRONTEND_VOLTAGE_GAIN     1.0f

/*
 * No AD9238 empirical points are valid for the new LTC2208/ADA4932 signal
 * path.  Add LTC2208 measurements only after a complete BNC-referred sweep.
 */
#define G_CONTEST_EMPIRICAL_CAL_POINT_COUNT 0U
#if G_CONTEST_EMPIRICAL_CAL_POINT_COUNT > 8U
#error "G contest supports at most eight empirical calibration points"
#endif

#endif /* G_CONTEST_CONFIG_H */
