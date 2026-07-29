#ifndef G_CONTEST_CONFIG_H
#define G_CONTEST_CONFIG_H

/* Set to zero to restore the repository's generic PLL/FPGA application. */
#define G_CONTEST_ENABLE                    1U

/* AD9280 runs at 32 MSPS. CIC3 decimation by 16 gives 2 MSPS. */
#define G_CONTEST_ADC_CLOCK_HZ              32000000U
#define G_CONTEST_FPGA_DECIMATION           16U
#define G_CONTEST_SAMPLE_RATE_HZ            2000000U
#define G_CONTEST_SAMPLE_COUNT              4096U
#define G_CONTEST_BIN_WIDTH_HZ              488.28125f
#define G_CONTEST_CAPTURE_TIMEOUT_MS        250U

/* External OPA695 gain after the six-pole 650 kHz low-pass filter. */
#define G_CONTEST_FRONT_END_GAIN            8.000000f
#define G_CONTEST_MIN_FREQ_HZ               10000.0f
#define G_CONTEST_MAX_FREQ_HZ               500000.0f
#define G_CONTEST_MIN_COMPONENT_PEAK_V      0.0005f

/* Default S2 module calibration. Replace with two measured points on site. */
#define G_CONTEST_CAL_LOW_CODE              0.0f
#define G_CONTEST_CAL_LOW_V                 (-5.0f)
#define G_CONTEST_CAL_HIGH_CODE             255.0f
#define G_CONTEST_CAL_HIGH_V                4.9609375f
#define G_CONTEST_CLIP_LOW_CODE             3U
#define G_CONTEST_CLIP_HIGH_CODE            252U

#endif /* G_CONTEST_CONFIG_H */
