#ifndef PLL_SELFTEST_CONFIG_H
#define PLL_SELFTEST_CONFIG_H

/* Set to 0 to restore the generic library-only startup. */
#define PLL_SELFTEST_ENABLE                    1U

/* TIM6 triggers ADC1+ADC2 from the same edge. The 500 kS/s default stays
 * below the 18.75 MHz / 25-cycle 16-bit ADC conversion limit (750 kS/s).
 * The previous /8 ADC clock missed every second 500 kS/s trigger. */
#define PLL_TEST_TIMER_CLOCK_HZ                240000000U
#define PLL_TEST_SAMPLE_RATE_HZ                500000U
#define PLL_TEST_ADC_HALF_SAMPLES              1024U
/* Multi-rate coherent undersampling resolves sine inputs before phase lock. */
#define PLL_TEST_MAX_OUTPUT_HZ                 40000000U
/* Automatic acquisition is intentionally bounded for the on-chip ADC path.
 * 6.5 MHz keeps the measured 6 MHz compatibility point while the specified
 * and validation range remains below 5 MHz. FPGA ADC code handles higher RF. */
#define PLL_TEST_AUTO_SEARCH_MAX_HZ             6500000U
#define PLL_TEST_RF_MIN_FREQ_HZ                1000000U
#define PLL_TEST_SWEEP_BLOCKS_PER_RATE         2U
/* Search the two strongest-rate seed sets: with at most one bad rate, one
 * base is guaranteed to be an inlier while keeping capture below one DMA half. */
#define PLL_TEST_SWEEP_BASE_COUNT               2U
#define PLL_TEST_REACQUIRE_MS                 1000U
#define PLL_TEST_ALIAS_EDGE_BINS               2U
#define PLL_TEST_ALIAS_MIN_SNR_DB              12.0f
/* This test board injects repeatable ADC1 tones near 3.375 and 4.375 MHz.
 * A listed tone is not rejected by frequency alone: replacement is allowed
 * only when three sample rates contain one coherent, PC4-dominant secondary
 * candidate. A genuine lone input at either listed frequency remains valid. */
#define PLL_TEST_BOARD_SPUR_REJECT_ENABLE       1U
#define PLL_TEST_BOARD_SPUR_HZ                  4375000.0f
#define PLL_TEST_BOARD_SPUR2_HZ                 3375000.0f
#define PLL_TEST_BOARD_SPUR_TOLERANCE_HZ        10000.0f
#define PLL_TEST_ALT_PEAK_MIN_REL_DB            (-6.0f)
#define PLL_TEST_ALT_PEAK_MIN_REF_FB_DB         12.0f
/*
 * Field evidence: a valid 100 kHz saw is 0.09..0.11, while disconnected PC4
 * coupling was only about 0.013. A 0.030 threshold rejects that false lock
 * with more than 2x margin without excluding the weakest validated waveform.
 */
#define PLL_TEST_MIN_REFERENCE_IQ_AMP           0.030f
#define PLL_TEST_MIN_FEEDBACK_IQ_AMP            0.003f
/* FPGA/AD9708 uses a 1024-point sine table and the complete 8-bit DAC span.
 * Amplitude 128 with offset 128 maps the normalized table to codes 0..255. */
#define PLL_TEST_AD9708_AMPLITUDE                128U
#define PLL_TEST_AD9708_OFFSET                   128U
#define PLL_TEST_AD9708_WAVE_POINTS             1024U
/* Automatic periodic-wave reconstruction. These field-tuning thresholds are
 * explicit because the PC4 analog frontend changes ideal harmonic ratios. */
#define PLL_TEST_WAVEFORM_DEFAULT                PLL_TEST_WAVE_AUTO
/*
 * The AM/FM estimator and modulation-phase loop are preserved as WIP, but
 * remain outside the default field-tested build. Set to 1 only when resuming
 * the dedicated FPGA 0x00020004 + hardware validation work.
 */
#define PLL_TEST_EXPERIMENTAL_AMFM_ENABLE          0U
#define PLL_TEST_WAVE_H2_SAW_MIN_DB              (-15.0f)
#define PLL_TEST_WAVE_H3_SQUARE_MIN_DB           (-15.0f)
#define PLL_TEST_WAVE_H3_SQUARE_RF_MIN_DB        (-24.0f)
#define PLL_TEST_WAVE_H3_TRIANGLE_MIN_DB         (-28.0f)
#define PLL_TEST_WAVE_LOW_SHAPE_MAX_HZ          120000.0f
#define PLL_TEST_WAVE_HARMONIC_MIN_RATES          2U
#define PLL_TEST_WAVE_HARMONIC_RADIUS_BINS        2U
#define PLL_TEST_WAVE_HARMONIC_GUARD_BINS         4U
#define PLL_TEST_LOCK_HARMONIC_MAX                 9U
#define PLL_TEST_WAVE_MONITOR_PERIOD_MS          250U
#define PLL_TEST_WAVE_MONITOR_CONFIRM_COUNT        3U
/* Single-tone AM/FM sideband reconstruction. A 1024-sample, approximately
 * 500 kS/s record resolves modulation from 0.8 to 20 kHz. Parameters remain
 * macros because the PC4 analogue passband must be field-characterized. */
#define PLL_TEST_MODULATION_MIN_HZ               800.0f
#define PLL_TEST_MODULATION_MAX_HZ             20000.0f
#define PLL_TEST_MODULATION_MIN_SIDEBAND_DB      (-30.0f)
#define PLL_TEST_MODULATION_MAX_SYMMETRY_DB         6.0f
#define PLL_TEST_MODULATION_MIN_PHASE_METRIC        0.55f
#define PLL_TEST_FM_MAX_BETA                        2.0f
#define PLL_TEST_MOD_PHASE_UPDATE_MS                50U
#define PLL_TEST_MOD_PHASE_GAIN                     0.50f
#define PLL_TEST_MOD_PHASE_MAX_STEP_DEG            45.0f
#define PLL_TEST_MOD_PHASE_LOCK_ERROR_DEG            8.0f
#define PLL_TEST_MOD_PHASE_LOCK_COUNT                3U
/* Consecutive acknowledged-write failures before the FPGA/AD9708 backend is
 * declared lost and the complete volatile state is reloaded automatically. */
#define PLL_TEST_AD9708_RECOVERY_ERROR_LIMIT    4U
#define PLL_TEST_MIN_AC_FREQ_HZ                0.5f
#define PLL_TEST_DEFAULT_START_FREQ_HZ         1000.0f
#define PLL_TEST_DEFAULT_LOOP_BW_HZ            10.0f
#define PLL_TEST_DEFAULT_DAMPING               0.70710678f
/* Keep instantaneous derivative feed-forward disabled: undersampled IQ phase
 * slopes are accurate only after filtering several complete DMA frames. */
#define PLL_TEST_FREQ_RATE_FEEDFORWARD          0.0f
/* Guarded FLL acquisition aid. A correction is applied only after N mutually
 * consistent block-to-block phase-rate estimates. The type-II PLL then closes
 * the residual frequency and phase error. These macros are board-test knobs,
 * not hidden gains inside pll_test_app.c. */
#define PLL_TEST_RATE_ASSIST_MIN_HZ              5.0f
#define PLL_TEST_RATE_ASSIST_STABLE_N            6U
#define PLL_TEST_RATE_ASSIST_REL_TOL              0.15f
#define PLL_TEST_RATE_ASSIST_ABS_TOL_HZ           5.0f
/* Lock declaration is separate from loop gain tuning. These limits cover the
 * measured PC4/PC5 frontend phase noise without accepting a cycle-slipping
 * input: a 5 Hz or larger residual is still rejected by the FLL/PLL path. */
#define PLL_TEST_LOCK_ERROR_DEG                    5.0f
#define PLL_TEST_LOCK_RATE_HZ                      2.0f
#define PLL_TEST_LOCK_COUNT                        8U
#define PLL_TEST_UNLOCK_COUNT                     12U
#define PLL_TEST_TARGET_PHASE_DEG              0.0f
#define PLL_TEST_FIXED_PHASE_CAL_DEG           0.0f
#define PLL_TEST_TELEMETRY_PERIOD_MS           100U
#define PLL_TEST_HOLDOVER_CONFIRM_MS           1000U
#define PLL_TEST_ACTIVITY_P2P_CODE             160U
#define PLL_TEST_MIN_SIGNAL_P2P_CODE           300U

#endif /* PLL_SELFTEST_CONFIG_H */
