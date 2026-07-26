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
#define PLL_TEST_RF_MIN_FREQ_HZ                1000000U
#define PLL_TEST_SWEEP_BLOCKS_PER_RATE         2U
#define PLL_TEST_REACQUIRE_MS                  300U
#define PLL_TEST_ALIAS_EDGE_BINS               2U
#define PLL_TEST_ALIAS_MIN_SNR_DB              12.0f
#define PLL_TEST_MIN_REFERENCE_IQ_AMP           0.005f
#define PLL_TEST_MIN_FEEDBACK_IQ_AMP            0.003f
#define PLL_TEST_AD9910_AMPLITUDE              16383U
#define PLL_TEST_MIN_AC_FREQ_HZ                0.5f
#define PLL_TEST_DEFAULT_START_FREQ_HZ         1000.0f
#define PLL_TEST_DEFAULT_LOOP_BW_HZ            10.0f
#define PLL_TEST_DEFAULT_DAMPING               0.70710678f
#define PLL_TEST_TARGET_PHASE_DEG              0.0f
#define PLL_TEST_FIXED_PHASE_CAL_DEG           0.0f
#define PLL_TEST_TELEMETRY_PERIOD_MS           100U
#define PLL_TEST_HOLDOVER_CONFIRM_MS           1000U
#define PLL_TEST_ACTIVITY_P2P_CODE             160U
#define PLL_TEST_MIN_SIGNAL_P2P_CODE           300U

#endif /* PLL_SELFTEST_CONFIG_H */


