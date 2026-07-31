#ifndef SIGNAL_ADC_AD9280_CALIBRATION_H
#define SIGNAL_ADC_AD9280_CALIBRATION_H

/*
 * Board-specific DC voltage calibration at the AD9280 module input.
 *
 * Fit points: -4, -3, -2, +2, +3 and +4 V.
 * Method: ordinary least squares in the FPGA signed-Q8.8 coordinate.
 * The +/-1 V points were excluded to reduce 8-bit mid-code DNL influence;
 * the +/-4.8 V points were excluded to avoid endpoint nonlinearity.
 *
 *     voltage_v = q8_8 * scale_v_per_q8_8 + offset_v
 */
#define SIGNAL_ADC_AD9280_CAL_SCALE_V_PER_Q8_8  0.00015502132915f
#define SIGNAL_ADC_AD9280_CAL_OFFSET_V          (-0.00330035338f)
#define SIGNAL_ADC_AD9280_CAL_SOURCE            \
    "2026-07-29 OLS +/-2,+/-3,+/-4V"

/*
 * Uniform full-chain AC amplitude correction.
 *
 * Reference Vpp at 166/332/498 kHz: 50, 80, 200 mV.
 * Reported Vpp before this correction: 52.999798, 84.248497,
 * 211.139938 mV.  A zero-intercept least-squares fit gives this one scalar.
 * It is deliberately not a frequency table, so it cannot reshape spectra.
 */
#define SIGNAL_ADC_AD9280_CAL_AC_AMPLITUDE_SCALE  0.947345031f
#define SIGNAL_ADC_AD9280_CAL_AC_SOURCE            \
    "2026-07-29 Vpp LS 166/332/498kHz"

#endif /* SIGNAL_ADC_AD9280_CALIBRATION_H */
