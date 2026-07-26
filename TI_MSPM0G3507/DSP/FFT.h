#ifndef FFT_H
#define FFT_H

#include "main.h"
#include "arm_math.h"
#include "arm_const_structs.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define FFT_length 512U /* 固定FFT点数。 */
#define FFT_MAX_HARMONICS 12U /* 最大谐波输出数量。 */

extern float FFT_Fs; /* 当前采样率，单位Hz。 */
extern float FFT_input[FFT_length]; /* FFT实数输入帧。 */
extern float FFT_Buffer[FFT_length * 2U]; /* 交错复数运算缓冲区。 */
extern float FFT_output[FFT_length]; /* 双边幅度谱。 */
extern float FFT_normalized_output[FFT_length / 2U + 1U]; /* 单边幅度谱。 */
extern float FFT_mag_max; /* 最近一次峰值谱幅。 */
extern uint32_t mag_max_index; /* 最近一次峰值频点索引。 */
extern volatile float FFT_Amplitude; /* 最近一次峰值幅值。 */
extern volatile float FFT_Frequency; /* 最近一次峰值频率，单位Hz。 */

void FFT_Input(const float *arr);
void Sin_Simulation(void);
void FFT_Start(void);
void Process_FFT_mag(void);
void Normalize_FFT_To_Single_Side(void);
void FFT_Data_Print(void);
float FFT_InterpolateHannPeak(uint32_t bin, float *corrected_magnitude);
float FFT_AmpAtFreq(const float *data, uint32_t len, float fs, float freq);
uint8_t FFT_GetHarmonics(const float *data, uint32_t len, float fs, float f0,
                         float max_freq, float freq_out[], float amp_out[]);
uint8_t FFT_GetHarmonics_FromFFT(float fs, float f0, float max_freq,
                                 float freq_out[], float amp_out[]);

typedef enum {
    WIN_RECT = 0,       /* 矩形窗。 */
    WIN_HANN,           /* Hann窗。 */
    WIN_HAMMING,        /* Hamming窗。 */
    WIN_BLACKMAN,       /* Blackman窗。 */
    WIN_BLACKMAN_HARRIS,/* Blackman-Harris窗。 */
    WIN_FLATTOP         /* 平顶窗。 */
} Window_Type_t;

void Window_Generate(float *w, uint32_t n, Window_Type_t type,
                     float *cg, float *enbw);
void Window_Apply(float *x, const float *w, uint32_t n);

#endif /* FFT_H */
