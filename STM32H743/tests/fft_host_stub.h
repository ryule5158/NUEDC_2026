#ifndef FFT_H
#define FFT_H
#include <stdbool.h>
#include <stdint.h>
#define FFT_length 4096U
extern float FFT_normalized_output[FFT_length / 2U + 1U];
bool FFT_StartN(const float *data, uint32_t len);
bool Normalize_FFT_To_Single_SideN(uint32_t len);
#endif
