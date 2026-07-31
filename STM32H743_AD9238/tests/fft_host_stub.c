#include "FFT.h"
#include <math.h>
#include <stddef.h>

#define HOST_PI 3.14159265358979323846
float FFT_normalized_output[FFT_length / 2U + 1U];

bool FFT_StartN(const float *data, uint32_t len)
{
    double win_sum = 0.0;
    if ((data == NULL) || (len < 64U) || (len > FFT_length) ||
        ((len & (len - 1U)) != 0U)) return false;

    for (uint32_t i = 0U; i < len; ++i) {
        win_sum += 0.5 - 0.5 * cos(2.0 * HOST_PI * (double)i /
                                  (double)(len - 1U));
    }
    for (uint32_t k = 0U; k <= len / 2U; ++k) {
        double re = 0.0;
        double im = 0.0;
        for (uint32_t i = 0U; i < len; ++i) {
            const double w = 0.5 - 0.5 * cos(2.0 * HOST_PI * (double)i /
                                             (double)(len - 1U));
            const double phase = -2.0 * HOST_PI * (double)k *
                                 (double)i / (double)len;
            re += (double)data[i] * w * cos(phase);
            im += (double)data[i] * w * sin(phase);
        }
        FFT_normalized_output[k] = (float)(sqrt(re * re + im * im) *
            ((k == 0U || k == len / 2U) ? 1.0 : 2.0) / win_sum);
    }
    return true;
}

bool Normalize_FFT_To_Single_SideN(uint32_t len)
{
    return (len >= 64U) && (len <= FFT_length) &&
           ((len & (len - 1U)) == 0U);
}
