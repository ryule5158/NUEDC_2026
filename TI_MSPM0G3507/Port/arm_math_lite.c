#include "arm_const_structs.h"
#include <math.h>
#include <string.h>

#define ARM_MATH_LITE_MAX_CFFT 512U /* 轻量复数FFT最大点数。 */
#define ARM_MATH_LITE_PI 3.14159265358979323846f /* 单精度圆周率。 */

/* CMSIS-DSP兼容的固定FFT长度描述符。 */
const arm_cfft_instance_f32 arm_cfft_sR_f32_len64 = {64U};
const arm_cfft_instance_f32 arm_cfft_sR_f32_len128 = {128U};
const arm_cfft_instance_f32 arm_cfft_sR_f32_len256 = {256U};
const arm_cfft_instance_f32 arm_cfft_sR_f32_len512 = {512U};

/* 使用C数学库实现公共驱动需要的单精度正弦接口。 */
float32_t arm_sin_f32(float32_t x)
{
    return sinf(x);
}

/* 判断FFT长度是否为2的整数次幂。 */
static uint8_t ARM_MathLite_IsPowerOfTwo(uint32_t value)
{
    return ((value != 0U) && ((value & (value - 1U)) == 0U)) ? 1U : 0U;
}

/* 对交错复数数组执行原地位逆序重排。 */
static void ARM_MathLite_BitReverse(float32_t *data, uint32_t length)
{
    uint32_t reversed = 0U;

    for (uint32_t index = 1U; index < length; index++) {
        uint32_t bit = length >> 1U;

        while ((reversed & bit) != 0U) {
            reversed ^= bit;
            bit >>= 1U;
        }
        reversed ^= bit;

        if (index < reversed) {
            float32_t temp;

            temp = data[2U * index];
            data[2U * index] = data[2U * reversed];
            data[2U * reversed] = temp;

            temp = data[(2U * index) + 1U];
            data[(2U * index) + 1U] = data[(2U * reversed) + 1U];
            data[(2U * reversed) + 1U] = temp;
        }
    }
}

/* 使用原地radix-2算法实现512点以内的兼容复数FFT。 */
void arm_cfft_f32(const arm_cfft_instance_f32 *S,
                  float32_t *p1,
                  uint8_t ifftFlag,
                  uint8_t bitReverseFlag)
{
    uint32_t length;

    if ((S == NULL) || (p1 == NULL)) {
        return;
    }

    length = S->fftLen;
    if ((length > ARM_MATH_LITE_MAX_CFFT) ||
        (ARM_MathLite_IsPowerOfTwo(length) == 0U)) {
        return;
    }

    ARM_MathLite_BitReverse(p1, length);

    for (uint32_t stage_size = 2U;
         stage_size <= length;
         stage_size <<= 1U) {
        uint32_t half_size = stage_size >> 1U;
        float32_t direction = (ifftFlag == 0U) ? -1.0f : 1.0f;
        float32_t angle = direction * 2.0f * ARM_MATH_LITE_PI /
                          (float32_t)stage_size;
        float32_t step_re = cosf(angle);
        float32_t step_im = sinf(angle);

        for (uint32_t block = 0U;
             block < length;
             block += stage_size) {
            float32_t twiddle_re = 1.0f;
            float32_t twiddle_im = 0.0f;

            for (uint32_t offset = 0U; offset < half_size; offset++) {
                uint32_t even = block + offset;
                uint32_t odd = even + half_size;
                float32_t odd_re = p1[2U * odd];
                float32_t odd_im = p1[(2U * odd) + 1U];
                float32_t product_re =
                    (twiddle_re * odd_re) - (twiddle_im * odd_im);
                float32_t product_im =
                    (twiddle_re * odd_im) + (twiddle_im * odd_re);
                float32_t even_re = p1[2U * even];
                float32_t even_im = p1[(2U * even) + 1U];
                float32_t next_twiddle_re =
                    (twiddle_re * step_re) - (twiddle_im * step_im);

                p1[2U * even] = even_re + product_re;
                p1[(2U * even) + 1U] = even_im + product_im;
                p1[2U * odd] = even_re - product_re;
                p1[(2U * odd) + 1U] = even_im - product_im;

                twiddle_im =
                    (twiddle_re * step_im) + (twiddle_im * step_re);
                twiddle_re = next_twiddle_re;
            }
        }
    }

    if (ifftFlag != 0U) {
        float32_t scale = 1.0f / (float32_t)length;

        for (uint32_t index = 0U; index < (2U * length); index++) {
            p1[index] *= scale;
        }
    }

    if (bitReverseFlag == 0U) {
        ARM_MathLite_BitReverse(p1, length);
    }
}

/* 计算交错复数数组的幅值。 */
void arm_cmplx_mag_f32(const float32_t *pSrc,
                       float32_t *pDst,
                       uint32_t numSamples)
{
    if ((pSrc == NULL) || (pDst == NULL)) {
        return;
    }

    for (uint32_t index = 0U; index < numSamples; index++) {
        float32_t real = pSrc[2U * index];
        float32_t imag = pSrc[(2U * index) + 1U];

        pDst[index] = sqrtf((real * real) + (imag * imag));
    }
}

/* 查找单精度数组的最大值及索引。 */
void arm_max_f32(const float32_t *pSrc,
                 uint32_t blockSize,
                 float32_t *pResult,
                 uint32_t *pIndex)
{
    float32_t max_value;
    uint32_t max_index = 0U;

    if ((pSrc == NULL) || (pResult == NULL) || (pIndex == NULL) ||
        (blockSize == 0U)) {
        return;
    }

    max_value = pSrc[0];
    for (uint32_t index = 1U; index < blockSize; index++) {
        if (pSrc[index] > max_value) {
            max_value = pSrc[index];
            max_index = index;
        }
    }

    *pResult = max_value;
    *pIndex = max_index;
}

/* 初始化轻量FIR实例并清零状态。 */
void arm_fir_init_f32(arm_fir_instance_f32 *S,
                      uint16_t numTaps,
                      const float32_t *pCoeffs,
                      float32_t *pState,
                      uint32_t blockSize)
{
    if (S == NULL) {
        return;
    }

    S->numTaps = numTaps;
    S->pCoeffs = pCoeffs;
    S->pState = pState;
    S->blockSize = blockSize;

    if ((pState != NULL) && (numTaps > 0U) && (blockSize > 0U)) {
        memset(pState,
               0,
               sizeof(float32_t) * ((uint32_t)numTaps + blockSize - 1U));
    }
}

/* 对输入数据执行轻量FIR滤波。 */
void arm_fir_f32(const arm_fir_instance_f32 *S,
                 const float32_t *pSrc,
                 float32_t *pDst,
                 uint32_t blockSize)
{
    uint32_t taps;

    if ((S == NULL) || (pSrc == NULL) || (pDst == NULL) ||
        (S->pCoeffs == NULL) || (S->pState == NULL) ||
        (S->numTaps == 0U)) {
        return;
    }

    taps = S->numTaps;
    for (uint32_t sample = 0U; sample < blockSize; sample++) {
        float32_t accumulator = 0.0f;

        /* CMSIS系数按时间逆序存放，状态按“最旧到最新”排列。 */
        for (uint32_t index = 0U; index < (taps - 1U); index++) {
            S->pState[index] = S->pState[index + 1U];
        }
        S->pState[taps - 1U] = pSrc[sample];

        for (uint32_t index = 0U; index < taps; index++) {
            accumulator += S->pState[index] * S->pCoeffs[index];
        }
        pDst[sample] = accumulator;
    }
}
