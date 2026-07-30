/*
 * 分数延时器主机回归测试。
 * 本文件直接调用 DSP/FilterEx.c 中已封装的实际 C 函数，不使用 Python 模型替代。
 */
#include <stdint.h>
#include <stddef.h>

#include "FilterEx.h"

#if defined(_MSC_VER)
#define FRACTIONAL_DELAY_TEST_EXPORT __declspec(dllexport) /* 导出给 Python 测试脚本调用。 */
#else
#define FRACTIONAL_DELAY_TEST_EXPORT
#endif

#define FRACTIONAL_DELAY_TEST_FS_HZ       1000000.0f /* 测试采样率：1 MSPS。 */
#define FRACTIONAL_DELAY_TEST_FREQ_HZ      100000.0f /* 测试信号频率：100 kHz。 */
#define FRACTIONAL_DELAY_TEST_SAMPLES          400u /* 稳态测量采样点数。 */
#define FRACTIONAL_DELAY_TEST_WARMUP            40u /* 丢弃的启动瞬态采样点数。 */
#define FRACTIONAL_DELAY_TEST_STORAGE           64u /* 延时器历史缓冲区长度。 */

/* 导出给测试脚本读取的实际 C 运行结果。 */
typedef struct {
    uint32_t failed_checks;       /* 失败的检查项位图，0 表示全部通过。 */
    float configured_delay;       /* SetPhase() 设置得到的延时点数。 */
    float sine_component;         /* 输出相对于输入正弦基的同相分量。 */
    float cosine_component;       /* 输出相对于输入余弦基的正交分量。 */
    float max_integer_error;      /* 50 kHz、90°整数延时的最大误差。 */
    float max_block_error;        /* 原地块处理的最大误差。 */
} FractionalDelay_HostResult_t;

/* 使用循环方式实现 memset，避免主机测试链接到 C 运行库。 */
void *FractionalDelay_HostMemset(void *destination, int value, size_t length)
{
    volatile uint8_t *bytes = (volatile uint8_t *)destination;
    for (size_t i = 0u; i < length; ++i) {
        bytes[i] = (uint8_t)value;
    }
    return destination;
}

/* 禁用 C 运行库后，链接器仍会要求的浮点标记。 */
int _fltused = 1;

/* FilterEx.c 中未在本测试调用的数学函数占位实现。 */
float fabsf(float x) { return (x >= 0.0f) ? x : -x; }
float cosf(float x) { return x; }
float expf(float x) { return x; }
float powf(float x, float y) { return x + y; }
float sinf(float x) { return x; }
float sqrtf(float x) { return x; }
float tanf(float x) { return x; }

/* 返回两个浮点数的绝对差。 */
static float FractionalDelay_TestAbsDiff(float left, float right)
{
    const float difference = left - right;
    return (difference >= 0.0f) ? difference : -difference;
}

/* 检查 SetPhase() 对频率和相移角的换算结果。 */
static uint32_t FractionalDelay_TestPhaseMapping(FilterEx_FractionalDelay_t *delay)
{
    static const float frequencies[] = { 10000.0f, 50000.0f, 100000.0f };
    static const float phases[] = { 0.0f, 45.0f, 90.0f, 135.0f, 180.0f };
    uint32_t failures = 0u;

    for (uint32_t f = 0u; f < 3u; ++f) {
        for (uint32_t p = 0u; p < 5u; ++p) {
            const float expected = FRACTIONAL_DELAY_TEST_FS_HZ * phases[p]
                                 / (360.0f * frequencies[f]);
            const uint32_t status = (uint32_t)FilterEx_FractionalDelaySetPhase(
                delay, FRACTIONAL_DELAY_TEST_FS_HZ, frequencies[f], phases[p]);
            const float actual = (float)delay->integer_delay + delay->fraction;

            if (status != (uint32_t)FILTEREX_OK
                || FractionalDelay_TestAbsDiff(actual, expected) > 0.0001f) {
                failures++;
            }
        }
    }
    return failures;
}

/* 运行实际 C 实现的分数延时器测试，返回 0 表示通过。 */
FRACTIONAL_DELAY_TEST_EXPORT uint32_t FractionalDelay_HostTest(
    FractionalDelay_HostResult_t *result)
{
    static const float sine_lut[10] = {
        0.0f, 0.587785252f, 0.951056516f, 0.951056516f, 0.587785252f,
        0.0f, -0.587785252f, -0.951056516f, -0.951056516f, -0.587785252f
    };
    static const float cosine_lut[10] = {
        1.0f, 0.809016994f, 0.309016994f, -0.309016994f, -0.809016994f,
        -1.0f, -0.809016994f, -0.309016994f, 0.309016994f, 0.809016994f
    };
    float storage[FRACTIONAL_DELAY_TEST_STORAGE]; /* 分数延时器历史缓冲区。 */
    FilterEx_FractionalDelay_t delay;             /* 被测分数延时器。 */
    uint32_t measured_samples = 0u;               /* 稳态测量点数。 */

    if (result == NULL) return 1u;
    FractionalDelay_HostMemset(result, 0, sizeof(*result));

    if (FilterEx_FractionalDelayInit(&delay, storage,
                                     FRACTIONAL_DELAY_TEST_STORAGE) != FILTEREX_OK) {
        result->failed_checks |= 1u;
        return result->failed_checks;
    }
    if (FilterEx_FractionalDelayInit(&delay, storage, 1u) != FILTEREX_ERROR_CAPACITY) {
        result->failed_checks |= 2u;
    }
    if (FilterEx_FractionalDelayInit(&delay, storage,
                                     FRACTIONAL_DELAY_TEST_STORAGE) != FILTEREX_OK) {
        result->failed_checks |= 4u;
        return result->failed_checks;
    }

    if (FractionalDelay_TestPhaseMapping(&delay) != 0u) {
        result->failed_checks |= 8u;
    }
    if (FilterEx_FractionalDelaySetPhase(&delay, FRACTIONAL_DELAY_TEST_FS_HZ,
                                         0.0f, 90.0f) != FILTEREX_ERROR_PARAMETER
        || FilterEx_FractionalDelaySetPhase(&delay, FRACTIONAL_DELAY_TEST_FS_HZ,
                                            50000.0f, 181.0f) != FILTEREX_ERROR_PARAMETER) {
        result->failed_checks |= 16u;
    }

    if (FilterEx_FractionalDelaySetPhase(&delay, FRACTIONAL_DELAY_TEST_FS_HZ,
                                         FRACTIONAL_DELAY_TEST_FREQ_HZ,
                                         90.0f) != FILTEREX_OK) {
        result->failed_checks |= 32u;
        return result->failed_checks;
    }
    result->configured_delay = (float)delay.integer_delay + delay.fraction;

    for (uint32_t n = 0u; n < FRACTIONAL_DELAY_TEST_SAMPLES
                              + FRACTIONAL_DELAY_TEST_WARMUP; ++n) {
        const uint32_t lut_index = n % 10u;
        const float output = FilterEx_FractionalDelayProcess(&delay, sine_lut[lut_index]);

        if (n >= FRACTIONAL_DELAY_TEST_WARMUP) {
            result->sine_component += output * sine_lut[lut_index];
            result->cosine_component += output * cosine_lut[lut_index];
            measured_samples++;
        }
    }
    result->sine_component *= 2.0f / (float)measured_samples;
    result->cosine_component *= 2.0f / (float)measured_samples;

    if (FilterEx_FractionalDelaySetPhase(&delay, FRACTIONAL_DELAY_TEST_FS_HZ,
                                         50000.0f, 90.0f) != FILTEREX_OK) {
        result->failed_checks |= 64u;
    } else {
        FilterEx_FractionalDelayReset(&delay);
        for (uint32_t n = 0u; n < 24u; ++n) {
            const float input = (float)n;
            const float output = FilterEx_FractionalDelayProcess(&delay, input);
            if (n >= 5u) {
                const float error = FractionalDelay_TestAbsDiff(output, (float)(n - 5u));
                if (error > result->max_integer_error) result->max_integer_error = error;
            }
        }
    }

    {
        float samples[6] = { 1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f }; /* 原地块处理样本。 */
        if (FilterEx_FractionalDelaySetSamples(&delay, 0.5f) != FILTEREX_OK) {
            result->failed_checks |= 128u;
        } else {
            FilterEx_FractionalDelayReset(&delay);
            FilterEx_FractionalDelayProcessBlock(&delay, samples, samples, 6u);
            for (uint32_t n = 0u; n < 6u; ++n) {
                const float expected = (float)n + 0.5f;
                const float error = FractionalDelay_TestAbsDiff(samples[n], expected);
                if (error > result->max_block_error) result->max_block_error = error;
            }
        }
    }

    if (result->configured_delay < 2.4999f || result->configured_delay > 2.5001f
        || FractionalDelay_TestAbsDiff(result->sine_component, 0.0f) > 0.001f
        || FractionalDelay_TestAbsDiff(result->cosine_component, -0.951056516f) > 0.001f
        || result->max_integer_error > 0.0001f
        || result->max_block_error > 0.0001f) {
        result->failed_checks |= 256u;
    }
    return result->failed_checks;
}
