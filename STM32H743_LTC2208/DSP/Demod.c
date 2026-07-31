/**
 * @file    Demod.c
 * @brief   AM/FM demodulation and analysis implementation.
 */
#include "Demod.h"
#include "arm_const_structs.h"
#include "FFT.h"
#include <float.h>
#include <math.h>
#include <string.h>

#define DEMOD_PI          3.14159265358979323846f /* 单精度圆周率。 */
#define DEMOD_2PI         6.28318530717958647692f /* 单精度二倍圆周率。 */
#define DEMOD_EPS         1.0e-12f               /* 浮点除法与幅值判定保护量。 */
#define DEMOD_MAX_POINTS  4096U                  /* 内部排序工作区最大点数。 */

#define DEMOD_LOOP_MIN_HZ          1.0f   /* 环路速度参数下限。 */
#define DEMOD_PD_LPF_RATIO         4.0f   /* 鉴相低通相对环路速度倍数。 */
#define DEMOD_CAPTURE_MIN_HZ       10.0f  /* 最小捕获频率范围。 */
#define DEMOD_CAPTURE_BW_RATIO     6.0f   /* 捕获范围相对环路速度倍数。 */
#define DEMOD_CAPTURE_FC_RATIO     0.05f  /* 捕获范围相对载频比例。 */
#define DEMOD_LEVEL_TC_SEC         0.001f /* 幅值估计时间常数。 */
#define DEMOD_ERROR_TC_SEC         0.0005f /* 鉴相误差平滑时间常数。 */
#define DEMOD_LOCK_TIME_SEC        0.0002f /* 锁定判定持续时间。 */
#define DEMOD_SIGNAL_GATE          1.0e-4f /* 信号幅值有效门限。 */
#define DEMOD_POWER_GATE           (DEMOD_SIGNAL_GATE * DEMOD_SIGNAL_GATE) /* 信号功率有效门限。 */
#define DEMOD_LOCK_ERR_LIMIT       0.10f  /* 锁定误差上限。 */
#define DEMOD_NORM_ERR_LIMIT       1.0f   /* 归一化鉴相误差限幅。 */
#define DEMOD_LOCK_COUNT_MAX       60000U /* 锁定计数器饱和值。 */

/* 兼容旧接口和单帧分析的静态工作区。
 * 注意：这两个缓冲区不是可重入设计；ISR/多任务并发调用时应改成由
 * Context 或调用者提供工作区。 */
static float s_order_work[DEMOD_MAX_POINTS]; /* 百分位与中值排序工作区。 */
static float s_freq_work[DEMOD_MAX_POINTS];  /* 瞬时频率统计工作区。 */

/* 查询解析信号所需的 float 工作区长度。 */
uint32_t Demod_AnalyticWorkFloats(uint32_t n)
{
    return (n <= (UINT32_MAX / 2U)) ? (2U * n) : 0U;
}

/* 查询 FM 处理所需的 float 工作区长度。 */
uint32_t Demod_FMWorkFloats(uint32_t n)
{
    return Demod_AnalyticWorkFloats(n);
}

/* 按点数选择 CMSIS-DSP 复数 FFT 实例。 */
static const arm_cfft_instance_f32 *Demod_CfftSel(uint32_t n)
{
    switch (n) {
    case 64:   return &arm_cfft_sR_f32_len64;
    case 128:  return &arm_cfft_sR_f32_len128;
    case 256:  return &arm_cfft_sR_f32_len256;
    case 512:  return &arm_cfft_sR_f32_len512;
    case 1024: return &arm_cfft_sR_f32_len1024;
    case 2048: return &arm_cfft_sR_f32_len2048;
    case 4096: return &arm_cfft_sR_f32_len4096;
    default:   return NULL;
    }
}

/* 将数值限制到给定闭区间。 */
static float Demod_ClampF(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

/* 返回两个单精度数中的较大值。 */
static float Demod_MaxF(float a, float b)
{
    return (a > b) ? a : b;
}

/* 返回有效正数，否则返回备用值。 */
static float Demod_PositiveOr(float x, float fallback)
{
    return (x > 0.0f) ? x : fallback;
}

/* 根据采样率和时间常数计算一阶平滑系数。 */
static float Demod_TimeAlpha(float fs, float tau_sec)
{
    float alpha;

    if ((fs <= 0.0f) || (tau_sec <= 0.0f)) {
        return 1.0f;
    }
    alpha = 1.0f - expf(-1.0f / (fs * tau_sec));
    return Demod_ClampF(alpha, 1.0e-6f, 1.0f);
}

/* 将持续时间换算为饱和的采样计数。 */
static uint16_t Demod_TimeCount(float fs, float seconds)
{
    float count;

    if ((fs <= 0.0f) || (seconds <= 0.0f)) {
        return 1U;
    }
    count = fs * seconds + 0.5f;
    count = Demod_ClampF(count, 1.0f, (float)DEMOD_LOCK_COUNT_MAX);
    return (uint16_t)count;
}

/* 将环路速度限制到采样率允许范围。 */
static float Demod_SafeLoopHz(float fs, float loop_hz)
{
    float max_hz = (fs > 0.0f) ? (0.45f * fs) : DEMOD_LOOP_MIN_HZ;

    return Demod_ClampF(Demod_PositiveOr(loop_hz, DEMOD_LOOP_MIN_HZ),
                        DEMOD_LOOP_MIN_HZ,
                        max_hz);
}

/* 选择鉴相器低通截止频率。 */
static float Demod_SelectPdLpfHz(float fs, float loop_hz)
{
    float pd_hz = Demod_SafeLoopHz(fs, loop_hz) * DEMOD_PD_LPF_RATIO;

    if (fs > 0.0f) {
        pd_hz = Demod_ClampF(pd_hz, DEMOD_LOOP_MIN_HZ, 0.45f * fs);
    }
    return pd_hz;
}

/* 根据采样率、载频和环路速度计算捕获频率范围。 */
static void Demod_LoopLimits(float fs,
                             float carrier_hz,
                             float loop_hz,
                             float *center,
                             float *min_freq,
                             float *max_freq)
{
    float safe_carrier = Demod_PositiveOr(carrier_hz, 0.0f);
    float span_hz = Demod_MaxF(DEMOD_CAPTURE_BW_RATIO * Demod_SafeLoopHz(fs, loop_hz),
                               DEMOD_CAPTURE_FC_RATIO * safe_carrier);

    span_hz = Demod_MaxF(span_hz, DEMOD_CAPTURE_MIN_HZ);
    if (fs > 0.0f) {
        float min_hz = Demod_ClampF(safe_carrier - span_hz, 0.0f, 0.49f * fs);
        float max_hz = Demod_ClampF(safe_carrier + span_hz, min_hz, 0.49f * fs);

        *center = DEMOD_2PI * safe_carrier / fs;
        *min_freq = DEMOD_2PI * min_hz / fs;
        *max_freq = DEMOD_2PI * max_hz / fs;
    } else {
        *center = 0.0f;
        *min_freq = 0.0f;
        *max_freq = 0.0f;
    }
}

/* 更新锁定计数器与锁定标志。 */
static void Demod_UpdateLock(uint8_t ok,
                             uint16_t target,
                             uint16_t *count,
                             uint8_t *locked)
{
    if (ok != 0U) {
        if (*count < DEMOD_LOCK_COUNT_MAX) {
            (*count)++;
        }
    } else {
        *count = 0U;
    }
    *locked = (*count >= target) ? 1U : 0U;
}

/* 判断单精度数是否为有限值。 */
static uint8_t Demod_IsFinite(float x)
{
    return ((x == x) && (x <= FLT_MAX) && (x >= -FLT_MAX)) ? 1U : 0U;
}

/* 生成单精度 NaN。 */
static float Demod_Nan(void)
{
    /* 用 NaN 标记“该样本无效”。后续统计函数会跳过 NaN，避免低幅度
     * 或未锁定阶段的错误频率污染中心频率、频偏和拟合结果。 */
    volatile float zero = 0.0f;
    return zero / zero;
}

/* 将弧度相位归一化到 [-pi, pi]。 */
static float Demod_WrapPi(float x)
{
    if (Demod_IsFinite(x) == 0U) return 0.0f;
    x = fmodf(x, DEMOD_2PI);
    if (x > DEMOD_PI) x -= DEMOD_2PI;
    if (x <= -DEMOD_PI) x += DEMOD_2PI;
    return x;
}

/* 判断两段内存范围是否重叠。 */
static uint8_t Demod_RangesOverlap(const void *a, uint32_t a_bytes,
                                   const void *b, uint32_t b_bytes)
{
    const uintptr_t pa = (uintptr_t)a;
    const uintptr_t pb = (uintptr_t)b;

    if (a == NULL || b == NULL || a_bytes == 0U || b_bytes == 0U) return 0U;
    if (pa <= pb) return ((pb - pa) < (uintptr_t)a_bytes) ? 1U : 0U;
    return ((pa - pb) < (uintptr_t)b_bytes) ? 1U : 0U;
}

/* 清零 AM 结果并写入状态码。 */
static void Demod_ClearAMResult(Demod_AMResult *result, Demod_Status status)
{
    if (result == NULL) return;
    memset(result, 0, sizeof(*result));
    result->status = status;
}

/* 清零 FM 结果并写入状态码。 */
static void Demod_ClearFMResult(Demod_FMResult *result, Demod_Status status)
{
    if (result == NULL) return;
    memset(result, 0, sizeof(*result));
    result->status = status;
}

/* 检查预处理配置是否合法。 */
static Demod_Status Demod_ValidatePreprocess(const Demod_PreprocessConfig *cfg)
{
    if (cfg == NULL) return DEMOD_OK;
    if ((Demod_IsFinite(cfg->fs_hz) == 0U) ||
        (Demod_IsFinite(cfg->carrier_min_hz) == 0U) ||
        (Demod_IsFinite(cfg->carrier_max_hz) == 0U) ||
        (Demod_IsFinite(cfg->amplitude_gate) == 0U) ||
        (cfg->amplitude_gate < 0.0f)) {
        return DEMOD_ERR_BAD_CONFIG;
    }
    if (cfg->bandpass_enable != 0U) {
        if ((cfg->fs_hz <= 0.0f) ||
            (cfg->carrier_min_hz <= 0.0f) ||
            (cfg->carrier_max_hz <= cfg->carrier_min_hz) ||
            (cfg->carrier_max_hz >= 0.5f * cfg->fs_hz)) {
            return DEMOD_ERR_BAD_CONFIG;
        }
    }
    return DEMOD_OK;
}

/* 计算丢弃两端样本后的有效分析区间。 */
static void Demod_Window(uint32_t n, uint32_t discard, uint32_t *begin, uint32_t *end)
{
    /* FFT Hilbert 的帧首/帧尾容易受周期延拓影响。这里仅把边缘样本从
     * 参数分析中排除，不会从输出波形数组中删除这些样本。 */
    if ((discard >= n) || ((n > 0U) && (discard > ((n - 1U) / 2U)))) {
        discard = 0U;
    }
    *begin = discard;
    *end = n - discard;
}

/* 计算指定区间内有限样本的均值。 */
static float Demod_Mean(const float *x, uint32_t begin, uint32_t end)
{
    float sum = 0.0f;
    uint32_t count = 0U;

    for (uint32_t i = begin; i < end; i++) {
        if (Demod_IsFinite(x[i]) != 0U) {
            sum += x[i];
            count++;
        }
    }
    return (count > 0U) ? (sum / (float)count) : 0.0f;
}

/* 统计指定区间内有限样本数量。 */
static uint32_t Demod_CountFinite(const float *x, uint32_t begin, uint32_t end)
{
    uint32_t count = 0U;

    if (x == NULL) return 0U;
    for (uint32_t i = begin; i < end; i++) {
        if (Demod_IsFinite(x[i]) != 0U) {
            count++;
        }
    }
    return count;
}

/* 计算指定区间相对中心值的均方根。 */
static float Demod_RmsAround(const float *x, uint32_t begin, uint32_t end, float center)
{
    float sum2 = 0.0f;
    uint32_t count = 0U;

    for (uint32_t i = begin; i < end; i++) {
        if (Demod_IsFinite(x[i]) != 0U) {
            float d = x[i] - center;
            sum2 += d * d;
            count++;
        }
    }
    return (count > 0U) ? sqrtf(sum2 / (float)count) : 0.0f;
}

/* 交换两个单精度数。 */
static void Demod_SwapF(float *a, float *b)
{
    float t = *a;
    *a = *b;
    *b = t;
}

/* 使用快速选择取得第 k 个顺序统计量。 */
static float Demod_QuickSelect(float *a, uint32_t n, uint32_t k)
{
    /* 只选第 k 个顺序统计量，不完整排序。比原来的直方图百分位更稳：
     * 极端尖峰不会拉宽统计范围，也不会降低百分位分辨率。 */
    uint32_t left = 0U;
    uint32_t right = n - 1U;

    while (left < right) {
        float pivot = a[(left + right) / 2U];
        uint32_t i = left;
        uint32_t j = right;

        while (i <= j) {
            while (a[i] < pivot) i++;
            while (a[j] > pivot) {
                if (j == 0U) break;
                j--;
            }
            if (i <= j) {
                Demod_SwapF(&a[i], &a[j]);
                i++;
                if (j == 0U) break;
                j--;
            }
        }

        if (k <= j) {
            right = j;
        } else if (k >= i) {
            left = i;
        } else {
            break;
        }
    }
    return a[k];
}

/* 将有效区间内的有限样本复制到内部排序工作区。 */
static Demod_Status Demod_CopyFiniteWindow(const float *x,
                                           uint32_t begin,
                                           uint32_t end,
                                           float *out,
                                           uint32_t out_cap,
                                           uint32_t *out_count)
{
    uint32_t count = 0U;

    if (x == NULL || out == NULL || out_count == NULL) return DEMOD_ERR_NULL;
    for (uint32_t i = begin; i < end; i++) {
        if (Demod_IsFinite(x[i]) != 0U) {
            if (count >= out_cap) return DEMOD_ERR_BUFFER_SIZE;
            out[count++] = x[i];
        }
    }
    *out_count = count;
    return (count > 0U) ? DEMOD_OK : DEMOD_ERR_NO_VALID_DATA;
}

/* 精确计算有效样本的指定百分位数。 */
static Demod_Status Demod_PercentileExact(const float *x,
                                          uint32_t begin,
                                          uint32_t end,
                                          float p,
                                          float *value)
{
    /* 百分位统计先复制有效样本到 s_order_work，再 QuickSelect。
     * NaN/Inf 会被跳过，因此幅度门控和 PLL 未锁定产生的无效点不会参与统计。 */
    uint32_t count = 0U;
    uint32_t k;
    Demod_Status st;

    if (value == NULL) return DEMOD_ERR_NULL;
    if (x == NULL || begin >= end) return DEMOD_ERR_NO_VALID_DATA;

    p = Demod_ClampF(p, 0.0f, 1.0f);
    st = Demod_CopyFiniteWindow(x, begin, end, s_order_work, DEMOD_MAX_POINTS, &count);
    if (st != DEMOD_OK) return st;

    k = (uint32_t)(p * (float)(count - 1U) + 0.5f);
    *value = Demod_QuickSelect(s_order_work, count, k);
    return DEMOD_OK;
}

/* 以已知频率正弦最小二乘拟合幅值、直流与残差。 */
static Demod_Status Demod_SineFit(const float *x,
                                  uint32_t begin,
                                  uint32_t end,
                                  float fs,
                                  float tone_hz,
                                  float *center,
                                  float *amp,
                                  float *residual_rms)
{
    /* 拟合 y = C + a*cos(wt) + b*sin(wt)。
     * 这比直接找峰谷抗噪，但要求调用者提供已知调制频率 tone_hz。 */
    float sum_y = 0.0f;
    float sum_c = 0.0f;
    float sum_s = 0.0f;
    float sum_cc = 0.0f;
    float sum_cs = 0.0f;
    float sum_ss = 0.0f;
    float sum_yc = 0.0f;
    float sum_ys = 0.0f;
    float c0;
    float a;
    float b;
    float res2 = 0.0f;
    uint32_t count = 0U;

    if (x == NULL || fs <= 0.0f || tone_hz <= 0.0f || begin >= end) {
        return DEMOD_ERR_BAD_FS;
    }

    for (uint32_t i = begin; i < end; i++) {
        float ph = DEMOD_2PI * tone_hz * (float)i / fs;
        float c = cosf(ph);
        float s = sinf(ph);
        float y = x[i];
        if (Demod_IsFinite(y) == 0U) {
            continue;
        }
        sum_y += y;
        sum_c += c;
        sum_s += s;
        sum_cc += c * c;
        sum_cs += c * s;
        sum_ss += s * s;
        sum_yc += y * c;
        sum_ys += y * s;
        count++;
    }

    if (count < 4U) {
        return DEMOD_ERR_LENGTH;
    }

    {
        float n_f = (float)count;
        float det =
            n_f * (sum_cc * sum_ss - sum_cs * sum_cs) -
            sum_c * (sum_c * sum_ss - sum_cs * sum_s) +
            sum_s * (sum_c * sum_cs - sum_cc * sum_s);
        float det_c;
        float det_a;
        float det_b;

        if (fabsf(det) <= DEMOD_EPS) {
            return DEMOD_ERR_NO_VALID_DATA;
        }

        det_c =
            sum_y * (sum_cc * sum_ss - sum_cs * sum_cs) -
            sum_c * (sum_yc * sum_ss - sum_cs * sum_ys) +
            sum_s * (sum_yc * sum_cs - sum_cc * sum_ys);
        det_a =
            n_f * (sum_yc * sum_ss - sum_cs * sum_ys) -
            sum_y * (sum_c * sum_ss - sum_cs * sum_s) +
            sum_s * (sum_c * sum_ys - sum_yc * sum_s);
        det_b =
            n_f * (sum_cc * sum_ys - sum_yc * sum_cs) -
            sum_c * (sum_c * sum_ys - sum_yc * sum_s) +
            sum_y * (sum_c * sum_cs - sum_cc * sum_s);

        c0 = det_c / det;
        a = det_a / det;
        b = det_b / det;
    }

    for (uint32_t i = begin; i < end; i++) {
        float ph = DEMOD_2PI * tone_hz * (float)i / fs;
        float fit = c0 + a * cosf(ph) + b * sinf(ph);
        if (Demod_IsFinite(x[i]) == 0U) {
            continue;
        }
        float e = x[i] - fit;
        res2 += e * e;
    }

    if (center != NULL) *center = c0;
    if (amp != NULL) *amp = sqrtf(a * a + b * b);
    if (residual_rms != NULL) *residual_rms = sqrtf(res2 / (float)count);
    return DEMOD_OK;
}

/* 填充预处理保守默认配置。 */
void Demod_PreprocessDefault(Demod_PreprocessConfig *cfg)
{
    if (cfg == NULL) return;
    cfg->remove_dc = 1U;
    cfg->normalize = 0U;
    cfg->bandpass_enable = 0U;
    cfg->fs_hz = 0.0f;
    cfg->carrier_min_hz = 0.0f;
    cfg->carrier_max_hz = 0.0f;
    cfg->amplitude_gate = 0.0f;
}

/* 填充 AM 解调保守默认配置。 */
void Demod_AMConfigDefault(Demod_AMConfig *cfg, float fs_hz)
{
    if (cfg == NULL) return;
    cfg->method = DEMOD_AM_HILBERT_MAG;
    Demod_PreprocessDefault(&cfg->preprocess);
    cfg->preprocess.fs_hz = fs_hz;
    cfg->estimator = DEMOD_EST_PERCENTILE;
    cfg->output_mode = DEMOD_AM_OUT_ENVELOPE_ABS;
    cfg->edge_discard_samples = 16U;
    cfg->rect_lpf_hz = (fs_hz > 0.0f) ? (0.02f * fs_hz) : 1.0f;
    cfg->rect_gain_correction = 1.0f;
    cfg->tone_hz = 0.0f;
    cfg->fs_hz = fs_hz;
    cfg->carrier_hz = 0.0f;
    cfg->pll_bw_hz = (fs_hz > 0.0f) ? (0.001f * fs_hz) : 1.0f;
    cfg->baseband_lpf_hz = (fs_hz > 0.0f) ? (0.02f * fs_hz) : 1.0f;
}

/* 填充 FM 解调保守默认配置。 */
void Demod_FMConfigDefault(Demod_FMConfig *cfg, float fs_hz)
{
    if (cfg == NULL) return;
    cfg->method = DEMOD_FM_CONJ_PRODUCT;
    Demod_PreprocessDefault(&cfg->preprocess);
    cfg->preprocess.fs_hz = fs_hz;
    cfg->preprocess.amplitude_gate = 0.0f;
    cfg->estimator = DEMOD_EST_PERCENTILE;
    cfg->fc_mode = DEMOD_FC_MEDIAN;
    cfg->edge_discard_samples = 16U;
    cfg->fs_hz = fs_hz;
    cfg->carrier_hz = 0.0f;
    cfg->mod_tone_hz = 0.0f;
    cfg->pll_bw_hz = (fs_hz > 0.0f) ? (0.02f * fs_hz) : 1.0f;
    cfg->baseband_lpf_enable = 0U;
    cfg->baseband_lpf_hz = (fs_hz > 0.0f) ? (0.02f * fs_hz) : 1.0f;
}

/* 检查 AM 配置及所选方法是否合法。 */
static Demod_Status Demod_ValidateAMConfig(const Demod_AMConfig *cfg)
{
    /* 配置检查不仅检查数值范围，也检查“方法/输出模式”是否物理可实现。
     * 例如 RECT/Hilbert 只能得到非负包络，不能恢复过调制的有符号包络。 */
    Demod_PreprocessConfig prep;

    if (cfg == NULL) return DEMOD_ERR_NULL;
    if ((cfg->method > DEMOD_AM_COHERENT_COSTAS) ||
        (cfg->estimator > DEMOD_EST_MEDIAN_MULTI_FRAME) ||
        (cfg->output_mode > DEMOD_AM_OUT_NORMALIZED)) {
        return DEMOD_ERR_UNSUPPORTED;
    }
    if ((Demod_IsFinite(cfg->fs_hz) == 0U) || (cfg->fs_hz <= 0.0f)) {
        return DEMOD_ERR_BAD_FS;
    }
    if ((Demod_IsFinite(cfg->rect_lpf_hz) == 0U) ||
        (Demod_IsFinite(cfg->rect_gain_correction) == 0U) ||
        (Demod_IsFinite(cfg->tone_hz) == 0U) ||
        (Demod_IsFinite(cfg->carrier_hz) == 0U) ||
        (Demod_IsFinite(cfg->pll_bw_hz) == 0U) ||
        (Demod_IsFinite(cfg->baseband_lpf_hz) == 0U)) {
        return DEMOD_ERR_BAD_CONFIG;
    }
    if ((cfg->method == DEMOD_AM_RECT) &&
        ((cfg->rect_lpf_hz <= 0.0f) || (cfg->rect_lpf_hz >= 0.5f * cfg->fs_hz))) {
        return DEMOD_ERR_BAD_CONFIG;
    }
    if ((cfg->method == DEMOD_AM_RECT) && (cfg->rect_gain_correction <= 0.0f)) {
        return DEMOD_ERR_BAD_CONFIG;
    }
    if (((cfg->method == DEMOD_AM_RECT) ||
         (cfg->method == DEMOD_AM_HILBERT_MAG)) &&
        (cfg->output_mode == DEMOD_AM_OUT_ENVELOPE_SIGNED)) {
        return DEMOD_ERR_UNSUPPORTED;
    }
    if ((cfg->tone_hz < 0.0f) || (cfg->tone_hz >= 0.5f * cfg->fs_hz)) {
        return DEMOD_ERR_BAD_CONFIG;
    }
    if ((cfg->method == DEMOD_AM_COHERENT_PLL) ||
        (cfg->method == DEMOD_AM_COHERENT_COSTAS)) {
        if ((cfg->carrier_hz <= 0.0f) ||
            (cfg->carrier_hz >= 0.5f * cfg->fs_hz) ||
            (cfg->pll_bw_hz <= 0.0f) ||
            (cfg->baseband_lpf_hz <= 0.0f) ||
            (cfg->baseband_lpf_hz >= 0.5f * cfg->fs_hz)) {
            return DEMOD_ERR_BAD_CONFIG;
        }
    }

    prep = cfg->preprocess;
    prep.fs_hz = cfg->fs_hz;
    return Demod_ValidatePreprocess(&prep);
}

/* 检查 FM 配置及所选方法是否合法。 */
static Demod_Status Demod_ValidateFMConfig(const Demod_FMConfig *cfg)
{
    /* QUAD_DERIV 是小相位步进近似。若已知数字中频占采样率比例较高，
     * 该方法会系统性低估频偏，直接拒绝比给出漂亮但错误的结果更安全。 */
    Demod_PreprocessConfig prep;

    if (cfg == NULL) return DEMOD_ERR_NULL;
    if ((cfg->method > DEMOD_FM_PLL) ||
        (cfg->estimator > DEMOD_EST_MEDIAN_MULTI_FRAME) ||
        (cfg->fc_mode > DEMOD_FC_MEDIAN)) {
        return DEMOD_ERR_UNSUPPORTED;
    }
    if ((Demod_IsFinite(cfg->fs_hz) == 0U) || (cfg->fs_hz <= 0.0f)) {
        return DEMOD_ERR_BAD_FS;
    }
    if ((Demod_IsFinite(cfg->carrier_hz) == 0U) ||
        (Demod_IsFinite(cfg->mod_tone_hz) == 0U) ||
        (Demod_IsFinite(cfg->pll_bw_hz) == 0U) ||
        (Demod_IsFinite(cfg->baseband_lpf_hz) == 0U)) {
        return DEMOD_ERR_BAD_CONFIG;
    }
    if ((cfg->carrier_hz < 0.0f) || (cfg->carrier_hz >= 0.5f * cfg->fs_hz)) {
        return DEMOD_ERR_BAD_CONFIG;
    }
    if ((cfg->method == DEMOD_FM_QUAD_DERIV) &&
        (cfg->carrier_hz > 0.0f) &&
        ((cfg->carrier_hz / cfg->fs_hz) > 0.05f)) {
        return DEMOD_ERR_UNSUPPORTED;
    }
    if ((cfg->mod_tone_hz < 0.0f) || (cfg->mod_tone_hz >= 0.5f * cfg->fs_hz)) {
        return DEMOD_ERR_BAD_CONFIG;
    }
    if ((cfg->method == DEMOD_FM_PLL) && (cfg->pll_bw_hz <= 0.0f)) {
        return DEMOD_ERR_BAD_CONFIG;
    }
    if ((cfg->baseband_lpf_enable != 0U) &&
        ((cfg->baseband_lpf_hz <= 0.0f) ||
         (cfg->baseband_lpf_hz >= 0.5f * cfg->fs_hz))) {
        return DEMOD_ERR_BAD_CONFIG;
    }

    prep = cfg->preprocess;
    prep.fs_hz = cfg->fs_hz;
    return Demod_ValidatePreprocess(&prep);
}

/* 全波整流并低通，按指定增益输出 AM 包络。 */
static void Demod_EnvelopeRectGain(const float *in,
                                   float *env,
                                   uint32_t len,
                                   float fs,
                                   float fc,
                                   float gain_correction,
                                   uint8_t remove_dc);

/* 使用 FFT-Hilbert 构造复数解析信号。 */
Demod_Status Demod_AnalyticSignal(const float *in,
                                  float *analytic_iq,
                                  uint32_t n,
                                  const Demod_PreprocessConfig *cfg)
{
    /* FFT Hilbert 路径：实信号 -> CFFT -> 正频翻倍/负频清零 -> ICFFT。
     * work 即 analytic_iq，长度必须至少 2*n float，且不能与输入重叠。 */
    const arm_cfft_instance_f32 *S = Demod_CfftSel(n);
    Demod_PreprocessConfig local_cfg;
    float mean = 0.0f;
    float max_abs = 0.0f;
    IIR_Cascade_t bpf;
    uint8_t use_bpf = 0U;

    if (in == NULL || analytic_iq == NULL) return DEMOD_ERR_NULL;
    if (n < 2U) return DEMOD_ERR_LENGTH;
    if (S == NULL) return DEMOD_ERR_FFT_SIZE;
    if (Demod_RangesOverlap(in, n * (uint32_t)sizeof(float),
                            analytic_iq, 2U * n * (uint32_t)sizeof(float)) != 0U) {
        return DEMOD_ERR_BUFFER_ALIAS;
    }

    if (cfg == NULL) {
        Demod_PreprocessDefault(&local_cfg);
        cfg = &local_cfg;
    }

    {
        Demod_Status st = Demod_ValidatePreprocess(cfg);
        if (st != DEMOD_OK) return st;
    }

    for (uint32_t i = 0U; i < n; i++) {
        if (Demod_IsFinite(in[i]) == 0U) return DEMOD_ERR_NO_VALID_DATA;
    }

    if (cfg->remove_dc != 0U) {
        for (uint32_t i = 0U; i < n; i++) {
            mean += in[i];
        }
        mean /= (float)n;
    }

    if ((cfg->bandpass_enable != 0U) &&
        (cfg->fs_hz > 0.0f) &&
        (cfg->carrier_max_hz > cfg->carrier_min_hz) &&
        (cfg->carrier_min_hz > 0.0f)) {
        float center = 0.5f * (cfg->carrier_min_hz + cfg->carrier_max_hz);
        float bw = cfg->carrier_max_hz - cfg->carrier_min_hz;
        float q = (bw > 0.0f) ? (center / bw) : 0.707f;
        if (q < 0.2f) q = 0.2f;
        IIR_CascadeInit(&bpf, IIR_BPF, cfg->fs_hz, center, q, 2U);
        use_bpf = 1U;
    }

    for (uint32_t i = 0U; i < n; i++) {
        float x = in[i] - mean;
        if (use_bpf != 0U) {
            x = IIR_CascadeProcess(&bpf, x);
        }
        analytic_iq[2U * i] = x;
        analytic_iq[2U * i + 1U] = 0.0f;
        if (fabsf(x) > max_abs) {
            max_abs = fabsf(x);
        }
    }

    if ((cfg->normalize != 0U) && (max_abs > DEMOD_EPS)) {
        float inv = 1.0f / max_abs;
        for (uint32_t i = 0U; i < n; i++) {
            analytic_iq[2U * i] *= inv;
        }
    }

    arm_cfft_f32(S, analytic_iq, 0, 1);

    uint32_t half = n / 2U;
    for (uint32_t k = 1U; k < half; k++) {
        analytic_iq[2U * k] *= 2.0f;
        analytic_iq[2U * k + 1U] *= 2.0f;
    }
    for (uint32_t k = half + 1U; k < n; k++) {
        analytic_iq[2U * k] = 0.0f;
        analytic_iq[2U * k + 1U] = 0.0f;
    }

    arm_cfft_f32(S, analytic_iq, 1, 1);
    return DEMOD_OK;
}

/* 对实信号块执行去直流、归一化和可选带通。 */
static Demod_Status Demod_PreprocessReal(const float *in,
                                         float *out,
                                         uint32_t n,
                                         const Demod_PreprocessConfig *cfg)
{
    /* 无状态实信号预处理，供 RECT 等非 Hilbert 路径复用。
     * Context 版本会保留滤波器状态；本函数每次调用都会重新开始滤波器状态。 */
    Demod_PreprocessConfig local_cfg;
    float mean = 0.0f;
    float max_abs = 0.0f;
    IIR_Cascade_t bpf;
    uint8_t use_bpf = 0U;
    Demod_Status st;

    if (in == NULL || out == NULL) return DEMOD_ERR_NULL;
    if (n == 0U) return DEMOD_ERR_LENGTH;

    if (cfg == NULL) {
        Demod_PreprocessDefault(&local_cfg);
        cfg = &local_cfg;
    }

    st = Demod_ValidatePreprocess(cfg);
    if (st != DEMOD_OK) return st;

    for (uint32_t i = 0U; i < n; i++) {
        if (Demod_IsFinite(in[i]) == 0U) return DEMOD_ERR_NO_VALID_DATA;
    }

    if (cfg->remove_dc != 0U) {
        for (uint32_t i = 0U; i < n; i++) {
            mean += in[i];
        }
        mean /= (float)n;
    }

    if (cfg->bandpass_enable != 0U) {
        float center = 0.5f * (cfg->carrier_min_hz + cfg->carrier_max_hz);
        float bw = cfg->carrier_max_hz - cfg->carrier_min_hz;
        float q = center / bw;
        if (q < 0.2f) q = 0.2f;
        IIR_CascadeInit(&bpf, IIR_BPF, cfg->fs_hz, center, q, 2U);
        use_bpf = 1U;
    }

    for (uint32_t i = 0U; i < n; i++) {
        float x = in[i] - mean;
        if (use_bpf != 0U) {
            x = IIR_CascadeProcess(&bpf, x);
        }
        out[i] = x;
        if (fabsf(x) > max_abs) max_abs = fabsf(x);
    }

    if ((cfg->normalize != 0U) && (max_abs > DEMOD_EPS)) {
        float inv = 1.0f / max_abs;
        for (uint32_t i = 0U; i < n; i++) {
            out[i] *= inv;
        }
    }

    return DEMOD_OK;
}

/* 更新一个样本的流式去直流与归一化状态。 */
static float Demod_PreprocessUpdate(const Demod_PreprocessConfig *cfg,
                                    IIR_Cascade_t *bpf,
                                    uint8_t use_bpf,
                                    float *dc_est,
                                    float *norm_est,
                                    float sample)
{
    /* Context/PLL/Costas 用的单点预处理。
     * 与帧处理不同，这里保留 DC、BPF 和归一化状态，保证连续块之间不重启。 */
    float x = sample;

    if (Demod_IsFinite(sample) == 0U) return Demod_Nan();

    if ((cfg != NULL) && (cfg->remove_dc != 0U) && (dc_est != NULL)) {
        *dc_est += 0.001f * (x - *dc_est);
        x -= *dc_est;
    }

    if ((use_bpf != 0U) && (bpf != NULL)) {
        x = IIR_CascadeProcess(bpf, x);
    }

    if ((cfg != NULL) && (cfg->normalize != 0U) && (norm_est != NULL)) {
        float ax = fabsf(x);
        *norm_est *= 0.999f;
        if (ax > *norm_est) {
            *norm_est = ax;
        }
        if (*norm_est > DEMOD_EPS) {
            x /= *norm_est;
        }
    }

    return x;
}

/* 按预处理频带初始化流式带通级联。 */
static uint8_t Demod_InitPreprocessBandpass(const Demod_PreprocessConfig *cfg,
                                            float fs,
                                            IIR_Cascade_t *bpf)
{
    float center;
    float bw;
    float q;

    if ((cfg == NULL) || (bpf == NULL) || (cfg->bandpass_enable == 0U)) {
        return 0U;
    }

    center = 0.5f * (cfg->carrier_min_hz + cfg->carrier_max_hz);
    bw = cfg->carrier_max_hz - cfg->carrier_min_hz;
    q = center / bw;
    if (q < 0.2f) q = 0.2f;

    IIR_CascadeInit(bpf, IIR_BPF, fs, center, q, 2U);
    return 1U;
}

/* 分析 AM 包络的载波幅值、调制度及质量。 */
static Demod_Status Demod_AnalyzeEnvelope(const float *env,
                                          uint32_t n,
                                          const Demod_AMConfig *cfg,
                                          Demod_AMResult *result)
{
    /* AM 参数分析只处理已有包络/基带，不负责解调。
     * result==NULL 的波形恢复路径不会调用这里，避免 DSB-SC 等零均值波形
     * 因无法计算传统调幅度而被误判为解调失败。 */
    uint32_t begin;
    uint32_t end;
    float lo;
    float hi;
    float carrier;
    float depth;
    float residual = 0.0f;
    float valid_ratio = 0.0f;
    Demod_AMConfig local_cfg;
    Demod_Status st;

    if (env == NULL || result == NULL) return DEMOD_ERR_NULL;
    Demod_ClearAMResult(result, DEMOD_OK);
    if (n < 2U) return DEMOD_ERR_LENGTH;

    if (cfg == NULL) {
        Demod_AMConfigDefault(&local_cfg, 0.0f);
        local_cfg.fs_hz = 1.0f;
        cfg = &local_cfg;
    }

    Demod_Window(n, cfg->edge_discard_samples, &begin, &end);
    if (begin >= end) return DEMOD_ERR_LENGTH;
    valid_ratio = (float)Demod_CountFinite(env, begin, end) / (float)(end - begin);

    if ((cfg->estimator == DEMOD_EST_SINE_FIT) &&
        (cfg->tone_hz > 0.0f) &&
        (cfg->fs_hz > 0.0f) &&
        (Demod_SineFit(env, begin, end, cfg->fs_hz, cfg->tone_hz,
                       &carrier, &depth, &residual) == DEMOD_OK)) {
        lo = carrier - depth;
        hi = carrier + depth;
    } else if (cfg->estimator == DEMOD_EST_PEAK_RAW) {
        lo = FLT_MAX;
        hi = -FLT_MAX;
        for (uint32_t i = begin; i < end; i++) {
            if (Demod_IsFinite(env[i]) == 0U) continue;
            if (env[i] < lo) lo = env[i];
            if (env[i] > hi) hi = env[i];
        }
        if (lo == FLT_MAX || hi == -FLT_MAX) return DEMOD_ERR_NO_VALID_DATA;
        carrier = 0.5f * (hi + lo);
    } else if (cfg->estimator == DEMOD_EST_PERCENTILE) {
        st = Demod_PercentileExact(env, begin, end, 0.01f, &lo);
        if (st != DEMOD_OK) return st;
        st = Demod_PercentileExact(env, begin, end, 0.99f, &hi);
        if (st != DEMOD_OK) return st;
        carrier = 0.5f * (hi + lo);
    } else {
        result->status = DEMOD_ERR_UNSUPPORTED;
        return DEMOD_ERR_UNSUPPORTED;
    }

    depth = ((hi + lo) > DEMOD_EPS) ? ((hi - lo) / (hi + lo)) : 0.0f;

    result->carrier_amp = carrier;
    result->modulation_depth = depth;
    result->env_low = lo;
    result->env_high = hi;
    result->valid_begin = begin;
    result->valid_end = end;
    result->quality.value = depth;
    result->quality.residual_rms = residual;
    result->quality.valid_ratio = valid_ratio;
    result->quality.confidence = (carrier > DEMOD_EPS) ? valid_ratio : 0.0f;
    result->quality.noise_rms = (cfg->estimator == DEMOD_EST_SINE_FIT) ? residual : 0.0f;
    result->quality.snr_est = ((cfg->estimator == DEMOD_EST_SINE_FIT) &&
                               (residual > DEMOD_EPS)) ?
                              (fabsf(depth * carrier) / residual) : 0.0f;
    result->status = (carrier > DEMOD_EPS) ? DEMOD_OK : DEMOD_ERR_LOW_SIGNAL;
    return result->status;
}

/* 仅估计有效样本的中心值。 */
static Demod_Status Demod_EstimateCenterOnly(const float *x,
                                             uint32_t n,
                                             uint32_t edge_discard,
                                             float *center)
{
    /* 仅为了输出模式做轻量中心估计，不计算调幅度/质量指标。
     * 这样 result==NULL 时仍然保持“只恢复波形”的语义。 */
    uint32_t begin;
    uint32_t end;
    Demod_Status st;

    if (x == NULL || center == NULL) return DEMOD_ERR_NULL;
    if (n < 2U) return DEMOD_ERR_LENGTH;

    Demod_Window(n, edge_discard, &begin, &end);
    st = Demod_PercentileExact(x, begin, end, 0.50f, center);
    if (st != DEMOD_OK) {
        *center = Demod_Mean(x, begin, end);
    }
    return DEMOD_OK;
}

/* 按配置将 AM 输出转换为包络或消息波形。 */
static Demod_Status Demod_ApplyAMOutputMode(float *out,
                                            uint32_t n,
                                            const Demod_AMConfig *cfg,
                                            float carrier)
{
    /* 统一 AM 输出语义。
     * 相干输出默认可能带符号；若用户要求 ABS，这里统一转成非负包络。 */
    if (out == NULL || cfg == NULL) return DEMOD_ERR_NULL;

    if (cfg->output_mode == DEMOD_AM_OUT_ENVELOPE_ABS) {
        if ((cfg->method == DEMOD_AM_COHERENT_PLL) ||
            (cfg->method == DEMOD_AM_COHERENT_COSTAS)) {
            for (uint32_t i = 0U; i < n; i++) {
                out[i] = fabsf(out[i]);
            }
        }
    } else if ((cfg->output_mode == DEMOD_AM_OUT_MESSAGE_ZERO_MEAN) ||
               (cfg->output_mode == DEMOD_AM_OUT_NORMALIZED)) {
        const float scale = ((cfg->output_mode == DEMOD_AM_OUT_NORMALIZED) &&
                             (fabsf(carrier) > DEMOD_EPS)) ?
                            (1.0f / carrier) : 1.0f;
        for (uint32_t i = 0U; i < n; i++) {
            out[i] = (out[i] - carrier) * scale;
        }
    }

    return DEMOD_OK;
}

/* 使用无状态方法恢复一块 AM 波形。 */
Demod_Status Demod_AM_Waveform(const float *in,
                               float *baseband_out,
                               uint32_t n,
                               float *work,
                               const Demod_AMConfig *cfg,
                               Demod_AMResult *result)
{
    /* 无状态 AM 波形恢复：只支持不需要跨块状态的 RECT/Hilbert。
     * 相干 PLL/Costas 需要 Demod_AMContext，否则每帧重新捕获会不稳定。 */
    Demod_AMConfig local_cfg;
    Demod_PreprocessConfig prep;
    Demod_Status st;

    if (in == NULL || baseband_out == NULL) return DEMOD_ERR_NULL;
    if (n == 0U) return DEMOD_ERR_LENGTH;

    if (cfg == NULL) {
        Demod_AMConfigDefault(&local_cfg, 1.0f);
        cfg = &local_cfg;
    }

    st = Demod_ValidateAMConfig(cfg);
    if (st != DEMOD_OK) {
        Demod_ClearAMResult(result, st);
        return st;
    }

    if ((cfg->method == DEMOD_AM_COHERENT_PLL) ||
        (cfg->method == DEMOD_AM_COHERENT_COSTAS)) {
        Demod_ClearAMResult(result, DEMOD_ERR_UNSUPPORTED);
        return DEMOD_ERR_UNSUPPORTED;
    }

    prep = cfg->preprocess;
    prep.fs_hz = cfg->fs_hz;

    if (cfg->method == DEMOD_AM_RECT) {
        const float *rect_in = in;
        uint8_t rect_remove_dc = prep.remove_dc;
        if ((prep.bandpass_enable != 0U) || (prep.normalize != 0U) || (prep.remove_dc == 0U)) {
            if (work == NULL) return DEMOD_ERR_NULL;
            st = Demod_PreprocessReal(in, work, n, &prep);
            if (st != DEMOD_OK) return st;
            rect_in = work;
            rect_remove_dc = 0U;
        }
        Demod_EnvelopeRectGain(rect_in, baseband_out, n, cfg->fs_hz,
                               cfg->rect_lpf_hz, cfg->rect_gain_correction,
                               rect_remove_dc);
        st = DEMOD_OK;
    } else if (cfg->method == DEMOD_AM_HILBERT_MAG) {
        if (work == NULL) return DEMOD_ERR_NULL;
        st = Demod_AnalyticSignal(in, work, n, &prep);
        if (st != DEMOD_OK) return st;
        for (uint32_t i = 0U; i < n; i++) {
            float re = work[2U * i];
            float im = work[2U * i + 1U];
            baseband_out[i] = sqrtf(re * re + im * im);
        }
    } else {
        Demod_ClearAMResult(result, DEMOD_ERR_UNSUPPORTED);
        return DEMOD_ERR_UNSUPPORTED;
    }

    if (result != NULL) {
        st = Demod_AnalyzeEnvelope(baseband_out, n, cfg, result);
        if (st != DEMOD_OK) return st;
        return Demod_ApplyAMOutputMode(baseband_out, n, cfg, result->carrier_amp);
    }

    if ((cfg->output_mode == DEMOD_AM_OUT_MESSAGE_ZERO_MEAN) ||
        (cfg->output_mode == DEMOD_AM_OUT_NORMALIZED)) {
        float carrier = 0.0f;
        st = Demod_EstimateCenterOnly(baseband_out, n, cfg->edge_discard_samples, &carrier);
        if (st != DEMOD_OK) return st;
        return Demod_ApplyAMOutputMode(baseband_out, n, cfg, carrier);
    }

    return DEMOD_OK;
}

/* 分析已有 AM 包络或基带的单音参数。 */
Demod_Status Demod_AM_AnalyzeTone(const float *baseband,
                                  uint32_t n,
                                  const Demod_AMConfig *cfg,
                                  Demod_AMResult *result)
{
    return Demod_AnalyzeEnvelope(baseband, n, cfg, result);
}

/* 全波整流并低通，按指定增益输出 AM 包络。 */
static void Demod_EnvelopeRectGain(const float *in,
                                   float *env,
                                   uint32_t len,
                                   float fs,
                                   float fc,
                                   float gain_correction,
                                   uint8_t remove_dc)
{
    /* 全波整流 + IIR 低通包络。
     * gain_correction 用于补偿实际低通、采样率和模拟前端带来的幅度误差。 */
    float mean = 0.0f;
    IIR_Cascade_t lp;
    const float gain = DEMOD_PI * 0.5f * gain_correction;

    if (in == NULL || env == NULL || len == 0U) return;
    if (fs <= 0.0f) fs = 1.0f;
    if (fc <= 0.0f) fc = 0.02f * fs;

    if (remove_dc != 0U) {
        for (uint32_t i = 0U; i < len; i++) {
            mean += in[i];
        }
        mean /= (float)len;
    }

    IIR_CascadeInit(&lp, IIR_LPF, fs, fc, 0.707f, 2U);
    for (uint32_t i = 0U; i < len; i++) {
        env[i] = gain * IIR_CascadeProcess(&lp, fabsf(in[i] - mean));
    }
}

/* 兼容旧接口：全波整流并低通得到 AM 包络。 */
void Demod_EnvelopeRect(const float *in, float *env, uint32_t len, float fs, float fc)
{
    Demod_EnvelopeRectGain(in, env, len, fs, fc, 1.0f, 1U);
}

/* 兼容旧接口：使用 Hilbert 解析信号得到 AM 包络。 */
int Demod_EnvelopeHilbert(const float *in, float *env, float *work, uint32_t n)
{
    Demod_Status st = Demod_AM_Waveform(in, env, n, work, NULL, NULL);
    return (st == DEMOD_OK) ? 0 : -1;
}

/* 兼容旧接口：由包络估计 AM 调制度。 */
float Demod_AM_Depth(const float *env, uint32_t len, float *carrier)
{
    Demod_AMResult result;
    Demod_AMConfig cfg;

    Demod_AMConfigDefault(&cfg, 0.0f);
    cfg.edge_discard_samples = 0U;
    cfg.estimator = DEMOD_EST_PERCENTILE;

    if (Demod_AnalyzeEnvelope(env, len, &cfg, &result) != DEMOD_OK) {
        if (carrier != NULL) *carrier = 0.0f;
        return 0.0f;
    }
    if (carrier != NULL) *carrier = result.carrier_amp;
    return result.modulation_depth;
}

/* 由解析信号按所选方法计算瞬时频率。 */
static Demod_Status Demod_FM_DetectFromAnalytic(float *analytic,
                                                float *freq_hz,
                                                uint32_t n,
                                                const Demod_FMConfig *cfg,
                                                Demod_FMResult *result)
{
    /* 从解析信号 I/Q 得到瞬时频率。
     * 低幅度点写 NaN；后续分析会跳过它们。PHASE_DIFF 遇到无效点后会
     * 重新建立 prev_phase，避免跨空洞相减造成假尖峰。 */
    float scale;
    float gate2;
    uint32_t valid = 0U;
    float prev_phase = 0.0f;
    uint8_t have_prev_valid = 0U;

    if (analytic == NULL || freq_hz == NULL || cfg == NULL) return DEMOD_ERR_NULL;
    if (cfg->fs_hz <= 0.0f) return DEMOD_ERR_BAD_FS;
    if (n < 2U) return DEMOD_ERR_LENGTH;
    if (cfg->method == DEMOD_FM_PLL) return DEMOD_ERR_UNSUPPORTED;

    scale = cfg->fs_hz / DEMOD_2PI;
    gate2 = cfg->preprocess.amplitude_gate * cfg->preprocess.amplitude_gate;

    freq_hz[0] = 0.0f;
    for (uint32_t i = 1U; i < n; i++) {
        float i0 = analytic[2U * (i - 1U)];
        float q0 = analytic[2U * (i - 1U) + 1U];
        float i1 = analytic[2U * i];
        float q1 = analytic[2U * i + 1U];
        float mag0 = i0 * i0 + q0 * q0;
        float mag1 = i1 * i1 + q1 * q1;
        float f;

        if ((Demod_IsFinite(mag0) == 0U) || (Demod_IsFinite(mag1) == 0U) ||
            (mag0 <= DEMOD_EPS) || (mag1 <= DEMOD_EPS) ||
            ((gate2 > 0.0f) && ((mag0 < gate2) || (mag1 < gate2)))) {
            f = Demod_Nan();
            have_prev_valid = 0U;
        } else if (cfg->method == DEMOD_FM_QUAD_DERIV) {
            float di = i1 - i0;
            float dq = q1 - q0;
            float denom = mag1;
            float dphi = (denom > DEMOD_EPS) ? ((i1 * dq - q1 * di) / denom) : 0.0f;
            f = dphi * scale;
        } else if (cfg->method == DEMOD_FM_PHASE_DIFF) {
            float ph = atan2f(q1, i1);
            if (have_prev_valid == 0U) {
                prev_phase = ph;
                have_prev_valid = 1U;
                f = Demod_Nan();
            } else {
                float dphi = ph - prev_phase;
                if (dphi > DEMOD_PI) {
                    dphi -= DEMOD_2PI;
                } else if (dphi <= -DEMOD_PI) {
                    dphi += DEMOD_2PI;
                }
                f = dphi * scale;
                prev_phase = ph;
            }
        } else if (cfg->method == DEMOD_FM_CONJ_PRODUCT) {
            float re = i1 * i0 + q1 * q0;
            float im = q1 * i0 - i1 * q0;
            float dphi = atan2f(im, re);
            f = dphi * scale;
        } else {
            return DEMOD_ERR_UNSUPPORTED;
        }
        if (Demod_IsFinite(f) != 0U) valid++;
        freq_hz[i] = f;
    }
    freq_hz[0] = (n > 1U) ? freq_hz[1] : 0.0f;

    if (result != NULL) {
        result->quality.valid_ratio = (n > 1U) ? ((float)valid / (float)(n - 1U)) : 0.0f;
    }
    return (valid > 0U) ? DEMOD_OK : DEMOD_ERR_NO_VALID_DATA;
}

/* 使用无状态 Hilbert 方法恢复 FM 瞬时频率。 */
Demod_Status Demod_FM_InstFreq(const float *in,
                               float *freq_hz,
                               float *analytic_work,
                               uint32_t n,
                               const Demod_FMConfig *cfg,
                               Demod_FMResult *result)
{
    /* 底层 FM 波形恢复：只产生 inst_freq_hz[]。
     * 不在这里估频偏，避免底层和高层重复统计。 */
    Demod_FMConfig local_cfg;
    Demod_Status st;

    if (in == NULL || freq_hz == NULL || analytic_work == NULL) return DEMOD_ERR_NULL;
    if (cfg == NULL) {
        Demod_FMConfigDefault(&local_cfg, 1.0f);
        cfg = &local_cfg;
    }
    if (result != NULL) Demod_ClearFMResult(result, DEMOD_OK);

    st = Demod_ValidateFMConfig(cfg);
    if (st != DEMOD_OK) {
        Demod_ClearFMResult(result, st);
        return st;
    }
    if (cfg->method == DEMOD_FM_PLL) {
        Demod_ClearFMResult(result, DEMOD_ERR_UNSUPPORTED);
        return DEMOD_ERR_UNSUPPORTED;
    }

    {
        Demod_PreprocessConfig prep = cfg->preprocess;
        prep.fs_hz = cfg->fs_hz;
        st = Demod_AnalyticSignal(in, analytic_work, n, &prep);
    }
    if (st != DEMOD_OK) return st;

    st = Demod_FM_DetectFromAnalytic(analytic_work, freq_hz, n, cfg, result);
    if (result != NULL) {
        result->status = st;
    }
    return st;
}

/* 分析瞬时频率的中心频率、频偏与质量。 */
Demod_Status Demod_FM_AnalyzeFreq(const float *freq_hz,
                                  uint32_t n,
                                  const Demod_FMConfig *cfg,
                                  Demod_FMResult *result)
{
    /* 对已有瞬时频率数组做统计。
     * NaN 表示无效样本，会被均值、百分位、RMS 和正弦拟合全部跳过。 */
    Demod_FMConfig local_cfg;
    uint32_t begin;
    uint32_t end;
    uint32_t finite_count;
    float center;
    float flo;
    float fhi;
    float amp = 0.0f;
    float residual = 0.0f;
    float input_valid_ratio = 0.0f;
    Demod_Status st;

    if (freq_hz == NULL || result == NULL) return DEMOD_ERR_NULL;
    Demod_ClearFMResult(result, DEMOD_OK);
    if (n < 2U) return DEMOD_ERR_LENGTH;

    if (cfg == NULL) {
        Demod_FMConfigDefault(&local_cfg, 1.0f);
        cfg = &local_cfg;
    }
    st = Demod_ValidateFMConfig(cfg);
    if (st != DEMOD_OK) {
        result->status = st;
        return st;
    }

    Demod_Window(n, cfg->edge_discard_samples, &begin, &end);
    if (begin >= end) return DEMOD_ERR_LENGTH;
    finite_count = Demod_CountFinite(freq_hz, begin, end);
    if (finite_count == 0U) {
        result->status = DEMOD_ERR_NO_VALID_DATA;
        return DEMOD_ERR_NO_VALID_DATA;
    }
    input_valid_ratio = (float)finite_count / (float)(end - begin);

    if (cfg->fc_mode == DEMOD_FC_KNOWN) {
        center = cfg->carrier_hz;
    } else if (cfg->fc_mode == DEMOD_FC_MEAN) {
        center = Demod_Mean(freq_hz, begin, end);
    } else {
        st = Demod_PercentileExact(freq_hz, begin, end, 0.50f, &center);
        if (st != DEMOD_OK) {
            result->status = st;
            return st;
        }
    }

    if ((cfg->estimator == DEMOD_EST_SINE_FIT) &&
        (cfg->mod_tone_hz > 0.0f) &&
        (cfg->fs_hz > 0.0f) &&
        (Demod_SineFit(freq_hz, begin, end, cfg->fs_hz, cfg->mod_tone_hz,
                       &center, &amp, &residual) == DEMOD_OK)) {
        flo = center - amp;
        fhi = center + amp;
    } else if (cfg->estimator == DEMOD_EST_PEAK_RAW) {
        flo = FLT_MAX;
        fhi = -FLT_MAX;
        for (uint32_t i = begin; i < end; i++) {
            if (Demod_IsFinite(freq_hz[i]) == 0U) continue;
            if (freq_hz[i] < flo) flo = freq_hz[i];
            if (freq_hz[i] > fhi) fhi = freq_hz[i];
        }
        amp = 0.5f * (fhi - flo);
    } else if (cfg->estimator == DEMOD_EST_RMS) {
        amp = 1.41421356237f * Demod_RmsAround(freq_hz, begin, end, center);
        flo = center - amp;
        fhi = center + amp;
    } else if (cfg->estimator == DEMOD_EST_PERCENTILE) {
        st = Demod_PercentileExact(freq_hz, begin, end, 0.01f, &flo);
        if (st != DEMOD_OK) {
            result->status = st;
            return st;
        }
        st = Demod_PercentileExact(freq_hz, begin, end, 0.99f, &fhi);
        if (st != DEMOD_OK) {
            result->status = st;
            return st;
        }
        amp = 0.5f * (fhi - flo);
    } else {
        result->status = DEMOD_ERR_UNSUPPORTED;
        return DEMOD_ERR_UNSUPPORTED;
    }

    result->center_hz = center;
    result->deviation_peak_hz = amp;
    result->deviation_rms_hz = Demod_RmsAround(freq_hz, begin, end, center);
    result->freq_low_hz = flo;
    result->freq_high_hz = fhi;
    result->valid_begin = begin;
    result->valid_end = end;
    result->quality.value = amp;
    result->quality.residual_rms = residual;
    result->quality.valid_ratio = input_valid_ratio;
    result->quality.noise_rms = (cfg->estimator == DEMOD_EST_SINE_FIT) ? residual : 0.0f;
    result->quality.snr_est = ((cfg->estimator == DEMOD_EST_SINE_FIT) &&
                               (residual > DEMOD_EPS)) ?
                              (amp / residual) : 0.0f;
    result->quality.confidence = result->quality.valid_ratio;
    result->status = DEMOD_OK;
    return DEMOD_OK;
}

/* 无状态完成 FM 瞬时频率、频偏和参数分析。 */
Demod_Status Demod_FM_Process(const float *in,
                              float *freq_hz,
                              float *deviation_hz,
                              float *work,
                              uint32_t n,
                              const Demod_FMConfig *cfg,
                              Demod_FMResult *result)
{
    /* 无状态一站式 FM：
     * 1) 先算瞬时频率；
     * 2) 从原始有效频率估 center；
     * 3) 生成 deviation_hz，并按需低通；
     * 4) 若调用者要 result，再用低通后的频偏波形估频偏。 */
    Demod_FMConfig local_cfg;
    Demod_FMResult local_result;
    float *freq_buf;
    Demod_Status st;
    uint8_t want_result = (result != NULL) ? 1U : 0U;

    if (in == NULL || work == NULL) return DEMOD_ERR_NULL;
    if ((freq_hz == NULL) && (deviation_hz == NULL)) return DEMOD_ERR_NULL;
    if (n > DEMOD_MAX_POINTS) return DEMOD_ERR_FFT_SIZE;
    if (((freq_hz != NULL) &&
         (Demod_RangesOverlap(freq_hz, n * (uint32_t)sizeof(float), work,
                              2U * n * (uint32_t)sizeof(float)) != 0U)) ||
        ((deviation_hz != NULL) &&
         (Demod_RangesOverlap(deviation_hz, n * (uint32_t)sizeof(float), work,
                              2U * n * (uint32_t)sizeof(float)) != 0U))) {
        return DEMOD_ERR_BUFFER_ALIAS;
    }

    if (cfg == NULL) {
        Demod_FMConfigDefault(&local_cfg, 1.0f);
        cfg = &local_cfg;
    }
    st = Demod_ValidateFMConfig(cfg);
    if (st != DEMOD_OK) {
        Demod_ClearFMResult(result, st);
        return st;
    }

    freq_buf = (freq_hz != NULL) ? freq_hz : deviation_hz;
    st = Demod_FM_InstFreq(in, freq_buf, work, n, cfg, want_result ? result : NULL);
    if (st != DEMOD_OK) return st;

    if (want_result != 0U) {
        st = Demod_FM_AnalyzeFreq(freq_buf, n, cfg, result);
        if (st != DEMOD_OK) return st;
    } else if (deviation_hz != NULL) {
        Demod_ClearFMResult(&local_result, DEMOD_OK);
        if (cfg->fc_mode == DEMOD_FC_KNOWN) {
            local_result.center_hz = cfg->carrier_hz;
        } else if (cfg->fc_mode == DEMOD_FC_MEAN) {
            uint32_t begin, end;
            Demod_Window(n, cfg->edge_discard_samples, &begin, &end);
            local_result.center_hz = Demod_Mean(freq_buf, begin, end);
        } else {
            uint32_t begin, end;
            Demod_Window(n, cfg->edge_discard_samples, &begin, &end);
            st = Demod_PercentileExact(freq_buf, begin, end, 0.50f, &local_result.center_hz);
            if (st != DEMOD_OK) return st;
        }
        result = &local_result;
    } else {
        return DEMOD_OK;
    }

    if (deviation_hz != NULL) {
        Demod_FMResult dev_result;
        Demod_FMConfig dev_cfg = *cfg;
        const float raw_center = result->center_hz;
        IIR_Cascade_t lpf;
        uint8_t use_lpf = 0U;
        float last_valid = 0.0f;
        if (cfg->baseband_lpf_enable != 0U) {
            IIR_CascadeInit(&lpf, IIR_LPF, cfg->fs_hz, cfg->baseband_lpf_hz, 0.707f, 2U);
            use_lpf = 1U;
        }
        for (uint32_t i = 0U; i < n; i++) {
            float d = freq_buf[i] - result->center_hz;
            if (Demod_IsFinite(d) == 0U) {
                d = last_valid;
            } else {
                last_valid = d;
            }
            deviation_hz[i] = (use_lpf != 0U) ? IIR_CascadeProcess(&lpf, d) : d;
        }

        dev_cfg.fc_mode = DEMOD_FC_KNOWN;
        dev_cfg.carrier_hz = 0.0f;
        if (want_result != 0U) {
            st = Demod_FM_AnalyzeFreq(deviation_hz, n, &dev_cfg, &dev_result);
        } else {
            st = DEMOD_OK;
        }
        if ((want_result != 0U) && (st == DEMOD_OK)) {
            const float raw_valid_ratio = result->quality.valid_ratio;
            result->center_hz = raw_center;
            result->deviation_peak_hz = dev_result.deviation_peak_hz;
            result->deviation_rms_hz = dev_result.deviation_rms_hz;
            result->freq_low_hz = raw_center + dev_result.freq_low_hz;
            result->freq_high_hz = raw_center + dev_result.freq_high_hz;
            result->quality = dev_result.quality;
            result->quality.valid_ratio = raw_valid_ratio;
            result->status = DEMOD_OK;
        }
        if ((want_result != 0U) && (st != DEMOD_OK)) return st;
    }
    return DEMOD_OK;
}

/* 一站式恢复 FM 频偏波形。 */
Demod_Status Demod_FM_Waveform(const float *in,
                               float *deviation_hz_out,
                               uint32_t n,
                               float *work,
                               const Demod_FMConfig *cfg,
                               Demod_FMResult *result)
{
    return Demod_FM_Process(in, NULL, deviation_hz_out, work, n, cfg, result);
}

/* 兼容旧接口：由输入信号计算瞬时频率。 */
int Demod_InstFreq(const float *in, float *freq_out, float *work, uint32_t n, float fs)
{
    Demod_FMConfig cfg;
    Demod_Status st;

    Demod_FMConfigDefault(&cfg, fs);
    cfg.edge_discard_samples = 0U;
    cfg.fc_mode = DEMOD_FC_MEAN;
    st = Demod_FM_InstFreq(in, freq_out, work, n, &cfg, NULL);
    return (st == DEMOD_OK) ? 0 : -1;
}

/* 兼容旧接口：估计 FM 峰值频偏和中心频率。 */
float Demod_FM_Deviation(const float *in, float *work, uint32_t n, float fs,
                         float *fc_center)
{
    Demod_FMConfig cfg;
    Demod_FMResult result;
    Demod_Status st;

    if (work == NULL) {
        if (fc_center != NULL) *fc_center = 0.0f;
        return 0.0f;
    }

    Demod_FMConfigDefault(&cfg, fs);
    cfg.edge_discard_samples = 16U;
    cfg.estimator = DEMOD_EST_PERCENTILE;
    cfg.fc_mode = DEMOD_FC_MEDIAN;

    if (n > DEMOD_MAX_POINTS) {
        if (fc_center != NULL) *fc_center = 0.0f;
        return 0.0f;
    }

    st = Demod_FM_InstFreq(in, s_freq_work, work, n, &cfg, &result);
    if (st == DEMOD_OK) {
        st = Demod_FM_AnalyzeFreq(s_freq_work, n, &cfg, &result);
    }
    if (st != DEMOD_OK) {
        if (fc_center != NULL) *fc_center = 0.0f;
        return 0.0f;
    }
    if (fc_center != NULL) *fc_center = result.center_hz;
    return result.deviation_peak_hz;
}

/*
 * 警告：Demod 在 PLL/Costas 未锁定或幅度低于门限时，输出数组的对应位置
 * 会被写入 NaN。NaN 参与任何算术/累加/FFT/Goertzel 操作都会直接产生 NaN，
 * 导致后续参数估计（中心频率、频偏、调幅度）全部失效。
 * 在对 Demod 输出做统计分析或频谱分析之前，必须先用 Demod_ReplaceNaN()
 * 把 NaN 替换为 0 或上一个有效值。
 */
/* 将数组中的非有限值替换为指定值或前一有效值。 */
void Demod_ReplaceNaN(float *data, uint32_t len, float fill)
{
    float last_valid;
    if (data == NULL) return;
    last_valid = (Demod_IsFinite(fill) != 0U) ? fill : 0.0f;
    for (uint32_t i = 0U; i < len; i++) {
        if (Demod_IsFinite(data[i]) != 0U) {
            last_valid = data[i];
        } else {
            data[i] = last_valid;
        }
    }
}

/* 使用显式环路参数初始化载波锁相环。 */
void PLL_InitManual(PLL_t *pll, float fs, float f0,
                float alpha, float beta, float zeta,
                float pd_lpf_hz)
{
    float loop_bw;

    if (pll == NULL || (Demod_IsFinite(fs) == 0U) || fs <= 0.0f ||
        (Demod_IsFinite(f0) == 0U) || (Demod_IsFinite(alpha) == 0U) ||
        (Demod_IsFinite(beta) == 0U) || (Demod_IsFinite(zeta) == 0U) ||
        (Demod_IsFinite(pd_lpf_hz) == 0U)) return;
    if (zeta < 0.5f) zeta = 0.5f;
    if (zeta > 2.0f) zeta = 2.0f;
    if (alpha <= 0.0f) alpha = 0.001f;
    if (beta  <= 0.0f) beta  = 1e-6f;

    /* 反算等效 loop_bw，仅用于 LoopLimits 频率限制窗 */
    loop_bw = sqrtf(beta) * fs / DEMOD_2PI;
    loop_bw = Demod_SafeLoopHz(fs, loop_bw);

    /* pd_lpf_hz <= 0 时自动计算，否则用调用者给定的值 */
    if (pd_lpf_hz <= 0.0f) {
        pd_lpf_hz = Demod_SelectPdLpfHz(fs, loop_bw);
    }

    pll->fs = fs;
    pll->phase = 0.0f;
    Demod_LoopLimits(fs, f0, loop_bw,
                     &pll->center_freq, &pll->min_freq, &pll->max_freq);
    pll->freq = pll->center_freq;
    pll->alpha = alpha;
    pll->beta  = beta;
    pll->pd_raw = 0.0f;
    pll->pd_low = 0.0f;
    pll->phase_error = 0.0f;
    pll->amplitude = 0.0f;
    pll->amplitude_gate = DEMOD_SIGNAL_GATE;
    pll->lock_metric = 1.0f;
    pll->lock_threshold = DEMOD_LOCK_ERR_LIMIT;
    pll->freq_delta = 0.0f;
    pll->amplitude_alpha = Demod_TimeAlpha(fs, DEMOD_LEVEL_TC_SEC);
    pll->error_alpha = Demod_TimeAlpha(fs, DEMOD_ERROR_TC_SEC);
    pll->lock_count = 0U;
    pll->lock_target_count = Demod_TimeCount(fs, DEMOD_LOCK_TIME_SEC);
    pll->locked = 0U;

    IIR_CascadeInit(&pll->pd_lpf, IIR_LPF, fs, pd_lpf_hz, 0.707f, 2U);
}

/* 按环路速度参数初始化载波锁相环。 */
void PLL_Init(PLL_t *pll, float fs, float f0, float loop_bw)
{
    if (pll == NULL) return;
    if ((Demod_IsFinite(fs) == 0U) || fs <= 0.0f ||
        (Demod_IsFinite(f0) == 0U) || (Demod_IsFinite(loop_bw) == 0U)) {
        memset(pll, 0, sizeof(*pll));
        return;
    }
    float wn = DEMOD_2PI * Demod_SafeLoopHz(fs, loop_bw) / fs;

    PLL_InitManual(pll, fs, f0,
               2.0f * 0.707f * wn,   /* alpha = 2ζωn */
               wn * wn,               /* beta  = ωn²   */
               0.707f,                /* zeta  = 0.707 */
               0.0f);                 /* pd_lpf_hz = 自动 */
}

/* 保留配置并复位锁相环运行状态。 */
void PLL_Reset(PLL_t *pll, float f0)
{
    float loop_bw;

    if (pll == NULL || (Demod_IsFinite(pll->fs) == 0U) || pll->fs <= 0.0f ||
        (Demod_IsFinite(f0) == 0U)) return;
    loop_bw = (pll->beta > 0.0f) ? (sqrtf(pll->beta) * pll->fs / DEMOD_2PI)
                                 : DEMOD_LOOP_MIN_HZ;
    pll->phase = 0.0f;
    Demod_LoopLimits(pll->fs, f0, loop_bw,
                     &pll->center_freq, &pll->min_freq, &pll->max_freq);
    pll->freq = pll->center_freq;
    pll->pd_raw = 0.0f;
    pll->pd_low = 0.0f;
    pll->phase_error = 0.0f;
    pll->amplitude = 0.0f;
    pll->lock_metric = 1.0f;
    pll->freq_delta = 0.0f;
    pll->lock_count = 0U;
    pll->locked = 0U;
    IIR_CascadeReset(&pll->pd_lpf);
}

/* 输入一个样本并更新载波锁相环。 */
float PLL_Update(PLL_t *pll, float sample)
{
    float sin_nco;
    float loop_err = 0.0f;
    float freq_stable_limit;
    uint8_t signal_ok;
    uint8_t lock_ok;

    if (pll == NULL || (Demod_IsFinite(pll->fs) == 0U) || pll->fs <= 0.0f) {
        return 0.0f;
    }
    if ((Demod_IsFinite(sample) == 0U) || (Demod_IsFinite(pll->phase) == 0U) ||
        (Demod_IsFinite(pll->freq) == 0U)) {
        pll->lock_count = 0U;
        pll->locked = 0U;
        pll->freq_delta = 0.0f;
        return (Demod_IsFinite(pll->freq) != 0U) ?
               (pll->freq * pll->fs / DEMOD_2PI) : 0.0f;
    }

    sin_nco = sinf(pll->phase);
    pll->pd_raw = sample * (-sin_nco);
    pll->pd_low = IIR_CascadeProcess(&pll->pd_lpf, pll->pd_raw);

    pll->amplitude += pll->amplitude_alpha * (fabsf(sample) - pll->amplitude);
    signal_ok = (pll->amplitude > pll->amplitude_gate) ? 1U : 0U;

    if (signal_ok != 0U) {
        loop_err = pll->pd_low / (pll->amplitude + DEMOD_EPS);
        loop_err = Demod_ClampF(loop_err, -DEMOD_NORM_ERR_LIMIT, DEMOD_NORM_ERR_LIMIT);
        pll->freq_delta = pll->beta * loop_err;
        pll->freq = Demod_ClampF(pll->freq + pll->freq_delta,
                                 pll->min_freq,
                                 pll->max_freq);
    } else {
        pll->freq_delta = 0.0f;
    }

    pll->phase += pll->freq + ((signal_ok != 0U) ? (pll->alpha * loop_err) : 0.0f);
    pll->phase = Demod_WrapPi(pll->phase);

    pll->phase_error = loop_err;
    pll->lock_metric += pll->error_alpha * (fabsf(loop_err) - pll->lock_metric);

    freq_stable_limit = Demod_MaxF(0.25f * pll->beta, 1.0e-8f);
    lock_ok = ((signal_ok != 0U) &&
               (pll->lock_metric < pll->lock_threshold) &&
               (fabsf(pll->freq_delta) < freq_stable_limit) &&
               (pll->freq >= pll->min_freq) &&
               (pll->freq <= pll->max_freq)) ? 1U : 0U;
    Demod_UpdateLock(lock_ok, pll->lock_target_count, &pll->lock_count, &pll->locked);
    return pll->freq * pll->fs / DEMOD_2PI;
}

/* 获取当前 NCO 正弦和余弦值。 */
void PLL_GetSinCos(const PLL_t *pll, float *sin_out, float *cos_out)
{
    if (pll == NULL) {
        if (sin_out != NULL) *sin_out = 0.0f;
        if (cos_out != NULL) *cos_out = 1.0f;
        return;
    }
    if (sin_out != NULL) *sin_out = sinf(pll->phase);
    if (cos_out != NULL) *cos_out = cosf(pll->phase);
}

/* 获取当前锁相环相位。 */
float PLL_GetPhase(const PLL_t *pll)
{
    return (pll != NULL) ? pll->phase : 0.0f;
}

/* 获取当前归一化鉴相误差。 */
float PLL_GetPhaseError(const PLL_t *pll)
{
    return (pll != NULL) ? pll->phase_error : 0.0f;
}

/* 获取当前锁相环频率。 */
float PLL_GetFrequency(const PLL_t *pll)
{
    if (pll == NULL || pll->fs <= 0.0f) return 0.0f;
    return pll->freq * pll->fs / DEMOD_2PI;
}

/* 查询锁相环是否已锁定。 */
uint8_t PLL_IsLocked(const PLL_t *pll)
{
    return (pll != NULL) ? pll->locked : 0U;
}

/* 初始化 PLL 相干 AM 解调器。 */
int Demod_AMCoherent_Init(Demod_AMCoherent_t *ctx,
                          float fs,
                          float carrier_hz,
                          float pll_bw_hz,
                          float baseband_bw_hz)
{
    if (ctx == NULL || (Demod_IsFinite(fs) == 0U) || fs <= 0.0f ||
        (Demod_IsFinite(carrier_hz) == 0U) || carrier_hz <= 0.0f ||
        carrier_hz >= 0.5f * fs || (Demod_IsFinite(pll_bw_hz) == 0U) ||
        (Demod_IsFinite(baseband_bw_hz) == 0U) || baseband_bw_hz <= 0.0f ||
        baseband_bw_hz >= 0.5f * fs) return -1;
    PLL_Init(&ctx->carrier_pll, fs, carrier_hz, pll_bw_hz);
    IIR_CascadeInit(&ctx->baseband_lpf, IIR_LPF, fs, baseband_bw_hz, 0.707f, 2U);
    ctx->dc_est = 0.0f;
    ctx->gain = 1.0f;
    return 0;
}

/* 输入一个样本并输出 PLL 相干 AM 基带。 */
float Demod_AMCoherent_Update(Demod_AMCoherent_t *ctx, float sample)
{
    /* 普通含载波 AM 的相干解调。
     * 对 DSB-SC/严重过调，普通 PLL 可能被 180 度相位翻转拉动，应优先用 Costas。 */
    float s;
    float c;
    float x;
    float mixed;

    if (ctx == NULL) return 0.0f;
    ctx->dc_est += 0.001f * (sample - ctx->dc_est);
    x = sample - ctx->dc_est;
    (void)PLL_Update(&ctx->carrier_pll, x);
    PLL_GetSinCos(&ctx->carrier_pll, &s, &c);
    mixed = 2.0f * x * c;
    return ctx->gain * IIR_CascadeProcess(&ctx->baseband_lpf, mixed);
}

/* 初始化 Costas 环 AM 解调器。 */
int Demod_Costas_Init(Demod_Costas_t *ctx,
                      float fs,
                      float carrier_hz,
                      float loop_bw_hz,
                      float baseband_bw_hz)
{
    const float zeta = 0.707f;
    float safe_loop_bw;
    float wn;

    if (ctx == NULL || (Demod_IsFinite(fs) == 0U) || fs <= 0.0f ||
        (Demod_IsFinite(carrier_hz) == 0U) || carrier_hz <= 0.0f ||
        carrier_hz >= 0.5f * fs || (Demod_IsFinite(loop_bw_hz) == 0U) ||
        (Demod_IsFinite(baseband_bw_hz) == 0U) || baseband_bw_hz <= 0.0f ||
        baseband_bw_hz >= 0.5f * fs) return -1;
    safe_loop_bw = Demod_SafeLoopHz(fs, loop_bw_hz);
    ctx->fs = fs;
    ctx->phase = 0.0f;
    Demod_LoopLimits(fs, carrier_hz, safe_loop_bw,
                     &ctx->center_freq, &ctx->min_freq, &ctx->max_freq);
    ctx->freq = ctx->center_freq;
    wn = DEMOD_2PI * safe_loop_bw / fs;
    ctx->alpha = 2.0f * zeta * wn;
    ctx->beta = wn * wn;
    ctx->dc_est = 0.0f;
    ctx->gain = 1.0f;
    ctx->phase_error = 0.0f;
    ctx->amplitude = 0.0f;
    ctx->power = 0.0f;
    ctx->power_gate = DEMOD_POWER_GATE;
    ctx->lock_metric = 1.0f;
    ctx->lock_threshold = DEMOD_LOCK_ERR_LIMIT;
    ctx->freq_delta = 0.0f;
    ctx->amplitude_alpha = Demod_TimeAlpha(fs, DEMOD_LEVEL_TC_SEC);
    ctx->error_alpha = Demod_TimeAlpha(fs, DEMOD_ERROR_TC_SEC);
    ctx->lock_count = 0U;
    ctx->lock_target_count = Demod_TimeCount(fs, DEMOD_LOCK_TIME_SEC);
    ctx->locked = 0U;
    IIR_CascadeInit(&ctx->i_lpf, IIR_LPF, fs, baseband_bw_hz, 0.707f, 2U);
    IIR_CascadeInit(&ctx->q_lpf, IIR_LPF, fs, baseband_bw_hz, 0.707f, 2U);
    return 0;
}

/* 处理一个 Costas 环样本并更新锁定状态。 */
static float Demod_CostasTrack(Demod_Costas_t *ctx, float x)
{
    float c;
    float s;
    float i_base;
    float q_base;
    float err = 0.0f;
    float freq_stable_limit;
    uint8_t signal_ok;
    uint8_t lock_ok;

    if (ctx == NULL || ctx->fs <= 0.0f) return 0.0f;
    if (Demod_IsFinite(x) == 0U) {
        ctx->lock_count = 0U;
        ctx->locked = 0U;
        ctx->freq_delta = 0.0f;
        return 0.0f;
    }

    c = cosf(ctx->phase);
    s = sinf(ctx->phase);

    i_base = IIR_CascadeProcess(&ctx->i_lpf, 2.0f * x * c);
    q_base = IIR_CascadeProcess(&ctx->q_lpf, -2.0f * x * s);
    ctx->power = i_base * i_base + q_base * q_base;
    signal_ok = (ctx->power > ctx->power_gate) ? 1U : 0U;

    if (signal_ok != 0U) {
        err = (2.0f * i_base * q_base) / (ctx->power + DEMOD_EPS);
        err = Demod_ClampF(err, -DEMOD_NORM_ERR_LIMIT, DEMOD_NORM_ERR_LIMIT);
        ctx->freq_delta = ctx->beta * err;
        ctx->freq = Demod_ClampF(ctx->freq + ctx->freq_delta,
                                 ctx->min_freq,
                                 ctx->max_freq);
    } else {
        ctx->freq_delta = 0.0f;
    }

    ctx->phase += ctx->freq + ((signal_ok != 0U) ? (ctx->alpha * err) : 0.0f);
    ctx->phase = Demod_WrapPi(ctx->phase);

    ctx->amplitude += ctx->amplitude_alpha * (sqrtf(ctx->power) - ctx->amplitude);
    ctx->phase_error = err;
    ctx->lock_metric += ctx->error_alpha * (fabsf(err) - ctx->lock_metric);

    freq_stable_limit = Demod_MaxF(0.25f * ctx->beta, 1.0e-8f);
    lock_ok = ((signal_ok != 0U) &&
               (ctx->amplitude > DEMOD_SIGNAL_GATE) &&
               (ctx->lock_metric < ctx->lock_threshold) &&
               (fabsf(ctx->freq_delta) < freq_stable_limit) &&
               (ctx->freq >= ctx->min_freq) &&
               (ctx->freq <= ctx->max_freq)) ? 1U : 0U;
    Demod_UpdateLock(lock_ok, ctx->lock_target_count, &ctx->lock_count, &ctx->locked);
    return ctx->gain * i_base;
}

/* 输入一个样本并输出 Costas 环基带。 */
float Demod_Costas_Update(Demod_Costas_t *ctx, float sample)
{
    /* Costas 环适合弱载波/抑制载波 AM，但存在整体正负号二义性。
     * 对需要绝对极性的场景，必须由外部参考或协议层决定是否翻转。 */
    float x;

    if (ctx == NULL || ctx->fs <= 0.0f) return 0.0f;
    ctx->dc_est += 0.001f * (sample - ctx->dc_est);
    x = sample - ctx->dc_est;
    return Demod_CostasTrack(ctx, x);
}

/* 对一个 AM 样本执行有状态预处理。 */
static float Demod_AMPreprocessSample(Demod_AMContext *ctx, float sample)
{
    return Demod_PreprocessUpdate(&ctx->cfg.preprocess,
                                  &ctx->preprocess_bpf,
                                  ctx->use_preprocess_bpf,
                                  &ctx->dc_est,
                                  &ctx->norm_est,
                                  sample);
}

/* 完成 AM 块的分析、输出模式转换和状态判定。 */
static Demod_Status Demod_AMFinishBlock(Demod_AMContext *ctx,
                                        float *out,
                                        uint32_t n,
                                        Demod_AMResult *result)
{
    Demod_Status st;

    if (ctx->cfg.output_mode == DEMOD_AM_OUT_ENVELOPE_ABS) {
        st = Demod_ApplyAMOutputMode(out, n, &ctx->cfg, 0.0f);
        if (st != DEMOD_OK) return st;
    }

    if (result != NULL) {
        st = Demod_AnalyzeEnvelope(out, n, &ctx->cfg, result);
        if (st != DEMOD_OK) return st;
        return Demod_ApplyAMOutputMode(out, n, &ctx->cfg, result->carrier_amp);
    }

    if ((ctx->cfg.output_mode == DEMOD_AM_OUT_MESSAGE_ZERO_MEAN) ||
        (ctx->cfg.output_mode == DEMOD_AM_OUT_NORMALIZED)) {
        float carrier = 0.0f;
        st = Demod_EstimateCenterOnly(out, n, ctx->cfg.edge_discard_samples, &carrier);
        if (st != DEMOD_OK) return st;
        return Demod_ApplyAMOutputMode(out, n, &ctx->cfg, carrier);
    }

    return Demod_ApplyAMOutputMode(out, n, &ctx->cfg, 0.0f);
}

/* 以有状态整流低通方式处理一块 AM 样本。 */
static void Demod_AMProcessRect(Demod_AMContext *ctx,
                                const float *in,
                                float *out,
                                uint32_t n)
{
    const float gain = DEMOD_PI * 0.5f * ctx->cfg.rect_gain_correction;

    for (uint32_t i = 0U; i < n; i++) {
        float x = Demod_AMPreprocessSample(ctx, in[i]);
        out[i] = (Demod_IsFinite(x) != 0U) ?
                 (gain * IIR_CascadeProcess(&ctx->rect_lpf, fabsf(x))) : Demod_Nan();
    }
}

/* 以 PLL 相干方式处理一块 AM 样本。 */
static uint8_t Demod_AMProcessPll(Demod_AMContext *ctx,
                                  const float *in,
                                  float *out,
                                  uint32_t n)
{
    uint8_t locked = 0U;

    for (uint32_t i = 0U; i < n; i++) {
        float s;
        float c;
        float x = Demod_AMPreprocessSample(ctx, in[i]);
        float y;

        (void)PLL_Update(&ctx->state.pll.carrier_pll, x);
        if (Demod_IsFinite(x) == 0U) {
            locked = 0U;
            out[i] = Demod_Nan();
            continue;
        }
        PLL_GetSinCos(&ctx->state.pll.carrier_pll, &s, &c);
        y = ctx->state.pll.gain *
            IIR_CascadeProcess(&ctx->state.pll.baseband_lpf, 2.0f * x * c);

        locked = PLL_IsLocked(&ctx->state.pll.carrier_pll);
        out[i] = (locked != 0U) ? y : Demod_Nan();
    }
    return locked;
}

/* 以 Costas 环方式处理一块 AM 样本。 */
static uint8_t Demod_AMProcessCostas(Demod_AMContext *ctx,
                                     const float *in,
                                     float *out,
                                     uint32_t n)
{
    uint8_t locked = 0U;

    for (uint32_t i = 0U; i < n; i++) {
        float x = Demod_AMPreprocessSample(ctx, in[i]);
        float y = Demod_CostasTrack(&ctx->state.costas, x);

        locked = ctx->state.costas.locked;
        out[i] = (locked != 0U) ? y : Demod_Nan();
    }
    return locked;
}

/* 初始化有状态 AM 解调上下文。 */
Demod_Status Demod_AM_Init(Demod_AMContext *ctx, const Demod_AMConfig *cfg)
{
    /* AM 有状态初始化。
     * RECT 保留低通/BPF/DC 状态；PLL/Costas 保留载波恢复状态；
     * Hilbert 仍然是整帧 FFT 算法，不会因为放进 Context 就变成真正流式 Hilbert。 */
    Demod_AMConfig local_cfg;
    Demod_Status st;

    if (ctx == NULL) return DEMOD_ERR_NULL;
    if (cfg == NULL) {
        Demod_AMConfigDefault(&local_cfg, 1.0f);
        cfg = &local_cfg;
    }

    st = Demod_ValidateAMConfig(cfg);
    if (st != DEMOD_OK) return st;

    memset(ctx, 0, sizeof(*ctx));
    ctx->cfg = *cfg;
    ctx->cfg.preprocess.fs_hz = ctx->cfg.fs_hz;
    ctx->use_preprocess_bpf =
        Demod_InitPreprocessBandpass(&ctx->cfg.preprocess,
                                     ctx->cfg.fs_hz,
                                     &ctx->preprocess_bpf);

    if (ctx->cfg.method == DEMOD_AM_COHERENT_PLL) {
        if (Demod_AMCoherent_Init(&ctx->state.pll, ctx->cfg.fs_hz,
                                  ctx->cfg.carrier_hz, ctx->cfg.pll_bw_hz,
                                  ctx->cfg.baseband_lpf_hz) != 0) {
            return DEMOD_ERR_BAD_CONFIG;
        }
    } else if (ctx->cfg.method == DEMOD_AM_COHERENT_COSTAS) {
        if (Demod_Costas_Init(&ctx->state.costas, ctx->cfg.fs_hz,
                              ctx->cfg.carrier_hz, ctx->cfg.pll_bw_hz,
                              ctx->cfg.baseband_lpf_hz) != 0) {
            return DEMOD_ERR_BAD_CONFIG;
        }
    } else if (ctx->cfg.method == DEMOD_AM_RECT) {
        IIR_CascadeInit(&ctx->rect_lpf, IIR_LPF, ctx->cfg.fs_hz,
                        ctx->cfg.rect_lpf_hz, 0.707f, 2U);
    } else if ((ctx->cfg.method != DEMOD_AM_RECT) &&
               (ctx->cfg.method != DEMOD_AM_HILBERT_MAG)) {
        return DEMOD_ERR_UNSUPPORTED;
    }

    ctx->ready = 1U;
    return DEMOD_OK;
}

/* 使用上下文处理一个 AM 数据块。 */
Demod_Status Demod_AM_ProcessBlock(Demod_AMContext *ctx,
                                   const float *in,
                                   float *out,
                                   uint32_t n,
                                   float *work,
                                   Demod_AMResult *result)
{
    /* AM 有状态块处理。
     * result==NULL 时只恢复波形；只有调用者要 result 时才做调幅度/质量分析。 */
    uint8_t locked = 0U;

    if (ctx == NULL || in == NULL || out == NULL) return DEMOD_ERR_NULL;
    if (ctx->ready == 0U) return DEMOD_ERR_BAD_CONFIG;
    if (n == 0U) return DEMOD_ERR_LENGTH;

    if (ctx->cfg.method == DEMOD_AM_HILBERT_MAG) {
        return Demod_AM_Waveform(in, out, n, work, &ctx->cfg, result);
    }

    if (ctx->cfg.method == DEMOD_AM_RECT) {
        Demod_AMProcessRect(ctx, in, out, n);
        return Demod_AMFinishBlock(ctx, out, n, result);
    }

    if (ctx->cfg.method == DEMOD_AM_COHERENT_PLL) {
        locked = Demod_AMProcessPll(ctx, in, out, n);
    } else if (ctx->cfg.method == DEMOD_AM_COHERENT_COSTAS) {
        locked = Demod_AMProcessCostas(ctx, in, out, n);
    } else {
        Demod_ClearAMResult(result, DEMOD_ERR_UNSUPPORTED);
        return DEMOD_ERR_UNSUPPORTED;
    }

    if (locked == 0U) {
        Demod_ClearAMResult(result, DEMOD_ERR_UNLOCKED);
        return DEMOD_ERR_UNLOCKED;
    }

    return Demod_AMFinishBlock(ctx, out, n, result);
}

/* 初始化有状态 FM 解调上下文。 */
Demod_Status Demod_FM_Init(Demod_FMContext *ctx, const Demod_FMConfig *cfg)
{
    /* FM 有状态初始化。
     * PLL 路径保存环路状态；非 PLL 路径主要保存基带低通状态。FFT Hilbert
     * 仍是逐帧处理，若要完全连续波形需后续实现 overlap-save 或 FIR Hilbert。 */
    Demod_FMConfig local_cfg;
    Demod_Status st;

    if (ctx == NULL) return DEMOD_ERR_NULL;
    if (cfg == NULL) {
        Demod_FMConfigDefault(&local_cfg, 1.0f);
        cfg = &local_cfg;
    }

    st = Demod_ValidateFMConfig(cfg);
    if (st != DEMOD_OK) return st;

    memset(ctx, 0, sizeof(*ctx));
    ctx->cfg = *cfg;
    ctx->cfg.preprocess.fs_hz = ctx->cfg.fs_hz;
    ctx->use_preprocess_bpf =
        Demod_InitPreprocessBandpass(&ctx->cfg.preprocess,
                                     ctx->cfg.fs_hz,
                                     &ctx->preprocess_bpf);

    if (ctx->cfg.method == DEMOD_FM_PLL) {
        if (ctx->cfg.carrier_hz <= 0.0f) return DEMOD_ERR_BAD_CONFIG;
        PLL_Init(&ctx->pll, ctx->cfg.fs_hz, ctx->cfg.carrier_hz,
                 ctx->cfg.pll_bw_hz);
    }
    if (ctx->cfg.baseband_lpf_enable != 0U) {
        IIR_CascadeInit(&ctx->baseband_lpf, IIR_LPF, ctx->cfg.fs_hz,
                        ctx->cfg.baseband_lpf_hz, 0.707f, 2U);
    }

    ctx->ready = 1U;
    return DEMOD_OK;
}

/* 对一个 FM 样本执行有状态预处理。 */
static float Demod_FMPreprocessSample(Demod_FMContext *ctx, float sample)
{
    return Demod_PreprocessUpdate(&ctx->cfg.preprocess,
                                  &ctx->preprocess_bpf,
                                  ctx->use_preprocess_bpf,
                                  &ctx->dc_est,
                                  &ctx->norm_est,
                                  sample);
}

/* 按配置从有效瞬时频率选择中心频率。 */
static Demod_Status Demod_FMSelectCenter(const float *freq_hz,
                                         uint32_t n,
                                         const Demod_FMConfig *cfg,
                                         Demod_FMResult *result)
{
    uint32_t begin;
    uint32_t end;

    if ((freq_hz == NULL) || (cfg == NULL) || (result == NULL)) {
        return DEMOD_ERR_NULL;
    }

    if (cfg->fc_mode == DEMOD_FC_KNOWN) {
        result->center_hz = cfg->carrier_hz;
        return DEMOD_OK;
    }

    Demod_Window(n, cfg->edge_discard_samples, &begin, &end);
    if (cfg->fc_mode == DEMOD_FC_MEAN) {
        result->center_hz = Demod_Mean(freq_hz, begin, end);
        return DEMOD_OK;
    }

    return Demod_PercentileExact(freq_hz, begin, end, 0.50f, &result->center_hz);
}

/* 根据频偏波形更新 FM 统计与质量指标。 */
static Demod_Status Demod_FMUpdateDeviationStats(const float *deviation_hz,
                                                 uint32_t n,
                                                 const Demod_FMConfig *cfg,
                                                 float raw_center,
                                                 Demod_FMResult *result)
{
    Demod_FMResult dev_result;
    Demod_FMConfig dev_cfg;
    Demod_Status st;

    if ((deviation_hz == NULL) || (cfg == NULL) || (result == NULL)) {
        return DEMOD_ERR_NULL;
    }

    dev_cfg = *cfg;
    dev_cfg.fc_mode = DEMOD_FC_KNOWN;
    dev_cfg.carrier_hz = 0.0f;
    st = Demod_FM_AnalyzeFreq(deviation_hz, n, &dev_cfg, &dev_result);
    if (st != DEMOD_OK) return st;

    const float raw_valid_ratio = result->quality.valid_ratio;
    result->center_hz = raw_center;
    result->deviation_peak_hz = dev_result.deviation_peak_hz;
    result->deviation_rms_hz = dev_result.deviation_rms_hz;
    result->freq_low_hz = raw_center + dev_result.freq_low_hz;
    result->freq_high_hz = raw_center + dev_result.freq_high_hz;
    result->quality = dev_result.quality;
    result->quality.valid_ratio = raw_valid_ratio;
    result->status = DEMOD_OK;
    return DEMOD_OK;
}

/* 以中心频率构造并可选低通频偏波形。 */
static void Demod_FMBuildDeviation(Demod_FMContext *ctx,
                                   const float *freq_hz,
                                   float center_hz,
                                   float *deviation_hz,
                                   uint32_t n,
                                   uint8_t keep_nan)
{
    float last_valid = 0.0f;

    for (uint32_t i = 0U; i < n; i++) {
        float d = freq_hz[i] - center_hz;

        if (Demod_IsFinite(d) == 0U) {
            if (keep_nan != 0U) {
                deviation_hz[i] = Demod_Nan();
                continue;
            }
            d = last_valid;
        } else {
            last_valid = d;
        }

        if (ctx->cfg.baseband_lpf_enable != 0U) {
            d = IIR_CascadeProcess(&ctx->baseband_lpf, d);
        }
        deviation_hz[i] = d;
    }
}

/* 使用非 PLL 方法处理一个 FM 数据块。 */
static Demod_Status Demod_FMProcessNonPll(Demod_FMContext *ctx,
                                          const float *in,
                                          float *freq_hz,
                                          float *deviation_hz,
                                          uint32_t n,
                                          float *work,
                                          Demod_FMResult *result)
{
    Demod_FMResult local_result;
    Demod_Status st;
    float *freq_buf = (freq_hz != NULL) ? freq_hz : s_freq_work;
    uint8_t want_result = (result != NULL) ? 1U : 0U;
    float raw_center;

    if (n > DEMOD_MAX_POINTS && freq_hz == NULL) return DEMOD_ERR_BUFFER_SIZE;
    if (work == NULL) return DEMOD_ERR_NULL;
    if (((freq_hz != NULL) &&
         (Demod_RangesOverlap(freq_hz, n * (uint32_t)sizeof(float), work,
                              2U * n * (uint32_t)sizeof(float)) != 0U)) ||
        ((deviation_hz != NULL) &&
         (Demod_RangesOverlap(deviation_hz, n * (uint32_t)sizeof(float), work,
                              2U * n * (uint32_t)sizeof(float)) != 0U))) {
        return DEMOD_ERR_BUFFER_ALIAS;
    }

    Demod_ClearFMResult(&local_result, DEMOD_OK);
    st = Demod_FM_InstFreq(in, freq_buf, work, n, &ctx->cfg,
                           want_result ? result : NULL);
    if (st != DEMOD_OK) return st;

    if (want_result != 0U) {
        st = Demod_FM_AnalyzeFreq(freq_buf, n, &ctx->cfg, result);
        if (st != DEMOD_OK) return st;
    } else if (deviation_hz != NULL) {
        st = Demod_FMSelectCenter(freq_buf, n, &ctx->cfg, &local_result);
        if (st != DEMOD_OK) return st;
        result = &local_result;
    } else {
        return DEMOD_OK;
    }

    raw_center = result->center_hz;
    if (deviation_hz != NULL) {
        Demod_FMBuildDeviation(ctx, freq_buf, raw_center, deviation_hz, n, 0U);
        if (want_result != 0U) {
            st = Demod_FMUpdateDeviationStats(deviation_hz, n, &ctx->cfg,
                                              raw_center, result);
            if (st != DEMOD_OK) return st;
        }
    }

    return DEMOD_OK;
}

/* 使用 PLL 方法处理一个 FM 数据块。 */
static Demod_Status Demod_FMProcessPll(Demod_FMContext *ctx,
                                       const float *in,
                                       float *freq_hz,
                                       float *deviation_hz,
                                       uint32_t n,
                                       Demod_FMResult *result)
{
    Demod_FMResult local_result;
    Demod_Status st;
    float *freq_buf = (freq_hz != NULL) ? freq_hz : deviation_hz;
    uint32_t locked_count = 0U;

    Demod_ClearFMResult(&local_result, DEMOD_OK);
    for (uint32_t i = 0U; i < n; i++) {
        float x = Demod_FMPreprocessSample(ctx, in[i]);
        float f = PLL_Update(&ctx->pll, x);

        if (PLL_IsLocked(&ctx->pll) != 0U) {
            freq_buf[i] = f;
            locked_count++;
        } else {
            freq_buf[i] = Demod_Nan();
        }
    }

    if (result != NULL) {
        Demod_ClearFMResult(result, DEMOD_OK);
        result->quality.valid_ratio = (float)locked_count / (float)n;
    } else {
        local_result.quality.valid_ratio = (float)locked_count / (float)n;
    }

    st = Demod_FM_AnalyzeFreq(freq_buf, n, &ctx->cfg, result ? result : &local_result);
    if (st != DEMOD_OK) return st;
    if (result == NULL) result = &local_result;
    result->quality.valid_ratio = (float)locked_count / (float)n;

    if (deviation_hz != NULL) {
        Demod_FMBuildDeviation(ctx, freq_buf, result->center_hz, deviation_hz, n, 1U);
        st = Demod_FMUpdateDeviationStats(deviation_hz, n, &ctx->cfg,
                                          result->center_hz, result);
        if (st != DEMOD_OK) return st;
    }

    if (PLL_IsLocked(&ctx->pll) == 0U) {
        result->status = DEMOD_ERR_UNLOCKED;
        return DEMOD_ERR_UNLOCKED;
    }

    return DEMOD_OK;
}

/* 使用上下文处理一个 FM 数据块。 */
Demod_Status Demod_FM_ProcessBlock(Demod_FMContext *ctx,
                                   const float *in,
                                   float *freq_hz,
                                   float *deviation_hz,
                                   uint32_t n,
                                   float *work,
                                   Demod_FMResult *result)
{
    /* FM 有状态块处理。
     * 非 PLL：瞬时频率仍由 FFT Hilbert 算出，但频偏基带低通使用 Context 状态。
     * PLL：未锁定的样本写 NaN，不参与中心频率和频偏估计。 */
    if (ctx == NULL || in == NULL) return DEMOD_ERR_NULL;
    if (ctx->ready == 0U) return DEMOD_ERR_BAD_CONFIG;
    if (n < 2U) return DEMOD_ERR_LENGTH;
    if ((freq_hz == NULL) && (deviation_hz == NULL)) return DEMOD_ERR_NULL;

    if (ctx->cfg.method != DEMOD_FM_PLL) {
        return Demod_FMProcessNonPll(ctx, in, freq_hz, deviation_hz, n, work, result);
    }

    return Demod_FMProcessPll(ctx, in, freq_hz, deviation_hz, n, result);
}

/* ==================================================================== */
/*  BPSK 相干解调（平方载波恢复 + I/Q 正交混频 + 二倍角相位投影）          */
/* ==================================================================== */

#define DEMOD_PSK_2PI   6.28318530717958647692f /* BPSK 处理使用的二倍圆周率。 */

/* 写入 BPSK 失败状态并返回该状态。 */
static Demod_Status Demod_PSKFail(Demod_PSKResult *result, Demod_Status status)
{
    if (result != NULL) {
        memset(result, 0, sizeof(*result));
        result->status = status;
    }
    return status;
}

/* 填充 BPSK 解调默认配置。 */
void Demod_PSKConfigDefault(Demod_PSKConfig *cfg, float fs_hz)
{
    if (cfg == NULL) return;
    cfg->fs_hz               = fs_hz;
    cfg->if_search_min_hz    = 360000.0f;
    cfg->if_search_max_hz    = 440000.0f;
    cfg->lpf_hz              = 50000.0f;
    cfg->edge_discard_samples = 512U;
}

/* 使用平方载波恢复和 I/Q 投影完成 BPSK 相干解调。 */
Demod_Status Demod_PSK_Demodulate(const float *in,
                                   float *baseband_out,
                                   float *work,
                                   uint32_t n,
                                   const Demod_PSKConfig *cfg,
                                   Demod_PSKResult *result)
{
    Demod_PSKConfig local_cfg;
    float *square_buf;          /* work[0..n-1]   平方数据 / Q 通道   */
    uint32_t i;
    float fs;
    float bin_hz;
    uint32_t lo_bin, hi_bin;
    float max_val;
    uint32_t peak_bin;
    float ym, y0, yp, denom, delta2;
    float doubled_if, if_hz;
    float mean_sq;
    IIR_Cascade_t lpf_i, lpf_q;
    uint32_t edge_begin, edge_end;
    float sum_cos2, sum_sin2;
    float axis_phase, cp, sp;
    float best_amp, best_base_freq;
    float candidates[3];

    /* ---- 参数校验 ---- */
    if (in == NULL || baseband_out == NULL || work == NULL) {
        return Demod_PSKFail(result, DEMOD_ERR_NULL);
    }
    if (n < 64U) return Demod_PSKFail(result, DEMOD_ERR_LENGTH);
    if (Demod_CfftSel(n) == NULL) return Demod_PSKFail(result, DEMOD_ERR_FFT_SIZE);
    if (Demod_RangesOverlap(in, n * (uint32_t)sizeof(float),
                            work, 2U * n * (uint32_t)sizeof(float)) != 0U) {
        return Demod_PSKFail(result, DEMOD_ERR_BUFFER_ALIAS);
    }
    if (cfg == NULL) {
        Demod_PSKConfigDefault(&local_cfg, 1.0f);
        cfg = &local_cfg;
    }
    if ((Demod_IsFinite(cfg->fs_hz) == 0U) || cfg->fs_hz <= 0.0f) {
        return Demod_PSKFail(result, DEMOD_ERR_BAD_FS);
    }
    if ((Demod_IsFinite(cfg->if_search_min_hz) == 0U) ||
        (Demod_IsFinite(cfg->if_search_max_hz) == 0U) ||
        (Demod_IsFinite(cfg->lpf_hz) == 0U) ||
        cfg->if_search_min_hz <= 0.0f ||
        cfg->if_search_max_hz <= cfg->if_search_min_hz ||
        cfg->if_search_max_hz >= 0.5f * cfg->fs_hz ||
        cfg->lpf_hz <= 0.0f || cfg->lpf_hz >= 0.5f * cfg->fs_hz) {
        return Demod_PSKFail(result, DEMOD_ERR_BAD_CONFIG);
    }

    fs = cfg->fs_hz;
    bin_hz = fs / (float)n;
    square_buf = work;                      /* work[0..n-1] 复用 */

    /* ---- 1. 拷贝到 square_buf，平方并去掉 DC ---- */
    mean_sq = 0.0f;
    for (i = 0U; i < n; i++) {
        float v = in[i];
        if (Demod_IsFinite(v) == 0U) {
            return Demod_PSKFail(result, DEMOD_ERR_NO_VALID_DATA);
        }
        mean_sq += v * v;
    }
    mean_sq /= (float)n;
    for (i = 0U; i < n; i++) {
        float v = in[i];
        square_buf[i] = v * v - mean_sq;
    }

    /* ---- 2. FFT，在 2×IF 预期范围内找峰 ---- */
    if (!FFT_StartN(square_buf, n)) {
        return Demod_PSKFail(result, DEMOD_ERR_FFT_SIZE);
    }

    lo_bin = (uint32_t)(cfg->if_search_min_hz / bin_hz);
    hi_bin = (uint32_t)(cfg->if_search_max_hz / bin_hz);
    if (lo_bin < 1U) lo_bin = 1U;
    if (hi_bin >= n/2U) hi_bin = n/2U - 1U;
    if (lo_bin >= hi_bin) return Demod_PSKFail(result, DEMOD_ERR_BAD_CONFIG);

    max_val = 0.0f;
    peak_bin = lo_bin;
    for (i = lo_bin; i <= hi_bin; i++) {
        if (FFT_output[i] > max_val) { max_val = FFT_output[i]; peak_bin = i; }
    }
    if ((Demod_IsFinite(max_val) == 0U) || max_val <= 1.0e-12f) {
        return Demod_PSKFail(result, DEMOD_ERR_NO_VALID_DATA);
    }

    /* 二次插值 */
    ym = (peak_bin > 0)     ? FFT_output[peak_bin - 1U] : 0.0f;
    y0 = FFT_output[peak_bin];
    yp = (peak_bin + 1U < n/2U) ? FFT_output[peak_bin + 1U] : 0.0f;
    denom = 2.0f * (ym - 2.0f * y0 + yp);
    delta2 = (fabsf(denom) > 1e-12f) ? ((ym - yp) / denom) : 0.0f;
    delta2 = Demod_ClampF(delta2, -0.5f, 0.5f);
    doubled_if = ((float)peak_bin + delta2) * bin_hz;
    if_hz = doubled_if * 0.5f;             /* 真载波 = 2×IF ÷ 2            */

    /* ---- 3. I/Q 正交混频 + 低通 ---- */
    IIR_CascadeInit(&lpf_i, IIR_LPF, fs, cfg->lpf_hz, 0.707f, 2U);
    IIR_CascadeInit(&lpf_q, IIR_LPF, fs, cfg->lpf_hz, 0.707f, 2U);
    for (i = 0U; i < n; i++) {
        float phase = DEMOD_PSK_2PI * if_hz * (float)i / fs;
        float c = cosf(phase);
        float s = -sinf(phase);
        float x = in[i];
        baseband_out[i] = IIR_CascadeProcess(&lpf_i, 2.0f * x * c);  /* I */
        work[n + i]     = IIR_CascadeProcess(&lpf_q, 2.0f * x * s);  /* Q */
    }

    /* ---- 4. 二倍角法估计星座轴，投影得到 NRZ ---- */
    Demod_Window(n, cfg->edge_discard_samples, &edge_begin, &edge_end);

    sum_cos2 = 0.0f;
    sum_sin2 = 0.0f;
    for (i = edge_begin; i < edge_end; i++) {
        float I = baseband_out[i];
        float Q = work[n + i];
        sum_cos2 += I * I - Q * Q;
        sum_sin2 += 2.0f * I * Q;
    }
    axis_phase = 0.5f * atan2f(sum_sin2, sum_cos2);
    cp = cosf(axis_phase);
    sp = sinf(axis_phase);

    for (i = 0U; i < n; i++) {
        float I = baseband_out[i];
        float Q = work[n + i];
        baseband_out[i] = I * cp + Q * sp;     /* 投影 = 干净 NRZ       */
    }

    /* ---- 5. Goertzel 搜 NRZ 基频 {3k,4k,5k} → Rc = 2×基频 ---- */
    candidates[0] = 3000.0f;
    candidates[1] = 4000.0f;
    candidates[2] = 5000.0f;
    best_amp       = 0.0f;
    best_base_freq = 0.0f;
    for (i = 0U; i < 3U; i++) {
        float amp = FFT_AmpAtFreq(&baseband_out[edge_begin],
                                   edge_end - edge_begin,
                                   fs, candidates[i]);
        if (amp > best_amp) {
            best_amp       = amp;
            best_base_freq = candidates[i];
        }
    }

    /* ---- 填充结果 ---- */
    if (result != NULL) {
        result->if_hz  = if_hz;
        result->rc_bps = (best_base_freq > 0.0f) ? (best_base_freq * 2.0f) : 0.0f;
        result->status = (best_base_freq > 0.0f) ? DEMOD_OK : DEMOD_ERR_NO_VALID_DATA;
    }

    return (best_base_freq > 0.0f) ? DEMOD_OK :
           Demod_PSKFail(result, DEMOD_ERR_NO_VALID_DATA);
}
