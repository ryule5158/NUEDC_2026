/**
 * @file    Demod.c
 * @brief   AM/FM demodulation and analysis implementation.
 */
#include "Demod.h"
#include "arm_const_structs.h"
#include <float.h>
#include <math.h>
#include <string.h>

#define DEMOD_PI          3.14159265358979323846f
#define DEMOD_2PI         6.28318530717958647692f
#define DEMOD_EPS         1.0e-12f
#define DEMOD_MAX_POINTS  512U

#define DEMOD_LOOP_MIN_HZ          1.0f
#define DEMOD_PD_LPF_RATIO         4.0f
#define DEMOD_CAPTURE_MIN_HZ       10.0f
#define DEMOD_CAPTURE_BW_RATIO     6.0f
#define DEMOD_CAPTURE_FC_RATIO     0.05f
#define DEMOD_LEVEL_TC_SEC         0.001f
#define DEMOD_ERROR_TC_SEC         0.0005f
#define DEMOD_LOCK_TIME_SEC        0.0002f
#define DEMOD_SIGNAL_GATE          1.0e-4f
#define DEMOD_POWER_GATE           (DEMOD_SIGNAL_GATE * DEMOD_SIGNAL_GATE)
#define DEMOD_LOCK_ERR_LIMIT       0.10f
#define DEMOD_NORM_ERR_LIMIT       1.0f
#define DEMOD_LOCK_COUNT_MAX       60000U

/* 兼容旧接口和单帧分析的静态工作区。
 * 注意：这两个缓冲区不是可重入设计；ISR/多任务并发调用时应改成由
 * Context 或调用者提供工作区。 */
static float s_order_work[DEMOD_MAX_POINTS];
static float s_freq_work[DEMOD_MAX_POINTS];

uint32_t Demod_AnalyticWorkFloats(uint32_t n)
{
    return 2U * n;
}

uint32_t Demod_FMWorkFloats(uint32_t n)
{
    return 2U * n;
}

static const arm_cfft_instance_f32 *Demod_CfftSel(uint32_t n)
{
    switch (n) {
    case 64:   return &arm_cfft_sR_f32_len64;
    case 128:  return &arm_cfft_sR_f32_len128;
    case 256:  return &arm_cfft_sR_f32_len256;
    case 512:  return &arm_cfft_sR_f32_len512;
    case 1024:
    case 2048:
    case 4096:
        return NULL;
    default:   return NULL;
    }
}

static float Demod_ClampF(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static float Demod_MaxF(float a, float b)
{
    return (a > b) ? a : b;
}

static float Demod_PositiveOr(float x, float fallback)
{
    return (x > 0.0f) ? x : fallback;
}

static float Demod_TimeAlpha(float fs, float tau_sec)
{
    float alpha;

    if ((fs <= 0.0f) || (tau_sec <= 0.0f)) {
        return 1.0f;
    }
    alpha = 1.0f - expf(-1.0f / (fs * tau_sec));
    return Demod_ClampF(alpha, 1.0e-6f, 1.0f);
}

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

static float Demod_SafeLoopHz(float fs, float loop_hz)
{
    float max_hz = (fs > 0.0f) ? (0.45f * fs) : DEMOD_LOOP_MIN_HZ;

    return Demod_ClampF(Demod_PositiveOr(loop_hz, DEMOD_LOOP_MIN_HZ),
                        DEMOD_LOOP_MIN_HZ,
                        max_hz);
}

static float Demod_SelectPdLpfHz(float fs, float loop_hz)
{
    float pd_hz = Demod_SafeLoopHz(fs, loop_hz) * DEMOD_PD_LPF_RATIO;

    if (fs > 0.0f) {
        pd_hz = Demod_ClampF(pd_hz, DEMOD_LOOP_MIN_HZ, 0.45f * fs);
    }
    return pd_hz;
}

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

static uint8_t Demod_IsFinite(float x)
{
    return ((x == x) && (x <= FLT_MAX) && (x >= -FLT_MAX)) ? 1U : 0U;
}

static float Demod_Nan(void)
{
    /* 用 NaN 标记“该样本无效”。后续统计函数会跳过 NaN，避免低幅度
     * 或未锁定阶段的错误频率污染中心频率、频偏和拟合结果。 */
    volatile float zero = 0.0f;
    return zero / zero;
}

static float Demod_WrapPi(float x)
{
    while (x > DEMOD_PI) {
        x -= DEMOD_2PI;
    }
    while (x <= -DEMOD_PI) {
        x += DEMOD_2PI;
    }
    return x;
}

static void Demod_ClearAMResult(Demod_AMResult *result, Demod_Status status)
{
    if (result == NULL) return;
    memset(result, 0, sizeof(*result));
    result->status = status;
}

static void Demod_ClearFMResult(Demod_FMResult *result, Demod_Status status)
{
    if (result == NULL) return;
    memset(result, 0, sizeof(*result));
    result->status = status;
}

static Demod_Status Demod_ValidatePreprocess(const Demod_PreprocessConfig *cfg)
{
    if (cfg == NULL) return DEMOD_OK;
    if (cfg->amplitude_gate < 0.0f) return DEMOD_ERR_BAD_CONFIG;
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

static void Demod_Window(uint32_t n, uint32_t discard, uint32_t *begin, uint32_t *end)
{
    /* FFT Hilbert 的帧首/帧尾容易受周期延拓影响。这里仅把边缘样本从
     * 参数分析中排除，不会从输出波形数组中删除这些样本。 */
    if (discard * 2U >= n) {
        discard = 0U;
    }
    *begin = discard;
    *end = n - discard;
}

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

static void Demod_SwapF(float *a, float *b)
{
    float t = *a;
    *a = *b;
    *b = t;
}

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

static Demod_Status Demod_ValidateAMConfig(const Demod_AMConfig *cfg)
{
    /* 配置检查不仅检查数值范围，也检查“方法/输出模式”是否物理可实现。
     * 例如 RECT/Hilbert 只能得到非负包络，不能恢复过调制的有符号包络。 */
    Demod_PreprocessConfig prep;

    if (cfg == NULL) return DEMOD_ERR_NULL;
    if (cfg->fs_hz <= 0.0f) return DEMOD_ERR_BAD_FS;
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

static Demod_Status Demod_ValidateFMConfig(const Demod_FMConfig *cfg)
{
    /* QUAD_DERIV 是小相位步进近似。若已知数字中频占采样率比例较高，
     * 该方法会系统性低估频偏，直接拒绝比给出漂亮但错误的结果更安全。 */
    Demod_PreprocessConfig prep;

    if (cfg == NULL) return DEMOD_ERR_NULL;
    if (cfg->fs_hz <= 0.0f) return DEMOD_ERR_BAD_FS;
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

static void Demod_EnvelopeRectGain(const float *in,
                                   float *env,
                                   uint32_t len,
                                   float fs,
                                   float fc,
                                   float gain_correction,
                                   uint8_t remove_dc);

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

    if (cfg == NULL) {
        Demod_PreprocessDefault(&local_cfg);
        cfg = &local_cfg;
    }

    {
        Demod_Status st = Demod_ValidatePreprocess(cfg);
        if (st != DEMOD_OK) return st;
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

    if (cfg->estimator == DEMOD_EST_SINE_FIT) {
        if ((cfg->tone_hz <= 0.0f) || (cfg->fs_hz <= 0.0f)) {
            result->status = DEMOD_ERR_BAD_CONFIG;
            return result->status;
        }
        st = Demod_SineFit(env, begin, end, cfg->fs_hz, cfg->tone_hz,
                           &carrier, &depth, &residual);
        if (st != DEMOD_OK) {
            result->status = st;
            return st;
        }
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
    } else if (cfg->estimator == DEMOD_EST_RMS) {
        carrier = Demod_Mean(env, begin, end);
        depth = 1.41421356237f * Demod_RmsAround(env, begin, end, carrier);
        lo = carrier - depth;
        hi = carrier + depth;
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

Demod_Status Demod_AM_AnalyzeTone(const float *baseband,
                                  uint32_t n,
                                  const Demod_AMConfig *cfg,
                                  Demod_AMResult *result)
{
    return Demod_AnalyzeEnvelope(baseband, n, cfg, result);
}

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

void Demod_EnvelopeRect(const float *in, float *env, uint32_t len, float fs, float fc)
{
    Demod_EnvelopeRectGain(in, env, len, fs, fc, 1.0f, 1U);
}

int Demod_EnvelopeHilbert(const float *in, float *env, float *work, uint32_t n)
{
    Demod_Status st = Demod_AM_Waveform(in, env, n, work, NULL, NULL);
    return (st == DEMOD_OK) ? 0 : -1;
}

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

        if ((gate2 > 0.0f) && ((mag0 < gate2) || (mag1 < gate2))) {
            f = Demod_Nan();
            have_prev_valid = 0U;
        } else if (cfg->method == DEMOD_FM_QUAD_DERIV) {
            float di = i1 - i0;
            float dq = q1 - q0;
            float denom = mag1;
            float dphi = (denom > DEMOD_EPS) ? ((i1 * dq - q1 * di) / denom) : 0.0f;
            f = dphi * scale;
            valid++;
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
                valid++;
            }
        } else if (cfg->method == DEMOD_FM_CONJ_PRODUCT) {
            float re = i1 * i0 + q1 * q0;
            float im = q1 * i0 - i1 * q0;
            float dphi = atan2f(im, re);
            f = dphi * scale;
            valid++;
        } else {
            return DEMOD_ERR_UNSUPPORTED;
        }
        freq_hz[i] = f;
    }
    freq_hz[0] = (n > 1U) ? freq_hz[1] : 0.0f;

    if (result != NULL) {
        result->quality.valid_ratio = (n > 1U) ? ((float)valid / (float)(n - 1U)) : 0.0f;
    }
    return (valid > 0U) ? DEMOD_OK : DEMOD_ERR_NO_VALID_DATA;
}

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
    float input_valid_ratio = (result != NULL) ? result->quality.valid_ratio : 0.0f;
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
    if (input_valid_ratio <= 0.0f) {
        input_valid_ratio = (float)finite_count / (float)(end - begin);
    }

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

    if (cfg->estimator == DEMOD_EST_SINE_FIT) {
        if ((cfg->mod_tone_hz <= 0.0f) || (cfg->fs_hz <= 0.0f)) {
            result->status = DEMOD_ERR_BAD_CONFIG;
            return result->status;
        }
        st = Demod_SineFit(freq_hz, begin, end, cfg->fs_hz,
                           cfg->mod_tone_hz, &center, &amp, &residual);
        if (st != DEMOD_OK) {
            result->status = st;
            return st;
        }
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
    if ((freq_hz == work) || (deviation_hz == work)) return DEMOD_ERR_BUFFER_ALIAS;

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
            result->center_hz = raw_center;
            result->deviation_peak_hz = dev_result.deviation_peak_hz;
            result->deviation_rms_hz = dev_result.deviation_rms_hz;
            result->freq_low_hz = raw_center + dev_result.freq_low_hz;
            result->freq_high_hz = raw_center + dev_result.freq_high_hz;
            result->quality = dev_result.quality;
            result->status = DEMOD_OK;
        }
    }
    return DEMOD_OK;
}

Demod_Status Demod_FM_Waveform(const float *in,
                               float *deviation_hz_out,
                               uint32_t n,
                               float *work,
                               const Demod_FMConfig *cfg,
                               Demod_FMResult *result)
{
    return Demod_FM_Process(in, NULL, deviation_hz_out, work, n, cfg, result);
}

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

void PLL_Init(PLL_t *pll, float fs, float f0, float loop_bw)
{
    const float zeta = 0.707f;
    float safe_loop_bw;
    float pd_lpf_hz;
    float wn;

    if (pll == NULL || fs <= 0.0f) return;
    safe_loop_bw = Demod_SafeLoopHz(fs, loop_bw);
    pll->fs = fs;
    pll->phase = 0.0f;
    Demod_LoopLimits(fs, f0, safe_loop_bw,
                     &pll->center_freq, &pll->min_freq, &pll->max_freq);
    pll->freq = pll->center_freq;
    wn = DEMOD_2PI * safe_loop_bw / fs;
    pll->alpha = 2.0f * zeta * wn;
    pll->beta = wn * wn;
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

    pd_lpf_hz = Demod_SelectPdLpfHz(fs, safe_loop_bw);
    IIR_CascadeInit(&pll->pd_lpf, IIR_LPF, fs, pd_lpf_hz, 0.707f, 2U);
}

void PLL_Reset(PLL_t *pll, float f0)
{
    float loop_bw;

    if (pll == NULL || pll->fs <= 0.0f) return;
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

float PLL_Update(PLL_t *pll, float sample)
{
    float sin_nco;
    float loop_err = 0.0f;
    float freq_stable_limit;
    uint8_t signal_ok;
    uint8_t lock_ok;

    if (pll == NULL || pll->fs <= 0.0f) return 0.0f;

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

float PLL_GetPhase(const PLL_t *pll)
{
    return (pll != NULL) ? pll->phase : 0.0f;
}

float PLL_GetPhaseError(const PLL_t *pll)
{
    return (pll != NULL) ? pll->phase_error : 0.0f;
}

float PLL_GetFrequency(const PLL_t *pll)
{
    if (pll == NULL || pll->fs <= 0.0f) return 0.0f;
    return pll->freq * pll->fs / DEMOD_2PI;
}

uint8_t PLL_IsLocked(const PLL_t *pll)
{
    return (pll != NULL) ? pll->locked : 0U;
}

int Demod_AMCoherent_Init(Demod_AMCoherent_t *ctx,
                          float fs,
                          float carrier_hz,
                          float pll_bw_hz,
                          float baseband_bw_hz)
{
    if (ctx == NULL || fs <= 0.0f || carrier_hz <= 0.0f) return -1;
    PLL_Init(&ctx->carrier_pll, fs, carrier_hz, pll_bw_hz);
    IIR_CascadeInit(&ctx->baseband_lpf, IIR_LPF, fs, baseband_bw_hz, 0.707f, 2U);
    ctx->dc_est = 0.0f;
    ctx->gain = 1.0f;
    return 0;
}

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

int Demod_Costas_Init(Demod_Costas_t *ctx,
                      float fs,
                      float carrier_hz,
                      float loop_bw_hz,
                      float baseband_bw_hz)
{
    const float zeta = 0.707f;
    float safe_loop_bw;
    float wn;

    if (ctx == NULL || fs <= 0.0f || carrier_hz <= 0.0f) return -1;
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

    if (ctx->cfg.preprocess.bandpass_enable != 0U) {
        float center = 0.5f * (ctx->cfg.preprocess.carrier_min_hz +
                               ctx->cfg.preprocess.carrier_max_hz);
        float bw = ctx->cfg.preprocess.carrier_max_hz -
                   ctx->cfg.preprocess.carrier_min_hz;
        float q = center / bw;
        if (q < 0.2f) q = 0.2f;
        IIR_CascadeInit(&ctx->preprocess_bpf, IIR_BPF, ctx->cfg.fs_hz,
                        center, q, 2U);
        ctx->use_preprocess_bpf = 1U;
    }

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

Demod_Status Demod_AM_ProcessBlock(Demod_AMContext *ctx,
                                   const float *in,
                                   float *out,
                                   uint32_t n,
                                   float *work,
                                   Demod_AMResult *result)
{
    /* AM 有状态块处理。
     * result==NULL 时只恢复波形；只有调用者要 result 时才做调幅度/质量分析。 */
    Demod_Status st;
    uint8_t locked = 0U;

    if (ctx == NULL || in == NULL || out == NULL) return DEMOD_ERR_NULL;
    if (ctx->ready == 0U) return DEMOD_ERR_BAD_CONFIG;
    if (n == 0U) return DEMOD_ERR_LENGTH;

    if (ctx->cfg.method == DEMOD_AM_HILBERT_MAG) {
        return Demod_AM_Waveform(in, out, n, work, &ctx->cfg, result);
    }

    if (ctx->cfg.method == DEMOD_AM_RECT) {
        /* RECT 路径使用 Context 内的 IIR 状态，避免每个块开头都出现低通启动暂态。 */
        const float gain = DEMOD_PI * 0.5f * ctx->cfg.rect_gain_correction;
        for (uint32_t i = 0U; i < n; i++) {
            float x = Demod_PreprocessUpdate(&ctx->cfg.preprocess,
                                             &ctx->preprocess_bpf,
                                             ctx->use_preprocess_bpf,
                                             &ctx->dc_est,
                                             &ctx->norm_est,
                                             in[i]);
            out[i] = gain * IIR_CascadeProcess(&ctx->rect_lpf, fabsf(x));
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
        return DEMOD_OK;
    }

    if (ctx->cfg.method == DEMOD_AM_COHERENT_PLL) {
        /* 未锁定样本输出 NaN。这样即使调用者要求 result，分析层也不会把捕获
         * 过程中的错误波形算进调幅度。 */
        for (uint32_t i = 0U; i < n; i++) {
            float x = Demod_PreprocessUpdate(&ctx->cfg.preprocess,
                                             &ctx->preprocess_bpf,
                                             ctx->use_preprocess_bpf,
                                             &ctx->dc_est,
                                             &ctx->norm_est,
                                             in[i]);
            float s;
            float c;
            float y;
            (void)PLL_Update(&ctx->state.pll.carrier_pll, x);
            PLL_GetSinCos(&ctx->state.pll.carrier_pll, &s, &c);
            y = ctx->state.pll.gain *
                IIR_CascadeProcess(&ctx->state.pll.baseband_lpf, 2.0f * x * c);
            locked = PLL_IsLocked(&ctx->state.pll.carrier_pll);
            out[i] = (locked != 0U) ? y : Demod_Nan();
        }
    } else if (ctx->cfg.method == DEMOD_AM_COHERENT_COSTAS) {
        for (uint32_t i = 0U; i < n; i++) {
            float x = Demod_PreprocessUpdate(&ctx->cfg.preprocess,
                                             &ctx->preprocess_bpf,
                                             ctx->use_preprocess_bpf,
                                             &ctx->dc_est,
                                             &ctx->norm_est,
                                             in[i]);
            float y = Demod_CostasTrack(&ctx->state.costas, x);

            locked = ctx->state.costas.locked;
            out[i] = (locked != 0U) ? y : Demod_Nan();
        }
    } else {
        Demod_ClearAMResult(result, DEMOD_ERR_UNSUPPORTED);
        return DEMOD_ERR_UNSUPPORTED;
    }

    if (locked == 0U) {
        Demod_ClearAMResult(result, DEMOD_ERR_UNLOCKED);
        return DEMOD_ERR_UNLOCKED;
    }

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

    if (ctx->cfg.preprocess.bandpass_enable != 0U) {
        float center = 0.5f * (ctx->cfg.preprocess.carrier_min_hz +
                               ctx->cfg.preprocess.carrier_max_hz);
        float bw = ctx->cfg.preprocess.carrier_max_hz -
                   ctx->cfg.preprocess.carrier_min_hz;
        float q = center / bw;
        if (q < 0.2f) q = 0.2f;
        IIR_CascadeInit(&ctx->preprocess_bpf, IIR_BPF, ctx->cfg.fs_hz,
                        center, q, 2U);
        ctx->use_preprocess_bpf = 1U;
    }

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
    Demod_Status st;
    Demod_FMResult local_result;

    if (ctx == NULL || in == NULL) return DEMOD_ERR_NULL;
    if (ctx->ready == 0U) return DEMOD_ERR_BAD_CONFIG;
    if (n < 2U) return DEMOD_ERR_LENGTH;
    if ((freq_hz == NULL) && (deviation_hz == NULL)) return DEMOD_ERR_NULL;

    if (ctx->cfg.method != DEMOD_FM_PLL) {
        float *freq_buf = (freq_hz != NULL) ? freq_hz : s_freq_work;
        Demod_FMResult dev_result;
        float raw_center;
        uint8_t want_result = (result != NULL) ? 1U : 0U;

        if (n > DEMOD_MAX_POINTS && freq_hz == NULL) return DEMOD_ERR_BUFFER_SIZE;
        if (work == NULL) return DEMOD_ERR_NULL;
        if ((freq_hz == work) || (deviation_hz == work)) return DEMOD_ERR_BUFFER_ALIAS;

        Demod_ClearFMResult(&local_result, DEMOD_OK);
        st = Demod_FM_InstFreq(in, freq_buf, work, n, &ctx->cfg, want_result ? result : NULL);
        if (st != DEMOD_OK) return st;
        if (want_result != 0U) {
            st = Demod_FM_AnalyzeFreq(freq_buf, n, &ctx->cfg, result);
            if (st != DEMOD_OK) return st;
        } else if (deviation_hz != NULL) {
            if (ctx->cfg.fc_mode == DEMOD_FC_KNOWN) {
                local_result.center_hz = ctx->cfg.carrier_hz;
            } else if (ctx->cfg.fc_mode == DEMOD_FC_MEAN) {
                uint32_t begin, end;
                Demod_Window(n, ctx->cfg.edge_discard_samples, &begin, &end);
                local_result.center_hz = Demod_Mean(freq_buf, begin, end);
            } else {
                uint32_t begin, end;
                Demod_Window(n, ctx->cfg.edge_discard_samples, &begin, &end);
                st = Demod_PercentileExact(freq_buf, begin, end, 0.50f, &local_result.center_hz);
                if (st != DEMOD_OK) return st;
            }
            result = &local_result;
        } else {
            return DEMOD_OK;
        }
        raw_center = result->center_hz;

        if (deviation_hz != NULL) {
            float last_valid = 0.0f;
            for (uint32_t i = 0U; i < n; i++) {
                float d = freq_buf[i] - raw_center;
                if (Demod_IsFinite(d) == 0U) {
                    d = last_valid;
                } else {
                    last_valid = d;
                }
                deviation_hz[i] = (ctx->cfg.baseband_lpf_enable != 0U) ?
                                  IIR_CascadeProcess(&ctx->baseband_lpf, d) : d;
            }

            {
                Demod_FMConfig dev_cfg = ctx->cfg;
                dev_cfg.fc_mode = DEMOD_FC_KNOWN;
                dev_cfg.carrier_hz = 0.0f;
                if (want_result != 0U) {
                    st = Demod_FM_AnalyzeFreq(deviation_hz, n, &dev_cfg, &dev_result);
                } else {
                    st = DEMOD_OK;
                }
                if ((want_result != 0U) && (st == DEMOD_OK)) {
                    result->center_hz = raw_center;
                    result->deviation_peak_hz = dev_result.deviation_peak_hz;
                    result->deviation_rms_hz = dev_result.deviation_rms_hz;
                    result->freq_low_hz = raw_center + dev_result.freq_low_hz;
                    result->freq_high_hz = raw_center + dev_result.freq_high_hz;
                    result->quality = dev_result.quality;
                    result->status = DEMOD_OK;
                }
            }
        }

        return DEMOD_OK;
    }

    {
        float *freq_buf = (freq_hz != NULL) ? freq_hz : deviation_hz;
        uint32_t locked_count = 0U;
        Demod_ClearFMResult(&local_result, DEMOD_OK);
        for (uint32_t i = 0U; i < n; i++) {
            float x = Demod_PreprocessUpdate(&ctx->cfg.preprocess,
                                             &ctx->preprocess_bpf,
                                             ctx->use_preprocess_bpf,
                                             &ctx->dc_est,
                                             &ctx->norm_est,
                                             in[i]);
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

        if (deviation_hz != NULL) {
            Demod_FMResult dev_result;
            Demod_FMConfig dev_cfg = ctx->cfg;
            for (uint32_t i = 0U; i < n; i++) {
                float d = freq_buf[i] - result->center_hz;
                if (Demod_IsFinite(d) == 0U) {
                    deviation_hz[i] = Demod_Nan();
                    continue;
                }
                if (ctx->cfg.baseband_lpf_enable != 0U) {
                    /* 未锁定样本不更新 IIR 状态，避免 NaN 或捕获瞬态污染后续输出。 */
                    d = IIR_CascadeProcess(&ctx->baseband_lpf, d);
                }
                deviation_hz[i] = d;
            }

            if (result != NULL) {
                /* PLL 路径也按“最终输出波形”重新估频偏：
                 * center_hz 保留原始 PLL 频率中心，deviation_* 来自低通后的频偏。 */
                dev_cfg.fc_mode = DEMOD_FC_KNOWN;
                dev_cfg.carrier_hz = 0.0f;
                st = Demod_FM_AnalyzeFreq(deviation_hz, n, &dev_cfg, &dev_result);
                if (st == DEMOD_OK) {
                    float raw_center = result->center_hz;
                    result->deviation_peak_hz = dev_result.deviation_peak_hz;
                    result->deviation_rms_hz = dev_result.deviation_rms_hz;
                    result->freq_low_hz = raw_center + dev_result.freq_low_hz;
                    result->freq_high_hz = raw_center + dev_result.freq_high_hz;
                    result->quality = dev_result.quality;
                    result->status = DEMOD_OK;
                }
            }
        }
    }

    if ((result != NULL) && (PLL_IsLocked(&ctx->pll) == 0U)) {
        result->status = DEMOD_ERR_UNLOCKED;
        return DEMOD_ERR_UNLOCKED;
    }

    return DEMOD_OK;
}
