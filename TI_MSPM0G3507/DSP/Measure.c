/**
 * @file    Measure.c
 * @brief   信号测量指标实现
 */
#include "Measure.h"
#include <math.h>
#include <string.h>

/* 判断浮点测量值是否有效。 */
static int Measure_IsFinite(float x)
{
    return isfinite(x) ? 1 : 0;
}

/* ====================== 时域测量 ====================== */

/* 计算有限样本的均值。 */
float Measure_Mean(const float *data, uint32_t len)
{
    if (data == NULL || len == 0u) return 0.0f;
    float s = 0.0f;
    uint32_t count = 0u;
    for (uint32_t i = 0; i < len; i++) {
        if (Measure_IsFinite(data[i])) { s += data[i]; count++; }
    }
    return (count > 0u) ? (s / (float)count) : 0.0f;
}

/* 计算有限样本的均方根值。 */
float Measure_RMS(const float *data, uint32_t len)
{
    if (data == NULL || len == 0u) return 0.0f;
    float s = 0.0f;
    uint32_t count = 0u;
    for (uint32_t i = 0; i < len; i++) {
        if (Measure_IsFinite(data[i])) { s += data[i] * data[i]; count++; }
    }
    return (count > 0u) ? sqrtf(s / (float)count) : 0.0f;
}

/* 计算去除直流后的交流均方根值。 */
float Measure_ACRMS(const float *data, uint32_t len)
{
    float dc = Measure_Mean(data, len);
    if (data == NULL || len == 0u) return 0.0f;
    float s = 0.0f;
    uint32_t count = 0u;
    for (uint32_t i = 0; i < len; i++) {
        if (!Measure_IsFinite(data[i])) continue;
        float ac = data[i] - dc;
        s += ac * ac;
        count++;
    }
    return (count > 0u) ? sqrtf(s / (float)count) : 0.0f;
}

/* 计算有限样本的峰峰值。 */
float Measure_Vpp(const float *data, uint32_t len)
{
    if (data == NULL || len == 0u) return 0.0f;
    uint32_t first = 0u;
    while (first < len && !Measure_IsFinite(data[first])) first++;
    if (first == len) return 0.0f;
    float vmin = data[first], vmax = data[first];
    for (uint32_t i = first + 1u; i < len; i++) {
        if (!Measure_IsFinite(data[i])) continue;
        if (data[i] > vmax) vmax = data[i];
        if (data[i] < vmin) vmin = data[i];
    }
    return vmax - vmin;
}

/* 一次计算直流、总RMS、交流RMS和峰峰值。 */
void Measure_TimeStats(const float *data, uint32_t len,
                       float *dc, float *rms, float *acrms, float *vpp)
{
    if (data == NULL || len == 0u) {
        if (dc)    *dc    = 0.0f;
        if (rms)   *rms   = 0.0f;
        if (acrms) *acrms = 0.0f;
        if (vpp)   *vpp   = 0.0f;
        return;
    }

    float mean = 0.0f;
    float m2 = 0.0f;
    float vmin = 0.0f, vmax = 0.0f;
    uint32_t count = 0u;
    for (uint32_t i = 0; i < len; i++) {
        float v = data[i];
        if (!Measure_IsFinite(v)) continue;
        if (count == 0u) vmin = vmax = v;
        count++;
        {
            float delta = v - mean;

            mean += delta / (float)count;
            m2 += delta * (v - mean);
        }
        if (v > vmax) vmax = v;
        if (v < vmin) vmin = v;
    }
    if (count == 0u) {
        if (dc) *dc = 0.0f;
        if (rms) *rms = 0.0f;
        if (acrms) *acrms = 0.0f;
        if (vpp) *vpp = 0.0f;
        return;
    }

    float ac_var = m2 / (float)count;
    if (ac_var < 0.0f) ac_var = 0.0f; /* 浮点舍入兜底。 */
    float meansq = ac_var + mean * mean;
    if (meansq < 0.0f) meansq = 0.0f;

    if (dc)    *dc    = mean;
    if (rms)   *rms   = sqrtf(meansq);
    if (acrms) *acrms = sqrtf(ac_var);
    if (vpp)   *vpp   = vmax - vmin;
}

/* ====================== 频域精测（谱插值） ====================== */

/* 对指定频谱峰执行三点抛物线插值。 */
void Measure_PeakInterp(const float *mag, uint32_t n, uint32_t k_peak,
                        float fs, Measure_Peak_t *res)
{
    if (res == NULL) return;
    res->bin = 0.0f;
    res->freq = 0.0f;
    res->amplitude = 0.0f;
    if (mag == NULL || n == 0u || !(fs > 0.0f)) return;
    if (k_peak == 0u || k_peak >= (n / 2u)) {
        /* 边界无法插值，退化为整数 bin 结果 */
        res->bin = (float)k_peak;
        res->freq = (float)k_peak * fs / (float)n;
        res->amplitude = (k_peak < (n / 2u + 1u)) ? mag[k_peak] : 0.0f;
        return;
    }

    /* 抛物线插值：用峰值及左右相邻 bin 拟合二次曲线，求顶点偏移 delta */
    float a = mag[k_peak - 1u];
    float b = mag[k_peak];
    float c = mag[k_peak + 1u];

    float denom = (a - 2.0f * b + c);
    float delta = 0.0f;
    if (fabsf(denom) > 1e-12f) {
        delta = 0.5f * (a - c) / denom;         /* delta ∈ (-0.5, 0.5) */
    }
    if (delta > 0.5f)  delta = 0.5f;
    if (delta < -0.5f) delta = -0.5f;

    float bin = (float)k_peak + delta;
    res->bin = bin;
    res->freq = bin * fs / (float)n;
    /* 顶点幅度修正（抛物线顶点值），抑制栅栏效应造成的幅度低估 */
    res->amplitude = b - 0.25f * (a - c) * delta;
}

/* 查找最大非直流频谱峰并执行插值。 */
uint32_t Measure_FindPeak(const float *mag, uint32_t n, float fs,
                          Measure_Peak_t *res)
{
    if (mag == NULL || n < 4u || !(fs > 0.0f)) {
        if (res) { res->freq = 0.0f; res->amplitude = 0.0f; res->bin = 0.0f; }
        return 0u;
    }

    /* 从 bin1 起找最大峰，跳过直流 bin0 */
    uint32_t k_max = 1u;
    int found = Measure_IsFinite(mag[1]) && (mag[1] > 0.0f);
    float v_max = found ? mag[1] : 0.0f;
    uint32_t half = n / 2u;
    for (uint32_t k = 2u; k < half; k++) {
        if (Measure_IsFinite(mag[k]) && (mag[k] > 0.0f) &&
            (!found || mag[k] > v_max)) {
            v_max = mag[k]; k_max = k; found = 1;
        }
    }
    if (!found) {
        if (res) { res->freq = 0.0f; res->amplitude = 0.0f; res->bin = 0.0f; }
        return 0u;
    }

    Measure_PeakInterp(mag, n, k_max, fs, res);
    return k_max;
}

/* ====================== 信号质量指标 ====================== */

/* 根据基波和谐波幅度计算总谐波失真。 */
float Measure_THD(const float *amp, uint32_t count, float *thd_db)
{
    if (amp == NULL || count < 2u || !Measure_IsFinite(amp[0]) || amp[0] <= 0.0f) {
        if (thd_db) *thd_db = -999.0f;
        return 0.0f;
    }
    float harm_sq = 0.0f;
    for (uint32_t h = 1u; h < count; h++) {     /* 从 2 次谐波(下标1)起 */
        if (Measure_IsFinite(amp[h])) harm_sq += amp[h] * amp[h];
    }
    float thd = sqrtf(harm_sq) / amp[0];
    if (thd_db) *thd_db = 20.0f * log10f(thd > 1e-12f ? thd : 1e-12f);
    return thd;
}

/* 填充面向电赛失真度题的默认配置。 */
void Measure_THDConfigDefault(Measure_THDConfig_t *config)
{
    if (config == NULL) return;
    config->fund_min_hz = 0.0f;
    config->fund_max_hz = 0.0f;
    config->min_fundamental_amplitude = 0.0f;
    config->max_harmonic = 5u;
    config->required_harmonic = 5u;
    config->search_half_width_bins = 1u;
    config->integration_half_width_bins = 2u;
    config->subtract_noise = 1u;
}

/* 清空 THD 结果并给出确定的失败默认值。 */
static void Measure_THDResultInit(Measure_THDResult_t *result)
{
    memset(result, 0, sizeof(*result));
    result->status = MEASURE_THD_ERROR_PARAM;
    result->thd_db = -240.0f;
    result->fund_snr_db = -240.0f;
}

/* 返回给定预测位置附近的最大有限谱线。 */
static uint32_t Measure_FindLocalPeak(const float *mag,
                                      uint32_t half,
                                      float predicted_bin,
                                      uint32_t search_bins)
{
    uint32_t center = (uint32_t)(predicted_bin + 0.5f);
    uint32_t lo = (center > search_bins) ? (center - search_bins) : 1u;
    uint32_t hi = center + search_bins;
    if (lo < 1u) lo = 1u;
    if (hi >= half) hi = half - 1u;
    if (lo > hi) return 0u;

    uint32_t best = 0u;
    float best_mag = 0.0f;
    for (uint32_t k = lo; k <= hi; k++) {
        if (Measure_IsFinite(mag[k]) && (best == 0u || mag[k] > best_mag)) {
            best = k;
            best_mag = mag[k];
        }
    }
    return best;
}

/* 聚合一个谱峰主瓣内的功率。 */
static float Measure_ClusterPowerRaw(const float *mag,
                                     uint32_t half,
                                     uint32_t center,
                                     uint32_t integration_bins,
                                     uint32_t *bin_count)
{
    uint32_t lo = (center > integration_bins) ? (center - integration_bins) : 1u;
    uint32_t hi = center + integration_bins;
    float power = 0.0f;
    uint32_t count = 0u;
    if (lo < 1u) lo = 1u;
    if (hi >= half) hi = half - 1u;

    for (uint32_t k = lo; k <= hi; k++) {
        if (Measure_IsFinite(mag[k])) {
            power += mag[k] * mag[k];
            count++;
        }
    }
    if (bin_count != NULL) *bin_count = count;
    return power;
}

/* 判断频点是否属于已定位的任一基波/谐波簇。 */
static int Measure_IsComponentBin(uint32_t bin,
                                  const uint32_t *centers,
                                  uint32_t center_count,
                                  uint32_t integration_bins)
{
    for (uint32_t i = 0u; i < center_count; i++) {
        uint32_t distance = (bin >= centers[i]) ?
                            (bin - centers[i]) : (centers[i] - bin);
        if (distance <= integration_bins) return 1;
    }
    return 0;
}

/* 从单边幅度谱自动定位基波和谐波并计算 THD。 */
Measure_THDStatus_t Measure_THDFromSpectrum(const float *mag,
                                            uint32_t n,
                                            float fs,
                                            const Measure_THDConfig_t *config,
                                            Measure_THDResult_t *result)
{
    Measure_THDConfig_t cfg;
    Measure_Peak_t fundamental_peak;
    uint32_t centers[MEASURE_THD_MAX_HARMONICS];
    float raw_power[MEASURE_THD_MAX_HARMONICS];
    uint32_t cluster_bins[MEASURE_THD_MAX_HARMONICS];
    uint32_t half;
    float df;

    if (result == NULL) return MEASURE_THD_ERROR_PARAM;
    Measure_THDResultInit(result);
    if (mag == NULL || n < 8u || (n & 1u) != 0u || !(fs > 0.0f)) {
        result->status = (n < 8u || (n & 1u) != 0u) ?
                         MEASURE_THD_ERROR_LENGTH : MEASURE_THD_ERROR_PARAM;
        return result->status;
    }

    Measure_THDConfigDefault(&cfg);
    if (config != NULL) cfg = *config;
    if (cfg.max_harmonic < 2u ||
        cfg.max_harmonic > MEASURE_THD_MAX_HARMONICS ||
        cfg.required_harmonic < 2u ||
        cfg.required_harmonic > cfg.max_harmonic ||
        cfg.fund_min_hz < 0.0f ||
        cfg.fund_max_hz < 0.0f ||
        cfg.min_fundamental_amplitude < 0.0f) {
        result->status = MEASURE_THD_ERROR_PARAM;
        return result->status;
    }

    half = n / 2u;
    df = fs / (float)n;
    result->requested_harmonic = cfg.max_harmonic;

    float search_min_hz = (cfg.fund_min_hz > 0.0f) ? cfg.fund_min_hz : df;
    float search_max_hz = (cfg.fund_max_hz > 0.0f) ?
                          cfg.fund_max_hz : (0.5f * fs - df);
    if (!(search_max_hz >= search_min_hz) || search_min_hz >= 0.5f * fs) {
        result->status = MEASURE_THD_ERROR_PARAM;
        return result->status;
    }

    /*
     * 把每个 FFT bin 视为中心频率 ±0.5 bin 的频带。这样即使低频搜索范围
     * 小于一个 bin，只要真实基波落在范围内，最近的谱线仍会参与搜索。
     */
    float min_bin_edge = search_min_hz / df - 0.5f;
    float max_bin_edge = search_max_hz / df + 0.5f;
    uint32_t k_min = (min_bin_edge > 0.0f) ?
                     (uint32_t)min_bin_edge : 1u;
    if ((float)k_min < min_bin_edge) k_min++;
    uint32_t k_max = (max_bin_edge > 0.0f) ?
                     (uint32_t)max_bin_edge : 0u;
    if (k_min < 1u) k_min = 1u;
    if (k_max >= half) k_max = half - 1u;
    if (k_min > k_max) {
        result->status = MEASURE_THD_ERROR_NO_FUNDAMENTAL;
        return result->status;
    }

    uint32_t k0 = 0u;
    float max_mag = 0.0f;
    for (uint32_t k = k_min; k <= k_max; k++) {
        if (Measure_IsFinite(mag[k]) && (k0 == 0u || mag[k] > max_mag)) {
            k0 = k;
            max_mag = mag[k];
        }
    }
    if (k0 == 0u || !(max_mag > 0.0f)) {
        result->status = MEASURE_THD_ERROR_NO_FUNDAMENTAL;
        return result->status;
    }

    Measure_PeakInterp(mag, n, k0, fs, &fundamental_peak);
    result->fund_freq = fundamental_peak.freq;
    if (fundamental_peak.amplitude < cfg.min_fundamental_amplitude) {
        result->status = MEASURE_THD_ERROR_LOW_AMPLITUDE;
        return result->status;
    }

    /* 分量间距必须大于搜索漂移和两个积分半宽，防止相邻谐波簇重叠。 */
    uint32_t min_spacing = 2u *
        ((uint32_t)cfg.search_half_width_bins +
         (uint32_t)cfg.integration_half_width_bins) + 1u;
    if (!(fundamental_peak.bin >= (float)min_spacing)) {
        result->status = MEASURE_THD_ERROR_RESOLUTION;
        return result->status;
    }

    uint32_t found = 0u;
    for (uint32_t harmonic = 1u; harmonic <= cfg.max_harmonic; harmonic++) {
        float predicted_bin = fundamental_peak.bin * (float)harmonic;
        uint32_t center;
        if (harmonic == 1u) {
            center = k0;
        } else {
            if (predicted_bin +
                (float)cfg.search_half_width_bins +
                (float)cfg.integration_half_width_bins >= (float)half) {
                break;
            }
            center = Measure_FindLocalPeak(mag,
                                           half,
                                           predicted_bin,
                                           cfg.search_half_width_bins);
            if (center == 0u) break;
        }

        if (center <= cfg.integration_half_width_bins ||
            center + cfg.integration_half_width_bins >= half) {
            break;
        }

        Measure_Peak_t component_peak;
        Measure_PeakInterp(mag, n, center, fs, &component_peak);
        centers[found] = center;
        raw_power[found] = Measure_ClusterPowerRaw(
            mag,
            half,
            center,
            cfg.integration_half_width_bins,
            &cluster_bins[found]);
        result->harmonic_freq[found] = component_peak.freq;
        found++;
    }

    if (found == 0u) {
        result->status = MEASURE_THD_ERROR_NO_FUNDAMENTAL;
        return result->status;
    }
    result->harmonics_found = (uint8_t)found;
    result->highest_harmonic = (uint8_t)found;

    float noise_power = 0.0f;
    uint32_t noise_bins = 0u;
    for (uint32_t k = 1u; k < half; k++) {
        if (Measure_IsComponentBin(k,
                                   centers,
                                   found,
                                   cfg.integration_half_width_bins)) {
            continue;
        }
        if (Measure_IsFinite(mag[k])) {
            noise_power += mag[k] * mag[k];
            noise_bins++;
        }
    }
    float noise_power_per_bin = (noise_bins > 0u) ?
                                (noise_power / (float)noise_bins) : 0.0f;
    /*
     * 再做一次 4 倍均值限幅，避免单个非谐波强杂散把噪声底抬高，
     * 进而把 H4/H5 的小功率全部扣掉。被剔除的杂散仍会计入 SINAD。
     */
    if (noise_bins > 0u && noise_power_per_bin > 0.0f) {
        float clipped_power = 0.0f;
        uint32_t clipped_bins = 0u;
        float clip_limit = 4.0f * noise_power_per_bin;
        for (uint32_t k = 1u; k < half; k++) {
            if (Measure_IsComponentBin(k,
                                       centers,
                                       found,
                                       cfg.integration_half_width_bins) ||
                !Measure_IsFinite(mag[k])) {
                continue;
            }
            float bin_power = mag[k] * mag[k];
            if (bin_power <= clip_limit) {
                clipped_power += bin_power;
                clipped_bins++;
            }
        }
        if (clipped_bins > 0u) {
            noise_power_per_bin = clipped_power / (float)clipped_bins;
        }
    }
    result->noise_rms_per_bin = sqrtf(noise_power_per_bin);

    for (uint32_t i = 0u; i < found; i++) {
        float component_power = raw_power[i];
        if (cfg.subtract_noise != 0u) {
            float correction = noise_power_per_bin * (float)cluster_bins[i];
            component_power = (component_power > correction) ?
                              (component_power - correction) : 0.0f;
        }
        result->harmonic_amp[i] = sqrtf(component_power);
    }

    result->fund_amplitude = result->harmonic_amp[0];
    if (!(result->fund_amplitude > 1e-12f)) {
        result->status = MEASURE_THD_ERROR_NO_FUNDAMENTAL;
        return result->status;
    }

    float harmonic_power = 0.0f;
    result->harmonic_ratio[0] = 1.0f;
    for (uint32_t i = 1u; i < found; i++) {
        result->harmonic_ratio[i] =
            result->harmonic_amp[i] / result->fund_amplitude;
        harmonic_power += result->harmonic_amp[i] * result->harmonic_amp[i];
    }
    result->thd = sqrtf(harmonic_power) / result->fund_amplitude;
    result->thd_percent = 100.0f * result->thd;
    result->thd_db = 20.0f * log10f(result->thd > 1e-12f ?
                                    result->thd : 1e-12f);

    float equal_band_noise = noise_power_per_bin * (float)cluster_bins[0];
    result->fund_snr_db = (equal_band_noise > 1e-20f) ?
        10.0f * log10f((result->fund_amplitude * result->fund_amplitude) /
                       equal_band_noise) : 240.0f;

    if (found < (uint32_t)cfg.required_harmonic) {
        result->status = MEASURE_THD_ERROR_BANDWIDTH;
        return result->status;
    }

    result->status = MEASURE_THD_OK;
    result->valid = 1u;
    return result->status;
}

/* 由单边幅度谱计算THD、SNR、SINAD和ENOB。 */
void Measure_Quality(const float *mag, uint32_t n, float fs,
                     uint32_t num_harm, uint32_t leak_bins,
                     Measure_Quality_t *q)
{
    Measure_THDConfig_t cfg;
    Measure_THDResult_t thd;
    if (q == NULL) return;
    q->thd = 0.0f; q->thd_db = -240.0f;
    q->snr_db = 0.0f; q->sinad_db = 0.0f; q->enob = 0.0f; q->fund_freq = 0.0f;
    if (mag == NULL || n < 8u || (n & 1u) != 0u || !(fs > 0.0f)) return;

    Measure_THDConfigDefault(&cfg);
    if (num_harm < 2u) num_harm = 2u;
    if (num_harm > MEASURE_THD_MAX_HARMONICS) {
        num_harm = MEASURE_THD_MAX_HARMONICS;
    }
    cfg.max_harmonic = (uint8_t)num_harm;
    cfg.required_harmonic = 2u;
    cfg.integration_half_width_bins =
        (leak_bins > 255u) ? 255u : (uint8_t)leak_bins;
    (void)Measure_THDFromSpectrum(mag, n, fs, &cfg, &thd);
    if (thd.harmonics_found == 0u || !(thd.fund_amplitude > 0.0f)) return;

    q->thd = thd.thd;
    q->thd_db = thd.thd_db;
    q->fund_freq = thd.fund_freq;

    float signal_power = thd.fund_amplitude * thd.fund_amplitude;
    float harmonic_power = 0.0f;
    for (uint32_t i = 1u; i < thd.harmonics_found; i++) {
        harmonic_power += thd.harmonic_amp[i] * thd.harmonic_amp[i];
    }

    float noise_power = 0.0f;
    uint32_t half = n / 2u;
    uint32_t centers[MEASURE_THD_MAX_HARMONICS];
    for (uint32_t i = 0u; i < thd.harmonics_found; i++) {
        centers[i] = (uint32_t)(thd.harmonic_freq[i] / fs * (float)n + 0.5f);
    }
    for (uint32_t k = 1u; k < half; k++) {
        if (!Measure_IsComponentBin(k,
                                    centers,
                                    thd.harmonics_found,
                                    leak_bins) &&
            Measure_IsFinite(mag[k])) {
            noise_power += mag[k] * mag[k];
        }
    }
    if (noise_power < 1e-20f) noise_power = 1e-20f;
    if (signal_power < 1e-20f) signal_power = 1e-20f;
    float noise_and_distortion = noise_power + harmonic_power;
    if (noise_and_distortion < 1e-20f) noise_and_distortion = 1e-20f;
    q->snr_db = 10.0f * log10f(signal_power / noise_power);
    q->sinad_db = 10.0f * log10f(signal_power / noise_and_distortion);
    q->enob = (q->sinad_db - 1.76f) / 6.02f;
}

/* ====================== 时域波形参数 ====================== */

/* 通过上升过零间隔估计信号频率。 */
float Measure_FreqZeroCross(const float *data, uint32_t len, float fs)
{
    if (data == NULL || len < 2u || fs <= 0.0f) return 0.0f;
    for (uint32_t i = 0u; i < len; i++) {
        if (!Measure_IsFinite(data[i])) return 0.0f;
    }

    float mean = Measure_Mean(data, len);

    float first_cross = 0.0f, last_cross = 0.0f;
    uint32_t count = 0u;

    for (uint32_t i = 1; i < len; i++) {
        float prev = data[i - 1] - mean;
        float cur  = data[i]     - mean;
        /* 上升过零：prev<0 且 cur>=0 */
        if (prev < 0.0f && cur >= 0.0f) {
            float denom = cur - prev;
            float frac  = (denom != 0.0f) ? (-prev / denom) : 0.0f; /* 线性插值 */
            float pos   = (float)(i - 1) + frac;
            if (count == 0u) first_cross = pos;
            last_cross = pos;
            count++;
        }
    }

    if (count < 2u) return 0.0f;
    float period_samples = (last_cross - first_cross) / (float)(count - 1u);
    if (period_samples <= 0.0f) return 0.0f;
    return fs / period_samples;
}

/* 提取从上升过零点开始的一个周期，并重采样为固定显示点数。 */
uint32_t Measure_ExtractPeriod(const float *data,
                               uint32_t len,
                               float fs,
                               float freq,
                               float *output,
                               uint32_t out_len)
{
    if (data == NULL || output == NULL || len < 4u || out_len < 2u ||
        !(fs > 0.0f) || freq < 0.0f) {
        return 0u;
    }
    for (uint32_t i = 0u; i < len; i++) {
        if (!Measure_IsFinite(data[i])) return 0u;
    }
    if (!(freq > 0.0f)) freq = Measure_FreqZeroCross(data, len, fs);
    if (!(freq > 0.0f)) return 0u;

    float period_samples = fs / freq;
    if (period_samples < 2.0f || period_samples > (float)(len - 1u)) return 0u;
    float mean = Measure_Mean(data, len);
    float start = -1.0f;

    for (uint32_t i = 1u; i < len; i++) {
        float prev = data[i - 1u] - mean;
        float cur = data[i] - mean;
        if (prev < 0.0f && cur >= 0.0f) {
            float denom = cur - prev;
            float frac = (fabsf(denom) > 1e-20f) ? (-prev / denom) : 0.0f;
            float candidate = (float)(i - 1u) + frac;
            if (candidate + period_samples <= (float)(len - 1u)) {
                start = candidate;
                break;
            }
        }
    }
    if (start < 0.0f) return 0u;

    for (uint32_t i = 0u; i < out_len; i++) {
        float position = start +
            period_samples * (float)i / (float)out_len;
        uint32_t index = (uint32_t)position;
        float fraction = position - (float)index;
        if (index + 1u >= len) return 0u;
        output[i] = data[index] +
                    (data[index + 1u] - data[index]) * fraction;
    }
    return out_len;
}

/* 根据上下电平中点计算占空比。 */
float Measure_DutyCycle(const float *data, uint32_t len)
{
    if (data == NULL || len == 0u) return 0.0f;
    for (uint32_t i = 0u; i < len; i++) {
        if (!Measure_IsFinite(data[i])) return 0.0f;
    }
    float vmin = data[0], vmax = data[0];
    for (uint32_t i = 1; i < len; i++) {
        if (data[i] > vmax) vmax = data[i];
        if (data[i] < vmin) vmin = data[i];
    }
    float mid = 0.5f * (vmax + vmin);
    uint32_t high = 0u;
    for (uint32_t i = 0; i < len; i++) {
        if (data[i] >= mid) high++;
    }
    return (float)high / (float)len;
}

/* 查找指定阈值的首个上升沿或下降沿位置。 */
int32_t Measure_Trigger(const float *data, uint32_t len, float level, int rising)
{
    if (data == NULL || len < 2u) return -1;
    for (uint32_t i = 1; i < len; i++) {
        if (rising) {
            if (data[i - 1] < level && data[i] >= level) return (int32_t)i;
        } else {
            if (data[i - 1] > level && data[i] <= level) return (int32_t)i;
        }
    }
    return -1;
}

/* 统计指定电压范围内的直方图。 */
void Measure_Histogram(const float *data, uint32_t len, float vmin, float vmax,
                       uint32_t *bins, uint32_t num_bins)
{
    if (data == NULL || bins == NULL || num_bins == 0u) return;
    for (uint32_t b = 0; b < num_bins; b++) bins[b] = 0u;

    if (!Measure_IsFinite(vmin) || !Measure_IsFinite(vmax)) return;
    float span = vmax - vmin;
    if (span <= 0.0f) return;
    float inv = (float)num_bins / span;

    for (uint32_t i = 0; i < len; i++) {
        if (!Measure_IsFinite(data[i])) continue;
        int32_t idx = (int32_t)((data[i] - vmin) * inv);
        if (idx < 0) idx = 0;
        if (idx >= (int32_t)num_bins) idx = (int32_t)num_bins - 1;
        bins[idx]++;
    }
}

/* 用直方图双峰法估计顶电平和底电平。 */
static void Measure_TopBase(const float *data, uint32_t len,
                            float vmin, float vmax, float *vtop, float *vbase)
{
    #define MEAS_HIST_BINS 64u /* 顶底电平估计的直方图箱数。 */
    uint32_t hist[MEAS_HIST_BINS];
    Measure_Histogram(data, len, vmin, vmax, hist, MEAS_HIST_BINS);

    float span = vmax - vmin;
    float bin_w = span / (float)MEAS_HIST_BINS;

    /* 下半区找众数 -> vbase */
    uint32_t lo_max = 0u, lo_idx = 0u;
    for (uint32_t b = 0; b < MEAS_HIST_BINS / 2u; b++) {
        if (hist[b] > lo_max) { lo_max = hist[b]; lo_idx = b; }
    }
    /* 上半区找众数 -> vtop */
    uint32_t hi_max = 0u, hi_idx = MEAS_HIST_BINS - 1u;
    for (uint32_t b = MEAS_HIST_BINS / 2u; b < MEAS_HIST_BINS; b++) {
        if (hist[b] > hi_max) { hi_max = hist[b]; hi_idx = b; }
    }

    *vbase = vmin + ((float)lo_idx + 0.5f) * bin_w;
    *vtop  = vmin + ((float)hi_idx + 0.5f) * bin_w;
    #undef MEAS_HIST_BINS
}

/* 从指定位置起查找首个阈值穿越的插值位置。 */
static float Measure_FindCross(const float *data, uint32_t len, uint32_t start,
                               float level, int dir)
{
    for (uint32_t i = (start > 0u ? start : 1u); i < len; i++) {
        float a = data[i - 1], b = data[i];
        int hit = (dir > 0) ? (a < level && b >= level)
                            : (a > level && b <= level);
        if (hit) {
            float denom = b - a;
            float frac = (denom != 0.0f) ? ((level - a) / denom) : 0.0f;
            return (float)(i - 1) + frac;
        }
    }
    return -1.0f;
}

/* 计算一帧波形的幅度、频率、占空比和边沿参数。 */
void Measure_Waveform(const float *data, uint32_t len, float fs, Measure_Wave_t *res)
{
    if (res == NULL) return;
    /* 所有字段都清零，避免参数错误时留下调用者栈上的旧值。 */
    memset(res, 0, sizeof(*res));
    if (data == NULL || len < 2u || fs <= 0.0f) return;
    for (uint32_t i = 0u; i < len; i++) {
        if (!Measure_IsFinite(data[i])) return;
    }

    /* 基础统计：min/max/mean/rms + 交流整流均值/交流峰值 */
    float vmin = data[0], vmax = data[0], sum = 0.0f, sumsq = 0.0f;
    for (uint32_t i = 0; i < len; i++) {
        float v = data[i];
        sum += v; sumsq += v * v;
        if (v > vmax) vmax = v;
        if (v < vmin) vmin = v;
    }
    float mean = sum / (float)len;
    res->vmax = vmax;
    res->vmin = vmin;
    res->vpp  = vmax - vmin;
    res->mean = mean;
    res->rms  = sqrtf(sumsq / (float)len);

    float arv = 0.0f, peak_ac = 0.0f;     /* 交流整流均值 / 交流峰值 */
    float acsq = 0.0f;
    for (uint32_t i = 0; i < len; i++) {
        float ac = data[i] - mean;
        float aac = fabsf(ac);
        arv += aac;
        acsq += ac * ac;
        if (aac > peak_ac) peak_ac = aac;
    }
    arv /= (float)len;
    float rms_ac = sqrtf(acsq / (float)len);
    res->form_factor  = (arv > 1e-9f) ? (rms_ac / arv) : 0.0f;       /* 正弦≈1.111 */
    res->crest_factor = (rms_ac > 1e-9f) ? (peak_ac / rms_ac) : 0.0f; /* 正弦≈1.414 */

    /* 频率/周期（过零法） */
    res->freq = Measure_FreqZeroCross(data, len, fs);
    res->period = (res->freq > 0.0f) ? (1.0f / res->freq) : 0.0f;

    /* 顶/底电平（直方图法） */
    Measure_TopBase(data, len, vmin, vmax, &res->vtop, &res->vbase);
    res->amplitude = res->vtop - res->vbase;

    /* 占空比 */
    res->duty = Measure_DutyCycle(data, len);

    /* 过冲：相对顶电平 */
    if (res->amplitude > 1e-9f) {
        res->overshoot = (vmax - res->vtop) / res->amplitude * 100.0f;
        if (res->overshoot < 0.0f) res->overshoot = 0.0f;
    }

    /* 上升/下降时间：10%~90% */
    if (res->amplitude > 1e-9f) {
        float lo = res->vbase + 0.1f * res->amplitude;
        float hi = res->vbase + 0.9f * res->amplitude;

        /* 上升沿：先过 10% 再过 90% */
        float t_lo = Measure_FindCross(data, len, 1u, lo, +1);
        if (t_lo >= 0.0f) {
            float t_hi = Measure_FindCross(data, len, (uint32_t)t_lo + 1u, hi, +1);
            if (t_hi > t_lo) res->rise_time = (t_hi - t_lo) / fs;
        }
        /* 下降沿：先过 90% 再过 10% */
        float t_hi2 = Measure_FindCross(data, len, 1u, hi, -1);
        if (t_hi2 >= 0.0f) {
            float t_lo2 = Measure_FindCross(data, len, (uint32_t)t_hi2 + 1u, lo, -1);
            if (t_lo2 > t_hi2) res->fall_time = (t_lo2 - t_hi2) / fs;
        }
    }
}
