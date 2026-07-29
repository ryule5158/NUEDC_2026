/**
 * @file    Demod.h
 * @brief   AM/FM demodulation and modulation analysis helpers.
 *
 * Design rule:
 *   - demodulation functions recover a waveform;
 *   - analysis functions estimate tone parameters from an existing waveform;
 *   - legacy APIs are kept as thin compatibility wrappers.
 */

/* 解调模块头文件保护宏。 */
#ifndef __DEMOD_H
#define __DEMOD_H /* 解调模块头文件保护宏。 */

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "arm_math.h"
#include "Filter.h"
#include <stdint.h>

/* 解调模块公共状态码。
 * Public functions return UNSUPPORTED when a declared
 * enum path is not valid for that API, instead of silently using another path. */
typedef enum {
    DEMOD_OK = 0,
    DEMOD_ERR_NULL,
    DEMOD_ERR_LENGTH,
    DEMOD_ERR_FFT_SIZE,
    DEMOD_ERR_BAD_FS,
    DEMOD_ERR_BAD_CONFIG,
    DEMOD_ERR_LOW_SIGNAL,
    DEMOD_ERR_UNLOCKED,
    DEMOD_ERR_NO_VALID_DATA,
    DEMOD_ERR_UNSUPPORTED,
    DEMOD_ERR_BUFFER_SIZE,
    DEMOD_ERR_BUFFER_ALIAS
} Demod_Status;

/* 参数估计方式。
 * MEDIAN_MULTI_FRAME 需要跨帧历史状态；当前单帧 Analyze API 会明确返回
 * DEMOD_ERR_UNSUPPORTED，而不是静默退化成其它算法。 */
typedef enum {
    DEMOD_EST_PEAK_RAW = 0,
    DEMOD_EST_PERCENTILE,
    DEMOD_EST_SINE_FIT,
    DEMOD_EST_RMS,
    DEMOD_EST_MEDIAN_MULTI_FRAME
} Demod_Estimator_t;

/* FM 中心频率的来源。
 * KNOWN 用调用者给定的 carrier_hz；MEAN/MEDIAN 从有效瞬时频率样本估计。 */
typedef enum {
    DEMOD_FC_KNOWN = 0,
    DEMOD_FC_MEAN,
    DEMOD_FC_MEDIAN
} Demod_FcMode_t;

/* AM 解调方法。 */
typedef enum {
    DEMOD_AM_RECT = 0,
    DEMOD_AM_HILBERT_MAG,
    DEMOD_AM_COHERENT_PLL,
    DEMOD_AM_COHERENT_COSTAS
} Demod_AMMethod_t;

/* AM 输出语义。
 * ENVELOPE_ABS: 非负包络，适合普通 AM 幅度显示。
 * ENVELOPE_SIGNED: 有符号相干输出，只适用于 PLL/Costas。
 * MESSAGE_ZERO_MEAN/NORMALIZED: 减去估计载波项，得到零均值/归一化消息波形。
 * 注意：Costas/DSB-SC 存在整体正负号二义性，这是信号本身决定的。 */
typedef enum {
    DEMOD_AM_OUT_ENVELOPE_ABS = 0,
    DEMOD_AM_OUT_ENVELOPE_SIGNED,
    DEMOD_AM_OUT_MESSAGE_ZERO_MEAN,
    DEMOD_AM_OUT_NORMALIZED
} Demod_AMOutputMode_t;

/* FM 鉴频方法。 */
typedef enum {
    DEMOD_FM_PHASE_DIFF = 0,
    DEMOD_FM_CONJ_PRODUCT,
    DEMOD_FM_QUAD_DERIV,
    DEMOD_FM_PLL
} Demod_FMMethod_t;

/* 统一预处理配置。
 * bandpass 是信道选择，不是纯载波提取。
 * AM/FM 高层 API 会把父配置里的 fs_hz 同步进来；如果直接调用
 * Demod_AnalyticSignal()，调用者必须自己保证 fs_hz 与采样率一致。 */
typedef struct {
    uint8_t remove_dc;
    uint8_t normalize;
    uint8_t bandpass_enable;
    float fs_hz;
    float carrier_min_hz;
    float carrier_max_hz;
    float amplitude_gate;
} Demod_PreprocessConfig;

/* 参数估计值及质量指标。 */
typedef struct {
    float value;
    float noise_rms;
    float snr_est;
    float residual_rms;
    float valid_ratio;
    float confidence;
} Demod_EstimateQuality;

/* AM 配置。
 * 无状态接口只支持 RECT/HILBERT_MAG；PLL/Costas 必须使用 Demod_AMContext，
 * 否则载波恢复状态每帧重置，结果不可靠。 */
typedef struct {
    Demod_AMMethod_t method;
    Demod_PreprocessConfig preprocess;
    Demod_Estimator_t estimator;
    Demod_AMOutputMode_t output_mode;
    uint32_t edge_discard_samples;
    float rect_lpf_hz;
    float rect_gain_correction;
    float tone_hz;
    float fs_hz;
    float carrier_hz;
    float pll_bw_hz;           /* 环路速度参数，非严格闭环带宽 */
    float baseband_lpf_hz;
} Demod_AMConfig;

/* AM 分析结果。
 * valid_begin/valid_end 是参数分析使用的有效区间，不表示输出数组边缘样本
 * 已经被删除；FFT Hilbert 的边缘样本仍会原样留在输出里。 */
typedef struct {
    float carrier_amp;
    float modulation_depth;
    float env_low;
    float env_high;
    uint32_t valid_begin;
    uint32_t valid_end;
    Demod_EstimateQuality quality;
    Demod_Status status;
} Demod_AMResult;

/* FM 配置。
 * CONJ_PRODUCT 是默认推荐鉴频方式；QUAD_DERIV 只适合较低归一化频率，
 * 在高数字中频下会返回 UNSUPPORTED，避免系统性低估频偏。 */
typedef struct {
    Demod_FMMethod_t method;
    Demod_PreprocessConfig preprocess;
    Demod_Estimator_t estimator;
    Demod_FcMode_t fc_mode;
    uint32_t edge_discard_samples;
    float fs_hz;
    float carrier_hz;
    float mod_tone_hz;
    float pll_bw_hz;           /* 环路速度参数，非严格闭环带宽 */
    uint8_t baseband_lpf_enable;
    float baseband_lpf_hz;
} Demod_FMConfig;

/* FM 分析结果。
 * center_hz 从原始有效瞬时频率估计；deviation_* 优先基于可选低通后的
 * 频偏波形估计，因此比直接对原始相位差分取极值更抗尖峰。 */
typedef struct {
    float center_hz;
    float deviation_peak_hz;
    float deviation_rms_hz;
    float freq_low_hz;
    float freq_high_hz;
    uint32_t valid_begin;
    uint32_t valid_end;
    Demod_EstimateQuality quality;
    Demod_Status status;
} Demod_FMResult;

/* 将数组中的 NaN/Inf 替换为指定值或上一个有效值。必须在 FFT/Goertzel 等累加算法前调用。 */
void Demod_ReplaceNaN(float *data, uint32_t len, float fill);

/* 填充预处理保守默认配置。 */
void Demod_PreprocessDefault(Demod_PreprocessConfig *cfg);
/* 填充 AM 解调保守默认配置。 */
void Demod_AMConfigDefault(Demod_AMConfig *cfg, float fs_hz);
/* 填充 FM 解调保守默认配置。 */
void Demod_FMConfigDefault(Demod_FMConfig *cfg, float fs_hz);

/* 工作区需求查询。
 * FFT Hilbert 需要 2*n 个 float。当前接口无法检查调用者实际分配长度，
 * 所以调用前应按这里返回值分配 work。 */
uint32_t Demod_AnalyticWorkFloats(uint32_t n);
/* 查询 FM 处理所需的 float 工作区长度。 */
uint32_t Demod_FMWorkFloats(uint32_t n);

/* 使用 FFT-Hilbert 在 analytic_iq[2*n] 中构造解析信号。
 * Requirement: in and analytic_iq must not overlap. */
/* 输入与输出工作区不得重叠。 */
Demod_Status Demod_AnalyticSignal(const float *in,
                                  float *analytic_iq,
                                  uint32_t n,
                                  const Demod_PreprocessConfig *cfg);

/* 无状态 AM 块处理。
 * 只负责恢复波形；result==NULL 时不会强制做调幅度分析。
 * 仅支持 RECT/HILBERT_MAG；相干解调请使用 Demod_AMContext。 */
Demod_Status Demod_AM_Waveform(const float *in,
                               float *baseband_out,
                               uint32_t n,
                               float *work,
                               const Demod_AMConfig *cfg,
                               Demod_AMResult *result);

/* 对已有 AM 包络/基带做单帧参数分析。 */
Demod_Status Demod_AM_AnalyzeTone(const float *baseband,
                                  uint32_t n,
                                  const Demod_AMConfig *cfg,
                                  Demod_AMResult *result);

/* 无状态 FM Hilbert 鉴频。
 * 支持 PHASE_DIFF/CONJ_PRODUCT/QUAD_DERIV；DEMOD_FM_PLL 必须用
 * Demod_FMContext，因为 PLL 捕获和锁定状态需要跨块保留。 */
Demod_Status Demod_FM_InstFreq(const float *in,
                               float *freq_hz,
                               float *analytic_work,
                               uint32_t n,
                               const Demod_FMConfig *cfg,
                               Demod_FMResult *result);

/* 对已有瞬时频率数组做中心频率/频偏估计；NaN 样本会被跳过。 */
Demod_Status Demod_FM_AnalyzeFreq(const float *freq_hz,
                                  uint32_t n,
                                  const Demod_FMConfig *cfg,
                                  Demod_FMResult *result);

/* 无状态 FM 一站式处理。
 * work 必须是 2*n float，且不能与 freq_hz/deviation_hz 别名。
 * result==NULL 时只恢复波形，不做完整频偏质量分析。 */
Demod_Status Demod_FM_Process(const float *in,
                              float *freq_hz,
                              float *deviation_hz,
                              float *work,
                              uint32_t n,
                              const Demod_FMConfig *cfg,
                              Demod_FMResult *result);

/* 一站式恢复 FM 频偏波形。 */
Demod_Status Demod_FM_Waveform(const float *in,
                               float *deviation_hz_out,
                               uint32_t n,
                               float *work,
                               const Demod_FMConfig *cfg,
                               Demod_FMResult *result);

/* 兼容旧接口：全波整流并低通得到 AM 包络。 */
void Demod_EnvelopeRect(const float *in, float *env, uint32_t len, float fs, float fc);
/* 兼容旧接口：使用 Hilbert 解析信号得到 AM 包络。 */
int Demod_EnvelopeHilbert(const float *in, float *env, float *work, uint32_t n);
/* 兼容旧接口：由包络估计 AM 调制度。 */
float Demod_AM_Depth(const float *env, uint32_t len, float *carrier);
/* 兼容旧接口：由输入信号计算瞬时频率。 */
int Demod_InstFreq(const float *in, float *freq_out, float *work, uint32_t n, float fs);
/* 兼容旧接口：估计 FM 峰值频偏和中心频率。 */
float Demod_FM_Deviation(const float *in, float *work, uint32_t n, float fs,
                         float *fc_center);

/* 二阶载波锁相环运行状态。 */
typedef struct {
    float fs;
    float phase;
    float freq;
    float center_freq;
    float min_freq;
    float max_freq;
    float alpha;
    float beta;
    float pd_raw;              /* 原始乘法鉴相输出，含 2fc 分量 */
    float pd_low;              /* 低通后的鉴相输出 */
    float phase_error;         /* 归一化低通鉴相量，不是严格弧度相差 */
    float amplitude;
    float amplitude_gate;
    float lock_metric;
    float lock_threshold;
    float freq_delta;
    float amplitude_alpha;
    float error_alpha;
    uint16_t lock_count;
    uint16_t lock_target_count;
    uint8_t locked;
    IIR_Cascade_t pd_lpf;
} PLL_t;

/* 锁相环单次输出与诊断结果。 */
typedef struct {
    float freq_hz;
    float deviation_hz;
    float phase_rad;
    float phase_error;
    float amplitude;
    float lock_metric;
    uint8_t locked;
} Demod_PLLResult;

/* 按环路速度参数初始化载波锁相环。 */
void PLL_Init(PLL_t *pll, float fs, float f0, float loop_bw);
/* 使用显式环路参数初始化载波锁相环。 */
void PLL_InitManual(PLL_t *pll, float fs, float f0,
                float alpha, float beta, float zeta,
                float pd_lpf_hz);
/* 保留配置并复位锁相环运行状态。 */
void PLL_Reset(PLL_t *pll, float f0);
/* 输入一个样本并更新锁相环。 */
float PLL_Update(PLL_t *pll, float sample);
/* 获取当前 NCO 正弦和余弦值。 */
void PLL_GetSinCos(const PLL_t *pll, float *sin_out, float *cos_out);
/* 获取当前锁相环相位。 */
float PLL_GetPhase(const PLL_t *pll);
/* 获取当前归一化鉴相误差。 */
float PLL_GetPhaseError(const PLL_t *pll);
/* 获取当前锁相环频率。 */
float PLL_GetFrequency(const PLL_t *pll);
/* 查询锁相环是否已锁定。 */
uint8_t PLL_IsLocked(const PLL_t *pll);

/* PLL 相干 AM 解调器状态。 */
typedef struct {
    PLL_t carrier_pll;
    IIR_Cascade_t baseband_lpf;
    float dc_est;
    float gain;
} Demod_AMCoherent_t;

/* 初始化 PLL 相干 AM 解调器。 */
int Demod_AMCoherent_Init(Demod_AMCoherent_t *ctx,
                          float fs,
                          float carrier_hz,
                          float pll_bw_hz,
                          float baseband_bw_hz);

/* 输入一个样本并输出 PLL 相干 AM 基带。 */
float Demod_AMCoherent_Update(Demod_AMCoherent_t *ctx, float sample);

/* Costas 环载波恢复与基带滤波状态。 */
typedef struct {
    float fs;
    float phase;
    float freq;
    float center_freq;
    float min_freq;
    float max_freq;
    float alpha;
    float beta;
    float dc_est;
    float gain;
    float phase_error;         /* 功率归一化鉴相量，不是严格弧度相差 */
    float amplitude;
    float power;
    float power_gate;
    float lock_metric;
    float lock_threshold;
    float freq_delta;
    float amplitude_alpha;
    float error_alpha;
    uint16_t lock_count;
    uint16_t lock_target_count;
    uint8_t locked;
    IIR_Cascade_t i_lpf;
    IIR_Cascade_t q_lpf;
} Demod_Costas_t;

/* 初始化 Costas 环 AM 解调器。 */
int Demod_Costas_Init(Demod_Costas_t *ctx,
                      float fs,
                      float carrier_hz,
                      float loop_bw_hz,
                      float baseband_bw_hz);

/* Costas 输出存在 0/pi 锁定二义性：对 DSB-SC 来说，out 可能整体反相。 */
float Demod_Costas_Update(Demod_Costas_t *ctx, float sample);

/* 有状态 AM 上下文。
 * RECT 路径保留低通/BPF/DC 状态；PLL/Costas 路径保留载波恢复状态；
 * Hilbert 仍是 FFT 帧处理，不等价于 overlap-save/FIR Hilbert 流式实现。 */
typedef struct {
    Demod_AMConfig cfg;
    union {
        Demod_AMCoherent_t pll;
        Demod_Costas_t costas;
    } state;
    IIR_Cascade_t rect_lpf;
    IIR_Cascade_t preprocess_bpf;
    float dc_est;
    float norm_est;
    uint8_t use_preprocess_bpf;
    uint8_t ready;
} Demod_AMContext;

/* 初始化 AM 上下文，必须在 Demod_AM_ProcessBlock() 前调用。 */
Demod_Status Demod_AM_Init(Demod_AMContext *ctx, const Demod_AMConfig *cfg);

/* 使用上下文处理一个 AM 块。
 * PLL/Costas 未锁定的样本会写 NaN，且块结束未锁定时返回 DEMOD_ERR_UNLOCKED。 */
Demod_Status Demod_AM_ProcessBlock(Demod_AMContext *ctx,
                                   const float *in,
                                   float *out,
                                   uint32_t n,
                                   float *work,
                                   Demod_AMResult *result);

/* 有状态 FM 上下文。
 * PLL 路径保留环路状态；非 PLL 路径保留基带低通状态，避免每块重置滤波器。 */
typedef struct {
    Demod_FMConfig cfg;
    PLL_t pll;
    IIR_Cascade_t baseband_lpf;
    IIR_Cascade_t preprocess_bpf;
    float dc_est;
    float norm_est;
    uint8_t use_preprocess_bpf;
    uint8_t ready;
} Demod_FMContext;

/* 初始化 FM 上下文，必须在 Demod_FM_ProcessBlock() 前调用。 */
Demod_Status Demod_FM_Init(Demod_FMContext *ctx, const Demod_FMConfig *cfg);

/* 使用上下文处理一个 FM 块。
 * freq_hz 可为 NULL；非 PLL 方法仍需要 2*n float 的 work。
 * PLL 未锁定样本写 NaN，不参与中心频率和频偏统计。 */
Demod_Status Demod_FM_ProcessBlock(Demod_FMContext *ctx,
                                   const float *in,
                                   float *freq_hz,
                                   float *deviation_hz,
                                   uint32_t n,
                                   float *work,
                                   Demod_FMResult *result);

/* BPSK 解调配置（平方载波恢复 + I/Q 正交解调 + 相位投影） */
typedef struct {
    float fs_hz;                    /* 采样率 */
    float if_search_min_hz;         /* 2×IF 搜索下限，例 360000 */
    float if_search_max_hz;         /* 2×IF 搜索上限，例 440000 */
    float lpf_hz;                   /* 基带低通截止，例 25000 */
    uint32_t edge_discard_samples;  /* 边缘丢弃点数 */
} Demod_PSKConfig;

/* BPSK 解调结果。 */
typedef struct {
    float if_hz;                    /* 估计的载波频率 (Hz) */
    float rc_bps;                   /* 估计的码速率 (bps) */
    Demod_Status status;
} Demod_PSKResult;

/* 填充 BPSK 解调默认配置。 */
void Demod_PSKConfigDefault(Demod_PSKConfig *cfg, float fs_hz);

/**
 * @brief  BPSK 相干解调（平方载波恢复 + I/Q 正交混频 + 相位投影）。
 * @param  in             输入实信号，长度 n
 * @param  baseband_out   解调 NRZ 基带输出，长度 n（正负交替方波）
 * @param  work           工作缓冲区，长度 >= 2*n float；不得与 in 重叠
 * @param  n              采样点数（需为 2 的幂，例 4096）
 * @param  cfg            配置参数
 * @param  result         [out] 载波频率、码速率等结果（可为 NULL）
 * @return DEMOD_OK 成功
 */
/* 使用平方载波恢复、正交混频和相位投影完成 BPSK 相干解调。 */
Demod_Status Demod_PSK_Demodulate(const float *in,
                                   float *baseband_out,
                                   float *work,
                                   uint32_t n,
                                   const Demod_PSKConfig *cfg,
                                   Demod_PSKResult *result);

#ifdef __cplusplus
}
#endif

#endif /* __DEMOD_H */
