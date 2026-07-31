/**
 * @file    Filter.h
 * @brief   数字滤波模块 — FIR(固定系数) + IIR(运行时可配双二阶)
 * @note    两类滤波各有所长，按需选用：
 *            ┌── FIR ──────────────────────────────────────────────┐
 *            │ 线性相位(无相位失真)、绝对稳定；系数离线设计后固定。 │
 *            │ 代价：阶数高(本例 101 阶)，每点运算量大。            │
 *            │ 适合：对相位敏感、需要陡峭且恒定群延迟的场合。       │
 *            └─────────────────────────────────────────────────────┘
 *            ┌── IIR(RBJ 双二阶) ──────────────────────────────────┐
 *            │ 运行时即可改类型/截止/Q；单级仅 5 乘 4 加，极省 CPU。│
 *            │ 代价：非线性相位；高阶需注意数值稳定。               │
 *            │ 适合：实时可调滤波、陷波(去工频)、带通选频等。       │
 *            └─────────────────────────────────────────────────────┘
 * @date    2026-06-18
 */

#ifndef FILTER_H
#define FILTER_H /* 数字滤波头文件包含保护。 */

#ifdef __cplusplus
extern "C" {
#endif

#include "arm_math.h"
#include <stdint.h>

/* ==================================================================== */
/*  第一部分：FIR 低通滤波（固定 101 阶，CMSIS arm_fir_f32 实现）         */
/* ==================================================================== */

#define FIFO_SIZE   256    /* FIR 滤波 FIFO 缓存长度 */
#define BLOCK_SIZE  32     /* FIR 滤波每次处理的数据块长度 */

extern const int   LP1;              /* FIR 抽头数 = 101 */
extern const float LP1_Resource[];   /* FIR 系数表 */

extern float fifo_data1_f[FIFO_SIZE]; /* FIR通用输入缓存。 */
extern float testOutput[FIFO_SIZE]; /* FIR通用输出缓存。 */
extern float firStateF32[BLOCK_SIZE + 101 - 1];  /* 使用实际数值，对应 LP1 */

/**
 * @brief  对输入数据数组进行 FIR 低通滤波（101 阶）。
 * @param  input        输入采样数组，长度 = num_samples
 * @param  output       滤波后输出数组，长度 = num_samples
 * @param  num_samples  要处理的采样点数；末尾不足 BLOCK_SIZE 的数据也会安全处理
 * @note   每次调用前会自动清零滤波器状态，保证独立调用之间无状态残留。
 *         如果需要连续流式滤波，请直接使用 arm_fir_f32 并自行管理状态。
 */
void arm_fir_f32_lp(float *input, float *output, uint32_t num_samples);


/* ==================================================================== */
/*  第二部分：IIR 双二阶滤波（RBJ Cookbook 设计 + DF2T 运行，可配可级联）  */
/* ==================================================================== */

/* 滤波器类型 */
typedef enum {
    IIR_LPF = 0,   /* 低通   */
    IIR_HPF,       /* 高通   */
    IIR_BPF,       /* 带通(0dB 峰值增益) */
    IIR_NOTCH,     /* 陷波(带阻，去某频点，如 50Hz 工频) */
    IIR_PEAK       /* 峰化/陷落(参数均衡，gain_db 决定增益方向) */
} IIR_Type_t;

/* 单级双二阶 */
typedef struct {
    float b0, b1, b2;   /* 分子系数(已按 a0 归一化) */
    float a1, a2;       /* 分母系数(已按 a0 归一化，a0=1) */
    float z1, z2;       /* DF2T 状态变量 */
} IIR_Biquad_t;

/* 级联滤波器 */
#define IIR_MAX_STAGES   4              /* 最多级数(每级2阶，最高8阶) */
typedef struct {
    IIR_Biquad_t stage[IIR_MAX_STAGES]; /* 各级双二阶滤波器。 */
    uint8_t num_stages; /* 当前有效级数。 */
} IIR_Cascade_t;

/* ---- 设计接口 ---- */

/**
 * @brief  设计单级双二阶系数（LP/HP/BP/NOTCH 用）。
 * @param  bq    [out] 双二阶实例
 * @param  type  滤波类型（LPF/HPF/BPF/NOTCH）
 * @param  fs    采样率 (Hz)
 * @param  fc    中心/截止频率 (Hz)，需满足 0 < fc < fs/2
 * @param  Q     品质因数：LP/HP 常取 0.707(巴特沃斯)；BP/NOTCH 越大越窄
 * @note   会清零状态。PEAK 型请用 IIR_DesignPeak()。
 */
void IIR_Design(IIR_Biquad_t *bq, IIR_Type_t type, float fs, float fc, float Q);

/**
 * @brief  设计峰化/陷落型(参数均衡)双二阶。
 * @param  gain_db  增益(dB)：>0 峰化(提升)，<0 陷落(衰减)
 */
void IIR_DesignPeak(IIR_Biquad_t *bq, float fs, float fc, float Q, float gain_db);

/* ---- 运行接口 ---- */

/** @brief 清零单级状态（切换信号或重设系数后调用，避免暂态残留）。 */
void IIR_Reset(IIR_Biquad_t *bq);

/**
 * @brief  单点滤波（DF2T）。
 * @return 滤波输出 y[n]
 */
static inline float IIR_Process(IIR_Biquad_t *bq, float x)
{
    float y  = bq->b0 * x + bq->z1;
    bq->z1   = bq->b1 * x - bq->a1 * y + bq->z2;
    bq->z2   = bq->b2 * x - bq->a2 * y;
    return y;
}

/** @brief 整块滤波（支持原地，out 可等于 in）。 */
void IIR_ProcessBlock(IIR_Biquad_t *bq, const float *in, float *out, uint32_t len);

/* ---- 级联接口 ---- */

/**
 * @brief  初始化级联滤波器：把 num_stages 个相同设定的 Biquad 串联，提高阶数/陡度。
 * @param  num_stages  级数 (1~IIR_MAX_STAGES)
 */
void IIR_CascadeInit(IIR_Cascade_t *c, IIR_Type_t type,
                     float fs, float fc, float Q, uint8_t num_stages);

/** @brief 级联单点滤波。 */
float IIR_CascadeProcess(IIR_Cascade_t *c, float x);

/** @brief 级联整块滤波（支持原地）。 */
void IIR_CascadeBlock(IIR_Cascade_t *c, const float *in, float *out, uint32_t len);

/** @brief 清零级联所有状态。 */
void IIR_CascadeReset(IIR_Cascade_t *c);


/* ==================================================================== */
/*  第三部分：非线性 / 多速率工具（中值、滑动平均、抽取降采样）          */
/* ==================================================================== */

/**
 * @brief  滑动平均(移动平均)滤波 —— 平滑随机噪声、做简单低通。
 * @param  in      输入数组
 * @param  out     输出数组(可与 in 不同；不支持原地)
 * @param  len     长度
 * @param  window  窗口长度(>=1)，越大越平滑、延迟越大
 * @note   采用滑动累加，复杂度 O(len)，与窗口大小无关，非常快。边界用可用样本数归一。
 */
void Filter_MovingAverage(const float *in, float *out, uint32_t len, uint32_t window);

/**
 * @brief  中值滤波 —— 去除脉冲/尖峰噪声(滑动平均去不掉的毛刺)。
 * @param  in      输入数组
 * @param  out     输出数组(不支持原地)
 * @param  len     长度
 * @param  window  窗口长度(奇数，1~31)，偶数会自动 -1
 * @note   每点对邻域排序取中值；窗口小(3/5/7)时很快。边界用镜像扩展。
 */
void Filter_Median(const float *in, float *out, uint32_t len, uint32_t window);

/**
 * @brief  抗混叠抽取降采样 —— 先 IIR 低通(防混叠)再每 factor 点取 1 点。
 * @param  in      输入数组
 * @param  len     输入长度
 * @param  factor  抽取因子(>=2)，输出采样率 = 原采样率 / factor
 * @param  out     [out] 降采样结果，长度需 >= floor(len/factor)
 * @return 实际输出点数 = floor(len/factor)
 * @note   防混叠低通截止取 0.8*(新奈奎斯特)。若只需简单抽取可自行跳过本函数。
 */
uint32_t Filter_Decimate(const float *in, uint32_t len, uint32_t factor, float *out);

#ifdef __cplusplus
}
#endif

#endif /* FILTER_H */
