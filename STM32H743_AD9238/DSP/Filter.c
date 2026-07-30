/**
 * @file    Filter.c
 * @brief   数字滤波模块实现 —— FIR(固定系数) + IIR(RBJ 双二阶)
 */
#include "Filter.h"
#include <math.h>

/* ==================================================================== */
/*  第一部分：FIR 低通滤波                                                */
/* ==================================================================== */
const int LP1 = 101; /* FIR低通抽头数。 */
/* FIR低通固定系数表。 */
const float LP1_Resource[101] = {
  -6.246264661e-19,-0.0003093869891,-0.0005282048369,-0.0005686038639,-0.0003847338376,
   8.00412014e-19,0.0004782403412,0.0008727228269,0.0009882040322, 0.000692866859,
  -1.310561491e-18,-0.0008915323997,-0.001632961212,-0.001844529994,-0.001284471364,
  2.105137849e-18, 0.001617582864,  0.00292513473, 0.003260490717, 0.002240399364,
  -3.10636264e-18,  -0.0027498852,-0.004913678393,-0.005416159518,-0.003683449002,
  4.216228504e-18, 0.004441405647, 0.007878025062, 0.008629448712, 0.005838892423,
  -5.326094782e-18,-0.006995317526, -0.01239417586,  -0.0135824373,-0.009210349992,
  6.327319572e-18,  0.01115160249,  0.01993717439,  0.02211486176,  0.01523578633,
  -7.121896034e-18, -0.01932843029, -0.03574078903, -0.04141482711, -0.03022013977,
  7.632045615e-18,  0.04616641253,   0.1002354994,   0.1510385275,    0.187203452,
     0.2002946883,    0.187203452,   0.1510385275,   0.1002354994,  0.04616641253,
  7.632045615e-18, -0.03022013977, -0.04141482711, -0.03574078903, -0.01932843029,
  -7.121896034e-18,  0.01523578633,  0.02211486176,  0.01993717439,  0.01115160249,
  6.327319572e-18,-0.009210349992,  -0.0135824373, -0.01239417586,-0.006995317526,
  -5.326094782e-18, 0.005838892423, 0.008629448712, 0.007878025062, 0.004441405647,
  4.216228504e-18,-0.003683449002,-0.005416159518,-0.004913678393,  -0.0027498852,
  -3.10636264e-18, 0.002240399364, 0.003260490717,  0.00292513473, 0.001617582864,
  2.105137849e-18,-0.001284471364,-0.001844529994,-0.001632961212,-0.0008915323997,
  -1.310561491e-18, 0.000692866859,0.0009882040322,0.0008727228269,0.0004782403412,
   8.00412014e-19,-0.0003847338376,-0.0005686038639,-0.0005282048369,-0.0003093869891,
  -6.246264661e-19
};
float fifo_data1_f[FIFO_SIZE] = {0}; /* FIR通用输入缓存。 */
float testOutput[FIFO_SIZE] = {0}; /* FIR通用输出缓存。 */
float firStateF32[BLOCK_SIZE + 101 - 1] = {0}; /* FIR状态缓存。 */
/**
  * @brief  对输入数据数组进行FIR低通滤波（101阶）。
  * @param  input  输入采样数组，长度 = num_samples
  * @param  output 滤波后输出数组，长度 = num_samples
  * @param  num_samples 要处理的采样点数，必须是BLOCK_SIZE的整数倍
  * @note   每次调用前会自动清零滤波器状态，保证独立调用之间无状态残留。
  *         如果需要连续流式滤波，请直接使用arm_fir_f32并自行管理状态。
  */
void arm_fir_f32_lp(float *input, float *output, uint32_t num_samples)
{
    uint32_t i;
    arm_fir_instance_f32 S;

    if (input == NULL || output == NULL || num_samples == 0U) return;

    /* 清零滤波器状态，确保每次调用独立 */
    for (i = 0; i < (BLOCK_SIZE + LP1 - 1); i++)
    {
        firStateF32[i] = 0.0f;
    }

    arm_fir_init_f32(&S, LP1, (float *)LP1_Resource, firStateF32, BLOCK_SIZE);

    /* 按 BLOCK_SIZE 分块；最后的短块复用同一状态，避免越界。 */
    for (i = 0; (i + BLOCK_SIZE) <= num_samples; i += BLOCK_SIZE)
    {
        arm_fir_f32(&S, input + i, output + i, BLOCK_SIZE);
    }
    if (i < num_samples) {
        arm_fir_f32(&S, input + i, output + i, num_samples - i);
    }
}


/* ==================================================================== */
/*  第二部分：IIR 双二阶滤波（RBJ Cookbook 设计 + DF2T 运行）             */
/* ==================================================================== */

#define IIR_2PI 6.28318530717958647692f /* 两倍圆周率。 */

/* 清零单级双二阶滤波器状态。 */
void IIR_Reset(IIR_Biquad_t *bq)
{
    if (bq == NULL) return;
    bq->z1 = 0.0f;
    bq->z2 = 0.0f;
}

/* 设计低通、高通、带通或陷波双二阶系数。 */
void IIR_Design(IIR_Biquad_t *bq, IIR_Type_t type, float fs, float fc, float Q)
{
    if (bq == NULL || !isfinite(fs) || !isfinite(fc) || !isfinite(Q) || fs <= 0.0f) return;

    /* 截止频率限幅到 (0, fs/2) 开区间，防止 tan/cos 异常 */
    if (fc <= 0.0f)        fc = 1.0f;
    if (fc >= 0.5f * fs)   fc = 0.499f * fs;
    if (Q  <= 0.0f)        Q  = 0.707f;

    const float w0    = IIR_2PI * fc / fs;
    const float cosw0 = cosf(w0);
    const float sinw0 = sinf(w0);
    const float alpha = sinw0 / (2.0f * Q);

    float b0, b1, b2, a0, a1, a2;

    switch (type) {
    case IIR_HPF:
        b0 =  (1.0f + cosw0) * 0.5f;
        b1 = -(1.0f + cosw0);
        b2 =  (1.0f + cosw0) * 0.5f;
        a0 =   1.0f + alpha;
        a1 =  -2.0f * cosw0;
        a2 =   1.0f - alpha;
        break;

    case IIR_BPF:   /* 常数 0dB 峰值增益带通 */
        b0 =  alpha;
        b1 =  0.0f;
        b2 = -alpha;
        a0 =  1.0f + alpha;
        a1 = -2.0f * cosw0;
        a2 =  1.0f - alpha;
        break;

    case IIR_NOTCH:
        b0 =  1.0f;
        b1 = -2.0f * cosw0;
        b2 =  1.0f;
        a0 =  1.0f + alpha;
        a1 = -2.0f * cosw0;
        a2 =  1.0f - alpha;
        break;

    case IIR_LPF:
    default:
        b0 = (1.0f - cosw0) * 0.5f;
        b1 =  1.0f - cosw0;
        b2 = (1.0f - cosw0) * 0.5f;
        a0 =  1.0f + alpha;
        a1 = -2.0f * cosw0;
        a2 =  1.0f - alpha;
        break;
    }

    /* 按 a0 归一化 */
    const float inv_a0 = 1.0f / a0;
    bq->b0 = b0 * inv_a0;
    bq->b1 = b1 * inv_a0;
    bq->b2 = b2 * inv_a0;
    bq->a1 = a1 * inv_a0;
    bq->a2 = a2 * inv_a0;

    IIR_Reset(bq);
}

/* 设计峰化或陷落型双二阶系数。 */
void IIR_DesignPeak(IIR_Biquad_t *bq, float fs, float fc, float Q, float gain_db)
{
    if (bq == NULL || !isfinite(fs) || !isfinite(fc) || !isfinite(Q) ||
        !isfinite(gain_db) || fs <= 0.0f) return;

    if (fc <= 0.0f)        fc = 1.0f;
    if (fc >= 0.5f * fs)   fc = 0.499f * fs;
    if (Q  <= 0.0f)        Q  = 0.707f;

    const float A     = powf(10.0f, gain_db / 40.0f);
    const float w0    = IIR_2PI * fc / fs;
    const float cosw0 = cosf(w0);
    const float sinw0 = sinf(w0);
    const float alpha = sinw0 / (2.0f * Q);

    const float b0 =  1.0f + alpha * A;
    const float b1 = -2.0f * cosw0;
    const float b2 =  1.0f - alpha * A;
    const float a0 =  1.0f + alpha / A;
    const float a1 = -2.0f * cosw0;
    const float a2 =  1.0f - alpha / A;

    const float inv_a0 = 1.0f / a0;
    bq->b0 = b0 * inv_a0;
    bq->b1 = b1 * inv_a0;
    bq->b2 = b2 * inv_a0;
    bq->a1 = a1 * inv_a0;
    bq->a2 = a2 * inv_a0;

    IIR_Reset(bq);
}

/* 对一块数据执行单级双二阶滤波。 */
void IIR_ProcessBlock(IIR_Biquad_t *bq, const float *in, float *out, uint32_t len)
{
    if (bq == NULL || in == NULL || out == NULL) return;
    for (uint32_t i = 0; i < len; i++) {
        out[i] = IIR_Process(bq, in[i]);
    }
}

/* ---- 级联 ---- */

/* 初始化指定级数的双二阶级联滤波器。 */
void IIR_CascadeInit(IIR_Cascade_t *c, IIR_Type_t type,
                     float fs, float fc, float Q, uint8_t num_stages)
{
    if (c == NULL) return;
    if (num_stages < 1u) num_stages = 1u;
    if (num_stages > IIR_MAX_STAGES) num_stages = IIR_MAX_STAGES;

    c->num_stages = num_stages;
    for (uint8_t i = 0; i < num_stages; i++) {
        c->stage[i].b0 = 1.0f;
        c->stage[i].b1 = 0.0f;
        c->stage[i].b2 = 0.0f;
        c->stage[i].a1 = 0.0f;
        c->stage[i].a2 = 0.0f;
        IIR_Reset(&c->stage[i]);
        if (!isfinite(fs) || !isfinite(fc) || !isfinite(Q) || fs <= 0.0f) continue;
        if (type == IIR_PEAK) {
            IIR_DesignPeak(&c->stage[i], fs, fc, Q, 0.0f);
        } else {
            IIR_Design(&c->stage[i], type, fs, fc, Q);
        }
    }
}

/* 对一个样本执行级联滤波。 */
float IIR_CascadeProcess(IIR_Cascade_t *c, float x)
{
    float y = x;
    if (c == NULL) return x;
    uint8_t stages = (c->num_stages <= IIR_MAX_STAGES) ? c->num_stages : IIR_MAX_STAGES;
    for (uint8_t i = 0; i < stages; i++) {
        y = IIR_Process(&c->stage[i], y);
    }
    return y;
}

/* 对一块数据执行级联滤波。 */
void IIR_CascadeBlock(IIR_Cascade_t *c, const float *in, float *out, uint32_t len)
{
    if (c == NULL || in == NULL || out == NULL) return;
    for (uint32_t i = 0; i < len; i++) {
        out[i] = IIR_CascadeProcess(c, in[i]);
    }
}

/* 清零级联滤波器全部状态。 */
void IIR_CascadeReset(IIR_Cascade_t *c)
{
    if (c == NULL) return;
    uint8_t stages = (c->num_stages <= IIR_MAX_STAGES) ? c->num_stages : IIR_MAX_STAGES;
    for (uint8_t i = 0; i < stages; i++) {
        IIR_Reset(&c->stage[i]);
    }
}


/* ==================================================================== */
/*  第三部分：非线性 / 多速率工具                                        */
/* ==================================================================== */

/* 对输入数据执行中心滑动平均。 */
void Filter_MovingAverage(const float *in, float *out, uint32_t len, uint32_t window)
{
    if (in == NULL || out == NULL || len == 0u) return;
    if (window < 1u) window = 1u;
    if (window > len) window = len;

    /* 以每个点为中心的对称窗；用滑动累加保持 O(len) */
    const int32_t left = (int32_t)((window - 1U) / 2U);
    const int32_t right = (int32_t)(window / 2U);
    float sum = 0.0f;
    int32_t cnt = 0;

    /* 初始化第 0 点窗口 [0, right]。偶数窗向右多取一个样本，
     * 从而保证内部窗口恰好等于调用者请求的 window。 */
    for (int32_t j = 0; j <= right && j < (int32_t)len; j++) {
        sum += in[j];
        cnt++;
    }

    for (int32_t i = 0; i < (int32_t)len; i++) {
        out[i] = sum / (float)cnt;

        int32_t add = i + right + 1;    /* 即将进入窗口右侧的新点 */
        int32_t rem = i - left;         /* 即将移出窗口左侧的旧点 */
        if (add < (int32_t)len) { sum += in[add]; cnt++; }
        if (rem >= 0)           { sum -= in[rem]; cnt--; }
    }
}

/* 内部：对长度 m 的小数组排序后取中值（插入排序，m 很小时最快） */
static float Filter_MedianOfBuf(float *buf, uint32_t m)
{
    for (uint32_t i = 1; i < m; i++) {
        float key = buf[i];
        int32_t j = (int32_t)i - 1;
        while (j >= 0 && buf[j] > key) {
            buf[j + 1] = buf[j];
            j--;
        }
        buf[j + 1] = key;
    }
    return buf[m / 2u];
}

/* 对输入数据执行滑动中值滤波。 */
void Filter_Median(const float *in, float *out, uint32_t len, uint32_t window)
{
    if (in == NULL || out == NULL || len == 0u) return;
    if (window < 1u)  window = 1u;
    if ((window & 1u) == 0u) window -= 1u;   /* 偶数化奇 */
    if (window < 1u)  window = 1u;
    if (window > 31u) window = 31u;          /* 限定小窗，保证局部缓冲足够 */

    float buf[31];
    const int32_t half = (int32_t)(window / 2u);

    for (int32_t i = 0; i < (int32_t)len; i++) {
        for (int32_t k = 0; k < (int32_t)window; k++) {
            int32_t idx = i - half + k;
            /* 边界镜像扩展，避免引入 0 造成边沿失真 */
            if (idx < 0)               idx = -idx;
            if (idx >= (int32_t)len)   idx = 2 * (int32_t)len - idx - 2;
            if (idx < 0)               idx = 0;
            if (idx >= (int32_t)len)   idx = (int32_t)len - 1;
            buf[k] = in[idx];
        }
        out[i] = Filter_MedianOfBuf(buf, window);
    }
}

/* 低通抗混叠后按指定因子抽取。 */
uint32_t Filter_Decimate(const float *in, uint32_t len, uint32_t factor, float *out)
{
    if (in == NULL || out == NULL || len == 0u) return 0u;
    if (factor < 2u) {
        for (uint32_t i = 0; i < len; i++) out[i] = in[i];
        return len;
    }

    /* 防混叠：2 级巴特沃斯低通，截止 = 0.8 * (fs/factor)/2，归一频率 = 0.8*0.5/factor */
    IIR_Cascade_t aa;
    float fc_norm = 0.8f * 0.5f / (float)factor;   /* 相对 fs 的归一截止 */
    IIR_CascadeInit(&aa, IIR_LPF, 1.0f, fc_norm, 0.707f, 2u);

    uint32_t out_n = 0u;
    const uint32_t output_count = len / factor;
    for (uint32_t i = 0; i < len; i++) {
        float y = IIR_CascadeProcess(&aa, in[i]);  /* 每点都滤，保持滤波器状态连续 */
        if (((i % factor) == 0u) && (out_n < output_count)) {
            out[out_n++] = y;
        }
    }
    return out_n;
}
