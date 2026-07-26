/**
 * @file    Filter.c
 * @brief   数字滤波模块实现 —— FIR(固定系数) + IIR(RBJ 双二阶)
 */
#include "Filter.h"
#include <math.h>
#include <stddef.h>

/* ==================================================================== */
/*  第一部分：FIR 低通滤波                                                */
/* ==================================================================== */
const int LP1 = 101;
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
float fifo_data1_f[FIFO_SIZE] = {0};
float testOutput[FIFO_SIZE] = {0};
float firStateF32[BLOCK_SIZE + 101 - 1] = {0};
/**
  * @brief  对输入数据数组进行FIR低通滤波（101阶）。
  * @param  input  输入采样数组，长度 = num_samples
  * @param  output 滤波后输出数组，长度 = num_samples
  * @param  num_samples 要处理的采样点数，可包含不足BLOCK_SIZE的末块
  * @note   每次调用前会自动清零滤波器状态，保证独立调用之间无状态残留。
  *         如果需要连续流式滤波，请直接使用arm_fir_f32并自行管理状态。
  */
void arm_fir_f32_lp(float *input, float *output, uint32_t num_samples)
{
    uint32_t i;
    arm_fir_instance_f32 S;

    /* 清零滤波器状态，确保每次调用独立 */
    for (i = 0; i < (BLOCK_SIZE + LP1 - 1); i++)
    {
        firStateF32[i] = 0.0f;
    }

    arm_fir_init_f32(&S, LP1, (float *)LP1_Resource, firStateF32, BLOCK_SIZE);

    /* 按BLOCK_SIZE分块处理，末块不足时只处理实际样本。 */
    for (i = 0; i < num_samples; i += BLOCK_SIZE)
    {
        uint32_t block_length = num_samples - i;

        if (block_length > BLOCK_SIZE) {
            block_length = BLOCK_SIZE;
        }
        arm_fir_f32(&S, input + i, output + i, block_length);
    }
}


/* ==================================================================== */
/*  第二部分：IIR 双二阶滤波（RBJ Cookbook 设计 + DF2T 运行）             */
/* ==================================================================== */

#define IIR_2PI   6.28318530717958647692f

/* 将双二阶设置为直通，保证无效配置也具有确定输出。 */
static void IIR_SetBypass(IIR_Biquad_t *bq)
{
    bq->b0 = 1.0f;
    bq->b1 = 0.0f;
    bq->b2 = 0.0f;
    bq->a1 = 0.0f;
    bq->a2 = 0.0f;
    IIR_Reset(bq);
}

void IIR_Reset(IIR_Biquad_t *bq)
{
    if (bq == NULL) return;
    bq->z1 = 0.0f;
    bq->z2 = 0.0f;
}

void IIR_Design(IIR_Biquad_t *bq, IIR_Type_t type, float fs, float fc, float Q)
{
    if (bq == NULL) return;
    if (fs <= 0.0f) {
        IIR_SetBypass(bq);
        return;
    }

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

void IIR_DesignPeak(IIR_Biquad_t *bq, float fs, float fc, float Q, float gain_db)
{
    if (bq == NULL) return;
    if (fs <= 0.0f) {
        IIR_SetBypass(bq);
        return;
    }

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

void IIR_ProcessBlock(IIR_Biquad_t *bq, const float *in, float *out, uint32_t len)
{
    if (bq == NULL || in == NULL || out == NULL) return;
    for (uint32_t i = 0; i < len; i++) {
        out[i] = IIR_Process(bq, in[i]);
    }
}

/* ---- 级联 ---- */

void IIR_CascadeInit(IIR_Cascade_t *c, IIR_Type_t type,
                     float fs, float fc, float Q, uint8_t num_stages)
{
    if (c == NULL) return;
    if (num_stages < 1u) num_stages = 1u;
    if (num_stages > IIR_MAX_STAGES) num_stages = IIR_MAX_STAGES;

    c->num_stages = num_stages;
    for (uint8_t i = 0; i < num_stages; i++) {
        IIR_Design(&c->stage[i], type, fs, fc, Q);
    }
}

void IIR_CascadePeakInit(IIR_Cascade_t *c, float fs, float fc, float Q,
                         float gain_db, uint8_t num_stages)
{
    if (c == NULL) return;
    if (num_stages < 1u) num_stages = 1u;
    if (num_stages > IIR_MAX_STAGES) num_stages = IIR_MAX_STAGES;

    c->num_stages = num_stages;
    for (uint8_t i = 0; i < num_stages; i++) {
        IIR_DesignPeak(&c->stage[i], fs, fc, Q, gain_db);
    }
}

float IIR_CascadeProcess(IIR_Cascade_t *c, float x)
{
    if (c == NULL) return x;
    float y = x;
    for (uint8_t i = 0; i < c->num_stages; i++) {
        y = IIR_Process(&c->stage[i], y);
    }
    return y;
}

void IIR_CascadeBlock(IIR_Cascade_t *c, const float *in, float *out, uint32_t len)
{
    if (c == NULL || in == NULL || out == NULL) return;
    for (uint32_t i = 0; i < len; i++) {
        out[i] = IIR_CascadeProcess(c, in[i]);
    }
}

void IIR_CascadeReset(IIR_Cascade_t *c)
{
    if (c == NULL) return;
    for (uint8_t i = 0; i < c->num_stages; i++) {
        IIR_Reset(&c->stage[i]);
    }
}


/* ==================================================================== */
/*  第三部分：非线性 / 多速率工具                                        */
/* ==================================================================== */

void Filter_MovingAverage(const float *in, float *out, uint32_t len, uint32_t window)
{
    if (in == NULL || out == NULL || len == 0u) return;
    if (window < 1u) window = 1u;
    if (window > len) window = len;

    /* 以每个点为中心的对称窗；用滑动累加保持 O(len) */
    const int32_t half = (int32_t)(window / 2u);
    float sum = 0.0f;
    int32_t cnt = 0;

    /* 初始化前半窗 [0, half] */
    for (int32_t j = 0; j <= half && j < (int32_t)len; j++) {
        sum += in[j];
        cnt++;
    }

    for (int32_t i = 0; i < (int32_t)len; i++) {
        out[i] = sum / (float)cnt;

        int32_t add = i + half + 1;     /* 即将进入窗口右侧的新点 */
        int32_t rem = i - half;         /* 即将移出窗口左侧的旧点 */
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
    for (uint32_t i = 0; i < len; i++) {
        float y = IIR_CascadeProcess(&aa, in[i]);  /* 每点都滤，保持滤波器状态连续 */
        if ((i % factor) == 0u) {
            out[out_n++] = y;
        }
    }
    return out_n;
}
