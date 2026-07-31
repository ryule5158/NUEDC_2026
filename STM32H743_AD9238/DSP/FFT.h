//
// FFT频谱分析公共接口。
//

#ifndef FFT_H
#define FFT_H /* FFT分析头文件包含保护。 */
#include <stdio.h>
#include "main.h"
#include "arm_math.h"
#include "arm_const_structs.h"
#include <stdint.h>
#include <stdbool.h>
#define FFT_length 4096      /* FFT固定采样长度。 */
#define FFT_MAX_HARMONICS 24 /* FFT最多统计的谐波数量。 */
extern float FFT_Fs; /* 当前采样率，单位Hz。 */
extern float FFT_input[FFT_length]; /* FFT实数输入缓存。 */
extern float FFT_Buffer[FFT_length*2]; /* FFT交错复数工作缓存。 */
extern float FFT_output[FFT_length]; /* FFT幅度谱缓存。 */
extern float FFT_normalized_output[FFT_length / 2 + 1]; /* 单边归一化幅度谱。 */
extern float FFT_mag_max; /* 当前最大频谱幅值。 */
extern uint32_t mag_max_index; /* 当前最大频谱索引。 */
extern volatile float FFT_Amplitude; /* 最近一次主峰幅度。 */
extern volatile float FFT_Frequency; /* 最近一次主峰频率。 */

/* 复制固定长度实数输入数据。 */
void FFT_Input(float arr[]);
/* 生成内部测试正弦波。 */
void Sin_Simulation();
/* 对内部输入缓存执行固定长度FFT。 */
void FFT_Start();
/* 对指定输入执行固定长度FFT。 */
void FFT_StartEx(const float *data);
/**
 * @brief 对支持的二次幂长度执行汉宁窗复数FFT。
 * @return 输入有效且长度受支持时返回true，否则返回false。
 */
bool FFT_StartN(const float *data, uint32_t len);
/* 从固定长度频谱提取主峰并打印结果。 */
void Process_FFT_mag(void);
/* 按指定采样率从频谱提取主峰。 */
void Process_FFT_magEx(float fs);
/* 将当前FFT幅度谱归一化为单边谱。 */
void Normalize_FFT_To_Single_Side();
/**
 * @brief 按实际 FFT 点数归一化当前频谱，支持 FFT_StartN() 的 64~4096 点结果。
 * @return len 合法时返回 true，否则清空首项并返回 false。
 */
bool Normalize_FFT_To_Single_SideN(uint32_t len);
/* 打印当前FFT原始结果。 */
void FFT_Data_Print();
/* 使用Goertzel算法计算指定频率幅度。 */
float FFT_AmpAtFreq(float data[], uint32_t len, float fs, float freq);
/* 从时域数据提取整数倍谐波。 */
uint8_t FFT_GetHarmonics(float data[], uint32_t len, float fs, float f0,
float max_freq, float freq_out[], float amp_out[]);
/* 从当前FFT频谱提取整数倍谐波。 */
uint8_t FFT_GetHarmonics_FromFFT(float fs, float f0, float max_freq,
                                 float freq_out[], float amp_out[]);
/**
 * @brief 从当前可变点数 FFT 的单边归一化频谱提取整数倍谐波。
 * @note  必须先依次调用 FFT_StartN() 和 Normalize_FFT_To_Single_SideN()。
 */
uint8_t FFT_GetHarmonics_FromFFTN(uint32_t len, float fs, float f0,
                                  float max_freq, float freq_out[],
                                  float amp_out[]);

/* ==================================================================== */
/*  窗函数库 — FFT 前加窗，抑制频谱泄漏                                   */
/*  选窗建议：测频→Hann；测幅度→Flattop(平顶,误差<0.01dB)；             */
/*            测谐波/THD→Blackman-Harris(旁瓣-92dB)；整周期采样→Rect    */
/* ==================================================================== */
/* 支持的窗函数类型。 */
typedef enum {
    WIN_RECT = 0,         /* 矩形窗(不加窗)        */
    WIN_HANN,             /* 汉宁窗                */
    WIN_HAMMING,          /* 汉明窗                */
    WIN_BLACKMAN,         /* 布莱克曼窗            */
    WIN_BLACKMAN_HARRIS,  /* 4 项布莱克曼-哈里斯   */
    WIN_FLATTOP           /* 5 项平顶窗(幅度测量)  */
} Window_Type_t;

/**
 * @brief  生成窗系数并给出修正因子。
 * @param  w     [out] 长度 n 的窗系数
 * @param  n     窗长(=FFT 点数)
 * @param  type  窗类型
 * @param  cg    [out,可空] 相干增益 = (1/n)Σw，幅度修正用：真幅度=2|X|/(n·cg)
 * @param  enbw  [out,可空] 等效噪声带宽(bin) = n·Σw²/(Σw)²，噪声/功率修正用
 */
void Window_Generate(float *w, uint32_t n, Window_Type_t type,
                     float *cg, float *enbw);

/** @brief 把窗就地乘到信号上：x[i] *= w[i]。 */
void Window_Apply(float *x, const float *w, uint32_t n);

#endif
