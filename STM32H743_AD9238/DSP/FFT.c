#include "FFT.h"
#include <stdio.h>
#define pi 3.1415926f /* 圆周率。 */
float FFT_Fs=20000.0f; /* 当前采样率，单位Hz。 */
float f=1000.0f; /* 内部测试正弦波频率，单位Hz。 */
float FFT_input[FFT_length]={0}; /* FFT实数输入缓存。 */
float FFT_Buffer[FFT_length*2]={0}; /* FFT交错复数工作缓存。 */
float FFT_normalized_output[FFT_length / 2 + 1] = {0}; /* 单边归一化幅度谱。 */
float FFT_output[FFT_length]={0}; /* FFT幅度谱缓存。 */
float FFT_mag_max=0; /* 当前最大频谱幅值。 */
uint32_t mag_max_index=0; /* 当前最大频谱索引。 */
volatile float FFT_Amplitude=0; /* 最近一次主峰幅度。 */
volatile float FFT_Frequency=0; /* 最近一次主峰频率。 */

/*
 * G题分析每帧都会使用同一个4096点Hann窗。缓存最近一次使用的窗，
 * 避免每个通道、每一帧都重复调用数千次cosf。窗系数仍由原公式生成，
 * 不改变FFT幅值标定和计算结果。
 */
static float s_hann_window[FFT_length];
static float s_hann_window_sum;
static uint32_t s_hann_window_length;


/* 复制固定长度实数输入数据。 */
void FFT_Input(float arr[]) {
	if (arr == NULL) return;
	for(int i=0;i<FFT_length;i++){
		FFT_input[i]=arr[i];
	}
}

/* 生成内部测试正弦波。 */
void Sin_Simulation(){
	for(int i=0;i<FFT_length;i++){
		FFT_input[i] = sinf(2.0f*pi*f*(float)i/FFT_Fs);
	}
};


//void FFT_Start() {
//	for (int i = 0; i < FFT_length; i++) {
//        FFT_Buffer[2*i] = FFT_input[i];   // 实部
//        FFT_Buffer[2*i+1] = 0.0f;         // 虚部
//    }
//	
//    arm_cfft_f32(&arm_cfft_sR_f32_len4096, FFT_Buffer, 0, 1);
//    
//    arm_cmplx_mag_f32(FFT_Buffer, FFT_output, FFT_length);

//}

/* 根据点数选择CMSIS复数FFT实例。 */
static const arm_cfft_instance_f32 *FFT_SelectCfft(uint32_t len)
{
    switch (len) {
    case 64U:   return &arm_cfft_sR_f32_len64;
    case 128U:  return &arm_cfft_sR_f32_len128;
    case 256U:  return &arm_cfft_sR_f32_len256;
    case 512U:  return &arm_cfft_sR_f32_len512;
    case 1024U: return &arm_cfft_sR_f32_len1024;
    case 2048U: return &arm_cfft_sR_f32_len2048;
    case 4096U: return &arm_cfft_sR_f32_len4096;
    default:    return NULL;
    }
}

/* 对支持的二次幂长度执行汉宁窗复数FFT。 */
bool FFT_StartN(const float *data, uint32_t len)
{
    const arm_cfft_instance_f32 *instance = FFT_SelectCfft(len);

    if ((data == NULL) || (instance == NULL) || (len > FFT_length) || (len < 2U)) {
        return false;
    }

    if (s_hann_window_length != len) {
        float win_sum = 0.0f;
        for (uint32_t i = 0U; i < len; i++) {
            const float win = 0.5f - 0.5f *
                cosf(2.0f * pi * (float)i / (float)(len - 1U));
            s_hann_window[i] = win;
            win_sum += win;
        }
        if (!(win_sum > 0.0f)) return false;
        s_hann_window_sum = win_sum;
        s_hann_window_length = len;
    }

    for (uint32_t i = 0U; i < len; i++) {
        FFT_Buffer[2U * i] = data[i] * s_hann_window[i];
        FFT_Buffer[2U * i + 1U] = 0.0f;
    }

    arm_cfft_f32(instance, FFT_Buffer, 0, 1);
    arm_cmplx_mag_f32(FFT_Buffer, FFT_output, len);

    for (uint32_t i = 0U; i < len; i++) {
        FFT_output[i] = FFT_output[i] *
                        (float)len / s_hann_window_sum;
    }
    return true;
}

/* 对内部输入缓存执行固定长度FFT。 */
void FFT_Start() {
    (void)FFT_StartN(FFT_input, FFT_length);
}

/* 对指定输入执行固定长度FFT。 */
void FFT_StartEx(const float *data) {
    (void)FFT_StartN(data, FFT_length);
}

/* 从固定长度频谱提取主峰并打印结果。 */
void Process_FFT_mag(){
	FFT_Frequency=0;
	FFT_Amplitude=0;
	
	arm_max_f32(FFT_output,FFT_length/2,&FFT_mag_max,&mag_max_index);
	
	/* 排除直流分量后重新寻找主峰。 */
	if(mag_max_index==0){
		FFT_output[0]=0;
		arm_max_f32(FFT_output,FFT_length/2,&FFT_mag_max,&mag_max_index);
	}
	
	FFT_Frequency=(float)mag_max_index*FFT_Fs/(float)FFT_length;

	FFT_Amplitude=FFT_mag_max*2.0f/(float)FFT_length;
	printf("001:%.3f\r\n",FFT_Frequency);
	printf("002:%.3f\r\n",FFT_mag_max);
	printf("003:%.3f\r\n",FFT_Amplitude);
}

/* 按指定采样率从频谱提取主峰。 */
void Process_FFT_magEx(float fs){
    FFT_Frequency=0;
    FFT_Amplitude=0;

    /* 跳过 DC 附近的 bin 0~3，避免汉宁窗的 DC 泄漏被误判为信号 */
    FFT_output[0] = 0;
    FFT_output[1] = 0;
    FFT_output[2] = 0;
    FFT_output[3] = 0;

    arm_max_f32(FFT_output,FFT_length/2,&FFT_mag_max,&mag_max_index);

    FFT_Frequency=(float)mag_max_index*fs/(float)FFT_length;

    FFT_Amplitude=FFT_mag_max*2.0f/(float)FFT_length;
}


/* 按实际 FFT 点数将当前幅度谱归一化为单边谱。 */
bool Normalize_FFT_To_Single_SideN(uint32_t len)
{
    if (FFT_SelectCfft(len) == NULL || len > FFT_length) {
        FFT_normalized_output[0] = 0.0f;
        return false;
    }
    float scale = 1.0f / (float)len;
    uint32_t single_side_length = len / 2u + 1u;

    for (uint32_t i = 0u; i < single_side_length; i++)
    {
        if (i == 0u)
        {
            FFT_normalized_output[i] = FFT_output[i] * scale;
        }
        else if (i == len / 2u)
        {
            FFT_normalized_output[i] = FFT_output[i] * scale;
        }
        else
        {
            FFT_normalized_output[i] = FFT_output[i] * 2.0f * scale;
        }
    }
    return true;
}

/* 兼容旧工程的固定 4096 点归一化入口。 */
void Normalize_FFT_To_Single_Side(void)
{
    (void)Normalize_FFT_To_Single_SideN(FFT_length);
}

/* 使用Goertzel算法计算指定频率幅度。 */
float FFT_AmpAtFreq(float data[], uint32_t len, float fs, float freq)
{
    if ((data == NULL) || (len < 2U) || !(fs > 0.0f) ||
        !(freq >= 0.0f) || (freq > 0.5f * fs)) {
        return 0.0f;
    }
    float w = 2.0f * pi * freq / fs;
    float coeff = 2.0f * cosf(w);

    float q0 = 0.0f, q1 = 0.0f, q2 = 0.0f;
    float win_sum = 0.0f;

    for (uint32_t i = 0; i < len; i++)
    {
        float win = 0.5f - 0.5f * cosf(2.0f * pi * (float)i / (float)(len - 1));

        win_sum += win;

        q0 = data[i] * win + coeff * q1 - q2;
        q2 = q1;
        q1 = q0;
    }

    float power = q1 * q1 + q2 * q2 - coeff * q1 * q2;

    if (power < 0.0f)
    {
        power = 0.0f;
    }

    return 2.0f * sqrtf(power) / win_sum;
}

/* 从时域数据提取整数倍谐波。 */
uint8_t FFT_GetHarmonics(float data[], uint32_t len, float fs, float f0,
                         float max_freq, float freq_out[], float amp_out[])
{
    uint8_t count = 0;

    if ((data == NULL) || (freq_out == NULL) || (amp_out == NULL) ||
        (len < 2U) || !(fs > 0.0f) || (f0 < 1.0f) || !(max_freq > 0.0f)) {
        return 0;
    }
    if (max_freq > 0.5f * fs) max_freq = 0.5f * fs;

    for (int h = 1; h * f0 <= max_freq && count < FFT_MAX_HARMONICS; h++)
    {
        freq_out[count] = h * f0;
        amp_out[count] = FFT_AmpAtFreq(data, len, fs, h * f0);
        count++;
    }

    return count;
}

/* 从当前可变点数 FFT 的单边归一化频谱提取整数倍谐波。 */
uint8_t FFT_GetHarmonics_FromFFTN(uint32_t len, float fs, float f0,
                                  float max_freq, float freq_out[],
                                  float amp_out[])
{
    uint8_t count = 0;
    float df = fs / (float)len;

    if ((freq_out == NULL) || (amp_out == NULL) || !(fs > 0.0f) ||
        (FFT_SelectCfft(len) == NULL) || (len > FFT_length) ||
        (f0 < 1.0f) || !(max_freq > 0.0f)) {
        return 0;
    }
    if (max_freq > 0.5f * fs) max_freq = 0.5f * fs;

    for (int h = 1; h * f0 <= max_freq && count < FFT_MAX_HARMONICS; h++)
    {
        float freq = h * f0;
        uint32_t k = (uint32_t)(freq / df + 0.5f);

        if (k == 0u || k >= len / 2u) break;

        uint32_t k_best = k;
        float mag = FFT_normalized_output[k];

        if (k > 1u && FFT_normalized_output[k - 1u] > mag)
        {
            mag = FFT_normalized_output[k - 1u];
            k_best = k - 1u;
        }

        if (k + 1u < len / 2u && FFT_normalized_output[k + 1u] > mag)
        {
            mag = FFT_normalized_output[k + 1u];
            k_best = k + 1u;
        }

        freq_out[count] = (float)k_best * df;
        amp_out[count] = mag;

        count++;
    }

    return count;
}

/* 兼容旧工程的固定 4096 点谐波提取入口。 */
uint8_t FFT_GetHarmonics_FromFFT(float fs, float f0, float max_freq,
                                 float freq_out[], float amp_out[])
{
    return FFT_GetHarmonics_FromFFTN(FFT_length,
                                     fs,
                                     f0,
                                     max_freq,
                                     freq_out,
                                     amp_out);
}

/* 打印当前FFT原始结果。 */
void FFT_Data_Print() {
	printf("\r\nRaw Results\r\n");
    for (int i = 0; i < FFT_length; i++) {
        printf("%.3f,", FFT_output[i]);
    }
	printf("\r\nFreq:%.3f\r\n",FFT_Frequency);
	printf("mag_max:%.3f\r\n",FFT_mag_max);
	printf("Amp:%.3f\r\n",FFT_Amplitude);

}

/* ==================================================================== */
/*  窗函数库实现                                                         */
/* ==================================================================== */
/* 生成指定窗系数及其相干增益和等效噪声带宽。 */
void Window_Generate(float *w, uint32_t n, Window_Type_t type,
                     float *cg, float *enbw)
{
    if (w == NULL || n == 0u) return;

    const float Nm1 = (n > 1u) ? (float)(n - 1u) : 1.0f;  /* 对称窗分母 */

    for (uint32_t i = 0; i < n; i++) {
        float a = 2.0f * pi * (float)i / Nm1;   /* 2*pi*i/(N-1) */
        float val;
        switch (type) {
        case WIN_HANN:
            val = 0.5f - 0.5f * cosf(a);
            break;
        case WIN_HAMMING:
            val = 0.54f - 0.46f * cosf(a);
            break;
        case WIN_BLACKMAN:
            val = 0.42f - 0.5f * cosf(a) + 0.08f * cosf(2.0f * a);
            break;
        case WIN_BLACKMAN_HARRIS:               /* 旁瓣 -92dB */
            val = 0.35875f - 0.48829f * cosf(a)
                + 0.14128f * cosf(2.0f * a) - 0.01168f * cosf(3.0f * a);
            break;
        case WIN_FLATTOP:                        /* Matlab flattopwin */
            val = 0.21557895f - 0.41663158f * cosf(a)
                + 0.277263158f * cosf(2.0f * a) - 0.083578947f * cosf(3.0f * a)
                + 0.006947368f * cosf(4.0f * a);
            break;
        case WIN_RECT:
        default:
            val = 1.0f;
            break;
        }
        w[i] = val;
    }

    if (cg != NULL || enbw != NULL) {
        float s1 = 0.0f, s2 = 0.0f;
        for (uint32_t i = 0; i < n; i++) {
            s1 += w[i];
            s2 += w[i] * w[i];
        }
        if (cg)   *cg   = s1 / (float)n;
        if (enbw) *enbw = (s1 > 0.0f) ? ((float)n * s2 / (s1 * s1)) : 1.0f;
    }
}

/* 将窗系数逐点乘到输入信号。 */
void Window_Apply(float *x, const float *w, uint32_t n)
{
    if (x == NULL || w == NULL) return;
    for (uint32_t i = 0; i < n; i++) {
        x[i] *= w[i];
    }
}
