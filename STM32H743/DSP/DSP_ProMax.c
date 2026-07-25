#include "DSP_ProMax.h"
#include <stdio.h>
#include <string.h>

#define DSP_PROMAX_QUALITY_HARMONICS  8U /* 质量分析默认统计到第8次谐波。 */
#define DSP_PROMAX_HANN_LEAK_BINS     2U /* Hann窗主瓣两侧计入的频点数。 */

static DSP_ProMaxResultTypeDef s_promax_result; /* 最近一次完整分析结果。 */

/* 填充一站式分析默认配置。 */
void DSP_ProMax_ConfigDefault(DSP_ProMaxConfigTypeDef *config)
{
  if (config == NULL)
  {
    return;
  }
  Measure_THDConfigDefault(&config->thd);
  config->quality_harmonics = DSP_PROMAX_QUALITY_HARMONICS;
  config->quality_leak_bins = DSP_PROMAX_HANN_LEAK_BINS;
}

/* 一次完成常用时域测量、可变点数FFT、谐波和信号质量分析。 */
DSP_ProMaxStatusTypeDef DSP_ProMax_AnalyzeEx(
    const float *data,
    uint32_t len,
    float fs_hz,
    const DSP_ProMaxConfigTypeDef *config,
    DSP_ProMaxResultTypeDef *result)
{
  DSP_ProMaxConfigTypeDef cfg;
  uint32_t peak_index;

  if ((data == NULL) || (result == NULL) || (len == 0U) || !(fs_hz > 0.0f))
  {
    return DSP_PROMAX_ERROR_PARAM;
  }

  DSP_ProMax_ConfigDefault(&cfg);
  if (config != NULL)
  {
    cfg = *config;
  }
  if ((cfg.quality_harmonics < 2U) ||
      (cfg.quality_harmonics > MEASURE_THD_MAX_HARMONICS))
  {
    return DSP_PROMAX_ERROR_PARAM;
  }

  memset(result, 0, sizeof(*result));
  Measure_TimeStats(data,
                    len,
                    &result->dc,
                    &result->rms,
                    &result->acrms,
                    &result->vpp);
  Measure_Waveform(data, len, fs_hz, &result->wave);
  result->time_valid = 1U;

  FFT_Fs = fs_hz;
  if (!FFT_StartN(data, len))
  {
    s_promax_result = *result;
    return DSP_PROMAX_ERROR_LENGTH;
  }
  if (!Normalize_FFT_To_Single_SideN(len))
  {
    s_promax_result = *result;
    return DSP_PROMAX_ERROR_FFT;
  }

  peak_index = Measure_FindPeak(FFT_normalized_output,
                                len,
                                fs_hz,
                                &result->peak);
  if ((peak_index != 0U) && (result->peak.amplitude > 0.0f))
  {
    (void)Measure_THDFromSpectrum(FFT_normalized_output,
                                  len,
                                  fs_hz,
                                  &cfg.thd,
                                  &result->thd);
    if (result->thd.harmonics_found != 0U)
    {
      uint32_t fundamental_bin = (uint32_t)(
          result->thd.fund_freq * (float)len / fs_hz + 0.5f);
      Measure_PeakInterp(FFT_normalized_output,
                         len,
                         fundamental_bin,
                         fs_hz,
                         &result->peak);
    }
    result->harmonic_count = result->thd.harmonics_found;
    for (uint32_t i = 0U; i < result->harmonic_count; i++)
    {
      result->harmonic_freq[i] = result->thd.harmonic_freq[i];
      result->harmonic_amp[i] = result->thd.harmonic_amp[i];
    }

    Measure_Quality(FFT_normalized_output,
                    len,
                    fs_hz,
                    cfg.quality_harmonics,
                    cfg.quality_leak_bins,
                    &result->quality);
    if (result->thd.harmonics_found != 0U)
    {
      /* 旧质量结构与新的专用 THD 结果保持同一个口径。 */
      result->quality.thd = result->thd.thd;
      result->quality.thd_db = result->thd.thd_db;
      result->quality.fund_freq = result->thd.fund_freq;
    }
    result->fft_valid = 1U;
  }

  s_promax_result = *result;
  return DSP_PROMAX_OK;
}

/* 使用默认配置的一站式分析兼容入口。 */
DSP_ProMaxStatusTypeDef DSP_ProMax_Analyze(const float *data,
                                           uint32_t len,
                                           float fs_hz,
                                           DSP_ProMaxResultTypeDef *result)
{
  DSP_ProMaxConfigTypeDef config;
  DSP_ProMax_ConfigDefault(&config);
  return DSP_ProMax_AnalyzeEx(data, len, fs_hz, &config, result);
}

/* 获取最近一次分析结果的只读指针。 */
const DSP_ProMaxResultTypeDef *DSP_ProMax_GetLastResult(void)
{
  return &s_promax_result;
}

/* 通过printf输出关键分析结果。 */
void DSP_ProMax_PrintResult(const DSP_ProMaxResultTypeDef *result)
{
  if (result == NULL)
  {
    result = &s_promax_result;
  }

  printf("DSP ProMax\r\n");
  printf("DC=%.4f RMS=%.4f AC_RMS=%.4f VPP=%.4f\r\n",
         result->dc,
         result->rms,
         result->acrms,
         result->vpp);
  printf("Wave freq=%.3fHz duty=%.3f rise=%.6fs fall=%.6fs\r\n",
         result->wave.freq,
         result->wave.duty,
         result->wave.rise_time,
         result->wave.fall_time);

  if (result->fft_valid != 0U)
  {
    printf("FFT peak=%.3fHz amp=%.4f bin=%.3f\r\n",
           result->peak.freq,
           result->peak.amplitude,
           result->peak.bin);
    printf("THD=%.3f%% THD_dB=%.2f SNR=%.2f SINAD=%.2f ENOB=%.2f status=%u\r\n",
           result->thd.thd_percent,
           result->thd.thd_db,
           result->quality.snr_db,
           result->quality.sinad_db,
           result->quality.enob,
           (unsigned int)result->thd.status);
    for (uint32_t i = 0U; i < result->thd.harmonics_found; i++)
    {
      printf("H%u freq=%.3fHz amp=%.6f norm=%.6f\r\n",
             (unsigned int)(i + 1U),
             result->thd.harmonic_freq[i],
             result->thd.harmonic_amp[i],
             result->thd.harmonic_ratio[i]);
    }
  }
}
