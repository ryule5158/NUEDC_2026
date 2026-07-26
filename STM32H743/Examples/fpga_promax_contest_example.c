#include "fpga_promax.h"
#include "fpga_promax_link.h"

/*
 * 这个文件是调用示例，不会自动加入Keil工程。
 * 复制所需函数到题目APP层，并在主循环中调用；不要在中断里操作SPI。
 */

#define EXAMPLE_ADC_SAMPLE_RATE_HZ 32000000U

static FpgaPromax s_promax;

/*
 * 8点滑动平均低通，每个系数为1/8。
 * Q1.16中：1.0=65536，1/8=8192。
 */
static const int32_t s_lowpass_q16[8] = {
  8192, 8192, 8192, 8192, 8192, 8192, 8192, 8192
};

/* 一阶差分高通：y[n]=0.5*x[n]-0.5*x[n-1]。 */
static const int32_t s_edge_q16[2] = {
  32768, -32768
};

/* 示例8点帧头模板；相关接口会自动反序后装入FIR。 */
static const int32_t s_preamble_q16[8] = {
  65536, 65536, -65536, 65536,
  -65536, -65536, 65536, -65536
};

/*
 * 一次完成硬件配置并启动连续数据面。
 * bank0、bank1和bank2会在每个ADC样点到来时同时运算。
 */
FpgaPromax_Result ContestPromax_Start(void)
{
  FpgaPromax_Result result;

  result = FPGA_ProMax_Init(&s_promax);
  if (result != FPGA_PROMAX_OK)
  {
    return result;
  }
  if ((FpgaPromax_HasRealtime(&s_promax) == 0U) ||
      (FpgaPromax_HasWaveStats(&s_promax) == 0U))
  {
    return FPGA_PROMAX_E_UNSUPPORTED;
  }

  result = FpgaPromax_SetRun(&s_promax, 0U);
  if (result != FPGA_PROMAX_OK)
  {
    return result;
  }
  result = FpgaPromax_LoadFirBank(&s_promax,
                                  0U,
                                  s_lowpass_q16,
                                  sizeof(s_lowpass_q16) /
                                  sizeof(s_lowpass_q16[0]));
  if (result != FPGA_PROMAX_OK)
  {
    return result;
  }
  result = FpgaPromax_LoadFirBank(&s_promax,
                                  1U,
                                  s_edge_q16,
                                  sizeof(s_edge_q16) /
                                  sizeof(s_edge_q16[0]));
  if (result != FPGA_PROMAX_OK)
  {
    return result;
  }
  result = FpgaPromax_LoadCorrelationTemplate(
    &s_promax,
    2U,
    s_preamble_q16,
    sizeof(s_preamble_q16) / sizeof(s_preamble_q16[0]));
  if (result != FPGA_PROMAX_OK)
  {
    return result;
  }

  /* 4096点统计块：32 MSPS下每约128 us更新一组完整结果。 */
  result = FpgaPromax_SetWaveBlockLog2(&s_promax, 12U);
  if (result != FPGA_PROMAX_OK)
  {
    return result;
  }
  result = FpgaPromax_ClearState(&s_promax);
  if (result != FPGA_PROMAX_OK)
  {
    return result;
  }
  return FpgaPromax_SetRun(&s_promax, 1U);
}

/*
 * 主循环低速调用。zero_code_voltage和volts_per_code来自实测标定：
 * voltage = zero_code_voltage + signed_adc_code * volts_per_code。
 */
FpgaPromax_Result ContestPromax_ReadMeasurements(
  float zero_code_voltage,
  float volts_per_code,
  FpgaPromax_WaveMetrics *metrics,
  int32_t fir_peak[FPGA_PROMAX_MF_BANKS],
  uint32_t peak_index[FPGA_PROMAX_MF_BANKS])
{
  FpgaPromax_Result result;
  FpgaPromax_WaveStats stats;
  uint8_t bank;

  if ((metrics == NULL) || (fir_peak == NULL) || (peak_index == NULL))
  {
    return FPGA_PROMAX_E_ARGUMENT;
  }

  result = FpgaPromax_ReadWaveStats(&s_promax, &stats, 100U);
  if (result != FPGA_PROMAX_OK)
  {
    return result;
  }
  result = FpgaPromax_ComputeWaveMetrics(&stats,
                                         EXAMPLE_ADC_SAMPLE_RATE_HZ,
                                         zero_code_voltage,
                                         volts_per_code,
                                         metrics);
  if (result != FPGA_PROMAX_OK)
  {
    return result;
  }

  for (bank = 0U; bank < FPGA_PROMAX_MF_BANKS; bank++)
  {
    result = FpgaPromax_GetFirPeak(&s_promax,
                                   bank,
                                   &fir_peak[bank],
                                   &peak_index[bank]);
    if (result != FPGA_PROMAX_OK)
    {
      return result;
    }
  }
  return FPGA_PROMAX_OK;
}

/* 8路DDC并行监测例子：35/37/39 MHz会自动按32 MSPS折叠。 */
FpgaPromax_Result ContestPromax_SetRfToneBank(void)
{
  static const uint32_t tones_hz[3] = {
    35000000U, 37000000U, 39000000U
  };

  return FpgaPromax_ConfigureToneBank(&s_promax,
                                     tones_hz,
                                     3U,
                                     EXAMPLE_ADC_SAMPLE_RATE_HZ);
}
