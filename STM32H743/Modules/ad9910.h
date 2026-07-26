/**
  ******************************************************************************
  * @file    ad9910.h
  * @brief   基于STM32 HAL GPIO和软件SPI的AD9910 DDS驱动接口。
  *
 * 基本使用流程：
 *   1. 在CubeMX中配置AD9910引脚，并在main.c中调用MX_GPIO_Init()。
 *   2. 调用AD9910_Init(NULL)。
  *   3. 选择一个输出接口：
   *        AD9910_SetRamWaveformCarrier(AD9910_WAVE_TRIANGLE, 0U, 1U, 1024U);
  *        AD9910_SetRamCustomWaveformCarrier(samples, count, 0U, 1U);
  *        AD9910_ConfigureFrequencySweep(..., AD9910_SWEEP_AUTO);
  *
 * 引脚映射保存在Modules/ad9910.c中，GPIO模式由CubeMX生成。
  ******************************************************************************
  */

#ifndef __AD9910_H
#define __AD9910_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include <stdint.h>

#ifndef HAL_SPI_MODULE_ENABLED
/* 未启用HAL硬件SPI时保留兼容的句柄类型声明。 */
typedef void SPI_HandleTypeDef;
#endif

#define AD9910_SYSCLK_HZ          1000000000ULL  /* AD9910芯片系统时钟, 单位Hz; 外部参考或PLL配置改变后必须同步修改 */
#define AD9910_MAX_OUTPUT_HZ      450000000UL    /* AD9910允许设置的最高输出频率, 单位Hz */
#define AD9910_MAX_AMPLITUDE      16383U         /* AD9910最大14位幅度值 */
#define AD9910_RAM_POINTS         1024U          /* AD9910内部RAM最大波形点数, 实际使用点数越多频率上限越低 */

/* AD9910驱动状态。 */
typedef enum
{
  AD9910_OK = 0,    /* 操作成功。 */
  AD9910_ERROR,     /* 通信或器件操作失败。 */
  AD9910_BAD_PARAM  /* 输入参数无效。 */
} AD9910_Status;

/* AD9910内置RAM波形类型。 */
typedef enum
{
  AD9910_WAVE_TRIANGLE = 0, /* 三角波。 */
  AD9910_WAVE_SQUARE,       /* 方波。 */
  AD9910_WAVE_SINC,         /* SINC波。 */
  AD9910_WAVE_SAWTOOTH      /* 锯齿波。 */
} AD9910_Waveform;

/* AD9910数字扫频控制方式。 */
typedef enum
{
  AD9910_SWEEP_MANUAL = 0, /* DRCTL引脚控制方向。 */
  AD9910_SWEEP_AUTO        /* 器件自动往返扫频。 */
} AD9910_SweepMode;

/* AD9910手动扫频方向。 */
typedef enum
{
  AD9910_SWEEP_DOWN = 0, /* 向下扫频。 */
  AD9910_SWEEP_UP        /* 向上扫频。 */
} AD9910_SweepDirection;

/**
  * @brief  复位AD9910并写入默认CFR寄存器。
  * @param  hspi 预留给硬件SPI模式使用；默认软件SPI模式传NULL。
  * @retval AD9910_OK表示成功，AD9910_BAD_PARAM表示参数错误，AD9910_ERROR表示通信失败。
  */
AD9910_Status AD9910_Init(SPI_HandleTypeDef *hspi);

/**
  * @brief  硬件复位AD9910，并将控制引脚设置为空闲状态。
  * @retval AD9910_OK。
  */
AD9910_Status AD9910_Reset(void);

/**
  * @brief  只更新当前单频输出的频率。
  * @param  freq_hz 输出频率，单位Hz，有效范围0~AD9910_MAX_OUTPUT_HZ。
  * @retval AD9910_OK、AD9910_BAD_PARAM或AD9910_ERROR。
  */
AD9910_Status AD9910_SetFrequencyHz(uint32_t freq_hz);

/* 设置AD9910小数Hz频率，用于H题锁频后的细微频率修正 */
AD9910_Status AD9910_SetFrequencyFineHz(float freq_hz);

/* 设置AD9910当前单频输出相位偏移字，0~65535对应0~360度 */
AD9910_Status AD9910_SetPhaseOffsetWord(uint16_t phase_word);

/**
  * @brief  原子更新当前Profile0的频率、相位和幅度。
  *
  * 该接口只写一次8字节Profile0并只产生一次IO_UPDATE，供PLL等高更新率
  * 闭环使用。调用前必须先执行AD9910_Init()，且不能处于RAM或数字扫频模式。
  *
  * @param  freq_hz 输出频率，单位Hz，允许小数。
  * @param  phase_word 16位相位偏移字，0~65535对应0~360度。
  * @param  amplitude 14位幅度控制值，范围0~AD9910_MAX_AMPLITUDE。
  * @retval AD9910_OK、AD9910_BAD_PARAM或AD9910_ERROR。
  */
AD9910_Status AD9910_UpdateSingleToneFast(double freq_hz,
                                           uint16_t phase_word,
                                           uint16_t amplitude);

/**
  * @brief  只更新当前单频输出的幅度。
  * @param  amplitude 14位幅度控制值，超过16383的值会被截取为14位。
  * @retval AD9910_OK或AD9910_ERROR。
  */
AD9910_Status AD9910_SetAmplitude(uint16_t amplitude);

/**
  * @brief  将1024点波形写入AD9910 RAM并开始连续播放。
  *
  * 该底层接口假设DDS载波FTW已经配置好。
  * 常规使用建议调用AD9910_SetRamWaveformCarrier()。
  *
  * @param  wave 内置波形：三角波、方波或SINC波。
  * @param  playback_step RAM播放步进M。AD9910 SYSCLK为1GHz时，M=1约输出244kHz。
  *         如果传入0，内部按1处理。
  * @retval AD9910_OK、AD9910_BAD_PARAM或AD9910_ERROR。
  */
AD9910_Status AD9910_SetRamWaveform(AD9910_Waveform wave, uint16_t playback_step);

/**
  * @brief  按自定义RAM点数写入波形并开始播放。
  *
  * 该底层接口假设DDS载波FTW已经配置好。
  * 常规使用建议调用AD9910_SetRamWaveformCarrier()。
  *
  * @param  wave 内置波形：三角波、方波或SINC波。
  * @param  playback_step RAM播放步进M。如果传入0，内部按1处理。
  * @param  points RAM点数，范围16~AD9910_RAM_POINTS。
  *         点数越小输出频率越高，便于示波器调试。
  * @retval AD9910_OK、AD9910_BAD_PARAM或AD9910_ERROR。
  */
AD9910_Status AD9910_SetRamWaveformPoints(AD9910_Waveform wave,
                                          uint16_t playback_step,
                                          uint16_t points);

/**
  * @brief  输出RAM波形。
  * @param  wave 内置波形：三角波、方波或SINC波。
  * @param  carrier_hz 传0表示直接RAM输出；大于0时将RAM表作为正弦载波包络。
  * @param  playback_step RAM播放步进M。如果传入0，内部按1处理。
  * @param  points RAM点数，范围16~AD9910_RAM_POINTS。
  * @retval AD9910_OK、AD9910_BAD_PARAM或AD9910_ERROR。
  *
  * 可见波形频率近似为：
  * AD9910_SYSCLK_HZ / (4 * playback_step * points)。
  * playback_step只能为整数, 因此RAM波形可能出现频率量化误差。
  */
AD9910_Status AD9910_SetRamWaveformCarrier(AD9910_Waveform wave,
                                           uint32_t carrier_hz,
                                           uint16_t playback_step,
                                           uint16_t points);

/**
  * @brief  输出用户自定义周期RAM波形。
  * @param  samples 一个周期的波形采样表，范围0~AD9910_MAX_AMPLITUDE。
  * @param  points 采样点数，范围16~AD9910_RAM_POINTS。
  * @param  carrier_hz 传0表示直接RAM输出；大于0时将RAM表作为正弦载波包络。
  * @param  playback_step RAM播放步进M。如果传入0，内部按1处理。
  * @retval AD9910_OK、AD9910_BAD_PARAM或AD9910_ERROR。
  *
  * 输出频率近似为：
  * AD9910_SYSCLK_HZ / (4 * playback_step * points)。
  * playback_step只能为整数, 因此RAM波形可能出现频率量化误差。
  */
AD9910_Status AD9910_SetRamCustomWaveformCarrier(const uint16_t *samples,
                                                 uint16_t points,
                                                 uint32_t carrier_hz,
                                                 uint16_t playback_step);

/**
  * @brief  输出带幅度控制的AD9910内置RAM波形。
  * @param  wave 内置波形：三角波、方波、锯齿波或SINC波。
  * @param  carrier_hz 载波频率，0表示直接RAM输出。
  * @param  playback_step RAM播放步进M，范围1~65535，越小输出波形频率越高。
  * @param  points RAM波形点数，范围16~AD9910_RAM_POINTS。
  * @param  amplitude 14位幅度控制值，范围0~AD9910_MAX_AMPLITUDE。
  * @param  retry_delay_ms 写RAM后再次重写的等待时间，0表示不重写。
  * @retval AD9910_OK、AD9910_BAD_PARAM或AD9910_ERROR。
  */
AD9910_Status AD9910_SetRamWaveformCarrierAmplitude(AD9910_Waveform wave,
                                                    uint32_t carrier_hz,
                                                    uint16_t playback_step,
                                                    uint16_t points,
                                                    uint16_t amplitude,
                                                    uint32_t retry_delay_ms);

/**
  * @brief  输出带幅度控制的AD9910用户RAM波形。
  * @param  samples 用户波形表，每点范围0~AD9910_MAX_AMPLITUDE。
  * @param  points RAM波形点数，范围16~AD9910_RAM_POINTS。
  * @param  carrier_hz 载波频率，0表示直接RAM输出。
  * @param  playback_step RAM播放步进M，范围1~65535，越小输出波形频率越高。
  * @param  amplitude 14位幅度控制值，范围0~AD9910_MAX_AMPLITUDE。
  * @param  retry_delay_ms 写RAM后再次重写的等待时间，0表示不重写。
  * @retval AD9910_OK、AD9910_BAD_PARAM或AD9910_ERROR。
  */
AD9910_Status AD9910_SetRamCustomWaveformCarrierAmplitude(const uint16_t *samples,
                                                          uint16_t points,
                                                          uint32_t carrier_hz,
                                                          uint16_t playback_step,
                                                          uint16_t amplitude,
                                                          uint32_t retry_delay_ms);

/* 输出带幅度控制的AD9910用户RAM双极性波形，适合低频三角波等交流重建输出 */
AD9910_Status AD9910_SetRamCustomWaveformCarrierPolarAmplitude(const uint16_t *samples,
                                                               uint16_t points,
                                                               uint32_t carrier_hz,
                                                               uint16_t playback_step,
                                                               uint16_t amplitude,
                                                               uint32_t retry_delay_ms);

/**
  * @brief  不重写RAM表，仅更新RAM播放速度。
  * @param  playback_step RAM播放步进M。如果传入0，内部按1处理。
  * @param  points 当前已加载波形使用的RAM点数，范围16~AD9910_RAM_POINTS。
  * @retval AD9910_OK、AD9910_BAD_PARAM或AD9910_ERROR。
  *
  * 该函数在AD9910_SetRamWaveformCarrier()或
  * AD9910_SetRamCustomWaveformCarrier()之后调用，适合RAM波形扫频。
  */
AD9910_Status AD9910_SetRamPlaybackStep(uint16_t playback_step, uint16_t points);

/**
  * @brief  根据播放步进计算RAM波形输出频率。
  * @param  playback_step RAM播放步进M。
  * @retval 近似输出频率，单位Hz。
  */
uint32_t AD9910_CalcRamOutputHz(uint16_t playback_step);

/**
  * @brief  根据自定义点数计算RAM波形输出频率。
  * @param  playback_step RAM播放步进M。
  * @param  points RAM点数。
  * @retval 近似输出频率，单位Hz；points为0时返回0。
  */
uint32_t AD9910_CalcRamOutputHzPoints(uint16_t playback_step, uint16_t points);

/**
  * @brief  根据目标频率计算AD9910 RAM播放步进。
  * @param  output_hz 目标波形频率，单位Hz。
  * @param  points RAM波形点数，范围16~AD9910_RAM_POINTS。
  * @retval RAM播放步进，范围1~65535；返回0表示目标频率超出可实现范围。
  *
  * 实际频率 = AD9910_SYSCLK_HZ / (4 * step * points)。
  * 由于step只能是整数, output_hz不一定能精确实现。
  * 例如1GHz系统时钟下100kHz: points=125时step=20刚好精确;
  * points=128或256时取整后实际约97.656kHz。
  */
uint16_t AD9910_CalcRamPlaybackStep(uint32_t output_hz, uint16_t points);

/**
  * @brief  使用AD9910数字斜坡发生器配置频率扫频。
  * @param  low_hz 扫频下限，单位Hz。
  * @param  high_hz 扫频上限，单位Hz，必须大于low_hz。
  * @param  up_step_hz 上扫频率步进。
  * @param  down_step_hz 下扫频率步进。
  * @param  up_rate 上扫驻留控制字。
  * @param  down_rate 下扫驻留控制字。
  * @param  mode AD9910_SWEEP_AUTO表示自动双向扫频，
  *         AD9910_SWEEP_MANUAL表示由DRCTL引脚控制扫频方向。
  * @retval AD9910_OK、AD9910_BAD_PARAM或AD9910_ERROR。
  *
  * 每个频点驻留时间近似为4 * rate / AD9910_SYSCLK_HZ。
  */
AD9910_Status AD9910_ConfigureFrequencySweep(uint32_t low_hz,
                                             uint32_t high_hz,
                                             uint32_t up_step_hz,
                                             uint32_t down_step_hz,
                                             uint16_t up_rate,
                                             uint16_t down_rate,
                                             AD9910_SweepMode mode);

/**
  * @brief  通过DRCTL设置手动扫频方向。
  * @param  direction AD9910_SWEEP_UP或AD9910_SWEEP_DOWN。
  */
void AD9910_SetSweepDirection(AD9910_SweepDirection direction);

/**
  * @brief  通过DRHOLD保持或释放数字斜坡发生器。
  * @param  hold GPIO_PIN_SET表示保持，GPIO_PIN_RESET表示释放。
  */
void AD9910_HoldSweep(GPIO_PinState hold);

/**
  * @brief  读取DROVER引脚状态。
  * @retval 返回1表示扫频到达端点，返回0表示未到达端点。
  */
uint8_t AD9910_IsSweepDone(void);

/**
  * @brief  设置AD9910 Profile选择引脚PF[2:0]。
  * @param  profile Profile编号，范围0~7。本工程默认使用Profile 0。
  */
void AD9910_SetProfile(uint8_t profile);

/**
  * @brief  控制AD9910外部掉电引脚。
  * @param  power_down GPIO_PIN_SET表示掉电，GPIO_PIN_RESET表示运行。
  */
void AD9910_PowerDown(GPIO_PinState power_down);

#ifdef __cplusplus
}
#endif

#endif /* __AD9910_H */
