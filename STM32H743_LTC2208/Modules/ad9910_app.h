#ifndef __AD9910_APP_H
#define __AD9910_APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ad9910.h"

#define AD9910_APP_WAVE_TABLE_SIZE         32U                   /* AD9910应用层任意波表默认点数 */
#define AD9910_APP_DEFAULT_AMPLITUDE       AD9910_MAX_AMPLITUDE  /* AD9910默认输出幅度, 16383为满幅 */
#define AD9910_APP_DEFAULT_RETRY_DELAY_MS  300U                  /* AD9910 RAM波形重写等待时间, 单位ms */
#define AD9910_APP_DEFAULT_RAM_POINTS      125U                  /* AD9910默认RAM波形点数, 1GHz时100kHz可精确对应step=20 */
#define AD9910_APP_DEFAULT_LOW_HZ          100000U               /* AD9910默认扫频下限, 单位Hz */
#define AD9910_APP_DEFAULT_HIGH_HZ         1000000U              /* AD9910默认扫频上限, 单位Hz */
#define AD9910_APP_DEFAULT_SWEEP_STEP_HZ   50000U                /* AD9910默认扫频频率步进, 单位Hz */
#define AD9910_APP_DEFAULT_DWELL_MS        100U                  /* AD9910默认每个扫频点驻留时间, 单位ms */

/* AD9910用户任意波表, 用户可直接改表内数值, 范围0~16383 */
extern uint16_t AD9910_AppArbitraryTable[AD9910_APP_WAVE_TABLE_SIZE];

/*
 * AD9910频率说明:
 * 正弦波使用DDS频率字, 频率分辨率很高。
 * 三角波、方波、锯齿波和任意波使用RAM播放, 实际频率 = AD9910_SYSCLK_HZ / (4 * points * step)。
 * step只能是整数, 所以output_hz是目标频率, 实际输出可能有量化误差。
 * 例如AD9910_SYSCLK_HZ=1GHz时, points=125输出100kHz正好step=20;
 * 若points=128或256输出100kHz, 实际会接近97.656kHz, 这是RAM步进取整造成的。
 */

/* 初始化AD9910应用层, 主函数先调用一次 */
AD9910_Status AD9910_AppInit(void);

/* 输出AD9910单频正弦波, freq_hz单位Hz, amplitude范围0~16383 */
AD9910_Status AD9910_AppOutputSine(uint32_t freq_hz,
                                   uint16_t amplitude);

/* 设置AD9910当前正弦输出相位偏移，phase_deg单位为度 */
AD9910_Status AD9910_AppSetSinePhaseOffsetDeg(float phase_deg);
/* 按相位索引输出AD9910内置RAM波形, 支持三角波、方波、锯齿波和SINC */
AD9910_Status AD9910_AppOutputRamWavePhaseIndex(AD9910_Waveform wave,
                                                uint32_t output_hz,
                                                uint16_t points,
                                                uint16_t amplitude,
                                                uint16_t phase_index,
                                                uint32_t retry_delay_ms);
/* 按相位索引输出AD9910用户任意波, phase_index为RAM表起点偏移 */
AD9910_Status AD9910_AppOutputArbitraryPhaseIndex(const uint16_t *wave_table,
                                                  uint16_t points,
                                                  uint32_t output_hz,
                                                  uint16_t amplitude,
                                                  uint16_t phase_index,
                                                  uint32_t retry_delay_ms);
/* 按相位索引输出AD9910三角波, phase_index为RAM表起点偏移, 0~points-1对应0~360度 */
AD9910_Status AD9910_AppOutputTrianglePhaseIndex(uint32_t output_hz,
                                                 uint16_t points,
                                                 uint16_t amplitude,
                                                 uint16_t phase_index,
                                                 uint32_t retry_delay_ms);
/* 按相位索引输出AD9910方波, phase_index为RAM表起点偏移, 0~points-1对应0~360度 */
AD9910_Status AD9910_AppOutputSquarePhaseIndex(uint32_t output_hz,
                                               uint16_t points,
                                               uint16_t amplitude,
                                               uint16_t phase_index,
                                               uint32_t retry_delay_ms);
/* 按相位索引输出AD9910锯齿波, phase_index为RAM表起点偏移, 0~points-1对应0~360度 */
AD9910_Status AD9910_AppOutputSawtoothPhaseIndex(uint32_t output_hz,
                                                 uint16_t points,
                                                 uint16_t amplitude,
                                                 uint16_t phase_index,
                                                 uint32_t retry_delay_ms);
/* 输出AD9910方波, output_hz为目标频率, points建议用AD9910_APP_DEFAULT_RAM_POINTS, amplitude范围0~16383 */
AD9910_Status AD9910_AppOutputSquare(uint32_t output_hz,
                                     uint16_t points,
                                     uint16_t amplitude,
                                     uint32_t retry_delay_ms);
/* 输出AD9910锯齿波, output_hz为目标频率, points建议用AD9910_APP_DEFAULT_RAM_POINTS, amplitude范围0~16383 */
AD9910_Status AD9910_AppOutputSawtooth(uint32_t output_hz,
                                       uint16_t points,
                                       uint16_t amplitude,
                                       uint32_t retry_delay_ms);
/* 按目标频率输出AD9910任意波, points会影响实际频率误差, amplitude范围0~16383 */
AD9910_Status AD9910_AppOutputArbitraryHz(const uint16_t *wave_table,
                                          uint16_t points,
                                          uint32_t output_hz,
                                          uint16_t amplitude,
                                          uint32_t retry_delay_ms);

/* 扫频方向模式 */
typedef enum
{
  AD9910_SWEEP_BIDIR = 0,     /* 双向来回: low->high->low->high... */
  AD9910_SWEEP_UNIDIR_LOOP,   /* 单向循环: low->high 到终点后跳回low重新开始 */
  AD9910_SWEEP_UNIDIR_STOP    /* 单向停止: low->high 到终点后停在high */
} AD9910_SweepDirMode;

/* 启动AD9910正弦软件扫频, 通过改写频率字支持ms~s级驻留, 需在while(1)调用AD9910_AppProcess() */
AD9910_Status AD9910_AppSweepSineMs(uint32_t low_hz,
                                    uint32_t high_hz,
                                    uint32_t frequency_step_hz,
                                    uint16_t amplitude,
                                    uint32_t dwell_ms,
                                    AD9910_SweepDirMode dir_mode);
/* 启动AD9910正弦扫频, 使用AD9910数字扫频功能 */
AD9910_Status AD9910_AppSweepSine(uint32_t low_hz,
                                  uint32_t high_hz,
                                  uint32_t up_step_hz,
                                  uint32_t down_step_hz,
                                  uint16_t up_rate,
                                  uint16_t down_rate,
                                  uint16_t amplitude,
                                  AD9910_SweepMode mode,
                                  AD9910_SweepDirection manual_direction,
                                  uint32_t manual_toggle_ms);
/* 启动AD9910三角波扫频, 频率参数单位Hz, 实际频点受points和整数step量化影响 */
AD9910_Status AD9910_AppSweepTriangle(uint32_t low_hz,
                                      uint32_t high_hz,
                                      uint32_t frequency_step_hz,
                                      uint16_t points,
                                      uint16_t amplitude,
                                      uint32_t dwell_ms,
                                      uint32_t retry_delay_ms);
/* 启动AD9910方波扫频, 频率参数单位Hz, 实际频点受points和整数step量化影响 */
AD9910_Status AD9910_AppSweepSquare(uint32_t low_hz,
                                    uint32_t high_hz,
                                    uint32_t frequency_step_hz,
                                    uint16_t points,
                                    uint16_t amplitude,
                                    uint32_t dwell_ms,
                                    uint32_t retry_delay_ms);
/* 启动AD9910锯齿波扫频, 频率参数单位Hz, 实际频点受points和整数step量化影响 */
AD9910_Status AD9910_AppSweepSawtooth(uint32_t low_hz,
                                      uint32_t high_hz,
                                      uint32_t frequency_step_hz,
                                      uint16_t points,
                                      uint16_t amplitude,
                                      uint32_t dwell_ms,
                                      uint32_t retry_delay_ms);
/* 启动AD9910任意波扫频, 频率参数单位Hz, 实际频点受points和整数step量化影响 */
AD9910_Status AD9910_AppSweepArbitrary(const uint16_t *wave_table,
                                       uint16_t points,
                                       uint32_t low_hz,
                                       uint32_t high_hz,
                                       uint32_t frequency_step_hz,
                                       uint16_t amplitude,
                                       uint32_t dwell_ms,
                                       uint32_t retry_delay_ms);

/* 丢弃首尾数据后降采样并转换为14位RAM采样表。 */
uint16_t AD9910_AppDownSample(const float *src,
                              uint32_t src_len,
                              uint16_t discard_front,
                              uint16_t discard_back,
                              uint16_t dst_max,
                              uint16_t *dst);
/* 从输入波形中提取一个周期并转换为14位RAM采样表。 */
uint16_t AD9910_AppExtractOneCycle(const float *src,
                                   uint32_t      len,
                                   uint16_t     *dst);

/* 停止AD9910后台扫频任务, 当前输出保持最后状态 */
void AD9910_AppStop(void);
/* AD9910后台处理函数, 主循环while(1)中周期调用 */
void AD9910_AppProcess(void);

#ifdef __cplusplus
}
#endif

#endif /* __AD9910_APP_H */
