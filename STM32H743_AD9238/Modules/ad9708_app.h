#ifndef __AD9708_APP_H
#define __AD9708_APP_H /* AD9708应用层头文件包含保护。 */

#ifdef __cplusplus
extern "C" {
#endif

#include "ad9708.h"
#include <stdint.h>

#define AD9708_APP_DEFAULT_RAM_POINTS      AD9708_DEFAULT_RAM_POINTS /* 默认RAM波形点数。 */
#define AD9708_APP_MAX_SWEEP_DWELL_US      AD9708_MAX_SWEEP_DWELL_US /* 最大扫频驻留时间，单位us。 */

/* 初始化高速DAC应用层，使用前调用一次。 */
AD9708_StatusTypeDef AD9708_AppInit(void);

/*
 * 设置两点电压校准值，参数为DAC码0和255的实测电压。
 * 调整模块电位器并确认输出未削顶后，再写入固定电位器下的实测值。
 * 允许任意有限电压范围，但必须满足code255_voltage_v > code0_voltage_v。
 */
AD9708_StatusTypeDef AD9708_AppSetVoltageCalibration(
    float code0_voltage_v,
    float code255_voltage_v);

/* 获取当前两点电压校准数据，只读使用。 */
const AD9708_VoltageCalibrationTypeDef *
AD9708_AppGetVoltageCalibration(void);

/* 读取FPGA和高速DAC当前状态。 */
AD9708_StatusTypeDef AD9708_AppPollStatus(void);

/* 设置当前相位偏移并立即生效，范围[0, 360)。 */
AD9708_StatusTypeDef AD9708_AppSetPhase(float phase_deg);

/* 输出校准范围内的恒定电压，单位V。 */
AD9708_StatusTypeDef AD9708_AppOutputConstant(float voltage_v);

/* 输出0~255恒定原始码，仅用于测量两点校准电压。 */
AD9708_StatusTypeDef AD9708_AppOutputCalibrationCode(uint8_t code);

/*
 * 按原始码输出正弦波，不读取或修改电压校准参数。
 * amplitude_code为峰值码，范围0~128；offset_code为中心码，范围0~255。
 */
AD9708_StatusTypeDef AD9708_AppOutputSineCode(float output_hz,
                                              uint8_t amplitude_code,
                                              uint8_t offset_code);

/*
 * 输出2~1024点用户任意波，幅度为Vpp、偏置为中心电压，单位V。
 * 波表应覆盖0~255才能得到完整的amplitude_vpp。
 */
AD9708_StatusTypeDef AD9708_AppOutputWave(const uint8_t *wave,
                                          uint16_t points,
                                          float output_hz,
                                          float amplitude_vpp,
                                          float offset_v);

/* 输出正弦波，频率(0,50MHz]，幅度为Vpp，偏置为中心电压，单位V。 */
AD9708_StatusTypeDef AD9708_AppOutputSine(float output_hz,
                                          float amplitude_vpp,
                                          float offset_v);

/* 输出三角波，频率(0,50MHz]，幅度为Vpp，偏置为中心电压，单位V。 */
AD9708_StatusTypeDef AD9708_AppOutputTriangle(float output_hz,
                                              float amplitude_vpp,
                                              float offset_v);

/* 输出50%方波，频率(0,50MHz]，幅度为Vpp，偏置为中心电压，单位V。 */
AD9708_StatusTypeDef AD9708_AppOutputSquare(float output_hz,
                                            float amplitude_vpp,
                                            float offset_v);

/* 输出上升锯齿波，频率(0,50MHz]，幅度为Vpp，偏置为中心电压，单位V。 */
AD9708_StatusTypeDef AD9708_AppOutputSawtooth(float output_hz,
                                              float amplitude_vpp,
                                              float offset_v);

/* 输出SINC波，频率(0,50MHz]，幅度为Vpp，偏置为中心电压，单位V。 */
AD9708_StatusTypeDef AD9708_AppOutputSinc(float output_hz,
                                          float amplitude_vpp,
                                          float offset_v);

/* 按采样点索引设置起始相位，频率和电压范围与标准波形接口相同。 */
AD9708_StatusTypeDef AD9708_AppOutputWavePhaseIndex(
    AD9708_WaveformTypeDef wave,
    float output_hz,
    uint16_t points,
    float amplitude_vpp,
    float offset_v,
    uint16_t phase_index);

/* 按采样点索引设置任意波起始相位，电压范围由两点校准值决定。 */
AD9708_StatusTypeDef AD9708_AppOutputArbitraryPhaseIndex(
    const uint8_t *wave,
    uint16_t points,
    float output_hz,
    float amplitude_vpp,
    float offset_v,
    uint16_t phase_index);

/* 暂停或继续硬件扫频。 */
AD9708_StatusTypeDef AD9708_AppHoldSweep(uint8_t hold);

/* 手动扫频模式下切换方向。 */
AD9708_StatusTypeDef AD9708_AppSetSweepDirection(
    AD9708_SweepDirectionTypeDef direction);

/* 停止扫频并保持停止瞬间的频率。 */
AD9708_StatusTypeDef AD9708_AppStopSweep(void);

/* 查询单向或手动扫频是否到达端点。 */
AD9708_StatusTypeDef AD9708_AppIsSweepDone(uint8_t *done);

/* 启动正弦扫频，幅度为Vpp、偏置为中心电压，单位V。 */
AD9708_StatusTypeDef AD9708_AppSweepSine(
    float low_hz,
    float high_hz,
    float step_hz,
    float amplitude_vpp,
    float offset_v,
    uint32_t dwell_us,
    AD9708_SweepModeTypeDef mode);

/* 启动三角波扫频，幅度为Vpp、偏置为中心电压，单位V。 */
AD9708_StatusTypeDef AD9708_AppSweepTriangle(
    float low_hz,
    float high_hz,
    float step_hz,
    float amplitude_vpp,
    float offset_v,
    uint32_t dwell_us,
    AD9708_SweepModeTypeDef mode);

/* 启动方波扫频，幅度为Vpp、偏置为中心电压，单位V。 */
AD9708_StatusTypeDef AD9708_AppSweepSquare(
    float low_hz,
    float high_hz,
    float step_hz,
    float amplitude_vpp,
    float offset_v,
    uint32_t dwell_us,
    AD9708_SweepModeTypeDef mode);

/* 启动锯齿波扫频，幅度为Vpp、偏置为中心电压，单位V。 */
AD9708_StatusTypeDef AD9708_AppSweepSawtooth(
    float low_hz,
    float high_hz,
    float step_hz,
    float amplitude_vpp,
    float offset_v,
    uint32_t dwell_us,
    AD9708_SweepModeTypeDef mode);

/* 启动SINC波扫频，幅度为Vpp、偏置为中心电压，单位V。 */
AD9708_StatusTypeDef AD9708_AppSweepSinc(
    float low_hz,
    float high_hz,
    float step_hz,
    float amplitude_vpp,
    float offset_v,
    uint32_t dwell_us,
    AD9708_SweepModeTypeDef mode);

/* 启动任意波扫频，幅度为Vpp、偏置为中心电压，单位V。 */
AD9708_StatusTypeDef AD9708_AppSweepArbitrary(
    const uint8_t *wave,
    uint16_t points,
    float low_hz,
    float high_hz,
    float step_hz,
    float amplitude_vpp,
    float offset_v,
    uint32_t dwell_us,
    AD9708_SweepModeTypeDef mode);

/* 停止扫频并关闭DAC输出。 */
AD9708_StatusTypeDef AD9708_AppStop(void);

/* 获取高速DAC底层状态，只读使用。 */
const AD9708_DataTypeDef *AD9708_AppGetData(void);

#ifdef __cplusplus
}
#endif

#endif /* __AD9708_APP_H */
