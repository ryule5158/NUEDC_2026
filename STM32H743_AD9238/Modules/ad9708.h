#ifndef __AD9708_H
#define __AD9708_H /* AD9708底层驱动头文件包含保护。 */

#ifdef __cplusplus
extern "C" {
#endif

#include "fpga_link.h"
#include <stdint.h>

#define AD9708_DAC_CLK_HZ             125000000UL  /* FPGA实际DAC更新时钟，125MSPS。 */
#define AD9708_MAX_OUTPUT_HZ          50000000.0f  /* 模块模拟通道允许尝试的最高输出频率。 */
#define AD9708_DIGITAL_NYQUIST_HZ     62500000.0f  /* 125MSPS对应的数字奈奎斯特极限。 */
#define AD9708_RAM_MIN_POINTS         2U           /* 任意波最少采样点数。 */
#define AD9708_RAM_MAX_POINTS         1024U        /* FPGA波形RAM最大采样点数。 */
#define AD9708_MID_CODE               128U         /* 模块双极性输出的零点码。 */
#define AD9708_MAX_CODE               255U         /* 8位DAC最大码值。 */
#define AD9708_MAX_AMPLITUDE          128U         /* 数字峰值幅度，128为满幅。 */
#define AD9708_MIN_AMPLITUDE_Q8       128U         /* 非零Q8.8幅度最小合法值0.5。 */
#define AD9708_MAX_AMPLITUDE_Q8       32768U       /* Q8.8数字峰值幅度上限128.0。 */
#define AD9708_READY_TIMEOUT_MS       100U         /* FPGA时钟锁定超时时间。 */
#define AD9708_MAX_SWEEP_DWELL_CYCLES 0xFFFFFFFFUL /* 扫频单点最大驻留周期数。 */
#define AD9708_DEFAULT_RAM_POINTS     1024U        /* 内置波形默认采样点数。 */
#define AD9708_MAX_SWEEP_DWELL_US     34359738UL   /* 125MSPS下32位驻留计数对应的最大us。 */

/* AD9708底层驱动返回状态。 */
typedef enum
{
  AD9708_OK = 0,          /* 操作成功。 */
  AD9708_ERROR_PARAM,     /* 输入参数超出有效范围。 */
  AD9708_ERROR_NOT_INIT,  /* 驱动尚未初始化。 */
  AD9708_ERROR_LINK,      /* STM32与FPGA通信失败。 */
  AD9708_ERROR_DEVICE,    /* FPGA设备标识或固件版本不匹配。 */
  AD9708_ERROR_PLL,       /* FPGA的DAC时钟未锁定。 */
  AD9708_ERROR_VERIFY,    /* 寄存器回读校验失败。 */
  AD9708_ERROR_CALIBRATION /* 尚未设置有效的电压校准参数。 */
} AD9708_StatusTypeDef;

/* FPGA波形发生器工作模式。 */
typedef enum
{
  AD9708_MODE_CONSTANT = 0U, /* 输出恒定8位DAC码。 */
  AD9708_MODE_RAM_DDS,       /* 使用RAM查表DDS输出任意波。 */
  AD9708_MODE_SAWTOOTH,      /* 使用硬件DDS输出上升锯齿波。 */
  AD9708_MODE_TRIANGLE,      /* 使用硬件DDS输出三角波。 */
  AD9708_MODE_SQUARE         /* 使用硬件DDS输出50%占空比方波。 */
} AD9708_ModeTypeDef;

/* FPGA-side real-time modulation. Requires firmware protocol 0x00020004. */
typedef enum
{
  AD9708_MODULATION_OFF = 0U,
  AD9708_MODULATION_AM,
  AD9708_MODULATION_FM
} AD9708_ModulationModeTypeDef;

/* FPGA硬件扫频运行模式。 */
typedef enum
{
  AD9708_SWEEP_BIDIRECTIONAL = 0U, /* 下限和上限之间往返扫频。 */
  AD9708_SWEEP_UP_LOOP,            /* 从下限扫到上限后跳回下限。 */
  AD9708_SWEEP_UP_ONCE,            /* 从下限扫到上限后停止扫频。 */
  AD9708_SWEEP_MANUAL              /* 由软件随时切换向上或向下。 */
} AD9708_SweepModeTypeDef;

/* FPGA硬件扫频方向。 */
typedef enum
{
  AD9708_SWEEP_DOWN = 0U, /* 从高频向低频扫描。 */
  AD9708_SWEEP_UP          /* 从低频向高频扫描。 */
} AD9708_SweepDirectionTypeDef;

/* 高速DAC可直接生成的标准波形类型。 */
typedef enum
{
  AD9708_WAVE_SINE = 0U, /* 正弦波。 */
  AD9708_WAVE_TRIANGLE,  /* 三角波。 */
  AD9708_WAVE_SQUARE,    /* 50%占空比方波。 */
  AD9708_WAVE_SAWTOOTH,  /* 上升锯齿波。 */
  AD9708_WAVE_SINC       /* 单周期SINC波。 */
} AD9708_WaveformTypeDef;

/* 高速DAC两点电压校准数据。 */
typedef struct
{
  float code0_voltage_v;   /* DAC码0对应的实测电压，单位V。 */
  float code255_voltage_v; /* DAC码255对应的实测电压，单位V。 */
  float volts_per_code;    /* 相邻DAC码对应的电压差，单位V。 */
  uint8_t valid;           /* 1表示校准有效，0表示尚未校准。 */
} AD9708_VoltageCalibrationTypeDef;

/* AD9708驱动及FPGA波形发生器的运行状态。 */
typedef struct
{
  uint8_t initialized;             /* 底层驱动初始化标志。 */
  uint8_t dac_ready;               /* FPGA的DAC复位已释放标志。 */
  uint8_t enabled;                 /* DAC波形输出使能标志。 */
  uint8_t pll_locked;              /* FPGA 125MHz时钟锁定标志。 */
  uint8_t sweep_enabled;           /* 硬件扫频功能使能标志。 */
  uint8_t sweep_running;           /* 硬件扫频正在运行标志。 */
  uint8_t sweep_hold;              /* 硬件扫频暂停标志。 */
  uint8_t sweep_done;              /* 单向或手动扫频到达端点标志。 */
  uint8_t last_code;               /* 最近设置的恒定输出码。 */
  uint8_t amplitude;               /* 当前数字峰值幅度，0~128。 */
  uint8_t amplitude_fraction;      /* 当前数字峰值幅度的小数部分，Q0.8格式。 */
  uint8_t offset;                  /* 当前数字中心偏置，0~255。 */
  uint8_t offset_fraction;         /* 当前中心偏置的小数部分，Q0.8格式。 */
  uint16_t ram_points;             /* 当前任意波采样点数。 */
  AD9708_ModeTypeDef mode;         /* 当前FPGA波形模式。 */
  AD9708_SweepModeTypeDef sweep_mode; /* 当前硬件扫频模式。 */
  AD9708_SweepDirectionTypeDef sweep_direction; /* 当前扫频方向。 */
  float output_hz;                 /* 基础DDS频率，单位Hz。 */
  float current_hz;                /* 当前实际扫频频点，单位Hz。 */
  float phase_deg;                 /* 当前相位偏移，单位度。 */
  float sweep_low_hz;              /* 扫频下限，单位Hz。 */
  float sweep_high_hz;             /* 扫频上限，单位Hz。 */
  float sweep_step_hz;             /* 扫频频率步进，单位Hz。 */
  uint32_t sweep_dwell_cycles;     /* 每个频点驻留的DAC周期数。 */
  uint32_t freq_word;              /* 当前基础DDS频率字。 */
  uint32_t phase_word;             /* 当前32位DDS相位字。 */
  uint32_t device_id;              /* FPGA固件设备标识。 */
  uint32_t firmware_version;       /* FPGA固件协议版本。 */
  AD9708_StatusTypeDef status;     /* 最近一次底层操作状态。 */
  /* Protocol 0x00020004 hardware-modulation state. */
  uint8_t modulation_supported;
  AD9708_ModulationModeTypeDef modulation_mode;
  float modulation_hz;
  float am_depth;
  float fm_deviation_hz;
  float modulation_phase_deg;
  uint32_t modulation_freq_word;
  uint32_t modulation_phase_word;
} AD9708_DataTypeDef;

/* 初始化并核验STM32、FPGA和AD9708完整链路。 */
AD9708_StatusTypeDef AD9708_Init(SPI_HandleTypeDef *hspi);

/* 恢复安全默认配置，输出停止在中点码。 */
AD9708_StatusTypeDef AD9708_Reset(void);

/* 读取FPGA时钟、输出和扫频状态。 */
AD9708_StatusTypeDef AD9708_PollStatus(void);

/* 等待FPGA的125MHz DAC时钟锁定。 */
AD9708_StatusTypeDef AD9708_WaitReady(uint32_t timeout_ms);

/* 启用或停止波形输出，停止时固定输出中点码。 */
AD9708_StatusTypeDef AD9708_SetEnable(uint8_t enable);

/* 将DDS累加器立即重置到已配置相位。 */
AD9708_StatusTypeDef AD9708_ResetPhase(void);

/* 设置FPGA波形发生模式。 */
AD9708_StatusTypeDef AD9708_SetMode(AD9708_ModeTypeDef mode);

/* 设置基础DDS频率，范围大于0且不超过50MHz。 */
AD9708_StatusTypeDef AD9708_SetFrequencyHz(float output_hz);

/* 直接设置基础DDS频率字。 */
AD9708_StatusTypeDef AD9708_SetFrequencyWord(uint32_t freq_word);

/* 设置32位DDS相位偏移字。 */
AD9708_StatusTypeDef AD9708_SetPhaseOffsetWord(uint32_t phase_word);

/*
 * PLL/servo fast path. Frequency and phase are written in one SPI session,
 * without the configuration-time readback overhead. Each FPGA write still
 * returns and validates the link ACK. When enable is zero, output_hz is
 * ignored and the DDS frequency word is forced to zero.
 */
AD9708_StatusTypeDef AD9708_UpdateDdsFast(float output_hz,
                                          uint32_t phase_word,
                                          uint8_t enable);

/*
 * Configure FPGA-rate modulation. Parameters are staged and committed
 * atomically, so the DAC never sees a mixture of old and new settings.
 * AM depth is 0..1. FM deviation is the peak deviation in Hz.
 */
AD9708_StatusTypeDef AD9708_ConfigureAm(float modulation_hz,
                                        float depth,
                                        uint32_t modulation_phase_word);
AD9708_StatusTypeDef AD9708_ConfigureFm(float modulation_hz,
                                        float deviation_hz,
                                        uint32_t modulation_phase_word);
AD9708_StatusTypeDef AD9708_DisableModulation(void);

/* Fast phase-only correction for locking the modulation envelope/instant. */
AD9708_StatusTypeDef AD9708_UpdateModulationPhaseFast(
    uint32_t modulation_phase_word);

/* 设置统一数字幅度和中心偏置，超范围部分由FPGA饱和。 */
AD9708_StatusTypeDef AD9708_SetLevel(uint8_t amplitude, uint8_t offset);

/* 设置Q8.8幅度和中心偏置，用于精确电压幅度与零点。 */
AD9708_StatusTypeDef AD9708_SetLevelFine(uint16_t amplitude_q8,
                                         uint16_t offset_q8);

/* 输出一个恒定8位DAC码。 */
AD9708_StatusTypeDef AD9708_OutputConstant(uint8_t code);

/* 向FPGA写入2~1024点任意波，写入后保持输出停止。 */
AD9708_StatusTypeDef AD9708_LoadWave(const uint8_t *samples,
                                     uint16_t points);

/*
 * Generate and load a full-scale sine or SINC table without enabling output.
 * This is intended for the once-per-boot preparation phase; runtime frequency
 * and phase changes should use AD9708_UpdateDdsFast().
 */
AD9708_StatusTypeDef AD9708_LoadGeneratedWave(AD9708_WaveformTypeDef wave,
                                              uint16_t points);

/* 配置硬件线性扫频，dwell_cycles以125MHz DAC周期为单位。 */
AD9708_StatusTypeDef AD9708_ConfigureFrequencySweep(
    float low_hz,
    float high_hz,
    float step_hz,
    uint32_t dwell_cycles,
    AD9708_SweepModeTypeDef mode,
    AD9708_SweepDirectionTypeDef initial_direction);

/* 从已配置的端点启动硬件扫频。 */
AD9708_StatusTypeDef AD9708_StartSweep(void);

/* 暂停或继续硬件扫频，当前频率和波形持续输出。 */
AD9708_StatusTypeDef AD9708_HoldSweep(uint8_t hold);

/* 手动扫频模式下切换扫频方向。 */
AD9708_StatusTypeDef AD9708_SetSweepDirection(
    AD9708_SweepDirectionTypeDef direction);

/* 停止扫频并保持停止瞬间的输出频率。 */
AD9708_StatusTypeDef AD9708_StopSweep(void);

/* 查询单向或手动扫频是否到达端点。 */
AD9708_StatusTypeDef AD9708_IsSweepDone(uint8_t *done);

/* 读取硬件当前使用的DDS频率。 */
AD9708_StatusTypeDef AD9708_GetCurrentFrequency(float *current_hz);

/* 获取最近一次底层运行状态，只读使用。 */
const AD9708_DataTypeDef *AD9708_GetData(void);

/* 将Hz转换为125MHz、32位DDS频率字。 */
uint32_t AD9708_HzToFreqWord(float output_hz);

/* 将32位DDS频率字转换为实际Hz。 */
float AD9708_FreqWordToHz(uint32_t freq_word);

/* 以下接口供ad9708_app.c使用，普通用户不直接调用。 */
/* 返回底层持有的电压校准数据。 */
AD9708_VoltageCalibrationTypeDef *
AD9708_InternalVoltageCalibration(void);
/* 将校准范围内的电压转换为8位DAC码。 */
AD9708_StatusTypeDef AD9708_InternalVoltageToCode(float voltage_v,
                                                   uint8_t *code);
/* 将电压幅度和偏置转换为Q8.8数字量。 */
AD9708_StatusTypeDef AD9708_InternalVoltageLevelToCode(
    float amplitude_vpp,
    float offset_v,
    uint16_t *amplitude_q8,
    uint16_t *offset_q8);
/* 将波表相位索引转换为32位DDS相位字。 */
AD9708_StatusTypeDef AD9708_InternalPhaseIndexToWord(
    uint16_t points,
    uint16_t phase_index,
    uint32_t *phase_word);
/* 按标准波形参数配置并启动DAC输出。 */
AD9708_StatusTypeDef AD9708_InternalOutputGeneratedVoltage(
    AD9708_WaveformTypeDef wave,
    float output_hz,
    uint16_t points,
    float amplitude_vpp,
    float offset_v,
    uint32_t phase_word);
/* 按任意波原始码、Q8.8电平和相位启动输出。 */
AD9708_StatusTypeDef AD9708_InternalOutputArbitraryCode(
    const uint8_t *wave,
    uint16_t points,
    float output_hz,
    uint16_t amplitude_q8,
    uint16_t offset_q8,
    uint32_t phase_word);
/* 启动已配置波形的硬件扫频。 */
AD9708_StatusTypeDef AD9708_InternalStartConfiguredSweep(
    float low_hz,
    float high_hz,
    float step_hz,
    uint32_t dwell_us,
    AD9708_SweepModeTypeDef mode);

#ifdef __cplusplus
}
#endif

#endif /* __AD9708_H */
