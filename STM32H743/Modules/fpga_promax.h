#ifndef __FPGA_PROMAX_H
#define __FPGA_PROMAX_H /* FPGA ProMax通用控制接口包含保护。 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>

#define FPGA_PROMAX_FRAME_SIZE       9U          /* ProMax逻辑协议固定帧长。 */
#define FPGA_PROMAX_REQUEST_SOF      0xA5U       /* ProMax请求帧头。 */
#define FPGA_PROMAX_RESPONSE_SOF     0x5AU       /* ProMax响应帧头。 */
#define FPGA_PROMAX_EXPECTED_ID      0x4258504DU /* ProMax设备标识，ASCII为BXPM。 */
#define FPGA_PROMAX_DDC_CHANNELS     8U          /* 实时档DDC通道数。 */
#define FPGA_PROMAX_MF_BANKS         4U          /* 实时档匹配滤波模板组数。 */
#define FPGA_PROMAX_MF_TAPS          32U         /* 每组匹配滤波抽头数。 */

#define FPGA_PROMAX_STATUS_RUN             (1U << 0) /* 实时数据面运行标志。 */
#define FPGA_PROMAX_STATUS_POWER_VALID     (1U << 1) /* 频带功率结果有效标志。 */
#define FPGA_PROMAX_STATUS_SCORE_VALID     (1U << 2) /* 匹配峰值结果有效标志。 */
#define FPGA_PROMAX_STATUS_WAVE_VALID      (1U << 7) /* 波形统计结果有效标志。 */
#define FPGA_PROMAX_STATUS_ALL_RESULTS     \
  (FPGA_PROMAX_STATUS_POWER_VALID | FPGA_PROMAX_STATUS_SCORE_VALID) /* 全部实时结果有效掩码。 */
#define FPGA_PROMAX_CAPABILITY_REALTIME    (1U << 27) /* FPGA具有实时DDC和匹配滤波能力。 */
#define FPGA_PROMAX_CAPABILITY_FFT         (1U << 28) /* FPGA具有流式FFT能力。 */
#define FPGA_PROMAX_CAPABILITY_WAVE_STATS  (1U << 29) /* FPGA具有连续波形统计能力。 */
#define FPGA_PROMAX_WAVE_MIN_BLOCK_LOG2    6U          /* 最短64点统计块。 */
#define FPGA_PROMAX_WAVE_MAX_BLOCK_LOG2    25U         /* 最长33554432点统计块。 */
#define FPGA_PROMAX_WAVE_CONTROL_CLEAR     (1U << 8)    /* 清除统计并重新学习门限。 */
#define FPGA_PROMAX_WAVE_STATUS_VALID      (1U << 0)    /* 波形统计快照有效。 */
#define FPGA_PROMAX_WAVE_STATUS_SETTLED    (1U << 1)    /* 自动门限已经稳定。 */
#define FPGA_PROMAX_FFT_CONTROL_ENABLE     (1U << 0)  /* FFT使能控制位。 */
#define FPGA_PROMAX_FFT_CONTROL_INVERSE    (1U << 1)  /* FFT逆变换控制位。 */
#define FPGA_PROMAX_FFT_CONTROL_RESTART    (1U << 2)  /* FFT流水线重启控制位。 */
#define FPGA_PROMAX_FFT_STATUS_CONFIGURED  (1U << 0)  /* FFT配置完成状态位。 */
#define FPGA_PROMAX_FFT_STATUS_ERROR       (1U << 1)  /* FFT粘滞错误状态位。 */
#define FPGA_PROMAX_FFT_STATUS_PEAK_VALID  (1U << 2)  /* FFT峰值结果有效状态位。 */

/* ProMax逻辑命令码。 */
typedef enum
{
  FPGA_PROMAX_CMD_PING = 0x00U,  /* 查询设备标识。 */
  FPGA_PROMAX_CMD_READ = 0x01U,  /* 读取32位逻辑寄存器。 */
  FPGA_PROMAX_CMD_WRITE = 0x02U  /* 写入32位逻辑寄存器。 */
} FpgaPromax_Command;

/* FPGA返回的逻辑协议状态。 */
typedef enum
{
  FPGA_PROMAX_STATUS_OK = 0x00U,        /* 命令执行成功。 */
  FPGA_PROMAX_STATUS_CRC = 0x01U,       /* 请求CRC错误。 */
  FPGA_PROMAX_STATUS_BAD_CMD = 0x02U,   /* 命令码不受支持。 */
  FPGA_PROMAX_STATUS_BAD_ADDR = 0x03U,  /* 逻辑地址不存在。 */
  FPGA_PROMAX_STATUS_READ_ONLY = 0x04U, /* 尝试写只读寄存器。 */
  FPGA_PROMAX_STATUS_BUSY = 0x05U       /* FPGA链路正忙。 */
} FpgaPromax_ProtocolStatus;

/* ProMax通用接口返回状态。 */
typedef enum
{
  FPGA_PROMAX_OK = 0,              /* 操作成功。 */
  FPGA_PROMAX_E_ARGUMENT = -1,     /* 输入参数无效。 */
  FPGA_PROMAX_E_TRANSPORT = -2,    /* 底层传输失败或超时。 */
  FPGA_PROMAX_E_BAD_FRAME = -3,    /* 响应帧格式错误。 */
  FPGA_PROMAX_E_CRC = -4,          /* 响应CRC错误。 */
  FPGA_PROMAX_E_SEQUENCE = -5,     /* 响应序号错误。 */
  FPGA_PROMAX_E_FPGA_STATUS = -6,  /* FPGA拒绝命令。 */
  FPGA_PROMAX_E_WRONG_DEVICE = -7, /* 设备标识不匹配。 */
  FPGA_PROMAX_E_RANGE = -8,        /* 数值超出硬件范围。 */
  FPGA_PROMAX_E_TIMEOUT = -9,      /* 等待结果超时。 */
  FPGA_PROMAX_E_UNSUPPORTED = -10  /* 当前FPGA档位不支持该功能。 */
} FpgaPromax_Result;

/* ProMax逻辑寄存器地址。 */
typedef enum
{
  FPGA_PROMAX_REG_ID = 0x00U,             /* 设备标识寄存器。 */
  FPGA_PROMAX_REG_VERSION = 0x01U,        /* 固件版本寄存器。 */
  FPGA_PROMAX_REG_CAPABILITY = 0x02U,     /* 硬件能力寄存器。 */
  FPGA_PROMAX_REG_CONTROL = 0x03U,        /* 实时数据面控制寄存器。 */
  FPGA_PROMAX_REG_STATUS = 0x04U,         /* 实时结果状态寄存器。 */
  FPGA_PROMAX_REG_SNAPSHOT = 0x05U,       /* 结果快照控制寄存器。 */
  FPGA_PROMAX_REG_PHASE_INC0 = 0x10U,     /* DDC0相位步进字首地址。 */
  FPGA_PROMAX_REG_PHASE_OFFSET0 = 0x20U,  /* DDC0相位偏移首地址。 */
  FPGA_PROMAX_REG_MF_SELECTOR = 0x40U,    /* 匹配滤波抽头选择寄存器。 */
  FPGA_PROMAX_REG_MF_COEFF = 0x41U,       /* 匹配滤波系数寄存器。 */
  FPGA_PROMAX_REG_POWER_LO0 = 0x50U,      /* 通道0功率低字首地址。 */
  FPGA_PROMAX_REG_POWER_HI0 = 0x58U,      /* 通道0功率高字首地址。 */
  FPGA_PROMAX_REG_MF_SCORE0 = 0x60U,      /* 模板0匹配峰值首地址。 */
  FPGA_PROMAX_REG_MF_PEAK_INDEX0 = 0x64U, /* 模板0峰值位置首地址。 */
  FPGA_PROMAX_REG_FFT_CONTROL = 0x80U,    /* FFT控制寄存器。 */
  FPGA_PROMAX_REG_FFT_STATUS = 0x81U,     /* FFT状态寄存器。 */
  FPGA_PROMAX_REG_FFT_PEAK_BIN = 0x82U,   /* FFT峰值频点寄存器。 */
  FPGA_PROMAX_REG_FFT_PEAK_LO = 0x83U,    /* FFT峰值平方低字寄存器。 */
  FPGA_PROMAX_REG_FFT_PEAK_HI = 0x84U,    /* FFT峰值平方高字寄存器。 */
  FPGA_PROMAX_REG_FFT_GENERATION = 0x85U, /* FFT结果代次寄存器。 */
  FPGA_PROMAX_REG_WAVE_CONTROL = 0x70U,   /* 统计块长度和清零控制。 */
  FPGA_PROMAX_REG_WAVE_STATUS = 0x71U,    /* 有效、门限稳定和代次。 */
  FPGA_PROMAX_REG_WAVE_MIN_MAX = 0x72U,   /* 最小、最大和判决门限。 */
  FPGA_PROMAX_REG_WAVE_SUM_LO = 0x73U,    /* 有符号样本和低32位。 */
  FPGA_PROMAX_REG_WAVE_SUM_HI = 0x74U,    /* 有符号样本和高8位。 */
  FPGA_PROMAX_REG_WAVE_SQ_LO = 0x75U,     /* 样本平方和低32位。 */
  FPGA_PROMAX_REG_WAVE_SQ_HI = 0x76U,     /* 样本平方和高16位。 */
  FPGA_PROMAX_REG_WAVE_HIGH = 0x77U,      /* 高于自动门限的样本数。 */
  FPGA_PROMAX_REG_WAVE_RISING = 0x78U,    /* 上升沿数量。 */
  FPGA_PROMAX_REG_WAVE_PERIOD = 0x79U,    /* 相邻上升沿间隔累计。 */
  FPGA_PROMAX_REG_WAVE_SAMPLES = 0x7AU    /* 统计块实际样本数。 */
} FpgaPromax_Register;

/* 完成一次逻辑请求和响应的底层传输回调。 */
typedef int32_t (*FpgaPromax_TransportFn)(
  void *user,
  const uint8_t request[FPGA_PROMAX_FRAME_SIZE],
  uint8_t response[FPGA_PROMAX_FRAME_SIZE],
  uint32_t timeout_ms);

/* 获取单调递增毫秒时基的回调。 */
typedef uint32_t (*FpgaPromax_TimeMsFn)(void *user);

/* 执行毫秒延时的回调。 */
typedef void (*FpgaPromax_DelayMsFn)(void *user, uint32_t delay_ms);

/* 实时档一次快照的全部压缩结果。 */
typedef struct
{
  uint64_t band_power[FPGA_PROMAX_DDC_CHANNELS];          /* 八路37位频带功率。 */
  int32_t matched_score[FPGA_PROMAX_MF_BANKS];            /* 四路24位有符号匹配峰值。 */
  uint32_t matched_peak_index[FPGA_PROMAX_MF_BANKS];      /* 四路匹配峰值样本位置。 */
  uint8_t power_generation;                               /* 功率结果代次。 */
  uint8_t score_generation;                               /* 匹配结果代次。 */
} FpgaPromax_AllResults;

/* FFT档的帧内峰值摘要。 */
typedef struct
{
  uint16_t bin;               /* 峰值频点编号。 */
  uint64_t magnitude_square;  /* 峰值幅度平方，低61位有效。 */
  uint8_t generation;         /* FFT结果代次。 */
} FpgaPromax_FftPeak;

/* FPGA满速数据面输出的原始波形统计。 */
typedef struct
{
  int8_t minimum_code;       /* ADC相对中心码的最小值。 */
  int8_t maximum_code;       /* ADC相对中心码的最大值。 */
  int8_t threshold_code;     /* 本块频率/占空比判决门限。 */
  int64_t sum_codes;         /* 样本码和，硬件有效位宽40位。 */
  uint64_t sum_square_codes; /* 样本码平方和，硬件有效位宽48位。 */
  uint32_t high_count;       /* 高于门限的样本数。 */
  uint32_t rising_count;     /* 本块检测到的上升沿数。 */
  uint32_t period_sum;       /* 相邻上升沿间隔的采样点数累计。 */
  uint32_t sample_count;     /* 本块样本数。 */
  uint8_t generation;        /* 波形统计代次。 */
  uint8_t block_log2;        /* sample_count对应的2的幂。 */
  uint8_t threshold_settled; /* 1表示已使用上一块学习出的门限。 */
} FpgaPromax_WaveStats;

/* 将原始ADC码按标定参数换算后的常用电赛测量量。 */
typedef struct
{
  float dc;                 /* 平均值，单位由标定斜率决定，通常为V。 */
  float rms;                /* 含直流的真有效值。 */
  float ac_rms;             /* 去直流后的交流有效值。 */
  float vpp;                /* 峰峰值。 */
  float frequency_hz;       /* 上升沿平均周期换算频率。 */
  float duty_percent;       /* 自动门限下的高电平占空比。 */
  uint8_t frequency_valid;  /* 上升沿不足两个时为0。 */
} FpgaPromax_WaveMetrics;

/* 三个并行DDC通道给出的普通单音AM调制度估计。 */
typedef struct
{
  uint64_t carrier_power;    /* 载频通道功率。 */
  uint64_t lower_power;      /* 下边带通道功率。 */
  uint64_t upper_power;      /* 上边带通道功率。 */
  float modulation_depth;    /* sqrt(2*(Pl+Pu)/Pc)，范围通常0~1。 */
} FpgaPromax_AmEstimate;

/* ProMax设备运行句柄。 */
typedef struct
{
  FpgaPromax_TransportFn transport; /* 底层传输回调。 */
  void *transport_user;             /* 底层传输上下文。 */
  uint32_t timeout_ms;              /* 单次逻辑事务超时，单位ms。 */
  uint8_t next_sequence;            /* 下一请求序号。 */
  uint8_t last_protocol_status;     /* 最近一次FPGA协议状态。 */
  int32_t last_transport_status;    /* 最近一次底层传输状态。 */
  uint32_t version;                 /* 已读取的FPGA固件版本。 */
  uint32_t capability;              /* 已读取的FPGA能力位。 */
  uint8_t snapshot_held;            /* 软件记录的快照冻结状态。 */
  FpgaPromax_TimeMsFn time_ms;      /* 可选毫秒时基回调。 */
  FpgaPromax_DelayMsFn delay_ms;    /* 可选毫秒延时回调。 */
  void *time_user;                  /* 时基回调上下文。 */
} FpgaPromax;

/* 初始化设备并读取设备标识、版本和能力。 */
FpgaPromax_Result FpgaPromax_Init(FpgaPromax *dev,
                                  FpgaPromax_TransportFn transport,
                                  void *transport_user,
                                  uint32_t timeout_ms);

/* 设置结果等待所用的可选毫秒时基。 */
void FpgaPromax_SetTimebase(FpgaPromax *dev,
                            FpgaPromax_TimeMsFn time_ms,
                            FpgaPromax_DelayMsFn delay_ms,
                            void *time_user);

/* 查询并返回ProMax设备标识。 */
FpgaPromax_Result FpgaPromax_Ping(FpgaPromax *dev, uint32_t *device_id);

/* 读取一个32位逻辑寄存器。 */
FpgaPromax_Result FpgaPromax_ReadRegister(FpgaPromax *dev,
                                         uint8_t address,
                                         uint32_t *value);

/* 写入一个32位逻辑寄存器。 */
FpgaPromax_Result FpgaPromax_WriteRegister(FpgaPromax *dev,
                                          uint8_t address,
                                          uint32_t value);

/* 启停实时数据面。 */
FpgaPromax_Result FpgaPromax_SetRun(FpgaPromax *dev, uint8_t enable);

/* 同步清除实时状态并保留已装载模板。 */
FpgaPromax_Result FpgaPromax_ClearState(FpgaPromax *dev);

/* 读取实时数据面状态。 */
FpgaPromax_Result FpgaPromax_GetStatus(FpgaPromax *dev, uint32_t *status);

/* 有界等待指定实时结果有效。 */
FpgaPromax_Result FpgaPromax_WaitResults(FpgaPromax *dev,
                                        uint32_t required_status_mask,
                                        uint32_t timeout_ms);

/* 冻结结果快照。 */
FpgaPromax_Result FpgaPromax_BeginResultSnapshot(FpgaPromax *dev);

/* 解冻结果快照。 */
FpgaPromax_Result FpgaPromax_EndResultSnapshot(FpgaPromax *dev);

/* 读取结果代次和快照冻结状态。 */
FpgaPromax_Result FpgaPromax_GetSnapshotInfo(FpgaPromax *dev,
                                            uint8_t *power_generation,
                                            uint8_t *score_generation,
                                            uint8_t *is_held);

/* 设置一路DDC的相位步进和偏移字。 */
FpgaPromax_Result FpgaPromax_SetDdcPhaseWord(FpgaPromax *dev,
                                            uint8_t channel,
                                            uint32_t phase_increment,
                                            uint32_t phase_offset);

/* 按频率和有效采样率设置一路DDC。 */
FpgaPromax_Result FpgaPromax_SetDdcFrequency(FpgaPromax *dev,
                                            uint8_t channel,
                                            uint32_t frequency_hz,
                                            uint32_t sample_rate_hz);

/* 装载一组Q1.16匹配模板，可选时间反转。 */
FpgaPromax_Result FpgaPromax_LoadMatchedTemplate(FpgaPromax *dev,
                                                uint8_t bank,
                                                const int32_t *coeff_q16,
                                                size_t count,
                                                uint8_t reverse);

/*
 * 把普通FIR系数装入指定硬件滤波器组。
 * FPGA中的4组32阶FIR会在每个ADC有效采样到来时同时运算，不由SPI逐点喂数据。
 */
FpgaPromax_Result FpgaPromax_LoadFirBank(FpgaPromax *dev,
                                        uint8_t bank,
                                        const int32_t *coeff_q16,
                                        size_t count);

/*
 * 把时域模板反序后装入FIR；这等价于相关/匹配滤波。
 * 模板长度最多32点，未使用抽头自动补零。
 */
FpgaPromax_Result FpgaPromax_LoadCorrelationTemplate(
  FpgaPromax *dev,
  uint8_t bank,
  const int32_t *template_q16,
  size_t count);

/* 读取一路37位频带功率。 */
FpgaPromax_Result FpgaPromax_GetBandPower(FpgaPromax *dev,
                                         uint8_t channel,
                                         uint64_t *power);

/* 读取一组24位有符号匹配峰值。 */
FpgaPromax_Result FpgaPromax_GetMatchedScore(FpgaPromax *dev,
                                            uint8_t bank,
                                            int32_t *score);

/* 读取一组匹配峰值及其样本位置。 */
FpgaPromax_Result FpgaPromax_GetMatchedPeak(FpgaPromax *dev,
                                           uint8_t bank,
                                           int32_t *score,
                                           uint32_t *sample_index);

/* 通用FIR命名的峰值读取接口；底层与匹配滤波峰值寄存器相同。 */
FpgaPromax_Result FpgaPromax_GetFirPeak(FpgaPromax *dev,
                                       uint8_t bank,
                                       int32_t *value,
                                       uint32_t *sample_index);

/* 在同一快照内读取全部实时结果。 */
FpgaPromax_Result FpgaPromax_ReadAllResults(FpgaPromax *dev,
                                           FpgaPromax_AllResults *results,
                                           uint32_t timeout_ms);

/* 判断当前FPGA是否具有实时数据面。 */
uint8_t FpgaPromax_HasRealtime(const FpgaPromax *dev);

/* 判断当前FPGA是否具有FFT数据面。 */
uint8_t FpgaPromax_HasFft(const FpgaPromax *dev);

/* 判断当前FPGA是否具有连续波形统计数据面。 */
uint8_t FpgaPromax_HasWaveStats(const FpgaPromax *dev);

/* 配置FFT启停、方向和重启。 */
FpgaPromax_Result FpgaPromax_SetFft(FpgaPromax *dev,
                                   uint8_t enable,
                                   uint8_t inverse,
                                   uint8_t restart);

/* 有界等待FFT峰值结果有效。 */
FpgaPromax_Result FpgaPromax_WaitFftPeak(FpgaPromax *dev,
                                        uint32_t timeout_ms);

/* 读取一帧FFT峰值摘要。 */
FpgaPromax_Result FpgaPromax_GetFftPeak(FpgaPromax *dev,
                                       FpgaPromax_FftPeak *peak,
                                       uint32_t timeout_ms);

/* 设置2^block_log2点统计块，并清除旧结果重新学习自动门限。 */
FpgaPromax_Result FpgaPromax_SetWaveBlockLog2(FpgaPromax *dev,
                                              uint8_t block_log2);

/* 读取一个门限已稳定的完整波形统计快照。 */
FpgaPromax_Result FpgaPromax_ReadWaveStats(FpgaPromax *dev,
                                           FpgaPromax_WaveStats *stats,
                                           uint32_t timeout_ms);

/*
 * 根据线性标定v=voltage_at_code_zero+code*volts_per_code换算工程量。
 * 若只关心ADC码，可传voltage_at_code_zero=0、volts_per_code=1。
 */
FpgaPromax_Result FpgaPromax_ComputeWaveMetrics(
  const FpgaPromax_WaveStats *stats,
  uint32_t sample_rate_hz,
  float voltage_at_code_zero,
  float volts_per_code,
  FpgaPromax_WaveMetrics *metrics);

/* 把任意RF/带通采样频率折叠到0~fs/2的数字别名频率。 */
uint32_t FpgaPromax_AliasFrequency(uint32_t frequency_hz,
                                  uint32_t sample_rate_hz);

/* 用前count路DDC同时监视多个频点，未使用通道自动关闭。 */
FpgaPromax_Result FpgaPromax_ConfigureToneBank(
  FpgaPromax *dev,
  const uint32_t *frequencies_hz,
  size_t count,
  uint32_t sample_rate_hz);

/* 读取前count路功率并返回最强通道索引。 */
FpgaPromax_Result FpgaPromax_ReadToneBank(
  FpgaPromax *dev,
  size_t count,
  uint64_t power[FPGA_PROMAX_DDC_CHANNELS],
  uint8_t *strongest_channel,
  uint32_t timeout_ms);

/* 配置通道0/1/2为载频、下边带和上边带。 */
FpgaPromax_Result FpgaPromax_ConfigureAmMeasurement(
  FpgaPromax *dev,
  uint32_t carrier_hz,
  uint32_t modulation_hz,
  uint32_t sample_rate_hz);

/* 读取三个通道并估计普通单音AM调制度。 */
FpgaPromax_Result FpgaPromax_ReadAmEstimate(
  FpgaPromax *dev,
  FpgaPromax_AmEstimate *estimate,
  uint32_t timeout_ms);

/* 计算CRC-8/ATM校验值。 */
uint8_t FpgaPromax_Crc8(const uint8_t *data, size_t length);

/* 返回通用结果状态的调试字符串。 */
const char *FpgaPromax_ResultString(FpgaPromax_Result result);

/* 返回FPGA协议状态的调试字符串。 */
const char *FpgaPromax_ProtocolStatusString(uint8_t status);

#ifdef __cplusplus
}
#endif

#endif /* __FPGA_PROMAX_H */
