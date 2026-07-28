#ifndef __FPGA_BOOT_RESTORE_H__
#define __FPGA_BOOT_RESTORE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "fpga_promax.h"
#include <stddef.h>
#include <stdint.h>

/*
 * 上电默认题目参数。这里只负责给 FPGA 的易失寄存器恢复一个确定的起点；
 * 锁相算法识别出基波后，可调用 FpgaBootRestore_SetToneBank() 原子更新频点。
 */
#ifndef FPGA_BOOT_ADC_SAMPLE_RATE_HZ
#define FPGA_BOOT_ADC_SAMPLE_RATE_HZ        32000000U
#endif
#ifndef FPGA_BOOT_QSPI_SETTLE_MS
#define FPGA_BOOT_QSPI_SETTLE_MS            500U
#endif
#ifndef FPGA_BOOT_RETRY_INTERVAL_MS
#define FPGA_BOOT_RETRY_INTERVAL_MS         500U
#endif
#ifndef FPGA_BOOT_POST_READY_GUARD_MS
#define FPGA_BOOT_POST_READY_GUARD_MS       500U
#endif
#ifndef FPGA_BOOT_WAVE_BLOCK_LOG2
#define FPGA_BOOT_WAVE_BLOCK_LOG2           12U
#endif
#ifndef FPGA_BOOT_INITIAL_FUNDAMENTAL_HZ
#define FPGA_BOOT_INITIAL_FUNDAMENTAL_HZ    100000U
#endif

typedef void (*FpgaBootRestore_LogFn)(void *user, const char *message);

/* 一组需在每次 FPGA 配置后重新下发的 Q1.16 FIR 系数。 */
typedef struct
{
  uint8_t bank;
  const int32_t *coeff_q16;
  size_t count;
} FpgaBootRestore_FirBank;

typedef struct
{
  uint32_t adc_sample_rate_hz;
  uint32_t qspi_settle_ms;
  uint32_t retry_interval_ms;
  uint32_t required_capability;
  uint8_t wave_block_log2;
  uint8_t run_after_restore;
  const FpgaBootRestore_FirBank *fir_banks;
  size_t fir_bank_count;
  const uint32_t *tone_frequencies_hz;
  size_t tone_count;
  FpgaBootRestore_LogFn log;
  void *log_user;
} FpgaBootRestore_Config;

typedef enum
{
  FPGA_BOOT_STATE_IDLE = 0,
  FPGA_BOOT_STATE_WAIT_QSPI,
  FPGA_BOOT_STATE_RESTORING,
  FPGA_BOOT_STATE_READY,
  FPGA_BOOT_STATE_RETRY_WAIT
} FpgaBootRestore_State;

typedef enum
{
  FPGA_BOOT_STAGE_NONE = 0,
  FPGA_BOOT_STAGE_INIT_LINK,
  FPGA_BOOT_STAGE_CHECK_CAPABILITY,
  FPGA_BOOT_STAGE_STOP,
  FPGA_BOOT_STAGE_LOAD_FIR,
  FPGA_BOOT_STAGE_CONFIGURE_TONES,
  FPGA_BOOT_STAGE_CONFIGURE_WAVE_STATS,
  FPGA_BOOT_STAGE_CLEAR_STATE,
  FPGA_BOOT_STAGE_START,
  FPGA_BOOT_STAGE_VERIFY_RUN
} FpgaBootRestore_Stage;

typedef struct
{
  FpgaBootRestore_State state;
  FpgaBootRestore_Stage stage;
  FpgaPromax_Result last_result;
  uint32_t attempt_count;
  uint32_t next_action_tick;
  uint32_t fpga_device_id;
  uint32_t fpga_version;
  uint32_t fpga_capability;
  uint32_t fpga_status;
  uint8_t ready;
} FpgaBootRestore_Status;

/* 生成可直接使用的默认配置：四组 FIR、八路谐波 DDC、32 MSPS。 */
void FpgaBootRestore_GetDefaultConfig(FpgaBootRestore_Config *config);

/*
 * 启动非阻塞恢复状态机。调用者应在 SPI2/UART 初始化后调用一次 Start，
 * 此后在主循环持续调用 Process；不要在中断内调用。
 */
FpgaPromax_Result FpgaBootRestore_Start(
  const FpgaBootRestore_Config *config);
void FpgaBootRestore_Process(void);

/*
 * Wait until QSPI configuration and the full volatile-register restore are
 * stable. timeout_ms == 0 waits indefinitely, allowing the FPGA rail to come
 * up after the MCU. A second restore after stable_guard_ms closes the race in
 * which an independently powered FPGA is reconfigured after the first reply.
 */
FpgaPromax_Result FpgaBootRestore_WaitReady(uint32_t timeout_ms,
                                                uint32_t stable_guard_ms);

/* 不等待 QSPI 延时，立即执行一次完整、幂等的停机-装载-启动事务。 */
FpgaPromax_Result FpgaBootRestore_RestoreNow(void);

/*
 * 原子更新 DDC 频点：先停数据面，写完全部通道并清状态，最后统一启动。
 * 新频点保存在 STM32 RAM 中，后续链路恢复会继续使用这组频点。
 */
FpgaPromax_Result FpgaBootRestore_SetToneBank(
  const uint32_t *frequencies_hz,
  size_t count);

/* FPGA 被外部重新配置后调用，状态机将在下一次主循环重新恢复参数。 */
void FpgaBootRestore_RequestRecovery(void);

const FpgaBootRestore_Status *FpgaBootRestore_GetStatus(void);
FpgaPromax *FpgaBootRestore_GetDevice(void);
const char *FpgaBootRestore_StageName(FpgaBootRestore_Stage stage);

#ifdef __cplusplus
}
#endif

#endif /* __FPGA_BOOT_RESTORE_H__ */
