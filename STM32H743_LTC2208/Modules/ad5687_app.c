#include "ad5687_app.h"
#include "main.h"

#define AD5687_APP_SCK_GPIO_PORT   GPIOA       /* 两片AD5687共用软件SPI时钟端口 */
#define AD5687_APP_SCK_PIN         GPIO_PIN_5  /* 两片AD5687共用软件SPI时钟引脚 */
#define AD5687_APP_MOSI_GPIO_PORT  GPIOA       /* 两片AD5687共用软件SPI数据端口 */
#define AD5687_APP_MOSI_PIN        GPIO_PIN_7  /* 两片AD5687共用软件SPI数据引脚 */
#define AD5687_APP_CS_GPIO_PORT    AD5687_CS_GPIO_Port /* 两片AD5687共用片选端口。 */
#define AD5687_APP_CS_PIN          AD5687_CS_Pin       /* 两片AD5687共用片选引脚。 */

/*
 * AD5687波形由主循环软件送点产生, 必须持续调用AD5687_AppProcess()。
 * output_hz越高需要越高送点速率，当前上限16000点/秒，16点表最高1000Hz。
 */

/* AD5687软件波形和扫频任务状态。 */
typedef struct
{
  const uint16_t *table;
  uint16_t size;
  uint16_t amplitude;
  uint16_t index;
  uint32_t frequency_hz;
  uint32_t last_sample_cycle;   /* 上一次送点时的CPU周期计数 */;
  uint64_t phase_accumulator;
  uint32_t low_hz;
  uint32_t high_hz;
  uint32_t frequency_step_hz;
  uint32_t dwell_ms;
  uint32_t last_sweep_tick;
  AD5687_OutputTypeDef output;
  int8_t sweep_direction;
  uint8_t running;
  uint8_t sweep_running;
} AD5687_AppWaveTaskTypeDef;

static AD5687_HandleTypeDef s_ad5687;          /* AD5687应用层驱动句柄。 */
static AD5687_AppWaveTaskTypeDef s_wave_task;  /* 当前软件波形任务。 */
static uint8_t s_ad5687_initialized;           /* 应用层初始化标志。 */

/* AD5687内置正弦波表。 */
const uint16_t AD5687_AppSineTable[AD5687_APP_WAVE_TABLE_SIZE] =
{
  2048U, 2831U, 3495U, 3939U,
  4095U, 3939U, 3495U, 2831U,
  2048U, 1264U,  600U,  156U,
     0U,  156U,  600U, 1264U
};

/* AD5687内置三角波表。 */
const uint16_t AD5687_AppTriangleTable[AD5687_APP_WAVE_TABLE_SIZE] =
{
     0U,  512U, 1024U, 1536U,
  2048U, 2560U, 3072U, 3584U,
  4095U, 3584U, 3072U, 2560U,
  2048U, 1536U, 1024U,  512U
};

/* AD5687内置方波表。 */
const uint16_t AD5687_AppSquareTable[AD5687_APP_WAVE_TABLE_SIZE] =
{
     0U,    0U,    0U,    0U,
     0U,    0U,    0U,    0U,
  4095U, 4095U, 4095U, 4095U,
  4095U, 4095U, 4095U, 4095U
};

/* AD5687内置锯齿波表。 */
const uint16_t AD5687_AppSawtoothTable[AD5687_APP_WAVE_TABLE_SIZE] =
{
     0U,  273U,  546U,  819U,
  1092U, 1365U, 1638U, 1911U,
  2184U, 2457U, 2730U, 3003U,
  3276U, 3549U, 3822U, 4095U
};

/* AD5687用户可编辑任意波表。 */
uint16_t AD5687_AppArbitraryTable[AD5687_APP_WAVE_TABLE_SIZE] =
{
     0U,  512U, 1024U, 1536U,
  2048U, 2560U, 3072U, 3584U,
  4095U, 3584U, 3072U, 2560U,
  2048U, 1536U, 1024U,  512U
};

/* 按需初始化AD5687应用层。 */
static HAL_StatusTypeDef EnsureInit(void);
/* 将CV输出编号映射到级联器件和通道。 */
static HAL_StatusTypeDef GetOutputAddress(AD5687_OutputTypeDef output,
                                          AD5687_DeviceTypeDef *device,
                                          AD5687_ChannelTypeDef *channel);
/* 启动指定波表的周期输出。 */
static HAL_StatusTypeDef StartWave(AD5687_OutputTypeDef output,
                                   const uint16_t *wave_table,
                                   uint16_t table_size,
                                   uint32_t output_hz,
                                   uint16_t amplitude);
/* 启动指定波表的往返扫频。 */
static HAL_StatusTypeDef StartSweep(AD5687_OutputTypeDef output,
                                    const uint16_t *wave_table,
                                    uint16_t table_size,
                                    uint32_t low_hz,
                                    uint32_t high_hz,
                                    uint32_t frequency_step_hz,
                                    uint16_t amplitude,
                                    uint32_t dwell_ms);
/* 按经过时间输出应送波形点。 */
static HAL_StatusTypeDef StepWave(uint8_t force);
/* 更新扫频方向和当前频率。 */
static void UpdateSweep(void);
/* 按目标幅度缩放波表码值。 */
static uint16_t ScaleCode(uint16_t code, uint16_t amplitude);
static void AD5687_AppTimebaseInit(void);  /* 初始化CPU周期计数器 */
static uint32_t AD5687_AppGetCycle(void);  /* 获取当前CPU周期计数 */

/************************************************************
 * Function :       AD5687_AppInit
 * Comment  :       初始化AD5687应用层实例
 * Parameter:       null
 * Return   :       HAL_OK表示成功, 其他值表示失败
 * Date     :       2026-06-12 V2
************************************************************/
HAL_StatusTypeDef AD5687_AppInit(void)
{
  AD5687_AppTimebaseInit();  /* 启用DWT周期计数，提高软件送点时间分辨率 */
  HAL_StatusTypeDef status;

  s_ad5687.sck_port = AD5687_APP_SCK_GPIO_PORT;
  s_ad5687.sck_pin = AD5687_APP_SCK_PIN;
  s_ad5687.mosi_port = AD5687_APP_MOSI_GPIO_PORT;
  s_ad5687.mosi_pin = AD5687_APP_MOSI_PIN;
  s_ad5687.cs_port = AD5687_APP_CS_GPIO_PORT;
  s_ad5687.cs_pin = AD5687_APP_CS_PIN;
  s_ad5687.spi_delay_cycles = AD5687_SOFT_SPI_DELAY_CYCLES;
  s_ad5687.vref_mv = AD5687_APP_VREF_MV;
  s_ad5687.gain = AD5687_APP_GAIN;

  AD5687_AppStop();
  status = AD5687_Init(&s_ad5687);
  s_ad5687_initialized = (status == HAL_OK) ? 1U : 0U;

  return status;
}

/************************************************************
 * Function :       AD5687_AppOutputRaw
 * Comment  :       按12位原始码值输出AD5687指定通道
 * Parameter:       output: CV1~CV4输出口; code: 12位码值, 范围0~4095
 * Return   :       HAL_OK表示成功, 其他值表示失败
 * Date     :       2026-06-12 V1
************************************************************/
HAL_StatusTypeDef AD5687_AppOutputRaw(AD5687_OutputTypeDef output,
                                      uint16_t code)
{
  HAL_StatusTypeDef status;
  AD5687_DeviceTypeDef device;
  AD5687_ChannelTypeDef channel;

  status = GetOutputAddress(output, &device, &channel);
  if (status != HAL_OK)
  {
    return status;
  }

  status = EnsureInit();
  if (status != HAL_OK)
  {
    return status;
  }

  AD5687_AppStop();
  return AD5687_WriteAndUpdateRaw(&s_ad5687, device, channel, code);
}

/************************************************************
 * Function :       AD5687_AppOutputVoltageMv
 * Comment  :       按毫伏值输出AD5687指定通道
 * Parameter:       output: CV1~CV4输出口; voltage_mv: 输出电压, 范围0~2500mV
 * Return   :       HAL_OK表示成功, 其他值表示失败
 * Date     :       2026-06-12 V1
************************************************************/
HAL_StatusTypeDef AD5687_AppOutputVoltageMv(AD5687_OutputTypeDef output,
                                            uint32_t voltage_mv)
{
  HAL_StatusTypeDef status;
  AD5687_DeviceTypeDef device;
  AD5687_ChannelTypeDef channel;

  status = GetOutputAddress(output, &device, &channel);
  if (status != HAL_OK)
  {
    return status;
  }

  status = EnsureInit();
  if (status != HAL_OK)
  {
    return status;
  }

  AD5687_AppStop();
  return AD5687_SetVoltageMv(&s_ad5687, device, channel, voltage_mv);
}

/************************************************************
 * Function :       AD5687_AppOutputFourVoltageMv
 * Comment  :       一次设置测试板CV1到CV4四路直流输出
 * Parameter:       cv1_mv~cv4_mv: 各路目标电压, 范围0~2500mV
 * Return   :       HAL_OK表示成功, 其他值表示失败
 * Date     :       2026-06-12 V2
************************************************************/
HAL_StatusTypeDef AD5687_AppOutputFourVoltageMv(uint32_t cv1_mv,
                                                uint32_t cv2_mv,
                                                uint32_t cv3_mv,
                                                uint32_t cv4_mv)
{
  HAL_StatusTypeDef status;

  status = EnsureInit();
  if (status != HAL_OK)
  {
    return status;
  }

  AD5687_AppStop();
  status = AD5687_SetBothVoltageMv(&s_ad5687,
                                    AD5687_DEVICE_CV1_CV2,
                                    cv1_mv,
                                    cv2_mv);
  if (status != HAL_OK)
  {
    return status;
  }

  return AD5687_SetBothVoltageMv(&s_ad5687,
                                  AD5687_DEVICE_CV3_CV4,
                                  cv3_mv,
                                  cv4_mv);
}

/************************************************************
 * Function :       AD5687_AppOutputSine
 * Comment  :       启动AD5687正弦波输出任务
 * Parameter:       output: CV1~CV4输出口; output_hz: 波形频率, 范围1~1000Hz; amplitude: 12位幅度, 范围0~4095
 * Return   :       HAL_OK表示成功, 其他值表示失败
 * Date     :       2026-06-12 V1
************************************************************/
HAL_StatusTypeDef AD5687_AppOutputSine(AD5687_OutputTypeDef output,
                                       uint32_t output_hz,
                                       uint16_t amplitude)
{
  return StartWave(output,
                   AD5687_AppSineTable,
                   AD5687_APP_WAVE_TABLE_SIZE,
                   output_hz,
                   amplitude);
}

/************************************************************
 * Function :       AD5687_AppOutputTriangle
 * Comment  :       启动AD5687三角波输出任务
 * Parameter:       output: CV1~CV4输出口; output_hz: 波形频率, 范围1~1000Hz; amplitude: 12位幅度, 范围0~4095
 * Return   :       HAL_OK表示成功, 其他值表示失败
 * Date     :       2026-06-12 V1
************************************************************/
HAL_StatusTypeDef AD5687_AppOutputTriangle(AD5687_OutputTypeDef output,
                                           uint32_t output_hz,
                                           uint16_t amplitude)
{
  return StartWave(output,
                   AD5687_AppTriangleTable,
                   AD5687_APP_WAVE_TABLE_SIZE,
                   output_hz,
                   amplitude);
}

/************************************************************
 * Function :       AD5687_AppOutputSquare
 * Comment  :       启动AD5687方波输出任务
 * Parameter:       output: CV1~CV4输出口; output_hz: 波形频率, 范围1~1000Hz; amplitude: 12位幅度, 范围0~4095
 * Return   :       HAL_OK表示成功, 其他值表示失败
 * Date     :       2026-06-12 V1
************************************************************/
HAL_StatusTypeDef AD5687_AppOutputSquare(AD5687_OutputTypeDef output,
                                         uint32_t output_hz,
                                         uint16_t amplitude)
{
  return StartWave(output,
                   AD5687_AppSquareTable,
                   AD5687_APP_WAVE_TABLE_SIZE,
                   output_hz,
                   amplitude);
}

/************************************************************
 * Function :       AD5687_AppOutputSawtooth
 * Comment  :       启动AD5687锯齿波输出任务
 * Parameter:       output: CV1~CV4输出口; output_hz: 波形频率, 范围1~1000Hz; amplitude: 12位幅度, 范围0~4095
 * Return   :       HAL_OK表示成功, 其他值表示失败
 * Date     :       2026-06-12 V1
************************************************************/
HAL_StatusTypeDef AD5687_AppOutputSawtooth(AD5687_OutputTypeDef output,
                                           uint32_t output_hz,
                                           uint16_t amplitude)
{
  return StartWave(output,
                   AD5687_AppSawtoothTable,
                   AD5687_APP_WAVE_TABLE_SIZE,
                   output_hz,
                   amplitude);
}

/************************************************************
 * Function :       AD5687_AppOutputArbitraryHz
 * Comment  :       启动AD5687任意波输出任务
 * Parameter:       output: CV1~CV4输出口; wave_table: 波形表; table_size: 点数; output_hz: 波形频率; amplitude: 12位幅度
 * Return   :       HAL_OK表示成功, 其他值表示失败
 * Date     :       2026-06-12 V1
************************************************************/
HAL_StatusTypeDef AD5687_AppOutputArbitraryHz(AD5687_OutputTypeDef output,
                                              const uint16_t *wave_table,
                                              uint16_t table_size,
                                              uint32_t output_hz,
                                              uint16_t amplitude)
{
  return StartWave(output,
                   wave_table,
                   table_size,
                   output_hz,
                   amplitude);
}

/************************************************************
 * Function :       AD5687_AppSweepSine
 * Comment  :       启动AD5687正弦波扫频任务
 * Parameter:       output: CV1~CV4输出口; low_hz/high_hz: 扫频范围Hz; frequency_step_hz: 频率步进Hz; amplitude: 12位幅度; dwell_ms: 每步停留时间
 * Return   :       HAL_OK表示成功, 其他值表示失败
 * Date     :       2026-06-12 V1
************************************************************/
HAL_StatusTypeDef AD5687_AppSweepSine(AD5687_OutputTypeDef output,
                                      uint32_t low_hz,
                                      uint32_t high_hz,
                                      uint32_t frequency_step_hz,
                                      uint16_t amplitude,
                                      uint32_t dwell_ms)
{
  return StartSweep(output,
                    AD5687_AppSineTable,
                    AD5687_APP_WAVE_TABLE_SIZE,
                    low_hz,
                    high_hz,
                    frequency_step_hz,
                    amplitude,
                    dwell_ms);
}

/************************************************************
 * Function :       AD5687_AppSweepTriangle
 * Comment  :       启动AD5687三角波扫频任务
 * Parameter:       output: CV1~CV4输出口; low_hz/high_hz: 扫频范围Hz; frequency_step_hz: 频率步进Hz; amplitude: 12位幅度; dwell_ms: 每步停留时间
 * Return   :       HAL_OK表示成功, 其他值表示失败
 * Date     :       2026-06-12 V1
************************************************************/
HAL_StatusTypeDef AD5687_AppSweepTriangle(AD5687_OutputTypeDef output,
                                          uint32_t low_hz,
                                          uint32_t high_hz,
                                          uint32_t frequency_step_hz,
                                          uint16_t amplitude,
                                          uint32_t dwell_ms)
{
  return StartSweep(output,
                    AD5687_AppTriangleTable,
                    AD5687_APP_WAVE_TABLE_SIZE,
                    low_hz,
                    high_hz,
                    frequency_step_hz,
                    amplitude,
                    dwell_ms);
}

/************************************************************
 * Function :       AD5687_AppSweepSquare
 * Comment  :       启动AD5687方波扫频任务
 * Parameter:       output: CV1~CV4输出口; low_hz/high_hz: 扫频范围Hz; frequency_step_hz: 频率步进Hz; amplitude: 12位幅度; dwell_ms: 每步停留时间
 * Return   :       HAL_OK表示成功, 其他值表示失败
 * Date     :       2026-06-12 V1
************************************************************/
HAL_StatusTypeDef AD5687_AppSweepSquare(AD5687_OutputTypeDef output,
                                        uint32_t low_hz,
                                        uint32_t high_hz,
                                        uint32_t frequency_step_hz,
                                        uint16_t amplitude,
                                        uint32_t dwell_ms)
{
  return StartSweep(output,
                    AD5687_AppSquareTable,
                    AD5687_APP_WAVE_TABLE_SIZE,
                    low_hz,
                    high_hz,
                    frequency_step_hz,
                    amplitude,
                    dwell_ms);
}

/************************************************************
 * Function :       AD5687_AppSweepSawtooth
 * Comment  :       启动AD5687锯齿波扫频任务
 * Parameter:       output: CV1~CV4输出口; low_hz/high_hz: 扫频范围Hz; frequency_step_hz: 频率步进Hz; amplitude: 12位幅度; dwell_ms: 每步停留时间
 * Return   :       HAL_OK表示成功, 其他值表示失败
 * Date     :       2026-06-12 V1
************************************************************/
HAL_StatusTypeDef AD5687_AppSweepSawtooth(AD5687_OutputTypeDef output,
                                          uint32_t low_hz,
                                          uint32_t high_hz,
                                          uint32_t frequency_step_hz,
                                          uint16_t amplitude,
                                          uint32_t dwell_ms)
{
  return StartSweep(output,
                    AD5687_AppSawtoothTable,
                    AD5687_APP_WAVE_TABLE_SIZE,
                    low_hz,
                    high_hz,
                    frequency_step_hz,
                    amplitude,
                    dwell_ms);
}

/************************************************************
 * Function :       AD5687_AppSweepArbitrary
 * Comment  :       启动AD5687任意波扫频任务
 * Parameter:       output: CV1~CV4输出口; wave_table: 波形表; table_size: 点数; low_hz/high_hz: 扫频范围Hz; frequency_step_hz: 频率步进Hz; amplitude: 12位幅度; dwell_ms: 每步停留时间
 * Return   :       HAL_OK表示成功, 其他值表示失败
 * Date     :       2026-06-12 V1
************************************************************/
HAL_StatusTypeDef AD5687_AppSweepArbitrary(AD5687_OutputTypeDef output,
                                           const uint16_t *wave_table,
                                           uint16_t table_size,
                                           uint32_t low_hz,
                                           uint32_t high_hz,
                                           uint32_t frequency_step_hz,
                                           uint16_t amplitude,
                                           uint32_t dwell_ms)
{
  return StartSweep(output,
                    wave_table,
                    table_size,
                    low_hz,
                    high_hz,
                    frequency_step_hz,
                    amplitude,
                    dwell_ms);
}

/************************************************************
 * Function :       AD5687_AppStop
 * Comment  :       停止AD5687应用层后台波形任务
 * Parameter:       null
 * Return   :       null
 * Date     :       2026-06-12 V1
************************************************************/
void AD5687_AppStop(void)
{
  s_wave_task.table = NULL;
  s_wave_task.size = 0U;
  s_wave_task.amplitude = 0U;
  s_wave_task.index = 0U;
  s_wave_task.frequency_hz = 0U;
  s_wave_task.last_sample_cycle = 0U;
  s_wave_task.phase_accumulator = 0ULL;
  s_wave_task.low_hz = 0U;
  s_wave_task.high_hz = 0U;
  s_wave_task.frequency_step_hz = 0U;
  s_wave_task.dwell_ms = 0U;
  s_wave_task.last_sweep_tick = 0U;
  s_wave_task.output = AD5687_OUTPUT_CV1;
  s_wave_task.sweep_direction = -1;
  s_wave_task.running = 0U;
  s_wave_task.sweep_running = 0U;
}

/************************************************************
 * Function :       AD5687_AppProcess
 * Comment  :       AD5687应用层后台处理函数, 放在while(1)中周期调用
 * Parameter:       null
 * Return   :       null
 * Date     :       2026-06-12 V1
************************************************************/
void AD5687_AppProcess(void)
{
  if (s_wave_task.running != 0U)
  {
    UpdateSweep();
    (void)StepWave(0U);
  }
}

/* 按需初始化AD5687应用层。 */
static HAL_StatusTypeDef EnsureInit(void)
{
  if (s_ad5687_initialized != 0U)
  {
    return HAL_OK;
  }

  return AD5687_AppInit();
}

/* 将CV输出编号映射到级联器件和通道。 */
static HAL_StatusTypeDef GetOutputAddress(AD5687_OutputTypeDef output,
                                          AD5687_DeviceTypeDef *device,
                                          AD5687_ChannelTypeDef *channel)
{
  if ((device == NULL) || (channel == NULL))
  {
    return HAL_ERROR;
  }

  switch (output)
  {
    case AD5687_OUTPUT_CV1:
      *device = AD5687_DEVICE_CV1_CV2;
      *channel = AD5687_CHANNEL_A;
      break;

    case AD5687_OUTPUT_CV2:
      *device = AD5687_DEVICE_CV1_CV2;
      *channel = AD5687_CHANNEL_B;
      break;

    case AD5687_OUTPUT_CV3:
      *device = AD5687_DEVICE_CV3_CV4;
      *channel = AD5687_CHANNEL_A;
      break;

    case AD5687_OUTPUT_CV4:
      *device = AD5687_DEVICE_CV3_CV4;
      *channel = AD5687_CHANNEL_B;
      break;

    default:
      return HAL_ERROR;
  }

  return HAL_OK;
}

/* 启动指定波表的周期输出。 */
static HAL_StatusTypeDef StartWave(AD5687_OutputTypeDef output,
                                   const uint16_t *wave_table,
                                   uint16_t table_size,
                                   uint32_t output_hz,
                                   uint16_t amplitude)
{
  HAL_StatusTypeDef status;
  AD5687_DeviceTypeDef device;
  AD5687_ChannelTypeDef channel;

  if ((wave_table == NULL) ||
      (table_size == 0U) ||
      (output_hz == 0U) ||
      (((uint64_t)table_size * (uint64_t)output_hz) >
       AD5687_APP_SAMPLE_RATE_HZ) ||
      (amplitude > AD5687_MAX_CODE))
  {
    return HAL_ERROR;
  }

  status = GetOutputAddress(output, &device, &channel);
  if (status != HAL_OK)
  {
    return status;
  }

  status = EnsureInit();
  if (status != HAL_OK)
  {
    return status;
  }

  s_wave_task.table = wave_table;
  s_wave_task.size = table_size;
  s_wave_task.amplitude = amplitude;
  s_wave_task.index = 0U;
  s_wave_task.frequency_hz = output_hz;
  s_wave_task.last_sample_cycle = HAL_GetTick();
  s_wave_task.phase_accumulator = 0ULL;
  s_wave_task.low_hz = 0U;
  s_wave_task.high_hz = 0U;
  s_wave_task.frequency_step_hz = 0U;
  s_wave_task.dwell_ms = 0U;
  s_wave_task.last_sweep_tick = 0U;
  s_wave_task.output = output;
  s_wave_task.sweep_direction = -1;
  s_wave_task.running = 1U;
  s_wave_task.sweep_running = 0U;

  return StepWave(1U);
}

/* 启动指定波表的往返扫频。 */
static HAL_StatusTypeDef StartSweep(AD5687_OutputTypeDef output,
                                    const uint16_t *wave_table,
                                    uint16_t table_size,
                                    uint32_t low_hz,
                                    uint32_t high_hz,
                                    uint32_t frequency_step_hz,
                                    uint16_t amplitude,
                                    uint32_t dwell_ms)
{
  HAL_StatusTypeDef status;

  if ((low_hz == 0U) ||
      (high_hz <= low_hz) ||
      (frequency_step_hz == 0U) ||
      (((uint64_t)table_size * (uint64_t)high_hz) >
       AD5687_APP_SAMPLE_RATE_HZ) ||
      (dwell_ms == 0U))
  {
    return HAL_ERROR;
  }

  status = StartWave(output,
                     wave_table,
                     table_size,
                     low_hz,
                     amplitude);
  if (status != HAL_OK)
  {
    return status;
  }

  s_wave_task.low_hz = low_hz;
  s_wave_task.high_hz = high_hz;
  s_wave_task.frequency_step_hz = frequency_step_hz;
  s_wave_task.dwell_ms = dwell_ms;
  s_wave_task.last_sweep_tick = HAL_GetTick();
  s_wave_task.sweep_direction = 1;
  s_wave_task.sweep_running = 1U;

  return HAL_OK;
}

/* 按经过时间输出应送波形点。 */
static HAL_StatusTypeDef StepWave(uint8_t force)
{
  uint32_t now_cycle;
  uint32_t elapsed_cycles;
  uint32_t samples_due;
  uint16_t output_code;
  HAL_StatusTypeDef status;
  AD5687_DeviceTypeDef device;
  AD5687_ChannelTypeDef channel;

  if (s_wave_task.running == 0U)
  {
    return HAL_OK;
  }

  status = GetOutputAddress(s_wave_task.output, &device, &channel);
  if (status != HAL_OK)
  {
    AD5687_AppStop();
    return status;
  }

  now_cycle = AD5687_AppGetCycle();

  if (force != 0U)
  {
    samples_due = 1U;
    s_wave_task.last_sample_cycle = now_cycle;
  }
  else
  {
    elapsed_cycles = now_cycle - s_wave_task.last_sample_cycle;
    if (elapsed_cycles == 0U)
    {
      return HAL_OK;
    }

    s_wave_task.last_sample_cycle = now_cycle;

    s_wave_task.phase_accumulator +=
      (uint64_t)elapsed_cycles *
      (uint64_t)s_wave_task.size *
      (uint64_t)s_wave_task.frequency_hz;

    samples_due =
      (uint32_t)(s_wave_task.phase_accumulator / (uint64_t)SystemCoreClock);

    s_wave_task.phase_accumulator %= (uint64_t)SystemCoreClock;

    if (samples_due == 0U)
    {
      return HAL_OK;
    }

    if (samples_due > s_wave_task.size)
    {
      samples_due = s_wave_task.size;
    }
  }

  while (samples_due > 0U)
  {
    output_code = ScaleCode(s_wave_task.table[s_wave_task.index],
                            s_wave_task.amplitude);

    status = AD5687_WriteAndUpdateRaw(&s_ad5687,
                                      device,
                                      channel,
                                      output_code);
    if (status != HAL_OK)
    {
      AD5687_AppStop();
      return status;
    }

    s_wave_task.index++;
    if (s_wave_task.index >= s_wave_task.size)
    {
      s_wave_task.index = 0U;
    }

    samples_due--;
  }

  return HAL_OK;
}

/* 更新扫频方向和当前频率。 */
static void UpdateSweep(void)
{
  uint32_t now_tick;
  uint32_t next_hz;

  if (s_wave_task.sweep_running == 0U)
  {
    return;
  }

  now_tick = HAL_GetTick();
  if ((now_tick - s_wave_task.last_sweep_tick) < s_wave_task.dwell_ms)
  {
    return;
  }
  s_wave_task.last_sweep_tick = now_tick;

  next_hz = s_wave_task.frequency_hz;
  if (s_wave_task.sweep_direction > 0)
  {
    if ((next_hz >= s_wave_task.high_hz) ||
        (s_wave_task.frequency_step_hz >=
         (s_wave_task.high_hz - next_hz)))
    {
      next_hz = s_wave_task.high_hz;
      s_wave_task.sweep_direction = -1;
    }
    else
    {
      next_hz += s_wave_task.frequency_step_hz;
    }
  }
  else
  {
    if ((next_hz <= s_wave_task.low_hz) ||
        (s_wave_task.frequency_step_hz >=
         (next_hz - s_wave_task.low_hz)))
    {
      next_hz = s_wave_task.low_hz;
      s_wave_task.sweep_direction = 1;
    }
    else
    {
      next_hz -= s_wave_task.frequency_step_hz;
    }
  }

  s_wave_task.frequency_hz = next_hz;
}

/* 按目标幅度缩放波表码值。 */
static uint16_t ScaleCode(uint16_t code, uint16_t amplitude)
{
  uint32_t value;

  if (code > AD5687_MAX_CODE)
  {
    code = AD5687_MAX_CODE;
  }

  value = ((uint32_t)code * (uint32_t)amplitude) +
          (AD5687_MAX_CODE / 2U);

  return (uint16_t)(value / AD5687_MAX_CODE);
}

/* 初始化DWT周期计数时基。 */
static void AD5687_AppTimebaseInit(void)
{
  SystemCoreClockUpdate();
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/* 获取当前DWT周期计数。 */
static uint32_t AD5687_AppGetCycle(void)
{
  return DWT->CYCCNT;
}
