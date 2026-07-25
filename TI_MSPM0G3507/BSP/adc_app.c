#include "adc_app.h"

#undef GPIOA
#undef GPIOB
#undef GPIOC
#undef GPIOD
#undef GPIOE
#undef GPIOH
#include "ti_msp_dl_config.h"

#include <stdio.h>

#define ADC_APP_DEFAULT_VREF_V       3.3f    /* 默认实测参考电压。 */
#define ADC_APP_DEFAULT_SAMPLE_RATE  100000.0f /* 默认每通道采样率。 */
#define ADC_APP_MAX_CODE             4095.0f /* 12位ADC最大码值。 */
#define ADC_APP_FIRST_HALF_MASK      0x01U   /* 连续采集前半区标志。 */
#define ADC_APP_SECOND_HALF_MASK     0x02U   /* 连续采集后半区标志。 */
#define ADC_APP_DMA_INTERRUPT_MASK                                           \
    (DL_DMA_INTERRUPT_CHANNEL0 | DL_DMA_INTERRUPT_CHANNEL1 |                  \
     DL_DMA_FULL_CH_INTERRUPT_EARLY_CHANNEL0 |                               \
     DL_DMA_FULL_CH_INTERRUPT_EARLY_CHANNEL1 |                               \
     DL_DMA_INTERRUPT_ADDR_ERROR | DL_DMA_INTERRUPT_DATA_ERROR) /* DMA中断集合。 */

/* 双通道ADC原始码缓冲区。 */
static uint16_t s_raw_buffer[ADC_APP_CHANNEL_COUNT][ADC_APP_MAX_SAMPLE_COUNT]
    __attribute__((aligned(4)));
static float s_voltage_buffer[ADC_APP_CHANNEL_COUNT][ADC_APP_MAX_SAMPLE_COUNT]; /* 各通道电压缓冲区。 */

static float s_vref_v = ADC_APP_DEFAULT_VREF_V;                /* 实测参考电压。 */
static float s_sample_rate_hz = ADC_APP_DEFAULT_SAMPLE_RATE;    /* 实际采样率。 */
static uint32_t s_timer_load_value = ADC_SAMPLE_TIMER_INST_LOAD_VALUE; /* 定时器装载值。 */
static uint16_t s_sample_count = ADC_APP_MAX_SAMPLE_COUNT;      /* 每通道采样点数。 */
static uint8_t s_print_raw;                                    /* 原始码打印开关。 */
static ADC_SampleMode_t s_mode = ADC_MODE_ONESHOT;              /* 当前采集模式。 */
static ADC_QualityMode_t s_quality = ADC_QUALITY_RAW;           /* 当前质量模式。 */
static uint8_t s_initialized;                                  /* 初始化完成标志。 */
static uint8_t s_active_channel = ADC_APP_CHANNEL_VOLTAGE;      /* 默认分析通道。 */

static volatile uint8_t s_running;                              /* ADC运行标志。 */
static volatile uint8_t s_data_ready;                           /* 数据就绪标志。 */
static volatile uint8_t s_ready_mask;                           /* 连续模式就绪半区。 */
static volatile uint8_t s_dma_seen[ADC_APP_CHANNEL_COUNT];      /* 两路DMA同步标志。 */
static volatile uint8_t s_sync_enable = 1U;                     /* 覆盖检测开关。 */
static volatile uint8_t s_buffer_locked;                        /* 消费者锁定标志。 */
static volatile uint8_t s_locked_half_mask;                     /* 消费者锁定半区。 */
static volatile uint8_t s_overrun_count;                        /* 数据覆盖计数。 */
static volatile uint8_t s_dma_error;                            /* DMA错误标志。 */
static volatile uint32_t s_frame_sequence;                      /* 帧或半区序号。 */

static ADC_HalfReady_t s_last_ready_half = ADC_HALF_NONE;       /* 最近取出的半区。 */
static uint8_t s_frame_converted;                               /* 单帧电压换算标志。 */

/* 饱和递增连续采集覆盖计数。 */
static void ADC_Sample_IncrementOverrun(void)
{
    if (s_overrun_count < UINT8_MAX) {
        s_overrun_count++;
    }
}

/* 停止定时器、两路DMA和两路ADC。 */
static void ADC_Sample_StopHardware(void)
{
    DL_TimerG_stopCounter(ADC_SAMPLE_TIMER_INST);
    DL_DMA_disableChannel(DMA, DMA_ADC_VOLTAGE_CHAN_ID);
    DL_DMA_disableChannel(DMA, DMA_ADC_CURRENT_CHAN_ID);
    DL_ADC12_stopConversion(ADC_VOLTAGE_INST);
    DL_ADC12_stopConversion(ADC_CURRENT_INST);
    s_running = 0U;
}

/* 清除一次采集关联的运行状态。 */
static void ADC_Sample_ResetState(void)
{
    s_data_ready = 0U;
    s_ready_mask = 0U;
    s_dma_seen[ADC_APP_CHANNEL_VOLTAGE] = 0U;
    s_dma_seen[ADC_APP_CHANNEL_CURRENT] = 0U;
    s_buffer_locked = 0U;
    s_locked_half_mask = 0U;
    s_overrun_count = 0U;
    s_dma_error = 0U;
    s_frame_sequence = 0U;
    s_last_ready_half = ADC_HALF_NONE;
    s_frame_converted = 0U;
}

/* 两路DMA到达同一半区后发布就绪通知。 */
static void ADC_Sample_PublishPairedHalf(uint8_t half_mask)
{
    uint8_t paired;

    paired = s_dma_seen[ADC_APP_CHANNEL_VOLTAGE] &
             s_dma_seen[ADC_APP_CHANNEL_CURRENT] & half_mask;
    if (paired == 0U) {
        return;
    }

    s_dma_seen[ADC_APP_CHANNEL_VOLTAGE] &= (uint8_t)~half_mask;
    s_dma_seen[ADC_APP_CHANNEL_CURRENT] &= (uint8_t)~half_mask;

    if (s_mode == ADC_MODE_ONESHOT) {
        if (half_mask == ADC_APP_SECOND_HALF_MASK) {
            ADC_Sample_StopHardware();
            s_data_ready = 1U;
            s_frame_sequence++;
        }
        return;
    }

    if ((s_ready_mask != 0U) ||
        ((s_sync_enable != 0U) &&
         (s_buffer_locked != 0U) &&
         ((s_locked_half_mask & half_mask) != 0U))) {
        ADC_Sample_IncrementOverrun();
    }

    /* 只发布最新半区，未及时消费的旧半区计为覆盖。 */
    s_ready_mask = half_mask;
    s_data_ready = 1U;
    s_frame_sequence++;
}

/* 记录一路DMA半区事件并尝试配对。 */
static void ADC_Sample_RecordDmaHalf(uint8_t channel, uint8_t half_mask)
{
    if ((s_running == 0U) || (channel >= ADC_APP_CHANNEL_COUNT)) {
        return;
    }

    s_dma_seen[channel] |= half_mask;
    ADC_Sample_PublishPairedHalf(half_mask);
}

/* 返回三个12位ADC码的中值。 */
static uint16_t ADC_Sample_Median3(uint16_t a, uint16_t b, uint16_t c)
{
    if (a > b) {
        uint16_t swap = a;
        a = b;
        b = swap;
    }
    if (b > c) {
        uint16_t swap = b;
        b = c;
        c = swap;
    }
    if (a > b) {
        b = a;
    }
    return b;
}

/* 将指定范围的原始码换算成电压。 */
static void ADC_Sample_ConvertRange(uint16_t offset, uint16_t length)
{
    uint32_t end = (uint32_t)offset + (uint32_t)length;
    uint32_t channel;
    uint32_t index;
    float scale = s_vref_v / ADC_APP_MAX_CODE;

    if (end > s_sample_count) {
        return;
    }

    for (channel = 0U; channel < ADC_APP_CHANNEL_COUNT; channel++) {
        for (index = offset; index < end; index++) {
            uint16_t code = s_raw_buffer[channel][index];

            if ((s_quality == ADC_QUALITY_MEDIAN3) &&
                (index > 0U) &&
                ((index + 1U) < s_sample_count)) {
                code = ADC_Sample_Median3(
                    s_raw_buffer[channel][index - 1U],
                    code,
                    s_raw_buffer[channel][index + 1U]);
            }
            s_voltage_buffer[channel][index] =
                (float)code * scale;
        }
    }
}

/* 填充适合高速波形采集的默认参数。 */
void ADC_Sample_GetDefaultConfig(ADC_SampleConfig_t *config)
{
    if (config == NULL) {
        return;
    }

    config->measured_vref_v = ADC_APP_DEFAULT_VREF_V;
    config->sample_rate_hz = ADC_APP_DEFAULT_SAMPLE_RATE;
    config->sample_count = ADC_APP_MAX_SAMPLE_COUNT;
    config->active_channel = ADC_APP_CHANNEL_VOLTAGE;
    config->mode = ADC_MODE_ONESHOT;
    config->quality = ADC_QUALITY_RAW;
    config->print_raw = 0U;
}

/* 校验参数并配置运行时采样率和DMA状态。 */
HAL_StatusTypeDef ADC_Sample_Init(const ADC_SampleConfig_t *config)
{
    uint32_t timer_cycles;

    if ((config == NULL) ||
        (config->measured_vref_v <= 0.0f) ||
        (config->sample_rate_hz < ADC_APP_MIN_SAMPLE_RATE_HZ) ||
        (config->sample_rate_hz > ADC_APP_MAX_SAMPLE_RATE_HZ) ||
        (config->sample_count < 2U) ||
        (config->sample_count > ADC_APP_MAX_SAMPLE_COUNT) ||
        (config->active_channel >= ADC_APP_CHANNEL_COUNT) ||
        ((config->mode != ADC_MODE_ONESHOT) &&
         (config->mode != ADC_MODE_CONTINUOUS)) ||
        ((config->quality != ADC_QUALITY_RAW) &&
         (config->quality != ADC_QUALITY_MEDIAN3))) {
        return HAL_ERROR;
    }
    if ((config->mode == ADC_MODE_CONTINUOUS) &&
        ((config->sample_count & 1U) != 0U)) {
        return HAL_ERROR;
    }

    timer_cycles =
        (uint32_t)(((float)CPUCLK_FREQ / config->sample_rate_hz) + 0.5f);
    if ((timer_cycles == 0U) || (timer_cycles > 65536U)) {
        return HAL_ERROR;
    }

    ADC_Sample_StopHardware();
    NVIC_DisableIRQ(DMA_INT_IRQn);

    s_vref_v = config->measured_vref_v;
    s_timer_load_value = timer_cycles - 1U;
    s_sample_rate_hz = (float)CPUCLK_FREQ / (float)timer_cycles;
    s_sample_count = config->sample_count;
    s_print_raw = (config->print_raw != 0U) ? 1U : 0U;
    s_mode = config->mode;
    s_quality = config->quality;
    s_initialized = 1U;
    s_active_channel = (uint8_t)config->active_channel;
    ADC_Sample_ResetState();

    DL_TimerG_setLoadValue(ADC_SAMPLE_TIMER_INST, s_timer_load_value);
    DL_DMA_clearInterruptStatus(DMA, ADC_APP_DMA_INTERRUPT_MASK);
    DL_DMA_enableInterrupt(
        DMA, DL_DMA_INTERRUPT_ADDR_ERROR | DL_DMA_INTERRUPT_DATA_ERROR);
    NVIC_ClearPendingIRQ(DMA_INT_IRQn);
    NVIC_EnableIRQ(DMA_INT_IRQn);

    return HAL_OK;
}

/* 按当前模式启动同步双ADC采集。 */
HAL_StatusTypeDef ADC_Sample_Start(void)
{
    DL_DMA_TRANSFER_MODE dma_transfer_mode;

    if ((s_initialized == 0U) || (s_running != 0U)) {
        return HAL_ERROR;
    }

    ADC_Sample_StopHardware();
    ADC_Sample_ResetState();

    DL_DMA_clearInterruptStatus(DMA, ADC_APP_DMA_INTERRUPT_MASK);

    /* 单帧禁止DMA回卷，连续模式才启用循环传输。 */
    dma_transfer_mode =
        (s_mode == ADC_MODE_ONESHOT) ?
            DL_DMA_SINGLE_TRANSFER_MODE :
            DL_DMA_FULL_CH_REPEAT_SINGLE_TRANSFER_MODE;
    DL_DMA_setTransferMode(
        DMA, DMA_ADC_VOLTAGE_CHAN_ID, dma_transfer_mode);
    DL_DMA_setTransferMode(
        DMA, DMA_ADC_CURRENT_CHAN_ID, dma_transfer_mode);

    DL_DMA_setSrcAddr(
        DMA,
        DMA_ADC_VOLTAGE_CHAN_ID,
        DL_ADC12_getMemResultAddress(
            ADC_VOLTAGE_INST, ADC_VOLTAGE_ADCMEM_0));
    DL_DMA_setDestAddr(
        DMA,
        DMA_ADC_VOLTAGE_CHAN_ID,
        (uint32_t)&s_raw_buffer[ADC_APP_CHANNEL_VOLTAGE][0]);
    DL_DMA_setTransferSize(
        DMA, DMA_ADC_VOLTAGE_CHAN_ID, s_sample_count);

    DL_DMA_setSrcAddr(
        DMA,
        DMA_ADC_CURRENT_CHAN_ID,
        DL_ADC12_getMemResultAddress(
            ADC_CURRENT_INST, ADC_CURRENT_ADCMEM_0));
    DL_DMA_setDestAddr(
        DMA,
        DMA_ADC_CURRENT_CHAN_ID,
        (uint32_t)&s_raw_buffer[ADC_APP_CHANNEL_CURRENT][0]);
    DL_DMA_setTransferSize(
        DMA, DMA_ADC_CURRENT_CHAN_ID, s_sample_count);

    DL_ADC12_enableConversions(ADC_VOLTAGE_INST);
    DL_ADC12_enableConversions(ADC_CURRENT_INST);
    DL_DMA_enableChannel(DMA, DMA_ADC_VOLTAGE_CHAN_ID);
    DL_DMA_enableChannel(DMA, DMA_ADC_CURRENT_CHAN_ID);

    s_running = 1U;
    DL_TimerG_setLoadValue(ADC_SAMPLE_TIMER_INST, s_timer_load_value);
    DL_TimerG_startCounter(ADC_SAMPLE_TIMER_INST);

    return HAL_OK;
}

/* 停止当前采集。 */
void ADC_Sample_Stop(void)
{
    ADC_Sample_StopHardware();
}

/* 在主循环换算单帧数据并按配置打印。 */
void ADC_Sample_Process(void)
{
    if ((s_mode == ADC_MODE_ONESHOT) &&
        (s_data_ready != 0U) &&
        (s_frame_converted == 0U)) {
        ADC_Sample_ConvertRange(0U, s_sample_count);
        s_frame_converted = 1U;

        if (s_print_raw != 0U) {
            ADC_Sample_PrintRaw();
        }
    }
}

/* 配置连续模式覆盖检测。 */
void ADC_Sample_ConfigSync(uint8_t enable)
{
    s_sync_enable = (enable != 0U) ? 1U : 0U;
}

/* 设置默认分析通道。 */
HAL_StatusTypeDef ADC_Sample_SetActiveChannel(uint8_t channel)
{
    if (channel >= ADC_APP_CHANNEL_COUNT) {
        return HAL_ERROR;
    }

    s_active_channel = channel;
    return HAL_OK;
}

/* 获取默认分析通道。 */
uint8_t ADC_Sample_GetActiveChannel(void)
{
    return s_active_channel;
}

/* 获取定时器实际采样率。 */
float ADC_Sample_GetSampleRateHz(void)
{
    return s_sample_rate_hz;
}

/* 获取当前每通道采样点数。 */
uint16_t ADC_Sample_GetSampleCount(void)
{
    return s_sample_count;
}

/* 查询采集是否正在运行。 */
uint8_t ADC_Sample_IsRunning(void)
{
    return s_running;
}

/* 查询当前模式是否有数据就绪。 */
uint8_t ADC_Sample_DataReady(void)
{
    return s_data_ready;
}

/* 原子清除单帧就绪通知。 */
void ADC_Sample_Acknowledge(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    s_data_ready = 0U;
    s_ready_mask = 0U;
    if (primask == 0U) {
        __enable_irq();
    }
}

/* 原子取出连续模式最新就绪半区。 */
ADC_HalfReady_t ADC_Sample_GetReadyHalf(void)
{
    uint32_t primask;
    ADC_HalfReady_t ready = ADC_HALF_NONE;

    if (s_mode != ADC_MODE_CONTINUOUS) {
        return ADC_HALF_NONE;
    }

    primask = __get_PRIMASK();
    __disable_irq();

    if ((s_ready_mask & ADC_APP_FIRST_HALF_MASK) != 0U) {
        s_ready_mask &= (uint8_t)~ADC_APP_FIRST_HALF_MASK;
        ready = ADC_HALF_FIRST;
    } else if ((s_ready_mask & ADC_APP_SECOND_HALF_MASK) != 0U) {
        s_ready_mask &= (uint8_t)~ADC_APP_SECOND_HALF_MASK;
        ready = ADC_HALF_SECOND;
    }

    if (s_ready_mask == 0U) {
        s_data_ready = 0U;
    }

    if (primask == 0U) {
        __enable_irq();
    }

    s_last_ready_half = ready;
    return ready;
}

/* 获取最近取出半区的起始下标。 */
uint16_t ADC_Sample_GetReadyOffset(void)
{
    return (s_last_ready_half == ADC_HALF_SECOND) ?
               (s_sample_count / 2U) :
               0U;
}

/* 获取最近取出半区的每通道长度。 */
uint16_t ADC_Sample_GetReadyLength(void)
{
    return (s_last_ready_half == ADC_HALF_NONE) ?
               0U :
               (s_sample_count / 2U);
}

/* 获取完成的帧或半区序号。 */
uint32_t ADC_Sample_GetFrameSequence(void)
{
    return s_frame_sequence;
}

/* 锁定消费者正在读取的连续采集半区。 */
void ADC_Sample_BufferLock(void)
{
    if (s_sync_enable != 0U) {
        if (s_last_ready_half == ADC_HALF_FIRST) {
            s_locked_half_mask = ADC_APP_FIRST_HALF_MASK;
        } else if (s_last_ready_half == ADC_HALF_SECOND) {
            s_locked_half_mask = ADC_APP_SECOND_HALF_MASK;
        } else {
            s_locked_half_mask = 0U;
        }
        s_buffer_locked = 1U;
    }
}

/* 解除连续采集半区锁定。 */
void ADC_Sample_BufferUnlock(void)
{
    s_buffer_locked = 0U;
    s_locked_half_mask = 0U;
}

/* 原子返回并清除连续采集覆盖次数。 */
uint8_t ADC_Sample_IsOverrun(void)
{
    uint32_t primask = __get_PRIMASK();
    uint8_t count;

    __disable_irq();
    count = s_overrun_count;
    s_overrun_count = 0U;
    if (primask == 0U) {
        __enable_irq();
    }

    return count;
}

/* 查询DMA地址或数据错误。 */
uint8_t ADC_Sample_HasError(void)
{
    return s_dma_error;
}

/* 获取默认通道原始码缓冲区。 */
const uint16_t *ADC_Sample_GetRawData(void)
{
    return s_raw_buffer[s_active_channel];
}

/* 获取指定通道原始码，参数非法时返回空指针。 */
const uint16_t *ADC_Sample_GetRawDataByChannel(uint8_t channel)
{
    if (channel >= ADC_APP_CHANNEL_COUNT) {
        return NULL;
    }
    return s_raw_buffer[channel];
}

/* 获取指定通道最近就绪半区的起始地址。 */
const uint16_t *ADC_Sample_GetReadyRawDataByChannel(uint8_t channel)
{
    const uint16_t *data = ADC_Sample_GetRawDataByChannel(channel);

    return (data == NULL) ? NULL : data + ADC_Sample_GetReadyOffset();
}

/* 获取默认通道电压缓冲区。 */
const float *ADC_Sample_GetVoltageData(void)
{
    return s_voltage_buffer[s_active_channel];
}

/* 获取指定通道电压，参数非法时返回空指针。 */
const float *ADC_Sample_GetVoltageDataByChannel(uint8_t channel)
{
    if (channel >= ADC_APP_CHANNEL_COUNT) {
        return NULL;
    }
    return s_voltage_buffer[channel];
}

/* 将指定连续采集半区换算为电压。 */
void ADC_Sample_ConvertHalf(ADC_HalfReady_t half)
{
    uint16_t half_count = s_sample_count / 2U;

    if (half == ADC_HALF_FIRST) {
        ADC_Sample_ConvertRange(0U, half_count);
    } else if (half == ADC_HALF_SECOND) {
        ADC_Sample_ConvertRange(half_count, half_count);
    }
}

/* 打印双通道原始ADC码。 */
void ADC_Sample_PrintRaw(void)
{
    uint32_t index;

    for (index = 0U; index < s_sample_count; index++) {
        printf("%lu,%u,%u\r\n",
               (unsigned long)index,
               (unsigned int)
                   s_raw_buffer[ADC_APP_CHANNEL_VOLTAGE][index],
               (unsigned int)
                   s_raw_buffer[ADC_APP_CHANNEL_CURRENT][index]);
    }
}

/* 处理两路ADC DMA半区、完成和错误事件。 */
void DMA_IRQHandler(void)
{
    DL_DMA_EVENT_IIDX pending;

    do {
        pending = DL_DMA_getPendingInterrupt(DMA);
        switch (pending) {
            case DL_DMA_FULL_CH_EVENT_IIDX_EARLY_IRQ_DMACH0:
                ADC_Sample_RecordDmaHalf(
                    ADC_APP_CHANNEL_VOLTAGE, ADC_APP_FIRST_HALF_MASK);
                break;

            case DL_DMA_FULL_CH_EVENT_IIDX_EARLY_IRQ_DMACH1:
                ADC_Sample_RecordDmaHalf(
                    ADC_APP_CHANNEL_CURRENT, ADC_APP_FIRST_HALF_MASK);
                break;

            case DL_DMA_EVENT_IIDX_DMACH0:
                ADC_Sample_RecordDmaHalf(
                    ADC_APP_CHANNEL_VOLTAGE, ADC_APP_SECOND_HALF_MASK);
                break;

            case DL_DMA_EVENT_IIDX_DMACH1:
                ADC_Sample_RecordDmaHalf(
                    ADC_APP_CHANNEL_CURRENT, ADC_APP_SECOND_HALF_MASK);
                break;

            case DL_DMA_EVENT_IIDX_ADDR_ERROR:
            case DL_DMA_EVENT_IIDX_DATA_ERROR:
                s_dma_error = 1U;
                ADC_Sample_IncrementOverrun();
                ADC_Sample_StopHardware();
                break;

            case DL_DMA_EVENT_IIDX_NO_INTR:
            default:
                break;
        }
    } while (pending != DL_DMA_EVENT_IIDX_NO_INTR);
}
