#include "adc_app.h"

#undef GPIOA
#undef GPIOB
#undef GPIOC
#undef GPIOD
#undef GPIOE
#undef GPIOH
#include "ti_msp_dl_config.h"

#include <stdio.h>

#define ADC_APP_DEFAULT_VREF_V       3.3f
#define ADC_APP_MAX_CODE             4095.0f
#define ADC_APP_FIRST_HALF_MASK      0x01U
#define ADC_APP_SECOND_HALF_MASK     0x02U
#define ADC_APP_DMA_INTERRUPT_MASK                                           \
    (DL_DMA_INTERRUPT_CHANNEL0 | DL_DMA_INTERRUPT_CHANNEL1 |                  \
     DL_DMA_FULL_CH_INTERRUPT_EARLY_CHANNEL0 |                               \
     DL_DMA_FULL_CH_INTERRUPT_EARLY_CHANNEL1 |                               \
     DL_DMA_INTERRUPT_ADDR_ERROR | DL_DMA_INTERRUPT_DATA_ERROR)

static uint16_t s_raw_buffer[ADC_APP_CHANNEL_COUNT][ADC_APP_SAMPLE_LENGTH]
    __attribute__((aligned(4)));
static float s_voltage_buffer[ADC_APP_CHANNEL_COUNT][ADC_APP_SAMPLE_LENGTH];

static float s_vref_v = ADC_APP_DEFAULT_VREF_V;
static float s_sample_rate_hz = ADC_APP_MAX_SAMPLE_RATE_HZ;
static float s_max_harmonic_hz;
static uint32_t s_timer_load_value = ADC_SAMPLE_TIMER_INST_LOAD_VALUE;
static uint8_t s_print_raw;
static ADC_SampleMode_t s_mode = ADC_MODE_ONESHOT;
static uint8_t s_initialized;
static uint8_t s_active_channel = ADC_APP_CHANNEL_VOLTAGE;

static volatile uint8_t s_running;
static volatile uint8_t s_data_ready;
static volatile uint8_t s_ready_mask;
static volatile uint8_t s_dma_seen[ADC_APP_CHANNEL_COUNT];
static volatile uint8_t s_sync_enable = 1U;
static volatile uint8_t s_buffer_locked;
static volatile uint8_t s_locked_half_mask;
static volatile uint8_t s_overrun_count;
static volatile uint8_t s_dma_error;
static volatile uint32_t s_frame_sequence;

static ADC_HalfReady_t s_last_ready_half = ADC_HALF_NONE;
static uint8_t s_frame_converted;

static void ADC_Sample_IncrementOverrun(void)
{
    if (s_overrun_count < UINT8_MAX) {
        s_overrun_count++;
    }
}

static void ADC_Sample_StopHardware(void)
{
    DL_TimerG_stopCounter(ADC_SAMPLE_TIMER_INST);
    DL_DMA_disableChannel(DMA, DMA_ADC_VOLTAGE_CHAN_ID);
    DL_DMA_disableChannel(DMA, DMA_ADC_CURRENT_CHAN_ID);
    DL_ADC12_stopConversion(ADC_VOLTAGE_INST);
    DL_ADC12_stopConversion(ADC_CURRENT_INST);
    s_running = 0U;
}

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

    /*
     * Publish only the newest completed half. If the previous half was not
     * consumed before this point, it is no longer a safe real-time result.
     */
    s_ready_mask = half_mask;
    s_data_ready = 1U;
    s_frame_sequence++;
}

static void ADC_Sample_RecordDmaHalf(uint8_t channel, uint8_t half_mask)
{
    if ((s_running == 0U) || (channel >= ADC_APP_CHANNEL_COUNT)) {
        return;
    }

    s_dma_seen[channel] |= half_mask;
    ADC_Sample_PublishPairedHalf(half_mask);
}

static void ADC_Sample_ConvertRange(uint16_t offset, uint16_t length)
{
    uint32_t end = (uint32_t)offset + (uint32_t)length;
    uint32_t channel;
    uint32_t index;
    float scale = s_vref_v / ADC_APP_MAX_CODE;

    if (end > ADC_APP_SAMPLE_LENGTH) {
        return;
    }

    for (channel = 0U; channel < ADC_APP_CHANNEL_COUNT; channel++) {
        for (index = offset; index < end; index++) {
            s_voltage_buffer[channel][index] =
                (float)s_raw_buffer[channel][index] * scale;
        }
    }
}

HAL_StatusTypeDef ADC_Sample_Init(float vref_v,
                                  float sample_rate_hz,
                                  float max_harmonic_hz,
                                  uint8_t print_raw,
                                  ADC_SampleMode_t mode)
{
    uint32_t timer_cycles;

    if ((vref_v <= 0.0f) ||
        (sample_rate_hz < ADC_APP_MIN_SAMPLE_RATE_HZ) ||
        (sample_rate_hz > ADC_APP_MAX_SAMPLE_RATE_HZ) ||
        (max_harmonic_hz < 0.0f) ||
        (max_harmonic_hz > (sample_rate_hz * 0.5f)) ||
        ((print_raw != ADC_APP_PRINT_RAW_OFF) &&
         (print_raw != ADC_APP_PRINT_RAW_ON)) ||
        ((mode != ADC_MODE_ONESHOT) && (mode != ADC_MODE_CONTINUOUS))) {
        return HAL_ERROR;
    }

    timer_cycles =
        (uint32_t)(((float)CPUCLK_FREQ / sample_rate_hz) + 0.5f);
    if ((timer_cycles == 0U) || (timer_cycles > 65536U)) {
        return HAL_ERROR;
    }

    ADC_Sample_StopHardware();
    NVIC_DisableIRQ(DMA_INT_IRQn);

    s_vref_v = vref_v;
    s_timer_load_value = timer_cycles - 1U;
    s_sample_rate_hz = (float)CPUCLK_FREQ / (float)timer_cycles;
    s_max_harmonic_hz = max_harmonic_hz;
    s_print_raw = print_raw;
    s_mode = mode;
    s_initialized = 1U;
    s_active_channel = ADC_APP_CHANNEL_VOLTAGE;
    ADC_Sample_ResetState();

    DL_TimerG_setLoadValue(ADC_SAMPLE_TIMER_INST, s_timer_load_value);
    DL_DMA_clearInterruptStatus(DMA, ADC_APP_DMA_INTERRUPT_MASK);
    DL_DMA_enableInterrupt(
        DMA, DL_DMA_INTERRUPT_ADDR_ERROR | DL_DMA_INTERRUPT_DATA_ERROR);
    NVIC_ClearPendingIRQ(DMA_INT_IRQn);
    NVIC_EnableIRQ(DMA_INT_IRQn);

    return HAL_OK;
}

HAL_StatusTypeDef ADC_Sample_Start(void)
{
    if ((s_initialized == 0U) || (s_running != 0U)) {
        return HAL_ERROR;
    }

    ADC_Sample_StopHardware();
    ADC_Sample_ResetState();

    DL_DMA_clearInterruptStatus(DMA, ADC_APP_DMA_INTERRUPT_MASK);

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
        DMA, DMA_ADC_VOLTAGE_CHAN_ID, ADC_APP_SAMPLE_LENGTH);

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
        DMA, DMA_ADC_CURRENT_CHAN_ID, ADC_APP_SAMPLE_LENGTH);

    DL_ADC12_enableConversions(ADC_VOLTAGE_INST);
    DL_ADC12_enableConversions(ADC_CURRENT_INST);
    DL_DMA_enableChannel(DMA, DMA_ADC_VOLTAGE_CHAN_ID);
    DL_DMA_enableChannel(DMA, DMA_ADC_CURRENT_CHAN_ID);

    s_running = 1U;
    DL_TimerG_setLoadValue(ADC_SAMPLE_TIMER_INST, s_timer_load_value);
    DL_TimerG_startCounter(ADC_SAMPLE_TIMER_INST);

    return HAL_OK;
}

void ADC_Sample_Stop(void)
{
    ADC_Sample_StopHardware();
}

void ADC_Sample_Process(void)
{
    if ((s_mode == ADC_MODE_ONESHOT) &&
        (s_data_ready != 0U) &&
        (s_frame_converted == 0U)) {
        ADC_Sample_ConvertRange(0U, ADC_APP_SAMPLE_LENGTH);
        s_frame_converted = 1U;

        if (s_print_raw == ADC_APP_PRINT_RAW_ON) {
            ADC_Sample_PrintRaw();
        }
    }
}

void ADC_Sample_ConfigSync(uint8_t enable)
{
    s_sync_enable = (enable != 0U) ? 1U : 0U;
}

HAL_StatusTypeDef ADC_Sample_SetActiveChannel(uint8_t channel)
{
    if (channel >= ADC_APP_CHANNEL_COUNT) {
        return HAL_ERROR;
    }

    s_active_channel = channel;
    return HAL_OK;
}

uint8_t ADC_Sample_GetActiveChannel(void)
{
    return s_active_channel;
}

float ADC_Sample_GetSampleRateHz(void)
{
    return s_sample_rate_hz;
}

uint8_t ADC_Sample_IsRunning(void)
{
    return s_running;
}

uint8_t ADC_Sample_DataReady(void)
{
    return s_data_ready;
}

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

uint16_t ADC_Sample_GetReadyOffset(void)
{
    return (s_last_ready_half == ADC_HALF_SECOND) ?
               ADC_APP_HALF_LENGTH :
               0U;
}

uint16_t ADC_Sample_GetReadyLength(void)
{
    return (s_last_ready_half == ADC_HALF_NONE) ?
               0U :
               ADC_APP_HALF_LENGTH;
}

uint32_t ADC_Sample_GetFrameSequence(void)
{
    return s_frame_sequence;
}

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

void ADC_Sample_BufferUnlock(void)
{
    s_buffer_locked = 0U;
    s_locked_half_mask = 0U;
}

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

uint8_t ADC_Sample_HasError(void)
{
    return s_dma_error;
}

const uint16_t *ADC_Sample_GetRawData(void)
{
    return s_raw_buffer[s_active_channel];
}

const uint16_t *ADC_Sample_GetRawDataByChannel(uint8_t channel)
{
    if (channel >= ADC_APP_CHANNEL_COUNT) {
        channel = ADC_APP_CHANNEL_VOLTAGE;
    }
    return s_raw_buffer[channel];
}

const uint16_t *ADC_Sample_GetReadyRawDataByChannel(uint8_t channel)
{
    return ADC_Sample_GetRawDataByChannel(channel) +
           ADC_Sample_GetReadyOffset();
}

const float *ADC_Sample_GetVoltageData(void)
{
    return s_voltage_buffer[s_active_channel];
}

const float *ADC_Sample_GetVoltageDataByChannel(uint8_t channel)
{
    if (channel >= ADC_APP_CHANNEL_COUNT) {
        channel = ADC_APP_CHANNEL_VOLTAGE;
    }
    return s_voltage_buffer[channel];
}

void ADC_Sample_ConvertHalf(ADC_HalfReady_t half)
{
    if (half == ADC_HALF_FIRST) {
        ADC_Sample_ConvertRange(0U, ADC_APP_HALF_LENGTH);
    } else if (half == ADC_HALF_SECOND) {
        ADC_Sample_ConvertRange(
            ADC_APP_HALF_LENGTH, ADC_APP_HALF_LENGTH);
    }
}

void ADC_Sample_PrintRaw(void)
{
    uint32_t index;

    for (index = 0U; index < ADC_APP_SAMPLE_LENGTH; index++) {
        printf("%lu,%u,%u\r\n",
               (unsigned long)index,
               (unsigned int)
                   s_raw_buffer[ADC_APP_CHANNEL_VOLTAGE][index],
               (unsigned int)
                   s_raw_buffer[ADC_APP_CHANNEL_CURRENT][index]);
    }
}

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
