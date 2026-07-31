#include "g_tjc_ui.h"

#include "g_tjc_plot_scale.h"
#include "usart.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define G_TJC_WAVEFORM_ID          1U
#define G_TJC_WAVEFORM_CHANNEL     0U
#define G_TJC_TX_TIMEOUT_MS        50U
#define G_TJC_BULK_READY_TIMEOUT_MS 250U
#define G_TJC_BULK_DONE_TIMEOUT_MS  250U
#define G_TJC_BULK_TX_MARGIN_MS      20U
#define G_TJC_COMMAND_CAPACITY     4U
#define G_TJC_COMPONENT_ROWS       3U

typedef enum {
    G_TJC_TRANSFER_IDLE = 0U,
    G_TJC_TRANSFER_WAIT_READY,
    G_TJC_TRANSFER_READY,
    G_TJC_TRANSFER_WAIT_DONE,
    G_TJC_TRANSFER_DONE,
    /*
     * The panel may still be in transparent-transfer mode.  No ASCII is
     * allowed after entering this state because it could be consumed as
     * waveform bytes and desynchronise every command that follows.
     */
    G_TJC_TRANSFER_LOCKED
} G_TjcTransferState;

static const uint8_t s_tjc_terminator[3] = {0xFFU, 0xFFU, 0xFFU};
static uint8_t s_rx_byte;
static char s_rx_command[G_TJC_COMMAND_CAPACITY];
static uint8_t s_rx_command_length;
static uint8_t s_rx_terminator_count;
static volatile uint8_t s_pending_mode;
static volatile uint8_t s_active_page;
static volatile uint8_t s_pending_page;
static volatile uint8_t s_transfer_state;
static volatile uint8_t s_reinitialize_safe;
static uint8_t s_plot[G_TJC_PLOT_POINTS];
/*
 * Keeping evaluated voltages here avoids evaluating the harmonic model
 * twice.  USART2 uses blocking/polling TX, not DMA, so the H7 D-cache does
 * not require clean/invalidate maintenance for either plot buffer.
 */
static float s_plot_mv[G_TJC_PLOT_POINTS];

static uint8_t G_TjcUi_IsUart2(const UART_HandleTypeDef *huart)
{
    return ((huart == &huart2) ||
            ((huart != NULL) && (huart->Instance == USART2))) ? 1U : 0U;
}

static uint8_t G_TjcUi_CanAccessMainObjects(void)
{
    return ((s_active_page == (uint8_t)G_TJC_PAGE_UNKNOWN) ||
            (s_active_page == (uint8_t)G_TJC_PAGE_MAIN)) ? 1U : 0U;
}

static uint8_t G_TjcUi_IsPanelError(uint8_t value)
{
    switch (value) {
    case 0x00U: /* Invalid instruction. */
    case 0x02U: /* Invalid component ID. */
    case 0x03U: /* Invalid page ID. */
    case 0x04U: /* Invalid picture ID. */
    case 0x05U: /* Invalid font ID. */
    case 0x06U: /* File operation failed. */
    case 0x09U: /* CRC failed. */
    case 0x11U: /* Invalid baud rate. */
    case 0x12U: /* Invalid waveform ID/channel. */
    case 0x1AU: /* Invalid variable name. */
    case 0x1BU: /* Invalid operation. */
    case 0x1CU: /* Assignment failed. */
    case 0x1DU: /* EEPROM operation failed. */
    case 0x1EU: /* Invalid parameter count. */
    case 0x1FU: /* I/O operation failed. */
    case 0x20U: /* Invalid escape. */
    case 0x23U: /* Variable name too long. */
    case 0x24U: /* Serial receive buffer overflow. */
        return 1U;
    default:
        return 0U;
    }
}

static HAL_StatusTypeDef G_TjcUi_SendCommand(const char *format, ...)
{
    uint8_t buffer[64];
    int length;
    va_list arguments;

    if (format == NULL) return HAL_ERROR;
    if (s_transfer_state != (uint8_t)G_TJC_TRANSFER_IDLE) {
        return HAL_BUSY;
    }

    va_start(arguments, format);
    length = vsnprintf((char *)buffer,
                       sizeof(buffer) - sizeof(s_tjc_terminator),
                       format,
                       arguments);
    va_end(arguments);
    if ((length < 0) ||
        ((uint32_t)length >
         sizeof(buffer) - sizeof(s_tjc_terminator) - 1U)) {
        return HAL_ERROR;
    }

    memcpy(&buffer[length],
           s_tjc_terminator,
           sizeof(s_tjc_terminator));
    return HAL_UART_Transmit(&huart2,
                             buffer,
                             (uint16_t)(length +
                                        (int)sizeof(s_tjc_terminator)),
                             G_TJC_TX_TIMEOUT_MS);
}

static void G_TjcUi_ResetRxPacket(void)
{
    s_rx_command_length = 0U;
    s_rx_terminator_count = 0U;
}

static void G_TjcUi_LockTransfer(void)
{
    uint32_t primask = __get_PRIMASK();
    uint8_t previous_state;

    __disable_irq();
    previous_state = s_transfer_state;
    /*
     * Once addt has been emitted, the panel may be consuming binary bytes.
     * Only an error that occurred while fully idle is safe for automatic
     * ASCII reinitialization.
     */
    s_reinitialize_safe =
        ((previous_state == (uint8_t)G_TJC_TRANSFER_IDLE) ||
         (previous_state == (uint8_t)G_TJC_TRANSFER_DONE)) ? 1U : 0U;
    s_transfer_state = (uint8_t)G_TJC_TRANSFER_LOCKED;
    G_TjcUi_ResetRxPacket();
    if (primask == 0U) __enable_irq();
}

static HAL_StatusTypeDef G_TjcUi_WaitForTransferState(
    G_TjcTransferState expected,
    uint32_t timeout_ms)
{
    const uint32_t start_tick = HAL_GetTick();

    for (;;) {
        const uint8_t state = s_transfer_state;

        if (state == (uint8_t)expected) return HAL_OK;
        if (state == (uint8_t)G_TJC_TRANSFER_LOCKED) return HAL_ERROR;
        if ((uint32_t)(HAL_GetTick() - start_tick) >= timeout_ms) {
            G_TjcUi_LockTransfer();
            return HAL_TIMEOUT;
        }
        /*
         * RX remains interrupt driven while HAL_UART_Transmit() uses the
         * independent TX state.  Do not disable interrupts or poll RDR here.
         */
        __NOP();
    }
}

static uint32_t G_TjcUi_BulkTxTimeoutMs(uint32_t byte_count)
{
    uint32_t baud = huart2.Init.BaudRate;
    uint32_t wire_time_ms;

    if (baud == 0U) baud = 115200U;
    /* 8N1 transfers ten line bits per payload byte; round upward. */
    wire_time_ms =
        (uint32_t)(((uint64_t)byte_count * 10000ULL + baud - 1U) / baud);
    return wire_time_ms + G_TJC_BULK_TX_MARGIN_MS;
}

static void G_TjcUi_ReversePlotForPanel(void)
{
    uint16_t left = 0U;
    uint16_t right = G_TJC_PLOT_POINTS - 1U;

    /*
     * This TJC waveform component maps the first addt byte to the right edge.
     * Reverse the common bulk buffer once so increasing time/frequency is
     * displayed from left to right for both waveform and spectrum modes.
     */
    while (left < right) {
        const uint8_t temporary = s_plot[left];
        s_plot[left] = s_plot[right];
        s_plot[right] = temporary;
        left++;
        right--;
    }
}

/*
 * Nextion/TJC transparent waveform transfer:
 *   addt <waveform>,<channel>,<count> FF FF FF
 *   panel -> FE FF FF FF
 *   MCU   -> exactly <count> raw bytes
 *   panel -> FD FF FF FF
 *
 * Once the addt command has been emitted, an absent/invalid handshake is
 * ambiguous: the panel might be waiting for binary data.  The transport is
 * therefore locked on every failure.  In particular, there is deliberately
 * no per-point ASCII fallback on that publication attempt.
 */
static HAL_StatusTypeDef G_TjcUi_SendPlotBulk(void)
{
    uint8_t command[24];
    int command_length;
    HAL_StatusTypeDef status;
    uint32_t primask;

    command_length = snprintf((char *)command,
                              sizeof(command) - sizeof(s_tjc_terminator),
                              "addt %u,%u,%u",
                              (unsigned)G_TJC_WAVEFORM_ID,
                              (unsigned)G_TJC_WAVEFORM_CHANNEL,
                              (unsigned)G_TJC_PLOT_POINTS);
    if ((command_length < 0) ||
        ((uint32_t)command_length >
         sizeof(command) - sizeof(s_tjc_terminator) - 1U)) {
        return HAL_ERROR;
    }
    memcpy(&command[command_length],
           s_tjc_terminator,
           sizeof(s_tjc_terminator));

    primask = __get_PRIMASK();
    __disable_irq();
    if (s_transfer_state != (uint8_t)G_TJC_TRANSFER_IDLE) {
        if (primask == 0U) __enable_irq();
        return HAL_BUSY;
    }
    G_TjcUi_ResetRxPacket();
    s_transfer_state = (uint8_t)G_TJC_TRANSFER_WAIT_READY;
    if (primask == 0U) __enable_irq();

    status = HAL_UART_Transmit(
        &huart2,
        command,
        (uint16_t)(command_length + (int)sizeof(s_tjc_terminator)),
        G_TJC_TX_TIMEOUT_MS);
    if (status != HAL_OK) {
        G_TjcUi_LockTransfer();
        return status;
    }
    status = G_TjcUi_WaitForTransferState(
        G_TJC_TRANSFER_READY,
        G_TJC_BULK_READY_TIMEOUT_MS);
    if (status != HAL_OK) return status;

    primask = __get_PRIMASK();
    __disable_irq();
    if (s_transfer_state != (uint8_t)G_TJC_TRANSFER_READY) {
        if (primask == 0U) __enable_irq();
        G_TjcUi_LockTransfer();
        return HAL_ERROR;
    }
    G_TjcUi_ResetRxPacket();
    s_transfer_state = (uint8_t)G_TJC_TRANSFER_WAIT_DONE;
    if (primask == 0U) __enable_irq();

    G_TjcUi_ReversePlotForPanel();
    status = HAL_UART_Transmit(
        &huart2,
        s_plot,
        (uint16_t)sizeof(s_plot),
        G_TjcUi_BulkTxTimeoutMs(sizeof(s_plot)));
    if (status != HAL_OK) {
        G_TjcUi_LockTransfer();
        return status;
    }
    status = G_TjcUi_WaitForTransferState(
        G_TJC_TRANSFER_DONE,
        G_TJC_BULK_DONE_TIMEOUT_MS);
    if (status != HAL_OK) return status;

    primask = __get_PRIMASK();
    __disable_irq();
    if (s_transfer_state != (uint8_t)G_TJC_TRANSFER_DONE) {
        if (primask == 0U) __enable_irq();
        G_TjcUi_LockTransfer();
        return HAL_ERROR;
    }
    G_TjcUi_ResetRxPacket();
    s_transfer_state = (uint8_t)G_TJC_TRANSFER_IDLE;
    if (primask == 0U) __enable_irq();
    return HAL_OK;
}

static HAL_StatusTypeDef G_TjcUi_SendText(const char *object,
                                         const char *text)
{
    if ((object == NULL) || (text == NULL)) return HAL_ERROR;
    return G_TjcUi_SendCommand("%s.txt=\"%s\"", object, text);
}

static HAL_StatusTypeDef G_TjcUi_SendAxisInteger(const char *object,
                                                int32_t value)
{
    char text[12];

    (void)snprintf(text, sizeof(text), "%ld", (long)value);
    return G_TjcUi_SendText(object, text);
}

static HAL_StatusTypeDef G_TjcUi_ClearPlot(void)
{
    return G_TjcUi_SendCommand("cle %u,%u",
                               (unsigned)G_TJC_WAVEFORM_ID,
                               (unsigned)G_TJC_WAVEFORM_CHANNEL);
}

static HAL_StatusTypeDef G_TjcUi_SetTimeAxis(uint16_t full_scale_mv,
                                            uint8_t cycles)
{
    HAL_StatusTypeDef status = HAL_OK;

    if (G_TjcUi_SendText("tyunit", "mV") != HAL_OK) status = HAL_ERROR;
    if (G_TjcUi_SendAxisInteger("tymax", full_scale_mv) != HAL_OK) {
        status = HAL_ERROR;
    }
    if (G_TjcUi_SendText("tymid", "0") != HAL_OK) status = HAL_ERROR;
    if (G_TjcUi_SendAxisInteger("tymin",
                                -(int32_t)full_scale_mv) != HAL_OK) {
        status = HAL_ERROR;
    }
    if (G_TjcUi_SendText("tx0", "0") != HAL_OK) status = HAL_ERROR;
    if (cycles == 3U) {
        if (G_TjcUi_SendText("txmid", "1.5T") != HAL_OK) {
            status = HAL_ERROR;
        }
        if (G_TjcUi_SendText("txmax", "3T") != HAL_OK) {
            status = HAL_ERROR;
        }
    } else {
        if (G_TjcUi_SendText("txmid", "0.5T") != HAL_OK) {
            status = HAL_ERROR;
        }
        if (G_TjcUi_SendText("txmax", "1T") != HAL_OK) {
            status = HAL_ERROR;
        }
    }
    return status;
}

static HAL_StatusTypeDef G_TjcUi_SetSpectrumAxis(
    uint16_t amplitude_full_scale_mv,
    uint32_t frequency_full_scale_hz)
{
    char text[12];
    HAL_StatusTypeDef status = HAL_OK;

    if (G_TjcUi_SendText("tyunit", "mV") != HAL_OK) status = HAL_ERROR;
    if (G_TjcUi_SendAxisInteger("tymax",
                                amplitude_full_scale_mv) != HAL_OK) {
        status = HAL_ERROR;
    }
    if (G_TjcUi_SendAxisInteger("tymid",
                                amplitude_full_scale_mv / 2U) != HAL_OK) {
        status = HAL_ERROR;
    }
    if (G_TjcUi_SendText("tymin", "0") != HAL_OK) status = HAL_ERROR;
    if (G_TjcUi_SendText("tx0", "0") != HAL_OK) status = HAL_ERROR;

    (void)snprintf(text,
                   sizeof(text),
                   "%luk",
                   (unsigned long)(frequency_full_scale_hz / 2000U));
    if (G_TjcUi_SendText("txmid", text) != HAL_OK) status = HAL_ERROR;
    (void)snprintf(text,
                   sizeof(text),
                   "%luk",
                   (unsigned long)(frequency_full_scale_hz / 1000U));
    if (G_TjcUi_SendText("txmax", text) != HAL_OK) status = HAL_ERROR;
    return status;
}

static uint8_t G_TjcUi_GetSortedComponents(
    const PeriodicAnalyzer_Result *result,
    uint8_t indices[G_TJC_COMPONENT_ROWS])
{
    uint8_t count = result->component_count;

    if (count > G_TJC_COMPONENT_ROWS) count = G_TJC_COMPONENT_ROWS;
    for (uint8_t i = 0U; i < count; ++i) indices[i] = i;
    for (uint8_t i = 0U; i < count; ++i) {
        for (uint8_t j = (uint8_t)(i + 1U); j < count; ++j) {
            if (result->component[indices[j]].frequency_hz <
                result->component[indices[i]].frequency_hz) {
                const uint8_t temporary = indices[i];
                indices[i] = indices[j];
                indices[j] = temporary;
            }
        }
    }
    return count;
}

static HAL_StatusTypeDef G_TjcUi_SendMeasurements(
    const PeriodicAnalyzer_Result *result)
{
    static const char *const rows[G_TJC_COMPONENT_ROWS] = {
        "tc1", "tc2", "tc3"
    };
    uint8_t indices[G_TJC_COMPONENT_ROWS];
    uint8_t count;
    char text[32];
    HAL_StatusTypeDef status = HAL_OK;

    (void)snprintf(text,
                   sizeof(text),
                   "%.3fmV",
                   (double)(1000.0f * result->vpp_v));
    if (G_TjcUi_SendText("tupp", text) != HAL_OK) status = HAL_ERROR;

    (void)snprintf(text,
                   sizeof(text),
                   "%.3fmV",
                   (double)(1000.0f * result->true_rms_v));
    if (G_TjcUi_SendText("trms", text) != HAL_OK) status = HAL_ERROR;

    count = G_TjcUi_GetSortedComponents(result, indices);
    for (uint8_t row = 0U; row < G_TJC_COMPONENT_ROWS; ++row) {
        if (row < count) {
            const PeriodicAnalyzer_Component *component =
                &result->component[indices[row]];
            const float frequency_khz =
                component->frequency_hz / 1000.0f;
            const float amplitude_mv =
                1000.0f * component->peak_amplitude_v;

            (void)snprintf(text,
                           sizeof(text),
                           "%.3f/%.3f",
                           (double)frequency_khz,
                           (double)amplitude_mv);
        } else {
            (void)strcpy(text, "---.---/----.---");
        }
        if (G_TjcUi_SendText(rows[row], text) != HAL_OK) {
            status = HAL_ERROR;
        }
    }
    return status;
}

static HAL_StatusTypeDef G_TjcUi_SendTimePlot(
    const PeriodicAnalyzer_Result *result,
    uint8_t cycles)
{
    float maximum_abs_mv;
    uint16_t full_scale_mv;
    HAL_StatusTypeDef status = HAL_OK;

    for (uint32_t i = 0U; i < G_TJC_PLOT_POINTS; ++i) {
        const float phase_turns =
            (float)cycles * (float)i / (float)(G_TJC_PLOT_POINTS - 1U);
        const float value_mv =
            1000.0f * PeriodicAnalyzer_Evaluate(result, phase_turns);

        s_plot_mv[i] = value_mv;
    }
    maximum_abs_mv = G_TjcPlotScale_RemoveDisplayDc(
        s_plot_mv,
        G_TJC_PLOT_POINTS);
    full_scale_mv = G_TjcPlotScale_SelectAmplitudeAxis(
        1.15f * maximum_abs_mv,
        30U,
        5000U);

    if (G_TjcUi_SetTimeAxis(full_scale_mv, cycles) != HAL_OK) {
        status = HAL_ERROR;
    }
    if (G_TjcUi_ClearPlot() != HAL_OK) status = HAL_ERROR;

    for (uint32_t i = 0U; i < G_TJC_PLOT_POINTS; ++i) {
        s_plot[i] = G_TjcPlotScale_MapBipolar(
            s_plot_mv[i],
            (float)full_scale_mv);
    }
    if (status != HAL_OK) return status;
    status = G_TjcUi_SendPlotBulk();
    return status;
}

static HAL_StatusTypeDef G_TjcUi_SendSpectrumPlot(
    const PeriodicAnalyzer_Result *result)
{
    uint8_t indices[G_TJC_COMPONENT_ROWS];
    uint8_t count = G_TjcUi_GetSortedComponents(result, indices);
    float maximum_frequency_hz = 0.0f;
    float maximum_amplitude_mv = 0.0f;
    uint32_t frequency_full_scale_hz;
    uint16_t amplitude_full_scale_mv;
    HAL_StatusTypeDef status = HAL_OK;

    memset(s_plot, 0, sizeof(s_plot));
    for (uint8_t i = 0U; i < count; ++i) {
        const PeriodicAnalyzer_Component *component =
            &result->component[indices[i]];
        const float amplitude_mv =
            1000.0f * component->peak_amplitude_v;
        if (component->frequency_hz > maximum_frequency_hz) {
            maximum_frequency_hz = component->frequency_hz;
        }
        if (amplitude_mv > maximum_amplitude_mv) {
            maximum_amplitude_mv = amplitude_mv;
        }
    }

    frequency_full_scale_hz =
        G_TjcPlotScale_SelectFrequencyAxis(maximum_frequency_hz);
    amplitude_full_scale_mv =
        G_TjcPlotScale_SelectAmplitudeAxis(
            1.15f * maximum_amplitude_mv,
            10U,
            5000U);

    for (uint8_t i = 0U; i < count; ++i) {
        const PeriodicAnalyzer_Component *component =
            &result->component[indices[i]];
        const float amplitude_mv =
            1000.0f * component->peak_amplitude_v;
        const uint32_t x = G_TjcPlotScale_MapFrequencyX(
            component->frequency_hz,
            (float)frequency_full_scale_hz);
        const uint8_t height = G_TjcPlotScale_MapUnipolar(
            amplitude_mv,
            (float)amplitude_full_scale_mv);

        if (height > s_plot[x]) s_plot[x] = height;
    }

    if (G_TjcUi_SetSpectrumAxis(amplitude_full_scale_mv,
                                frequency_full_scale_hz) != HAL_OK) {
        status = HAL_ERROR;
    }
    if (G_TjcUi_ClearPlot() != HAL_OK) status = HAL_ERROR;
    if (status != HAL_OK) return status;
    status = G_TjcUi_SendPlotBulk();
    return status;
}

static void G_TjcUi_AcceptCommand(void)
{
    G_TjcUiMode mode;
    G_TjcUiPage page;

    if (s_rx_command_length != 2U) return;

    if (s_rx_command[0] == 'P') {
        if (s_rx_command[1] == 'I') {
            page = G_TJC_PAGE_INDEX;
        } else if (s_rx_command[1] == 'M') {
            page = G_TJC_PAGE_MAIN;
        } else if (s_rx_command[1] == 'S') {
            page = G_TJC_PAGE_SET;
        } else {
            return;
        }
        s_active_page = (uint8_t)page;
        s_pending_page = (uint8_t)page;
        return;
    }

    if ((s_rx_command[0] != 'G') ||
        (G_TjcUi_CanAccessMainObjects() == 0U)) {
        return;
    }
    if (s_rx_command[1] == '1') {
        mode = G_TJC_MODE_WAVE_1T;
    } else if (s_rx_command[1] == '3') {
        mode = G_TJC_MODE_WAVE_3T;
    } else if (s_rx_command[1] == 'S') {
        mode = G_TJC_MODE_SPECTRUM;
    } else {
        return;
    }
    s_pending_mode = (uint8_t)mode;
}

HAL_StatusTypeDef G_TjcUi_Init(void)
{
    /*
     * Init is also the recovery entry after a UART or transparent-transfer
     * failure.  Stop any stale one-byte receive before resetting protocol
     * state and arming it again.
     */
    if ((s_transfer_state == (uint8_t)G_TJC_TRANSFER_LOCKED) &&
        (s_reinitialize_safe == 0U)) {
        return HAL_ERROR;
    }
    (void)HAL_UART_AbortReceive(&huart2);
    G_TjcUi_ResetRxPacket();
    s_rx_byte = 0U;
    s_transfer_state = (uint8_t)G_TJC_TRANSFER_IDLE;
    s_reinitialize_safe = 1U;
    /* Measurement starts only after a touch command or a USART1 command. */
    s_pending_mode = 0U;
    s_active_page = (uint8_t)G_TJC_PAGE_UNKNOWN;
    s_pending_page = (uint8_t)G_TJC_PAGE_UNKNOWN;

    if (HAL_UART_Receive_IT(&huart2, &s_rx_byte, 1U) != HAL_OK) {
        return HAL_ERROR;
    }
    /*
     * 页面跳转完全由 HMI 本地事件完成。MCU 不在启动时强制 main，
     * 这样 index 页面可作为真正首页，也不会打断 set 页面。
     */
    return HAL_OK;
}

uint8_t G_TjcUi_IsHealthy(void)
{
    return (s_transfer_state != (uint8_t)G_TJC_TRANSFER_LOCKED) ? 1U : 0U;
}

uint8_t G_TjcUi_CanReinitialize(void)
{
    return s_reinitialize_safe;
}

uint8_t G_TjcUi_TakeRequest(G_TjcUiMode *mode)
{
    uint8_t pending;
    uint32_t primask;

    if (mode == NULL) return 0U;
    primask = __get_PRIMASK();
    __disable_irq();
    pending = s_pending_mode;
    s_pending_mode = 0U;
    if (primask == 0U) __enable_irq();

    if ((pending != (uint8_t)G_TJC_MODE_WAVE_1T) &&
        (pending != (uint8_t)G_TJC_MODE_WAVE_3T) &&
        (pending != (uint8_t)G_TJC_MODE_SPECTRUM)) {
        return 0U;
    }
    *mode = (G_TjcUiMode)pending;
    return 1U;
}

uint8_t G_TjcUi_TakePageEvent(G_TjcUiPage *page)
{
    uint8_t pending;
    uint32_t primask;

    if (page == NULL) return 0U;
    primask = __get_PRIMASK();
    __disable_irq();
    pending = s_pending_page;
    s_pending_page = (uint8_t)G_TJC_PAGE_UNKNOWN;
    if (primask == 0U) __enable_irq();

    if ((pending != (uint8_t)G_TJC_PAGE_INDEX) &&
        (pending != (uint8_t)G_TJC_PAGE_MAIN) &&
        (pending != (uint8_t)G_TJC_PAGE_SET)) {
        return 0U;
    }
    *page = (G_TjcUiPage)pending;
    return 1U;
}

uint8_t G_TjcUi_IsMainPageActive(void)
{
    return G_TjcUi_CanAccessMainObjects();
}

HAL_StatusTypeDef G_TjcUi_ShowIdle(void)
{
    HAL_StatusTypeDef status = HAL_OK;

    if (G_TjcUi_CanAccessMainObjects() == 0U) return HAL_BUSY;
    if (G_TjcUi_SendText("tview", "WAIT") != HAL_OK) {
        status = HAL_ERROR;
    }
    if (G_TjcUi_ClearPlot() != HAL_OK) status = HAL_ERROR;
    return status;
}

HAL_StatusTypeDef G_TjcUi_ShowAcquiring(G_TjcUiMode mode)
{
    (void)mode;
    if (G_TjcUi_CanAccessMainObjects() == 0U) return HAL_BUSY;
    return G_TjcUi_SendText("tview", "ACQUIRING");
}

HAL_StatusTypeDef G_TjcUi_ShowError(const char *status_text)
{
    HAL_StatusTypeDef status = HAL_OK;
    const char *text = (status_text != NULL) ?
                       status_text : "MEASURE ERROR";

    if (G_TjcUi_CanAccessMainObjects() == 0U) return HAL_BUSY;
    if (G_TjcUi_SendText("tview", text) != HAL_OK) status = HAL_ERROR;
    if (G_TjcUi_SendText("tupp", "-----.---mV") != HAL_OK) {
        status = HAL_ERROR;
    }
    if (G_TjcUi_SendText("trms", "-----.---mV") != HAL_OK) {
        status = HAL_ERROR;
    }
    if (G_TjcUi_SendText("tc1", "---.---/----.---") != HAL_OK) {
        status = HAL_ERROR;
    }
    if (G_TjcUi_SendText("tc2", "---.---/----.---") != HAL_OK) {
        status = HAL_ERROR;
    }
    if (G_TjcUi_SendText("tc3", "---.---/----.---") != HAL_OK) {
        status = HAL_ERROR;
    }
    if (G_TjcUi_ClearPlot() != HAL_OK) status = HAL_ERROR;
    return status;
}

HAL_StatusTypeDef G_TjcUi_Publish(
    const PeriodicAnalyzer_Result *result,
    G_TjcUiMode mode)
{
    HAL_StatusTypeDef status = HAL_OK;
    const char *title;

    if ((result == NULL) ||
        (result->valid == 0U) ||
        (result->status != PERIODIC_ANALYZER_OK)) {
        return G_TjcUi_ShowError("NO SIGNAL");
    }

    if (G_TjcUi_CanAccessMainObjects() == 0U) return HAL_BUSY;
    /*
     * 坐标轴和曲线处于用户感知的关键路径，先发送；右侧数值和状态
     * 随后更新，比原顺序更早约一组文本命令时间看到完整曲线。
     */
    if (mode == G_TJC_MODE_SPECTRUM) {
        if (G_TjcUi_SendSpectrumPlot(result) != HAL_OK) return HAL_ERROR;
        title = "SPECTRUM";
    } else if (mode == G_TJC_MODE_WAVE_3T) {
        if (G_TjcUi_SendTimePlot(result, 3U) != HAL_OK) return HAL_ERROR;
        title = "3 PERIOD";
    } else {
        if (G_TjcUi_SendTimePlot(result, 1U) != HAL_OK) return HAL_ERROR;
        title = "1 PERIOD";
    }
    if (G_TjcUi_CanAccessMainObjects() == 0U) return HAL_BUSY;
    if (G_TjcUi_SendMeasurements(result) != HAL_OK) return HAL_ERROR;
    if (result->clipped != 0U) title = "OVERLOAD";
    if (G_TjcUi_SendText("tview", title) != HAL_OK) status = HAL_ERROR;
    return status;
}

void G_TjcUi_UartRxComplete(UART_HandleTypeDef *huart)
{
    if (G_TjcUi_IsUart2(huart) == 0U) return;

    if (s_rx_byte == 0xFFU) {
        if (s_rx_terminator_count < 3U) s_rx_terminator_count++;
        if (s_rx_terminator_count == 3U) {
            if ((s_rx_command_length == 1U) &&
                ((uint8_t)s_rx_command[0] == 0xFEU) &&
                (s_transfer_state ==
                 (uint8_t)G_TJC_TRANSFER_WAIT_READY)) {
                s_transfer_state = (uint8_t)G_TJC_TRANSFER_READY;
            } else if ((s_rx_command_length == 1U) &&
                       ((uint8_t)s_rx_command[0] == 0xFDU) &&
                       (s_transfer_state ==
                        (uint8_t)G_TJC_TRANSFER_WAIT_DONE)) {
                s_transfer_state = (uint8_t)G_TJC_TRANSFER_DONE;
            } else if ((s_rx_command_length == 1U) &&
                       (G_TjcUi_IsPanelError(
                            (uint8_t)s_rx_command[0]) != 0U)) {
                /*
                 * Idle-state errors indicate a rejected ASCII command and are
                 * safe to retry.  During addt, preserve the unsafe state
                 * because the panel may still be consuming waveform bytes.
                 */
                G_TjcUi_LockTransfer();
            } else {
                /* G1/G3/GS touch commands remain live outside handshakes. */
                G_TjcUi_AcceptCommand();
            }
            s_rx_command_length = 0U;
            s_rx_terminator_count = 0U;
        }
    } else {
        if (s_rx_terminator_count != 0U) {
            s_rx_command_length = 0U;
            s_rx_terminator_count = 0U;
        }
        if (s_rx_command_length < G_TJC_COMMAND_CAPACITY) {
            s_rx_command[s_rx_command_length++] = (char)s_rx_byte;
        } else {
            s_rx_command_length = 0U;
        }
    }
    if (HAL_UART_Receive_IT(&huart2, &s_rx_byte, 1U) != HAL_OK) {
        G_TjcUi_LockTransfer();
    }
}

void G_TjcUi_UartError(UART_HandleTypeDef *huart)
{
    if (G_TjcUi_IsUart2(huart) == 0U) return;
    G_TjcUi_ResetRxPacket();
    G_TjcUi_LockTransfer();
}
