#include "g_tjc_ui.h"

#include "usart.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define G_TJC_WAVEFORM_ID          1U
#define G_TJC_WAVEFORM_CHANNEL     0U
#define G_TJC_PLOT_POINTS          270U
#define G_TJC_TX_TIMEOUT_MS        50U
#define G_TJC_COMMAND_CAPACITY     4U
#define G_TJC_COMPONENT_ROWS       3U

static const uint8_t s_tjc_terminator[3] = {0xFFU, 0xFFU, 0xFFU};
static uint8_t s_rx_byte;
static char s_rx_command[G_TJC_COMMAND_CAPACITY];
static uint8_t s_rx_command_length;
static uint8_t s_rx_terminator_count;
static volatile uint8_t s_pending_mode;
static uint8_t s_plot[G_TJC_PLOT_POINTS];

static uint8_t G_TjcUi_IsUart2(const UART_HandleTypeDef *huart)
{
    return ((huart == &huart2) ||
            ((huart != NULL) && (huart->Instance == USART2))) ? 1U : 0U;
}

static HAL_StatusTypeDef G_TjcUi_SendCommand(const char *format, ...)
{
    uint8_t buffer[64];
    int length;
    va_list arguments;

    if (format == NULL) return HAL_ERROR;

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

static uint16_t G_TjcUi_RoundScale10(float required,
                                    uint16_t minimum,
                                    uint16_t maximum)
{
    uint16_t units;
    uint16_t scale;

    if (!(required > 0.0f)) return minimum;
    units = (uint16_t)(required / 10.0f);
    if ((float)(10U * units) < required) units++;
    scale = (uint16_t)(10U * units);
    if (scale < minimum) scale = minimum;
    if (scale > maximum) scale = maximum;
    return scale;
}

static HAL_StatusTypeDef G_TjcUi_SendPoint(uint8_t value)
{
    return G_TjcUi_SendCommand("add %u,%u,%u",
                               (unsigned)G_TJC_WAVEFORM_ID,
                               (unsigned)G_TJC_WAVEFORM_CHANNEL,
                               (unsigned)value);
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

static uint32_t G_TjcUi_SelectFrequencyScale(float maximum_hz)
{
    if (maximum_hz <= 50000.0f) return 50000U;
    if (maximum_hz <= 100000.0f) return 100000U;
    if (maximum_hz <= 200000.0f) return 200000U;
    return 500000U;
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
                   "U:%.1fmV",
                   (double)(1000.0f * result->vpp_v));
    if (G_TjcUi_SendText("tupp", text) != HAL_OK) status = HAL_ERROR;

    (void)snprintf(text,
                   sizeof(text),
                   "R:%.1fmV",
                   (double)(1000.0f * result->true_rms_v));
    if (G_TjcUi_SendText("trms", text) != HAL_OK) status = HAL_ERROR;

    if (result->fundamental_hz < 100000.0f) {
        (void)snprintf(text,
                       sizeof(text),
                       "f:%.3fk",
                       (double)(result->fundamental_hz / 1000.0f));
    } else {
        (void)snprintf(text,
                       sizeof(text),
                       "f:%.1fk",
                       (double)(result->fundamental_hz / 1000.0f));
    }
    if (G_TjcUi_SendText("tf1", text) != HAL_OK) status = HAL_ERROR;

    count = G_TjcUi_GetSortedComponents(result, indices);
    for (uint8_t row = 0U; row < G_TJC_COMPONENT_ROWS; ++row) {
        if (row < count) {
            const PeriodicAnalyzer_Component *component =
                &result->component[indices[row]];
            const float frequency_khz =
                component->frequency_hz / 1000.0f;
            const float amplitude_mv =
                1000.0f * component->peak_amplitude_v;

            /*
             * The 16-pixel full-width font fits at most nine characters in
             * the 153-pixel row. Keep one frequency decimal only when the
             * amplitude has fewer than three integer digits.
             */
            if ((frequency_khz < 100.0f) &&
                (amplitude_mv < 99.5f)) {
                (void)snprintf(text,
                               sizeof(text),
                               "%.1fk%.0fmV",
                               (double)frequency_khz,
                               (double)amplitude_mv);
            } else {
                (void)snprintf(text,
                               sizeof(text),
                               "%.0fk%.0fmV",
                               (double)frequency_khz,
                               (double)amplitude_mv);
            }
        } else {
            (void)strcpy(text, "--.-k--mV");
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
    float maximum_abs_mv = 0.0f;
    uint16_t full_scale_mv;
    HAL_StatusTypeDef status = HAL_OK;

    for (uint32_t i = 0U; i < G_TJC_PLOT_POINTS; ++i) {
        const float phase_turns =
            (float)cycles * (float)i / (float)(G_TJC_PLOT_POINTS - 1U);
        const float value_mv =
            1000.0f * PeriodicAnalyzer_Evaluate(result, phase_turns);
        const float absolute_mv = fabsf(value_mv);
        if (absolute_mv > maximum_abs_mv) maximum_abs_mv = absolute_mv;
    }
    full_scale_mv = G_TjcUi_RoundScale10(1.15f * maximum_abs_mv,
                                         30U,
                                         150U);

    if (G_TjcUi_SetTimeAxis(full_scale_mv, cycles) != HAL_OK) {
        status = HAL_ERROR;
    }
    if (G_TjcUi_ClearPlot() != HAL_OK) status = HAL_ERROR;

    for (uint32_t i = 0U; i < G_TJC_PLOT_POINTS; ++i) {
        const float phase_turns =
            (float)cycles * (float)i / (float)(G_TJC_PLOT_POINTS - 1U);
        const float value_mv =
            1000.0f * PeriodicAnalyzer_Evaluate(result, phase_turns);
        float mapped = 128.0f +
                       120.0f * value_mv / (float)full_scale_mv;
        uint8_t code;

        if (mapped < 8.0f) mapped = 8.0f;
        if (mapped > 248.0f) mapped = 248.0f;
        code = (uint8_t)(mapped + 0.5f);
        if (G_TjcUi_SendPoint(code) != HAL_OK) status = HAL_ERROR;
    }
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
        G_TjcUi_SelectFrequencyScale(maximum_frequency_hz);
    amplitude_full_scale_mv =
        G_TjcUi_RoundScale10(1.15f * maximum_amplitude_mv, 10U, 250U);

    for (uint8_t i = 0U; i < count; ++i) {
        const PeriodicAnalyzer_Component *component =
            &result->component[indices[i]];
        const float amplitude_mv =
            1000.0f * component->peak_amplitude_v;
        uint32_t x = 1U + (uint32_t)(
            component->frequency_hz *
            (float)(G_TJC_PLOT_POINTS - 3U) /
            (float)frequency_full_scale_hz);
        uint32_t height = (uint32_t)(
            240.0f * amplitude_mv /
            (float)amplitude_full_scale_mv + 0.5f);

        if (x > G_TJC_PLOT_POINTS - 2U) x = G_TJC_PLOT_POINTS - 2U;
        if (height > 240U) height = 240U;
        if (height > s_plot[x]) s_plot[x] = (uint8_t)height;
    }

    if (G_TjcUi_SetSpectrumAxis(amplitude_full_scale_mv,
                                frequency_full_scale_hz) != HAL_OK) {
        status = HAL_ERROR;
    }
    if (G_TjcUi_ClearPlot() != HAL_OK) status = HAL_ERROR;
    for (uint32_t i = 0U; i < G_TJC_PLOT_POINTS; ++i) {
        if (G_TjcUi_SendPoint(s_plot[i]) != HAL_OK) status = HAL_ERROR;
    }
    return status;
}

static void G_TjcUi_AcceptCommand(void)
{
    G_TjcUiMode mode;

    if ((s_rx_command_length != 2U) ||
        (s_rx_command[0] != 'G')) {
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
    s_rx_command_length = 0U;
    s_rx_terminator_count = 0U;
    s_rx_byte = 0U;
    /* Schedule a first 1-period measurement without requiring a touch. */
    s_pending_mode = (uint8_t)G_TJC_MODE_WAVE_1T;

    if (HAL_UART_Receive_IT(&huart2, &s_rx_byte, 1U) != HAL_OK) {
        return HAL_ERROR;
    }
    if (G_TjcUi_SendCommand("page main") != HAL_OK) return HAL_ERROR;
    if (G_TjcUi_SendText("tview", "WAIT") != HAL_OK) return HAL_ERROR;
    return G_TjcUi_ClearPlot();
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

HAL_StatusTypeDef G_TjcUi_ShowAcquiring(G_TjcUiMode mode)
{
    (void)mode;
    return G_TjcUi_SendText("tview", "ACQUIRING");
}

HAL_StatusTypeDef G_TjcUi_ShowError(const char *status_text)
{
    HAL_StatusTypeDef status = HAL_OK;
    const char *text = (status_text != NULL) ?
                       status_text : "MEASURE ERROR";

    if (G_TjcUi_SendText("tview", text) != HAL_OK) status = HAL_ERROR;
    if (G_TjcUi_SendText("tupp", "U:---.-mV") != HAL_OK) {
        status = HAL_ERROR;
    }
    if (G_TjcUi_SendText("trms", "R:---.-mV") != HAL_OK) {
        status = HAL_ERROR;
    }
    if (G_TjcUi_SendText("tf1", "f:---.---k") != HAL_OK) {
        status = HAL_ERROR;
    }
    if (G_TjcUi_SendText("tc1", "--.-k--mV") != HAL_OK) {
        status = HAL_ERROR;
    }
    if (G_TjcUi_SendText("tc2", "--.-k--mV") != HAL_OK) {
        status = HAL_ERROR;
    }
    if (G_TjcUi_SendText("tc3", "--.-k--mV") != HAL_OK) {
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

    if (G_TjcUi_SendMeasurements(result) != HAL_OK) status = HAL_ERROR;
    if (mode == G_TJC_MODE_SPECTRUM) {
        if (G_TjcUi_SendSpectrumPlot(result) != HAL_OK) status = HAL_ERROR;
        title = "SPECTRUM";
    } else if (mode == G_TJC_MODE_WAVE_3T) {
        if (G_TjcUi_SendTimePlot(result, 3U) != HAL_OK) status = HAL_ERROR;
        title = "3 PERIOD";
    } else {
        if (G_TjcUi_SendTimePlot(result, 1U) != HAL_OK) status = HAL_ERROR;
        title = "1 PERIOD";
    }
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
            G_TjcUi_AcceptCommand();
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
    (void)HAL_UART_Receive_IT(&huart2, &s_rx_byte, 1U);
}

void G_TjcUi_UartError(UART_HandleTypeDef *huart)
{
    if (G_TjcUi_IsUart2(huart) == 0U) return;
    s_rx_command_length = 0U;
    s_rx_terminator_count = 0U;
    (void)HAL_UART_Receive_IT(&huart2, &s_rx_byte, 1U);
}
