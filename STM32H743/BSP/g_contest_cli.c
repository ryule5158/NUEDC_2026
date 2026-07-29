#include "g_contest_cli.h"

#include "g_contest_app.h"
#include "g_contest_config.h"
#include "g_tjc_ui.h"
#include "periodic_analyzer.h"
#include "signal_adc.h"
#include "usart.h"

#include <ctype.h>
#include <stdio.h>

#define G_CLI_RX_RING_SIZE 256U
#define G_CLI_LINE_SIZE     96U

static uint8_t s_rx_byte;
static volatile uint8_t s_rx_ring[G_CLI_RX_RING_SIZE];
static volatile uint16_t s_rx_head;
static volatile uint16_t s_rx_tail;
static char s_line[G_CLI_LINE_SIZE];
static uint16_t s_line_length;
static uint8_t s_pending_cycles;
static uint8_t s_calibration_mode;
static uint8_t s_pending_calibration;
static uint32_t s_rx_overflow_count;
static uint32_t s_command_count;
static volatile uint8_t s_rx_rearm_pending;

static HAL_StatusTypeDef GContestCli_ArmReceive(void)
{
    HAL_StatusTypeDef status;
    uint32_t primask;

    if (huart1.RxState != HAL_UART_STATE_READY) {
        (void)HAL_UART_AbortReceive(&huart1);
    }
    /*
     * HAL_UART_Receive_IT enables RX interrupts before it returns.  Keep the
     * final pending-flag update in the same short critical section so an
     * immediately arriving byte cannot have its rearm failure overwritten.
     */
    primask = __get_PRIMASK();
    __disable_irq();
    status = HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1U);
    s_rx_rearm_pending = (status == HAL_OK) ? 0U : 1U;
    if (primask == 0U) {
        __enable_irq();
    }
    return status;
}

static uint8_t GContestCli_Equals(const char *text, const char *command)
{
    while ((*text != '\0') && (*command != '\0')) {
        if ((char)toupper((unsigned char)*text) !=
            (char)toupper((unsigned char)*command)) {
            return 0U;
        }
        text++;
        command++;
    }
    return ((*text == '\0') && (*command == '\0')) ? 1U : 0U;
}

static void GContestCli_PrintHelp(void)
{
    printf("[CLI] test commands (single byte, CR/LF optional):\r\n");
    printf("  1                    capture, print 1 period, then results\r\n");
    printf("  3                    capture, print 3 periods, then results\r\n");
    printf("[CLI] diagnostic commands (finish with CR/LF):\r\n");
    printf("  HELP                 show this help\r\n");
    printf("  INFO                 show G measurement parameters\r\n");
    printf("  STATUS               show latest result and CLI state\r\n");
    printf("  PING                 check command link\r\n");
    printf("[CLI] raw voltage-calibration mode (finish with CR/LF):\r\n");
    printf("  CAL                  enter raw DC calibration mode\r\n");
    printf("  Q                    capture 16 frames and print raw Q\r\n");
    printf("  EXIT                 leave raw DC calibration mode\r\n");
}

static void GContestCli_PrintInfo(void)
{
    const SignalAdc_Info *info = SignalAdc_GetInfo();

    printf("[CLI] G remote solution\r\n");
    if (info != NULL) {
        printf("[CLI] ADC=%s physicalFs=%luHz Fs=%.0fHz "
               "physical_bits=%u physical_lsb_nominal=%.6fmV "
               "transport=%s transport_scale=%.9fmV/Q "
               "transport_offset=%.6fmV\r\n",
               info->name,
               (unsigned long)info->physical_sample_clock_hz,
               (double)info->output_sample_rate_hz,
               (unsigned)info->resolution_bits,
               (double)(1000.0f * info->nominal_lsb_v),
               info->transport_format,
               (double)(1000.0f * info->transport_lsb_v),
               (double)(1000.0f * info->transport_offset_v));
        printf("[CLI] voltage_transfer=%s calibrated=%u source=%s\r\n",
               (info->voltage_transfer_model != NULL) ?
                   info->voltage_transfer_model : "unspecified",
               (unsigned)info->voltage_calibrated,
               (info->voltage_calibration_source != NULL) ?
                   info->voltage_calibration_source : "none");
        printf("[CLI] ac_amplitude_scale=%.9f source=%s "
               "frequency_table=OFF\r\n",
               (double)info->ac_amplitude_scale,
               (info->ac_amplitude_calibration_source != NULL) ?
                   info->ac_amplitude_calibration_source : "none");
        printf("[CLI] N=%lu df=%.5fHz\r\n",
               (unsigned long)G_CONTEST_SAMPLE_COUNT,
               (double)(info->output_sample_rate_hz /
                        (float)G_CONTEST_SAMPLE_COUNT));
    }
    printf("[CLI] band=%.1f..%.1fHz analytic_response=ON "
           "empirical_calibration=OFF\r\n",
           (double)G_CONTEST_MIN_FREQ_HZ,
           (double)G_CONTEST_MAX_FREQ_HZ);
    printf("[CLI] interference_rejection=joint_model expected=%.0fHz "
           "search=+/-%.0fHz min_post_dsp_rms=%.3fmV\r\n",
           (double)G_CONTEST_INTERFERENCE_FREQ_HZ,
           (double)G_CONTEST_INTERFERENCE_SEARCH_HZ,
           (double)(1000.0f * G_CONTEST_INTERFERENCE_MIN_RMS_V));
}

static void GContestCli_PrintStatus(void)
{
    const SignalAdc_Diagnostic *diagnostic =
        SignalAdc_GetDiagnostic();
    const PeriodicAnalyzer_Result *result = GContest_GetLastResult();

    printf("[CLI] state=%s backend=%s pending_cycles=%u frame=%lu latency=%lums "
           "rx_overflow=%lu commands=%lu\r\n",
           (s_pending_calibration != 0U) ? "CAL_PENDING" :
           ((s_calibration_mode != 0U) ? "CAL" :
            ((s_pending_cycles != 0U) ? "PENDING" : "IDLE")),
           (GContest_IsReady() != 0U) ? "READY" : "WAIT",
           (unsigned)s_pending_cycles,
           (unsigned long)GContest_GetFrameNumber(),
           (unsigned long)GContest_GetLastLatencyMs(),
           (unsigned long)s_rx_overflow_count,
           (unsigned long)s_command_count);
    if ((result != NULL) && (result->valid != 0U)) {
        printf("[CLI] latest f0=%.2fHz vpp=%.3fmV rms=%.3fmV lines=%u clip=%u\r\n",
               (double)result->fundamental_hz,
               (double)(1000.0f * result->vpp_v),
               (double)(1000.0f * result->true_rms_v),
               (unsigned)result->component_count,
               (unsigned)result->clipped);
    } else if (result != NULL) {
        printf("[CLI] latest invalid analyzer_status=%d\r\n",
               (int)result->status);
    } else {
        printf("[CLI] latest result unavailable\r\n");
    }
    if (diagnostic != NULL) {
        printf("[CLI] ADC stage=%s native_status=%lu "
               "ready=%u busy=%u done=%u captured=%lu read=%lu\r\n",
               SignalAdc_StageName(diagnostic->stage),
               (unsigned long)diagnostic->native_status,
               (unsigned)diagnostic->ready,
               (unsigned)diagnostic->busy,
               (unsigned)diagnostic->done,
               (unsigned long)diagnostic->captured_count,
               (unsigned long)diagnostic->read_count);
    }
}

static void GContestCli_HandleLine(const char *line)
{
    s_command_count++;

    if (GContestCli_Equals(line, "CAL") != 0U) {
        s_pending_cycles = 0U;
        s_pending_calibration = 0U;
        s_calibration_mode = 1U;
        printf("[CAL] mode entered; apply a stable DC voltage at the ADC input\r\n");
        printf("[CAL] send Q after settling; send EXIT when finished\r\n");
        printf("[CAL] output is raw transport Q only; voltage calibration is NOT applied\r\n");
    } else if (GContestCli_Equals(line, "EXIT") != 0U) {
        if (s_calibration_mode != 0U) {
            s_calibration_mode = 0U;
            s_pending_calibration = 0U;
            printf("[CAL] mode exited; normal measurement commands restored\r\n");
        } else {
            printf("[CLI] not in CAL mode\r\n");
        }
    } else if (GContestCli_Equals(line, "Q") != 0U) {
        if (s_calibration_mode != 0U) {
            s_pending_calibration = 1U;
            printf("[CAL] Q capture accepted; keep the DC input stable\r\n");
        } else {
            printf("[CLI] Q requires CAL mode; send CAL first\r\n");
        }
    } else if (GContestCli_Equals(line, "1") != 0U) {
        if (s_calibration_mode != 0U) {
            printf("[CAL] normal measurement disabled; send EXIT first\r\n");
        } else {
            s_pending_cycles = 1U;
            printf("[CLI] command=1 accepted; capture starts now\r\n");
        }
    } else if (GContestCli_Equals(line, "3") != 0U) {
        if (s_calibration_mode != 0U) {
            printf("[CAL] normal measurement disabled; send EXIT first\r\n");
        } else {
            s_pending_cycles = 3U;
            printf("[CLI] command=3 accepted; capture starts now\r\n");
        }
    } else if (GContestCli_Equals(line, "HELP") != 0U) {
        GContestCli_PrintHelp();
    } else if (GContestCli_Equals(line, "INFO") != 0U) {
        GContestCli_PrintInfo();
    } else if (GContestCli_Equals(line, "STATUS") != 0U) {
        GContestCli_PrintStatus();
    } else if (GContestCli_Equals(line, "PING") != 0U) {
        printf("[CLI] PONG\r\n");
    } else if (line[0] != '\0') {
        printf("[CLI] unknown command: %s\r\n", line);
        printf("[CLI] type HELP\r\n");
    }
}

HAL_StatusTypeDef GContestCli_Init(void)
{
    s_rx_head = 0U;
    s_rx_tail = 0U;
    s_line_length = 0U;
    s_pending_cycles = 0U;
    s_calibration_mode = 0U;
    s_pending_calibration = 0U;
    s_rx_overflow_count = 0U;
    s_command_count = 0U;
    s_rx_byte = 0U;
    s_rx_rearm_pending = 0U;

    if (GContestCli_ArmReceive() != HAL_OK) {
        return HAL_ERROR;
    }

    printf("[CLI] READY USART1 115200 8N1; send 1/3, or CAL for raw Q\r\n");
    return HAL_OK;
}

void GContestCli_Poll(void)
{
    if (s_rx_rearm_pending != 0U) {
        (void)GContestCli_ArmReceive();
    }

    while (s_rx_tail != s_rx_head) {
        uint8_t byte = s_rx_ring[s_rx_tail];
        s_rx_tail = (uint16_t)((s_rx_tail + 1U) % G_CLI_RX_RING_SIZE);

        if (((byte == (uint8_t)'1') || (byte == (uint8_t)'3')) &&
            (s_line_length == 0U)) {
            char numeric_command[2];
            numeric_command[0] = (char)byte;
            numeric_command[1] = '\0';
            GContestCli_HandleLine(numeric_command);
        } else if ((byte == '\r') || (byte == '\n')) {
            if (s_line_length > 0U) {
                s_line[s_line_length] = '\0';
                GContestCli_HandleLine(s_line);
                s_line_length = 0U;
            }
        } else if ((byte == '\b') || (byte == 0x7FU)) {
            if (s_line_length > 0U) s_line_length--;
        } else if (isprint((unsigned char)byte) != 0) {
            if (s_line_length < (G_CLI_LINE_SIZE - 1U)) {
                s_line[s_line_length++] = (char)byte;
            }
        }
    }
}

uint8_t GContestCli_TakeMeasurementRequest(uint8_t *waveform_cycles)
{
    if ((waveform_cycles != NULL) && (s_pending_cycles != 0U)) {
        *waveform_cycles = s_pending_cycles;
        s_pending_cycles = 0U;
        return 1U;
    }
    return 0U;
}

uint8_t GContestCli_TakeCalibrationRequest(void)
{
    if (s_pending_calibration != 0U) {
        s_pending_calibration = 0U;
        return 1U;
    }
    return 0U;
}

uint8_t GContestCli_IsCalibrationMode(void)
{
    return s_calibration_mode;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if ((huart == &huart1) ||
        ((huart != NULL) && (huart->Instance == USART1))) {
        uint16_t next = (uint16_t)((s_rx_head + 1U) % G_CLI_RX_RING_SIZE);
        if (next != s_rx_tail) {
            s_rx_ring[s_rx_head] = s_rx_byte;
            s_rx_head = next;
        } else {
            s_rx_overflow_count++;
        }
        if (HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1U) != HAL_OK) {
            s_rx_rearm_pending = 1U;
        }
        return;
    }
    G_TjcUi_UartRxComplete(huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if ((huart == &huart1) ||
        ((huart != NULL) && (huart->Instance == USART1))) {
        s_rx_rearm_pending = 1U;
        return;
    }
    G_TjcUi_UartError(huart);
}
