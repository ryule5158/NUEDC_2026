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
static uint32_t s_rx_overflow_count;
static uint32_t s_command_count;

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
    printf("  ADCDBG               show cached AD9238/FPGA diagnostics\r\n");
    printf("  PING                 check command link\r\n");
}

static void GContestCli_PrintInfo(void)
{
    const SignalAdc_Info *info = SignalAdc_GetInfo();

    printf("[CLI] G remote solution\r\n");
    if (info != NULL) {
        printf("[CLI] ADC=%s physicalFs=%luHz Fs=%.0fHz "
               "physical_bits=%u physical_lsb=%.6fmV "
               "transport=%s transport_lsb=%.6fmV\r\n",
               info->name,
               (unsigned long)info->physical_sample_clock_hz,
               (double)info->output_sample_rate_hz,
               (unsigned)info->resolution_bits,
               (double)(1000.0f * info->nominal_lsb_v),
               info->transport_format,
               (double)(1000.0f * info->transport_lsb_v));
        printf("[CLI] N=%lu df=%.5fHz\r\n",
               (unsigned long)G_CONTEST_SAMPLE_COUNT,
               (double)(info->output_sample_rate_hz /
                        (float)G_CONTEST_SAMPLE_COUNT));
    }
    printf("[CLI] band=%.1f..%.1fHz FIR307_CIC3_R25_RTL=ON "
           "empirical_points=%u span=100000..500000Hz "
           "extrapolation=OFF endpoint500=INFERRED_HOLD\r\n",
           (double)G_CONTEST_MIN_FREQ_HZ,
           (double)G_CONTEST_MAX_FREQ_HZ,
           (unsigned)G_CONTEST_EMPIRICAL_CAL_POINT_COUNT);
}

static void GContestCli_PrintStatus(void)
{
    const SignalAdc_Diagnostic *diagnostic =
        SignalAdc_GetDiagnostic();
    const PeriodicAnalyzer_Result *result = GContest_GetLastResult();

    printf("[CLI] state=%s pending_cycles=%u frame=%lu latency=%lums "
           "rx_overflow=%lu commands=%lu tx_pending=%lu tx_drop=%lu\r\n",
           (s_pending_cycles != 0U) ? "PENDING" : "IDLE",
           (unsigned)s_pending_cycles,
           (unsigned long)GContest_GetFrameNumber(),
           (unsigned long)GContest_GetLastLatencyMs(),
           (unsigned long)s_rx_overflow_count,
           (unsigned long)s_command_count,
           (unsigned long)DebugUart1_GetPendingByteCount(),
           (unsigned long)DebugUart1_GetDroppedByteCount());
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

/*
 * Print a cached snapshot only.  This command deliberately performs no new
 * SPI transaction, so it is safe to use while diagnosing a completed or
 * failed capture and cannot disturb the FPGA read pointer.
 */
static void GContestCli_PrintAdcDebug(void)
{
    const SignalAdc_Info *info = SignalAdc_GetInfo();
    const SignalAdc_Diagnostic *diagnostic =
        SignalAdc_GetDiagnostic();
    float transport_lsb_mv;

    if ((info == NULL) || (diagnostic == NULL)) {
        printf("[ADCDBG] unavailable\r\n");
        return;
    }

    transport_lsb_mv = 1000.0f * info->transport_lsb_v;
    printf("[ADCDBG] id=0x%08lX fw=0x%08lX physical_clk=%luHz "
           "word=%u frac=%u transport=%s\r\n",
           (unsigned long)diagnostic->device_id,
           (unsigned long)diagnostic->firmware_version,
           (unsigned long)diagnostic->physical_sample_clock_hz,
           (unsigned)diagnostic->sample_word_bits,
           (unsigned)diagnostic->sample_fraction_bits,
           info->transport_format);
    printf("[ADCDBG] cfg samples=%lu decimation=%lu stored_Fs=%.0fHz "
           "trigger_mode=%u threshold=%lu\r\n",
           (unsigned long)diagnostic->configured_sample_count,
           (unsigned long)diagnostic->configured_decimation,
           (double)diagnostic->stored_sample_rate_hz,
           (unsigned)diagnostic->trigger_mode,
           (unsigned long)diagnostic->trigger_threshold);
    printf("[ADCDBG] state native=%lu stage=%s ready=%u busy=%u done=%u "
           "triggered=%u waiting=%u overrange=%u captured=%lu read=%lu\r\n",
           (unsigned long)diagnostic->native_status,
           SignalAdc_StageName(diagnostic->stage),
           (unsigned)diagnostic->ready,
           (unsigned)diagnostic->busy,
           (unsigned)diagnostic->done,
           (unsigned)diagnostic->triggered,
           (unsigned)diagnostic->waiting_trigger,
           (unsigned)diagnostic->overrange,
           (unsigned long)diagnostic->captured_count,
           (unsigned long)diagnostic->read_count);
    printf("[ADCDBG] q8_8 min=%d max=%d latest=%d mean=%.3f sum=%ld "
           "min=%.3fmV max=%.3fmV latest=%.3fmV otr_clocks=%lu\r\n",
           (int)diagnostic->min_sample_q8_8,
           (int)diagnostic->max_sample_q8_8,
           (int)diagnostic->latest_sample_q8_8,
           (double)diagnostic->mean_sample_q8_8,
           (long)diagnostic->sample_sum_q8_8,
           (double)((float)diagnostic->min_sample_q8_8 *
                    transport_lsb_mv),
           (double)((float)diagnostic->max_sample_q8_8 *
                    transport_lsb_mv),
           (double)((float)diagnostic->latest_sample_q8_8 *
                    transport_lsb_mv),
           (unsigned long)diagnostic->overrange_count);
}

static void GContestCli_HandleLine(const char *line)
{
    s_command_count++;

    if (GContestCli_Equals(line, "1") != 0U) {
        s_pending_cycles = 1U;
        printf("[CLI] command=1 accepted; capture starts now\r\n");
    } else if (GContestCli_Equals(line, "3") != 0U) {
        s_pending_cycles = 3U;
        printf("[CLI] command=3 accepted; capture starts now\r\n");
    } else if (GContestCli_Equals(line, "HELP") != 0U) {
        GContestCli_PrintHelp();
    } else if (GContestCli_Equals(line, "INFO") != 0U) {
        GContestCli_PrintInfo();
    } else if (GContestCli_Equals(line, "STATUS") != 0U) {
        GContestCli_PrintStatus();
    } else if (GContestCli_Equals(line, "ADCDBG") != 0U) {
        GContestCli_PrintAdcDebug();
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
    s_rx_overflow_count = 0U;
    s_command_count = 0U;
    s_rx_byte = 0U;

    if (HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1U) != HAL_OK) {
        return HAL_ERROR;
    }

    printf("[CLI] READY USART1 115200 8N1; send 1 or 3\r\n");
    return HAL_OK;
}

void GContestCli_Poll(void)
{
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
        (void)HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1U);
        return;
    }
    G_TjcUi_UartRxComplete(huart);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    DebugUart1_TxComplete(huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if ((huart == &huart1) ||
        ((huart != NULL) && (huart->Instance == USART1))) {
        (void)HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1U);
        return;
    }
    G_TjcUi_UartError(huart);
}
