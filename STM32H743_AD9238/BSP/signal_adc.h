#ifndef SIGNAL_ADC_H
#define SIGNAL_ADC_H

#include <stdint.h>

/*
 * ADC-independent acquisition contract.
 *
 * Exactly one backend implementation must be linked into the target.  The
 * backend owns raw-code width, transport, decimation and code-to-volt
 * conversion; measurement code always receives SI-unit float samples.
 */
typedef enum {
    SIGNAL_ADC_OK = 0,
    SIGNAL_ADC_E_ARGUMENT,
    SIGNAL_ADC_E_NOT_INITIALIZED,
    SIGNAL_ADC_E_LINK,
    SIGNAL_ADC_E_DEVICE,
    SIGNAL_ADC_E_FORMAT,
    SIGNAL_ADC_E_NOT_READY,
    SIGNAL_ADC_E_BUSY,
    SIGNAL_ADC_E_TIMEOUT,
    SIGNAL_ADC_E_NO_DATA,
    SIGNAL_ADC_E_UNKNOWN
} SignalAdc_Status;

typedef enum {
    SIGNAL_ADC_STAGE_IDLE = 0,
    SIGNAL_ADC_STAGE_START,
    SIGNAL_ADC_STAGE_WAIT,
    SIGNAL_ADC_STAGE_READ,
    SIGNAL_ADC_STAGE_STATUS,
    SIGNAL_ADC_STAGE_COMPLETE,
    SIGNAL_ADC_STAGE_UNKNOWN
} SignalAdc_Stage;

typedef struct {
    const char *name;
    uint32_t physical_sample_clock_hz;
    float output_sample_rate_hz;
    uint32_t max_samples;
    uint8_t resolution_bits; /* Physical ADC resolution, before FPGA DSP. */
    float nominal_lsb_v;     /* Physical ADC input volts per native code. */
    const char *transport_format; /* Sample representation received by MCU. */
    float transport_lsb_v;        /* Input volts represented by one transport LSB. */
} SignalAdc_Info;

typedef struct {
    uint32_t sample_count;
    float sample_rate_hz;
    float min_v;
    float max_v;
    uint8_t clipped;
} SignalAdc_FrameInfo;

typedef struct {
    SignalAdc_Stage stage;
    uint32_t native_status;
    uint32_t device_id;
    uint32_t firmware_version;
    uint32_t physical_sample_clock_hz;
    uint32_t configured_sample_count;
    uint32_t configured_decimation;
    uint32_t trigger_threshold;
    uint32_t captured_count;
    uint32_t read_count;
    uint32_t overrange_count;
    int32_t sample_sum_q8_8;
    int16_t min_sample_q8_8;
    int16_t max_sample_q8_8;
    int16_t latest_sample_q8_8;
    float mean_sample_q8_8;
    float stored_sample_rate_hz;
    uint8_t sample_word_bits;
    uint8_t sample_fraction_bits;
    uint8_t trigger_mode;
    uint8_t ready;
    uint8_t busy;
    uint8_t done;
    uint8_t triggered;
    uint8_t waiting_trigger;
    uint8_t overrange;
} SignalAdc_Diagnostic;

SignalAdc_Status SignalAdc_Init(void);
SignalAdc_Status SignalAdc_Capture(float *destination_v,
                                  uint32_t capacity,
                                  uint32_t requested_count,
                                  uint32_t timeout_ms,
                                  SignalAdc_FrameInfo *frame);

/*
 * Capture CH1 and CH2 from the same FPGA start event. CH1 uses its calibrated
 * FIR+CIC measurement path; CH2 uses the FIR-only anti-alias reference path.
 */
SignalAdc_Status SignalAdc_CaptureDual(float *ch1_destination_v,
                                      float *ch2_destination_v,
                                      uint32_t capacity,
                                      uint32_t requested_count,
                                      uint32_t timeout_ms,
                                      SignalAdc_FrameInfo *ch1_frame,
                                      SignalAdc_FrameInfo *ch2_frame);
SignalAdc_Status SignalAdc_Abort(void);

const SignalAdc_Info *SignalAdc_GetInfo(void);
const SignalAdc_Diagnostic *SignalAdc_GetDiagnostic(void);
const char *SignalAdc_StatusName(SignalAdc_Status status);
const char *SignalAdc_StageName(SignalAdc_Stage stage);

#endif /* SIGNAL_ADC_H */
