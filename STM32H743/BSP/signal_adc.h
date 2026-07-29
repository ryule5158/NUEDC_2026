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

/*
 * Deterministic digital transfer function of the selected acquisition
 * backend.  Magnitude is linear V/V; phase is the output phase minus the
 * ADC-input phase in radians.  Callers must stay inside the advertised
 * response band.
 */
typedef float (*SignalAdc_DigitalResponseFn)(float frequency_hz);

typedef struct {
    const char *name;
    uint32_t physical_sample_clock_hz;
    float output_sample_rate_hz;
    uint32_t max_samples;
    uint8_t resolution_bits; /* Physical ADC resolution, before FPGA DSP. */
    float nominal_lsb_v;     /* Physical ADC input volts per native code. */
    const char *transport_format; /* Sample representation received by MCU. */
    float transport_lsb_v;        /* Active linear volts per transport LSB. */
    float transport_offset_v;     /* Input volts represented by transport zero. */
    uint8_t transport_fraction_bits;
    float transport_counts_per_native_code;
    const char *voltage_transfer_model;
    const char *voltage_calibration_source;
    uint8_t voltage_calibrated;
    float ac_amplitude_scale;
    const char *ac_amplitude_calibration_source;
    const char *amplitude_path;    /* Human-readable digital signal path. */
    const char *digital_response_model;
    float digital_response_min_hz; /* Inclusive deterministic-model band. */
    float digital_response_max_hz;
    SignalAdc_DigitalResponseFn digital_magnitude_response;
    SignalAdc_DigitalResponseFn digital_phase_response_rad;
    uint8_t deterministic_response_valid;
    uint8_t requires_external_antialias;
} SignalAdc_Info;

typedef struct {
    uint32_t sample_count;
    float sample_rate_hz;
    float min_v;
    float max_v;
    uint8_t clipped;
} SignalAdc_FrameInfo;

/*
 * Backend-native integer statistics used only by voltage calibration.
 * No nominal volts/code scale or empirical frequency calibration is applied.
 */
typedef struct {
    int64_t sample_sum;
    int32_t min_sample;
    int32_t max_sample;
    uint32_t sample_count;
    uint32_t overrange_count;
    uint8_t clipped;
} SignalAdc_RawFrameStats;

typedef struct {
    SignalAdc_Stage stage;
    uint32_t native_status;
    uint32_t device_id;
    uint32_t firmware_version;
    uint32_t response_profile;
    uint32_t captured_count;
    uint32_t read_count;
    uint8_t ready;
    uint8_t busy;
    uint8_t done;
} SignalAdc_Diagnostic;

SignalAdc_Status SignalAdc_Init(void);
SignalAdc_Status SignalAdc_Capture(float *destination_v,
                                  uint32_t capacity,
                                  uint32_t requested_count,
                                  uint32_t timeout_ms,
                                  SignalAdc_FrameInfo *frame);
SignalAdc_Status SignalAdc_CaptureRawStats(
    uint32_t requested_count,
    uint32_t timeout_ms,
    SignalAdc_RawFrameStats *stats);
SignalAdc_Status SignalAdc_Abort(void);

const SignalAdc_Info *SignalAdc_GetInfo(void);
const SignalAdc_Diagnostic *SignalAdc_GetDiagnostic(void);
const char *SignalAdc_StatusName(SignalAdc_Status status);
const char *SignalAdc_StageName(SignalAdc_Stage stage);

#endif /* SIGNAL_ADC_H */
