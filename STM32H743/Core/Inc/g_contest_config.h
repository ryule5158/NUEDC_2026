#ifndef G_CONTEST_CONFIG_H
#define G_CONTEST_CONFIG_H

#define G_CONTEST_SAMPLE_COUNT              4096U
#define G_CONTEST_CAPTURE_TIMEOUT_MS        250U
#define G_CONTEST_CAPTURE_MAX_ATTEMPTS      2U
#define G_CONTEST_CAPTURE_RETRY_DELAY_MS    2U

/* Raw two-point voltage-calibration acquisition (USART1 CAL/Q mode). */
#define G_CONTEST_CALIBRATION_FRAMES        16U
#define G_CONTEST_CALIBRATION_SAMPLES       4096U

#define G_CONTEST_MIN_FREQ_HZ               10000.0f
#define G_CONTEST_MAX_FREQ_HZ               500000.0f
#define G_CONTEST_MIN_COMPONENT_PEAK_V      0.0005f

/*
 * 第三问的 1 MHz 带外干扰采用“联合建模后剔除”，不让 10~500 kHz
 * 被测信号经过额外陷波器。搜索宽度覆盖信号源与采样时钟的小量频偏；
 * 阈值单位是 FPGA 数字链路之后、分析器输入端的 V_RMS。
 */
#define G_CONTEST_INTERFERENCE_FREQ_HZ       1000000.0f
#define G_CONTEST_INTERFERENCE_SEARCH_HZ     5000.0f
#define G_CONTEST_INTERFERENCE_MIN_RMS_V     0.001f

#define G_CONTEST_WAVEFORM_POINTS_PER_CYCLE 256U
#define G_CONTEST_MAX_WAVEFORM_CYCLES       3U

#endif /* G_CONTEST_CONFIG_H */
