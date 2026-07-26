"""TI DSP 数值回归：覆盖 FFT、IQ、相关、FIR、峰值滤波和数值稳定性。"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
FFT_LENGTHS = (64, 128, 256, 512)  # 轻量FFT公开支持的长度。


def radix2_fft(values: np.ndarray, inverse: bool = False) -> np.ndarray:
    """按固件相同的原地 radix-2 流程计算复数FFT。"""
    data = np.asarray(values, dtype=np.complex64).copy()
    length = data.size
    bits = int(math.log2(length))
    order = np.array(
        [int(f"{index:0{bits}b}"[::-1], 2) for index in range(length)]
    )
    data = data[order]

    stage_size = 2
    direction = 1.0 if inverse else -1.0
    while stage_size <= length:
        half_size = stage_size // 2
        angle = direction * 2.0 * math.pi / stage_size
        step = np.complex64(complex(math.cos(angle), math.sin(angle)))
        for block in range(0, length, stage_size):
            twiddle = np.complex64(1.0 + 0.0j)
            for offset in range(half_size):
                even = block + offset
                odd = even + half_size
                product = np.complex64(twiddle * data[odd])
                even_value = data[even]
                data[even] = np.complex64(even_value + product)
                data[odd] = np.complex64(even_value - product)
                twiddle = np.complex64(twiddle * step)
        stage_size *= 2

    if inverse:
        data /= np.float32(length)
    return data


def test_fft() -> None:
    """验证所有公开FFT长度的正变换和逆变换。"""
    rng = np.random.default_rng(2023)
    for length in FFT_LENGTHS:
        signal = (rng.normal(size=length) + 1j * rng.normal(size=length)).astype(
            np.complex64
        )
        actual = radix2_fft(signal)
        expected = np.fft.fft(signal)
        error = float(np.max(np.abs(actual - expected)))
        assert error < 2.0e-3, (length, error)

        restored = radix2_fft(actual, inverse=True)
        error = float(np.max(np.abs(restored - signal)))
        assert error < 2.0e-4, (length, error)


def test_adc_fft_and_iq() -> None:
    """验证含ADC直流偏置、非整周期正弦的测频、测幅和IQ相位。"""
    length = 512
    sample_rate = 1_000_000.0
    frequency = 100_370.0
    amplitude = 0.82
    phase = 0.43
    index = np.arange(length, dtype=np.float32)
    signal = 1.65 + amplitude * np.cos(
        2.0 * np.pi * frequency * index / sample_rate + phase
    )

    centered = signal - np.mean(signal)
    window = np.hanning(length).astype(np.float32)
    spectrum = np.abs(radix2_fft((centered * window).astype(np.complex64)))
    spectrum *= length / float(np.sum(window))
    peak = int(np.argmax(spectrum[1 : length // 2]) + 1)
    left, center, right = spectrum[peak - 1 : peak + 2]
    log_left, log_center, log_right = np.log((left, center, right))
    offset = 0.5 * (log_left - log_right) / (
        log_left - 2.0 * log_center + log_right
    )
    angle = np.pi * abs(float(offset))
    hann_response = abs(
        np.sin(angle) / (angle * (1.0 - float(offset) * float(offset)))
    )
    measured_frequency = (peak + float(offset)) * sample_rate / length
    measured_amplitude = float(spectrum[peak] / hann_response * 2.0 / length)
    assert abs(measured_frequency - frequency) < 160.0
    assert abs(measured_amplitude - amplitude) < 0.02

    angle = 2.0 * np.pi * frequency * index / sample_rate
    component_i = 2.0 * float(np.sum(centered * np.cos(angle))) / length
    component_q = -2.0 * float(np.sum(centered * np.sin(angle))) / length
    iq_amplitude = math.hypot(component_i, component_q)
    iq_phase = math.atan2(component_q, component_i)
    phase_error = math.atan2(math.sin(iq_phase - phase), math.cos(iq_phase - phase))
    assert abs(iq_amplitude - amplitude) < 0.01
    assert abs(phase_error) < 0.015


def test_time_delay_sign() -> None:
    """验证y相对x延迟时返回正延迟。"""
    rng = np.random.default_rng(7)
    source = rng.normal(size=128).astype(np.float32) + 1.65
    expected_delay = 7
    delayed = np.zeros_like(source)
    delayed[expected_delay:] = source[:-expected_delay]
    delayed[:expected_delay] = 1.65
    source_centered = source - np.mean(source)
    delayed_centered = delayed - np.mean(delayed)
    max_lag = 16
    correlations = []
    for lag in range(-max_lag, max_lag + 1):
        start = max(lag, 0)
        end = len(source) + min(lag, 0)
        correlations.append(
            float(
                np.dot(
                    source_centered[start:end],
                    delayed_centered[start - lag : end - lag],
                )
            )
        )
    measured_delay = -int(np.argmax(correlations) - max_lag)
    assert measured_delay == expected_delay


def test_fir_cmsis_order() -> None:
    """验证CMSIS时间逆序系数能产生自然时序的冲激响应。"""
    natural_coefficients = np.array([0.2, -0.3, 0.7], dtype=np.float32)
    cmsis_coefficients = natural_coefficients[::-1]
    state = np.zeros(3, dtype=np.float32)
    output = []
    for sample in (1.0, 0.0, 0.0, 0.0):
        state[:-1] = state[1:]
        state[-1] = sample
        output.append(float(np.dot(state, cmsis_coefficients)))
    np.testing.assert_allclose(output[:3], natural_coefficients, atol=1.0e-7)


def test_peak_filter_gain() -> None:
    """验证峰值均衡在中心频率处确实产生设定增益。"""
    sample_rate = 20_000.0
    center_frequency = 2_000.0
    gain_db = 6.0
    quality = 1.2
    amplitude_gain = 10.0 ** (gain_db / 40.0)
    omega = 2.0 * math.pi * center_frequency / sample_rate
    alpha = math.sin(omega) / (2.0 * quality)
    b = np.array(
        [1.0 + alpha * amplitude_gain, -2.0 * math.cos(omega), 1.0 - alpha * amplitude_gain]
    )
    a = np.array(
        [1.0 + alpha / amplitude_gain, -2.0 * math.cos(omega), 1.0 - alpha / amplitude_gain]
    )
    z = complex(math.cos(omega), -math.sin(omega))
    response = (b[0] + b[1] * z + b[2] * z * z) / (
        a[0] + a[1] * z + a[2] * z * z
    )
    measured_db = 20.0 * math.log10(abs(response))
    assert abs(measured_db - gain_db) < 1.0e-5


def test_source_contracts() -> None:
    """检查修复后的实现约束没有被回退。"""
    math_source = (ROOT / "Port" / "arm_math_lite.c").read_text(encoding="utf-8")
    fft_source = (ROOT / "DSP" / "FFT.c").read_text(encoding="utf-8")
    filter_source = (ROOT / "DSP" / "Filter.c").read_text(encoding="utf-8")
    demod_header = (ROOT / "DSP" / "Demod.h").read_text(encoding="utf-8")
    assert "stage_size <<= 1U" in math_source
    assert "ARM_MATH_LITE_MAX_CFFT 512U" in math_source
    assert "FFT_RotateUnit" in fft_source
    assert "IIR_CascadePeakInit" in filter_source
    assert "DEMOD_EST_MEDIAN_MULTI_FRAME" not in demod_header


def main() -> None:
    """运行全部数值回归。"""
    test_fft()
    test_adc_fft_and_iq()
    test_time_delay_sign()
    test_fir_cmsis_order()
    test_peak_filter_gain()
    test_source_contracts()
    print("DSP verification passed: FFT/IQ/correlation/FIR/IIR/source contracts")


if __name__ == "__main__":
    main()
