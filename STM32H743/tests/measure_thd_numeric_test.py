"""Pure-Python numerical regression for the THD spectrum algorithm."""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass
class Config:
    fund_min_hz: float = 0.0
    fund_max_hz: float = 0.0
    min_fundamental_amplitude: float = 0.0
    max_harmonic: int = 5
    required_harmonic: int = 5
    search_bins: int = 1
    integration_bins: int = 2
    subtract_noise: bool = True


def _peak_interp(magnitude: list[float], index: int) -> tuple[float, float]:
    left, center, right = magnitude[index - 1 : index + 2]
    denominator = left - 2.0 * center + right
    delta = (
        0.0
        if abs(denominator) <= 1e-12
        else 0.5 * (left - right) / denominator
    )
    delta = min(max(delta, -0.5), 0.5)
    amplitude = center - 0.25 * (left - right) * delta
    return index + delta, amplitude


def measure_thd(
    magnitude: list[float], sample_rate: float, config: Config
) -> dict[str, object]:
    fft_length = 2 * (len(magnitude) - 1)
    half = fft_length // 2
    bin_width = sample_rate / fft_length
    search_min = (
        config.fund_min_hz if config.fund_min_hz > 0.0 else bin_width
    )
    search_max = (
        config.fund_max_hz
        if config.fund_max_hz > 0.0
        else 0.5 * sample_rate - bin_width
    )
    first = max(1, math.ceil(search_min / bin_width - 0.5))
    last = min(half - 1, math.floor(search_max / bin_width + 0.5))
    if first > last:
        return {"status": "no_fundamental"}
    fundamental_index = max(
        range(first, last + 1), key=magnitude.__getitem__
    )
    fundamental_bin, fundamental_peak = _peak_interp(
        magnitude, fundamental_index
    )
    if fundamental_peak < config.min_fundamental_amplitude:
        return {"status": "low_amplitude"}

    minimum_spacing = 2 * (config.search_bins + config.integration_bins) + 1
    if fundamental_bin < minimum_spacing:
        return {"status": "resolution"}

    centers: list[int] = []
    frequencies: list[float] = []
    raw_powers: list[float] = []
    cluster_counts: list[int] = []
    for harmonic in range(1, config.max_harmonic + 1):
        predicted = harmonic * fundamental_bin
        if harmonic == 1:
            center = fundamental_index
        else:
            if (
                predicted + config.search_bins + config.integration_bins
                >= half
            ):
                break
            rounded = int(predicted + 0.5)
            low = max(1, rounded - config.search_bins)
            high = min(half - 1, rounded + config.search_bins)
            center = max(range(low, high + 1), key=magnitude.__getitem__)
        if (
            center <= config.integration_bins
            or center + config.integration_bins >= half
        ):
            break
        component_bin, _ = _peak_interp(magnitude, center)
        low = center - config.integration_bins
        high = center + config.integration_bins
        cluster = magnitude[low : high + 1]
        centers.append(center)
        frequencies.append(component_bin * bin_width)
        raw_powers.append(sum(value * value for value in cluster))
        cluster_counts.append(len(cluster))

    def is_component_bin(bin_index: int) -> bool:
        return any(
            abs(bin_index - center) <= config.integration_bins
            for center in centers
        )

    noise_samples = [
        magnitude[index]
        for index in range(1, half)
        if not is_component_bin(index)
    ]
    noise_power_per_bin = (
        sum(value * value for value in noise_samples) / len(noise_samples)
        if noise_samples
        else 0.0
    )
    if noise_power_per_bin > 0.0:
        clipped_noise = [
            value
            for value in noise_samples
            if value * value <= 4.0 * noise_power_per_bin
        ]
        if clipped_noise:
            noise_power_per_bin = sum(
                value * value for value in clipped_noise
            ) / len(clipped_noise)

    powers = list(raw_powers)
    if config.subtract_noise:
        powers = [
            max(power - noise_power_per_bin * count, 0.0)
            for power, count in zip(powers, cluster_counts)
        ]
    amplitudes = [math.sqrt(power) for power in powers]
    ratios = [amplitude / amplitudes[0] for amplitude in amplitudes]
    thd = math.sqrt(sum(amplitude * amplitude for amplitude in amplitudes[1:]))
    thd /= amplitudes[0]
    status = "ok" if len(centers) >= config.required_harmonic else "bandwidth"
    return {
        "status": status,
        "frequency": frequencies[0],
        "frequencies": frequencies,
        "ratios": ratios,
        "thd": thd,
        "thd_percent": 100.0 * thd,
    }


def make_spectrum(
    sample_rate: float,
    fundamental: float,
    ratios: tuple[float, ...],
    *,
    fft_length: int = 1024,
    spur: tuple[float, float] | None = None,
) -> list[float]:
    samples: list[float] = []
    window: list[float] = []
    for index in range(fft_length):
        time = index / sample_rate
        value = 0.21
        for harmonic, ratio in enumerate(ratios, start=1):
            value += ratio * math.sin(
                2.0 * math.pi * harmonic * fundamental * time
                + 0.17 * harmonic
            )
        if spur is not None:
            spur_frequency, spur_amplitude = spur
            value += spur_amplitude * math.sin(
                2.0 * math.pi * spur_frequency * time + 0.31
            )
        value += 0.0005 * math.sin(
            2.0 * math.pi * 7311.0 * time + 0.9
        )
        samples.append(value)
        window.append(
            0.5
            - 0.5
            * math.cos(2.0 * math.pi * index / (fft_length - 1))
        )

    window_sum = sum(window)
    spectrum: list[float] = []
    for bin_index in range(fft_length // 2 + 1):
        real = 0.0
        imaginary = 0.0
        angle_step = -2.0 * math.pi * bin_index / fft_length
        for index, (sample, coefficient) in enumerate(zip(samples, window)):
            angle = angle_step * index
            windowed = sample * coefficient
            real += windowed * math.cos(angle)
            imaginary += windowed * math.sin(angle)
        scale = 1.0 if bin_index in (0, fft_length // 2) else 2.0
        spectrum.append(scale * math.hypot(real, imaginary) / window_sum)
    return spectrum


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def main() -> None:
    ratios = (1.0, 0.30, 0.20, 0.10, 0.05)
    expected_thd = math.sqrt(sum(value * value for value in ratios[1:]))

    # Non-coherent sampling at both ends of the 2021 bonus range.
    for sample_rate, fundamental in (
        (40_000.0, 1_000.37),
        (1_200_000.0, 99_937.0),
    ):
        config = Config(
            fund_min_hz=0.9 * fundamental,
            fund_max_hz=1.1 * fundamental,
        )
        result = measure_thd(
            make_spectrum(sample_rate, fundamental, ratios),
            sample_rate,
            config,
        )
        require(result["status"] == "ok", f"{fundamental:g} Hz status")
        require(
            abs(float(result["thd"]) - expected_thd) < 0.003,
            f"{fundamental:g} Hz THD accuracy",
        )
        require(
            abs(float(result["frequency"]) - fundamental)
            < sample_rate / 1024,
            f"{fundamental:g} Hz frequency accuracy",
        )
        require(
            abs(float(result["ratios"][4]) - ratios[4]) < 0.003,
            f"{fundamental:g} Hz H5 normalized amplitude",
        )

    # A larger out-of-band spur must not replace the configured fundamental
    # or raise the clipped noise-floor correction enough to erase H4/H5.
    sample_rate = 1_200_000.0
    fundamental = 10_000.0
    config = Config(fund_min_hz=9_000.0, fund_max_hz=11_000.0)
    result = measure_thd(
        make_spectrum(
            sample_rate,
            fundamental,
            ratios,
            spur=(350_000.0, 1.5),
        ),
        sample_rate,
        config,
    )
    require(result["status"] == "ok", "range-limited search status")
    require(
        abs(float(result["frequency"]) - fundamental)
        < sample_rate / 1024,
        "strong out-of-band spur rejected",
    )
    require(
        abs(float(result["thd"]) - expected_thd) < 0.003,
        "strong spur does not corrupt THD noise correction",
    )

    # Five harmonics are impossible unless fs/2 is above 5*f0.
    low_sample_rate = 200_000.0
    high_fundamental = 30_000.0
    config = Config(fund_min_hz=29_000.0, fund_max_hz=31_000.0)
    result = measure_thd(
        make_spectrum(low_sample_rate, high_fundamental, ratios),
        low_sample_rate,
        config,
    )
    require(result["status"] == "bandwidth", "fifth-harmonic bandwidth status")
    require(len(result["frequencies"]) == 3, "partial H1-H3 preservation")

    print("measure_thd_numeric_test: PASS")


if __name__ == "__main__":
    main()
