#!/usr/bin/env python3
"""Verify the exact quantized CIC5/R65 + FIR81 magnitude response.

The FIR coefficients are parsed from the production RTL rather than copied
into this script.  The default interference sweep covers every 500 Hz from
1 MHz through the 65 MHz Nyquist edge of the 130 MSPS converter.
"""

from __future__ import annotations

import argparse
import math
import pathlib
import re
import sys


ADC_HZ = 130_000_000.0
DECIMATION = 65
OUTPUT_HZ = ADC_HZ / DECIMATION
CIC_STAGES = 5
FIR_FRACTION_BITS = 47
FIR_CENTER = 40
PASS_LIMIT_DB = 0.030
STOP_LIMIT_DB = -42.0
INTERFERENCE_VPP_MV = 200.0


def parse_unique_coefficients(rtl_path: pathlib.Path) -> list[int]:
    text = rtl_path.read_text(encoding="utf-8")
    matches = re.findall(
        r"6'd(\d+):\s*FIR81_Coefficient\s*=\s*"
        r"(-?)18'sd(\d+)\s*;",
        text,
    )
    values: dict[int, int] = {}
    for index_text, sign, magnitude_text in matches:
        index = int(index_text)
        magnitude = int(magnitude_text)
        values[index] = -magnitude if sign else magnitude
    expected = list(range(41))
    if sorted(values) != expected:
        raise RuntimeError(
            f"expected FIR81 coefficient indices 0..40, got {sorted(values)}"
        )
    return [values[index] for index in expected]


def cic_raw_magnitude(frequency_hz: float) -> float:
    x = math.pi * frequency_hz / ADC_HZ
    denominator = math.sin(x)
    if abs(denominator) < 1.0e-15:
        return float(DECIMATION**CIC_STAGES)
    return abs(math.sin(DECIMATION * x) / denominator) ** CIC_STAGES


def fir_magnitude(unique: list[int], frequency_hz: float) -> float:
    omega = 2.0 * math.pi * frequency_hz / OUTPUT_HZ
    symmetric_sum = float(unique[FIR_CENTER])
    for index in range(FIR_CENTER):
        symmetric_sum += 2.0 * unique[index] * math.cos(
            omega * (FIR_CENTER - index)
        )
    return abs(symmetric_sum) / float(1 << FIR_FRACTION_BITS)


def combined_gain(unique: list[int], frequency_hz: float) -> float:
    return cic_raw_magnitude(frequency_hz) * fir_magnitude(
        unique, frequency_hz
    )


def to_db(gain: float) -> float:
    if gain <= 0.0:
        return float("-inf")
    return 20.0 * math.log10(gain)


def inclusive_frequencies(start_hz: int, stop_hz: int, step_hz: int):
    frequency = start_hz
    while frequency <= stop_hz:
        yield frequency
        frequency += step_hz
    if (stop_hz - start_hz) % step_hz:
        yield stop_hz


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--step-hz",
        type=int,
        default=500,
        help="dense 1..65 MHz sweep step (default: 500 Hz)",
    )
    args = parser.parse_args()
    if args.step_hz <= 0:
        parser.error("--step-hz must be positive")

    root_dir = pathlib.Path(__file__).resolve().parents[1]
    rtl_path = root_dir / "rtl" / "ltc2208_cic5_fir81_decimator.v"
    unique = parse_unique_coefficients(rtl_path)

    pass_min = (float("inf"), 0)
    pass_max = (float("-inf"), 0)
    for frequency in inclusive_frequencies(10_000, 500_000, 500):
        gain_db = to_db(combined_gain(unique, float(frequency)))
        if gain_db < pass_min[0]:
            pass_min = (gain_db, frequency)
        if gain_db > pass_max[0]:
            pass_max = (gain_db, frequency)

    worst_stop = (float("-inf"), 0, 0.0)
    point_count = 0
    for frequency in inclusive_frequencies(
        1_000_000, 65_000_000, args.step_hz
    ):
        gain = combined_gain(unique, float(frequency))
        gain_db = to_db(gain)
        point_count += 1
        if gain_db > worst_stop[0]:
            worst_stop = (gain_db, frequency, gain)

    print(f"RTL={rtl_path}")
    print(
        "CHAIN=130MSPS -> CIC5/R65 -> FIR81(Q47) -> 2MSPS "
        f"coefficients=41 sweep_points={point_count} step_hz={args.step_hz}"
    )
    for frequency in (
        10_000,
        100_000,
        200_000,
        300_000,
        400_000,
        500_000,
    ):
        gain = combined_gain(unique, float(frequency))
        print(
            f"PASS frequency_hz={frequency} gain={gain:.9f} "
            f"gain_db={to_db(gain):+.6f}"
        )
    print(
        f"PASS_BAND min_db={pass_min[0]:+.6f}@{pass_min[1]}Hz "
        f"max_db={pass_max[0]:+.6f}@{pass_max[1]}Hz "
        f"ripple_db={pass_max[0] - pass_min[0]:.6f}"
    )
    residual_vpp_mv = worst_stop[2] * INTERFERENCE_VPP_MV
    print(
        f"INTERFERENCE_WORST frequency_hz={worst_stop[1]} "
        f"gain_db={worst_stop[0]:.6f} "
        f"residual_for_200mVpp_mVpp={residual_vpp_mv:.6f}"
    )
    print(
        "STEADY_STATE_GROUP_DELAY cic_adc_clocks=160 "
        "fir_output_samples=40 nominal_total_us=21.230769"
    )

    failures: list[str] = []
    if max(abs(pass_min[0]), abs(pass_max[0])) > PASS_LIMIT_DB:
        failures.append(
            f"passband exceeds +/-{PASS_LIMIT_DB:.3f} dB"
        )
    if worst_stop[0] > STOP_LIMIT_DB:
        failures.append(
            f"1..65 MHz worst leakage exceeds {STOP_LIMIT_DB:.1f} dB"
        )
    if residual_vpp_mv > 1.6:
        failures.append("200 mVpp interference residual exceeds 1.6 mVpp")

    if failures:
        for failure in failures:
            print(f"FAIL: {failure}", file=sys.stderr)
        return 1
    print("LTC2208_DENSE_RESPONSE_ANALYSIS_PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
