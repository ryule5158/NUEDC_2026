#!/usr/bin/env python3
"""Fail-closed transport-grid comparison for LTC2208 and AD9238 G builds.

This proves that the LTC2208 CIC/FIR fractional output is not truncated before
the MCU. It deliberately does not treat the three retained fractional bits as
new ADC ENOB, nor claim the unmeasured analog path is already calibrated.
"""

from __future__ import annotations

import math
import pathlib
import re
import sys


PROJECT = pathlib.Path(__file__).resolve().parents[1]
REPOSITORY = PROJECT.parent
AD9238_PROJECT = REPOSITORY / "STM32H743_AD9238"


def read(path: pathlib.Path) -> str:
    if not path.is_file():
        raise RuntimeError(f"required source not found: {path}")
    return path.read_text(encoding="utf-8")


def macro_number(text: str, name: str) -> float:
    match = re.search(
        rf"(?m)^\s*#define\s+{re.escape(name)}\s+"
        rf"\(?\s*([-+]?\d+(?:\.\d+)?)f?\s*\)?",
        text,
    )
    if not match:
        raise RuntimeError(f"numeric macro not found: {name}")
    return float(match.group(1))


def macro_uint(text: str, name: str) -> int:
    return int(round(macro_number(text, name)))


def macro_hex(text: str, name: str) -> int:
    match = re.search(
        rf"(?m)^\s*#define\s+{re.escape(name)}\s+"
        rf"(0x[0-9a-fA-F]+)[uUlL]*",
        text,
    )
    if not match:
        raise RuntimeError(f"hex macro not found: {name}")
    return int(match.group(1), 16)


def main() -> int:
    ltc_header = read(PROJECT / "Modules" / "ltc2208.h")
    ltc_config = read(PROJECT / "Core" / "Inc" / "g_contest_config.h")
    ad_header = read(AD9238_PROJECT / "Modules" / "ad9238.h")
    ad_config = read(
        AD9238_PROJECT / "Core" / "Inc" / "g_contest_config.h"
    )

    ltc_span_v = (
        macro_number(ltc_header, "LTC2208_MODULE_INPUT_MAX_V")
        - macro_number(ltc_header, "LTC2208_MODULE_INPUT_MIN_V")
    )
    ad_span_v = (
        macro_number(ad_header, "AD9238_MODULE_INPUT_MAX_V")
        - macro_number(ad_header, "AD9238_MODULE_INPUT_MIN_V")
    )
    ltc_gain = macro_number(
        ltc_config, "G_CONTEST_FRONTEND_VOLTAGE_GAIN"
    )
    ad_gain = macro_number(
        ad_config, "G_CONTEST_FRONTEND_VOLTAGE_GAIN"
    )
    ltc_word_bits = macro_uint(ltc_header, "LTC2208_SAMPLE_WORD_BITS")
    ltc_fraction_bits = macro_uint(
        ltc_header, "LTC2208_SAMPLE_FRACTION_BITS"
    )
    ltc_native_bits = macro_uint(
        ltc_header, "LTC2208_NATIVE_RESOLUTION_BITS"
    )
    ltc_protocol = macro_hex(
        ltc_header, "LTC2208_CAPTURE_PROTOCOL_Q3_VERSION"
    )
    ad_word_bits = macro_uint(ad_header, "AD9238_SAMPLE_WORD_BITS")
    ad_fraction_bits = macro_uint(
        ad_header, "AD9238_SAMPLE_FRACTION_BITS"
    )

    ltc_physical_lsb_v = ltc_span_v / (2.0**ltc_native_bits) / ltc_gain
    ltc_transport_grid_v = ltc_physical_lsb_v / (2.0**ltc_fraction_bits)
    ad_transport_grid_v = ad_span_v / (2.0**ad_word_bits) / ad_gain
    improvement = ad_transport_grid_v / ltc_transport_grid_v
    minimum_component_peak_v = 0.005
    ltc_minimum_transport_counts = (
        minimum_component_peak_v / ltc_transport_grid_v
    )
    ltc_minimum_physical_codes = minimum_component_peak_v / ltc_physical_lsb_v
    ad_minimum_transport_counts = (
        minimum_component_peak_v / ad_transport_grid_v
    )
    official_max_vpp_v = 0.250
    design_max_vpp_v = 0.450
    official_utilization = official_max_vpp_v * ltc_gain / ltc_span_v
    design_utilization = design_max_vpp_v * ltc_gain / ltc_span_v
    transport_grid_rms_uv = (
        ltc_transport_grid_v / math.sqrt(12.0) * 1.0e6
    )

    print(
        f"LTC2208 nominal_physical_lsb_uV={ltc_physical_lsb_v * 1e6:.6f} "
        f"transport_grid_uV={ltc_transport_grid_v * 1e6:.6f} "
        f"word_bits={ltc_word_bits} fraction_bits={ltc_fraction_bits} "
        f"protocol=0x{ltc_protocol:08X}"
    )
    print(
        f"AD9238 configured_Q8.8_input_grid_uV="
        f"{ad_transport_grid_v * 1e6:.6f} "
        f"word_bits={ad_word_bits} fraction_bits={ad_fraction_bits} "
        f"frontend_gain={ad_gain:.6f}"
    )
    print(
        f"TRANSPORT_GRID_ADVANTAGE={improvement:.6f}x "
        f"minimum_5mV_peak_counts_ltc={ltc_minimum_transport_counts:.3f} "
        f"ad9238={ad_minimum_transport_counts:.3f} "
        f"ltc_physical_codes={ltc_minimum_physical_codes:.3f}"
    )
    print(
        f"LTC2208_transport_grid_rms_uV={transport_grid_rms_uv:.6f} "
        f"official_250mVpp_utilization={official_utilization:.6f} "
        f"design_450mVpp_utilization={design_utilization:.6f}"
    )

    failures: list[str] = []
    if ltc_word_bits != 20 or ltc_fraction_bits != 3:
        failures.append("LTC2208 transport is not signed-20 Q3")
    if ltc_protocol != 0x00030002:
        failures.append("LTC2208 protocol version is not 0x00030002")
    if not re.search(
        r"typedef\s+int32_t\s+LTC2208_SampleTypeDef\s*;",
        ltc_header,
    ):
        failures.append("LTC2208 sample typedef is not int32_t")
    if ad_word_bits != 16 or ad_fraction_bits != 8:
        failures.append("AD9238 reference transport is not signed Q8.8")
    if abs(ltc_gain - 1.0) > 1.0e-7:
        failures.append("external OPA690 gain is not configured as unity")
    if ltc_physical_lsb_v > 35.0e-6:
        failures.append("LTC2208 nominal physical LSB exceeds 35 uV")
    if not 4.29e-6 <= ltc_transport_grid_v <= 4.30e-6:
        failures.append("LTC2208 Q3 transport grid is not about 4.2915 uV")
    if not 15.9e-6 <= ad_transport_grid_v <= 16.1e-6:
        failures.append("AD9238 Q8.8 input grid is not about 15.978 uV")
    if improvement < 3.7:
        failures.append("LTC2208 transport-grid advantage is below 3.7x")
    if ltc_minimum_physical_codes < 140.0:
        failures.append("official 5 mV peak component uses fewer than 140 codes")
    if ltc_minimum_transport_counts < 1100.0:
        failures.append("official 5 mV peak uses fewer than 1100 Q3 counts")
    if design_utilization >= 0.25:
        failures.append("450 mVpp design maximum has insufficient headroom")

    if failures:
        for failure in failures:
            print(f"FAIL: {failure}", file=sys.stderr)
        return 1
    print("LTC2208_Q3_PRECISION_BUDGET_PASS")
    print("Q3_RETENTION_IS_NOT_THREE_EXTRA_ADC_BITS")
    print("PHYSICAL_BNC_CALIBRATION_STILL_REQUIRED")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

