#!/usr/bin/env python3
"""De-embed AD9238 channel mismatch from BODE THRU/FILTER serial logs."""

from __future__ import annotations

import argparse
import bisect
import csv
import math
from pathlib import Path

CALIBRATION_FREQUENCIES_HZ = (
    10_000.0,
    100_000.0,
    125_000.0,
    150_000.0,
    200_000.0,
    250_000.0,
    400_000.0,
    500_000.0,
)


def parse_logs(paths: list[Path], include_low_quality: bool) -> dict[str, list[dict[str, float]]]:
    modes: dict[str, list[dict[str, float]]] = {"THRU": [], "FILTER": []}
    for path in paths:
        for raw_line in path.read_text(encoding="utf-8", errors="ignore").splitlines():
            line = raw_line.strip()
            marker = line.find("BODE,")
            if marker < 0:
                continue
            fields = line[marker:].split(",")
            if len(fields) != 14 or fields[1] not in modes:
                continue
            try:
                point = {
                    "frequency_hz": float(fields[2]),
                    "ch1_vpp_mv": float(fields[3]),
                    "ch2_vpp_mv": float(fields[4]),
                    "gain_linear": float(fields[5]),
                    "gain_db": float(fields[6]),
                    "phase_wrapped_deg": float(fields[7]),
                    "phase_unwrapped_deg": float(fields[8]),
                    "ch1_residual_pct": float(fields[9]),
                    "ch2_residual_pct": float(fields[10]),
                    "ch1_clip": float(fields[11]),
                    "ch2_clip": float(fields[12]),
                    "quality": float(fields[13]),
                }
            except ValueError:
                continue
            if include_low_quality or point["quality"] >= 0.5:
                modes[fields[1]].append(point)
    for points in modes.values():
        points.sort(key=lambda point: point["frequency_hz"])
        previous_wrapped: float | None = None
        previous_unwrapped = 0.0
        for point in points:
            wrapped = point["phase_wrapped_deg"]
            if previous_wrapped is None:
                unwrapped = wrapped
            else:
                unwrapped = previous_unwrapped + wrap_degrees(
                    wrapped - previous_wrapped)
            point["phase_unwrapped_deg"] = unwrapped
            previous_wrapped = wrapped
            previous_unwrapped = unwrapped
    return modes


def interpolate(points: list[dict[str, float]], frequency_hz: float, field: str) -> float:
    frequencies = [point["frequency_hz"] for point in points]
    if not frequencies or frequency_hz < frequencies[0] or frequency_hz > frequencies[-1]:
        raise ValueError("frequency outside baseline span")
    right = bisect.bisect_left(frequencies, frequency_hz)
    if right == 0:
        return points[0][field]
    if right == len(points):
        return points[-1][field]
    left = right - 1
    f0, f1 = frequencies[left], frequencies[right]
    if f1 == f0:
        return 0.5 * (points[left][field] + points[right][field])
    fraction = (frequency_hz - f0) / (f1 - f0)
    return points[left][field] + fraction * (points[right][field] - points[left][field])


def wrap_degrees(phase_deg: float) -> float:
    return (phase_deg + 180.0) % 360.0 - 180.0


def corrected_points(thru: list[dict[str, float]], filt: list[dict[str, float]]) -> list[dict[str, float]]:
    output: list[dict[str, float]] = []
    previous_wrapped: float | None = None
    previous_unwrapped = 0.0
    for point in filt:
        frequency_hz = point["frequency_hz"]
        try:
            thru_gain = interpolate(thru, frequency_hz, "gain_linear")
            thru_phase = interpolate(thru, frequency_hz, "phase_unwrapped_deg")
        except ValueError:
            continue
        if thru_gain <= 0.0 or point["gain_linear"] <= 0.0:
            continue
        gain = point["gain_linear"] / thru_gain
        wrapped = wrap_degrees(point["phase_unwrapped_deg"] - thru_phase)
        if previous_wrapped is None:
            unwrapped = wrapped
        else:
            unwrapped = previous_unwrapped + wrap_degrees(wrapped - previous_wrapped)
        output.append(
            {
                "frequency_hz": frequency_hz,
                "lc_gain_linear": gain,
                "lc_gain_db": 20.0 * math.log10(gain),
                "lc_phase_wrapped_deg": wrapped,
                "lc_phase_unwrapped_deg": unwrapped,
                "amplitude_correction": 1.0 / gain,
                "phase_correction_deg": -unwrapped,
                "filter_ch1_vpp_mv": point["ch1_vpp_mv"],
                "filter_ch2_vpp_mv": point["ch2_vpp_mv"],
            }
        )
        previous_wrapped = wrapped
        previous_unwrapped = unwrapped
    return output


def write_csv(path: Path, rows: list[dict[str, float]]) -> None:
    if not rows:
        raise ValueError("no corrected points")
    with path.open("w", newline="", encoding="utf-8-sig") as output:
        writer = csv.DictWriter(output, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def sample_calibration(points: list[dict[str, float]]) -> list[dict[str, float]]:
    result: list[dict[str, float]] = []
    for frequency_hz in CALIBRATION_FREQUENCIES_HZ:
        result.append(
            {
                "frequency_hz": frequency_hz,
                "amplitude_correction": interpolate(points, frequency_hz, "amplitude_correction"),
                "phase_correction_deg": interpolate(points, frequency_hz, "phase_correction_deg"),
            }
        )
    return result


def svg_polyline(points: list[dict[str, float]], field: str,
                 x0: float, y0: float, width: float, height: float) -> tuple[str, float, float]:
    values = [point[field] for point in points]
    low, high = min(values), max(values)
    if math.isclose(low, high):
        low -= 1.0
        high += 1.0
    padding = 0.08 * (high - low)
    low -= padding
    high += padding
    fmin = points[0]["frequency_hz"]
    fmax = points[-1]["frequency_hz"]
    coordinates = []
    for point in points:
        x = x0 + width * (point["frequency_hz"] - fmin) / (fmax - fmin)
        y = y0 + height * (high - point[field]) / (high - low)
        coordinates.append(f"{x:.2f},{y:.2f}")
    return " ".join(coordinates), low, high


def write_svg(path: Path, points: list[dict[str, float]]) -> None:
    width, height = 1100, 720
    plot_x, plot_width, plot_height = 90.0, 950.0, 235.0
    gain_line, gain_low, gain_high = svg_polyline(
        points, "lc_gain_db", plot_x, 75.0, plot_width, plot_height)
    phase_line, phase_low, phase_high = svg_polyline(
        points, "lc_phase_unwrapped_deg", plot_x, 410.0, plot_width, plot_height)
    fmin = points[0]["frequency_hz"] / 1000.0
    fmax = points[-1]["frequency_hz"] / 1000.0
    svg = f'''<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">
<rect width="100%" height="100%" fill="white"/>
<style>text{{font-family:Arial,"Microsoft YaHei",sans-serif;fill:#222}} .axis{{stroke:#333;stroke-width:1}} .curve{{fill:none;stroke:#1261a0;stroke-width:2}}</style>
<text x="550" y="32" text-anchor="middle" font-size="22">LC 滤波器校准后幅频与相频曲线</text>
<text x="20" y="190" transform="rotate(-90 20 190)" text-anchor="middle">增益 / dB</text>
<line class="axis" x1="{plot_x}" y1="75" x2="{plot_x}" y2="310"/><line class="axis" x1="{plot_x}" y1="310" x2="1040" y2="310"/>
<polyline class="curve" points="{gain_line}"/><text x="82" y="82" text-anchor="end">{gain_high:.2f}</text><text x="82" y="310" text-anchor="end">{gain_low:.2f}</text>
<text x="20" y="525" transform="rotate(-90 20 525)" text-anchor="middle">相位 / °</text>
<line class="axis" x1="{plot_x}" y1="410" x2="{plot_x}" y2="645"/><line class="axis" x1="{plot_x}" y1="645" x2="1040" y2="645"/>
<polyline class="curve" points="{phase_line}"/><text x="82" y="417" text-anchor="end">{phase_high:.1f}</text><text x="82" y="645" text-anchor="end">{phase_low:.1f}</text>
<text x="90" y="680">{fmin:.1f} kHz</text><text x="1040" y="680" text-anchor="end">{fmax:.1f} kHz</text><text x="565" y="705" text-anchor="middle">频率</text>
</svg>'''
    path.write_text(svg, encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Process STM32 BODE THRU/FILTER logs into de-embedded LC curves.")
    parser.add_argument("logs", nargs="+", type=Path, help="serial log file(s)")
    parser.add_argument("--output-dir", type=Path, default=Path("bode_output"))
    parser.add_argument("--include-low-quality", action="store_true")
    args = parser.parse_args()

    modes = parse_logs(args.logs, args.include_low_quality)
    if len(modes["THRU"]) < 2 or len(modes["FILTER"]) < 2:
        raise SystemExit("Need at least two valid THRU and FILTER points.")
    corrected = corrected_points(modes["THRU"], modes["FILTER"])
    if len(corrected) < 2:
        raise SystemExit("THRU and FILTER frequency spans do not overlap.")

    args.output_dir.mkdir(parents=True, exist_ok=True)
    curve_path = args.output_dir / "lc_response_corrected.csv"
    calibration_path = args.output_dir / "lc_inverse_calibration_8points.csv"
    svg_path = args.output_dir / "lc_response_corrected.svg"
    write_csv(curve_path, corrected)
    write_csv(calibration_path, sample_calibration(corrected))
    write_svg(svg_path, corrected)
    print(f"Wrote {len(corrected)} corrected points: {curve_path}")
    print(f"Wrote firmware calibration points: {calibration_path}")
    print(f"Wrote plot: {svg_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
