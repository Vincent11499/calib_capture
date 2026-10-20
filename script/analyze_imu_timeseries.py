#!/usr/bin/env python3
"""Plot IMU timestamp intervals, six axes, and temperature on a PC."""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

from calib_capture.imu_diagnostics import (
    AXES,
    AnalysisOutput,
    ImuDiagnosticError,
    column_statistics,
    default_output_directory,
    extrema_downsample,
    interval_histogram,
    load_window,
    report_base,
    scan_imu_csv,
    time_axis,
    timeseries_window,
    timing_statistics,
    write_json,
    write_statistics_csv,
)


def _pyplot():
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as pyplot
    except ImportError as exc:
        raise RuntimeError(
            "time-series analysis needs NumPy and Matplotlib in the PC environment"
        ) from exc
    return pyplot


def render_timestamp_intervals(
    timestamps, gaps_ms, histogram: dict, output: Path, max_points: int, dpi: int
) -> None:
    import numpy as np
    from matplotlib.ticker import StrMethodFormatter

    pyplot = _pyplot()
    elapsed, x_label, scale = time_axis(timestamps)
    plot_x, plot_y = extrema_downsample(elapsed[1:], gaps_ms, max_points)
    figure, (line_axis, histogram_axis) = pyplot.subplots(
        2, 1, figsize=(16, 10), gridspec_kw={"height_ratios": [2, 1]}
    )
    line_axis.plot(
        plot_x, plot_y, color="#70AD47", linewidth=0.75,
        label=f"IMU ({len(plot_y):,} of {len(gaps_ms):,} intervals)",
    )
    median = float(np.median(gaps_ms))
    line_axis.axhline(
        median, color="#C00000", linestyle="--", linewidth=0.9,
        label=f"median {median:.6f} ms",
    )
    lower = histogram["minimum_ms"]
    upper = histogram["maximum_ms"]
    line_axis.axhline(
        lower, color="#777777", linestyle="--", linewidth=0.8,
        label=f"lower threshold {lower:g} ms",
    )
    line_axis.axhline(
        upper, color="#444444", linestyle=":", linewidth=0.9,
        label=f"upper threshold {upper:g} ms",
    )
    duration_s = float(timestamps[-1] - timestamps[0]) / 1e9
    duration_minutes = int(round(duration_s / 60.0))
    duration_hours, duration_remainder = divmod(duration_minutes, 60)
    line_axis.set_title(
        "IMU header timestamp delta\n"
        f"Total time: {duration_hours:02d} h {duration_remainder:02d} min"
    )
    line_axis.set_xlabel(x_label)
    line_axis.set_ylabel("Interval (ms)")
    line_axis.grid(True, linewidth=0.5, alpha=0.35)
    line_axis.legend(loc="best")

    bins = histogram["bins"]
    positions = np.arange(len(bins))
    counts = np.asarray([item["count"] for item in bins], dtype=np.int64)
    bars = histogram_axis.bar(positions, counts, color="#70AD47", alpha=0.9)
    histogram_axis.set_xticks(positions)
    histogram_axis.set_xticklabels(
        [item["label"] for item in bins], rotation=45, ha="right"
    )
    histogram_axis.set_title(
        "IMU header timestamp delta distribution\n"
        f"Observed range: {float(gaps_ms.min()):.3f} to "
        f"{float(gaps_ms.max()):.3f} ms"
    )
    histogram_axis.set_xlabel("Interval (ms), right-closed bins")
    histogram_axis.set_ylabel("Sample count")
    histogram_axis.yaxis.set_major_formatter(StrMethodFormatter("{x:,.0f}"))
    histogram_axis.grid(axis="y", linewidth=0.5, alpha=0.25)
    maximum_count = max(int(counts.max()), 1)
    histogram_axis.set_ylim(0, maximum_count * 1.26)
    for bar, item in zip(bars, bins):
        count = item["count"]
        histogram_axis.text(
            bar.get_x() + bar.get_width() / 2,
            count + maximum_count * 0.012,
            f"{count:,}",
            ha="center", va="bottom", rotation=90, fontsize=7,
            color="#548235",
        )
    below = histogram["at_or_below_minimum"]
    above = histogram["above_maximum"]
    histogram_axis.text(
        0.98, 0.96,
        f"IMU >{upper:g} ms ratio: {above['count']:,}/"
        f"{histogram['total_intervals']:,} = {above['ratio']:.6%}\n"
        f"IMU <={lower:g} ms ratio: {below['count']:,}/"
        f"{histogram['total_intervals']:,} = {below['ratio']:.6%}",
        transform=histogram_axis.transAxes,
        ha="right", va="top", color="#548235", fontweight="bold",
        bbox=dict(facecolor="white", edgecolor="#548235", alpha=0.88),
    )
    figure.tight_layout()
    figure.savefig(output, dpi=dpi)
    pyplot.close(figure)


def render_axes_temperature(
    timestamps, values, output: Path, max_points: int, dpi: int
) -> None:
    pyplot = _pyplot()
    elapsed, x_label, _ = time_axis(timestamps)
    figure, panels = pyplot.subplots(
        3, 1, figsize=(16, 12), sharex=True,
        gridspec_kw={"height_ratios": [1, 1, 0.65]},
    )
    colors = ("#4472C4", "#ED7D31", "#70AD47")
    for index, name in enumerate(AXES[:3]):
        x, y = extrema_downsample(elapsed, values[:, index], max_points)
        panels[0].plot(x, y, linewidth=0.7, color=colors[index], label=name)
    panels[0].set_title("Gyroscope")
    panels[0].set_ylabel("Angular velocity (rad/s)")
    panels[0].legend(loc="best", ncol=3)

    for index, name in enumerate(AXES[3:]):
        x, y = extrema_downsample(elapsed, values[:, index + 3], max_points)
        panels[1].plot(x, y, linewidth=0.7, color=colors[index], label=name)
    panels[1].set_title("Accelerometer")
    panels[1].set_ylabel("Acceleration (m/s²)")
    panels[1].legend(loc="best", ncol=3)

    x, y = extrema_downsample(elapsed, values[:, 6], max_points)
    panels[2].plot(x, y, linewidth=0.8, color="#A64D79", label="temperature")
    panels[2].set_title("Temperature (latest received value held between updates)")
    panels[2].set_ylabel("Temperature (°C)")
    panels[2].set_xlabel(x_label)
    panels[2].legend(loc="best")
    for axis in panels:
        axis.grid(True, linewidth=0.5, alpha=0.35)
    figure.tight_layout()
    figure.savefig(output, dpi=dpi)
    pyplot.close(figure)


def analyze(
    dataset: str | Path,
    *,
    output: str | Path | None = None,
    start_s: float | None = None,
    end_s: float | None = None,
    max_plot_points: int = 20_000,
    hist_min_ms: float = 7.5,
    hist_max_ms: float = 20.0,
    hist_bin_width_ms: float = 1.0,
    dpi: int = 160,
) -> tuple[Path, dict]:
    if max_plot_points < 4:
        raise ImuDiagnosticError("--max-plot-points must be at least 4")
    if dpi <= 0:
        raise ImuDiagnosticError("--dpi must be greater than zero")
    scan = scan_imu_csv(dataset)
    window = timeseries_window(scan, start_s, end_s)
    selection = load_window(scan, window)
    timing, gaps_ms = timing_statistics(selection.timestamps_ns)
    histogram = interval_histogram(
        gaps_ms, hist_min_ms, hist_max_ms, hist_bin_width_ms
    )
    columns = column_statistics(selection.values)
    final = (
        Path(output).expanduser().resolve()
        if output
        else default_output_directory(scan, "timeseries", window)
    )
    report = report_base("imu_timeseries_analysis", scan, selection)
    report.update(
        status="pass",
        output=str(final),
        timing=timing,
        timestamp_histogram=histogram,
        sensors=columns,
        plots={
            "timestamp_intervals": "timestamp_intervals.png",
            "imu_axes_temperature": "imu_axes_temperature.png",
        },
        plotting={
            "max_points_per_line": max_plot_points,
            "downsampling": "per-bucket min/max envelope",
            "dpi": dpi,
        },
    )
    with AnalysisOutput(final) as staging:
        render_timestamp_intervals(
            selection.timestamps_ns,
            gaps_ms,
            histogram,
            staging / "timestamp_intervals.png",
            max_plot_points,
            dpi,
        )
        render_axes_temperature(
            selection.timestamps_ns,
            selection.values,
            staging / "imu_axes_temperature.png",
            max_plot_points,
            dpi,
        )
        write_statistics_csv(
            staging / "statistics.csv", timing, columns, histogram
        )
        write_json(staging / "summary.json", report)
    return final, report


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dataset", help="IMU session directory or <session>/imu/imu0.csv")
    parser.add_argument("--output", help="exact new result directory")
    parser.add_argument("--start-s", type=float, help="inclusive seconds from first timestamp")
    parser.add_argument("--end-s", type=float, help="exclusive seconds from first timestamp")
    parser.add_argument("--max-plot-points", type=int, default=20_000)
    parser.add_argument("--hist-min-ms", type=float, default=7.5)
    parser.add_argument("--hist-max-ms", type=float, default=20.0)
    parser.add_argument("--hist-bin-width-ms", type=float, default=1.0)
    parser.add_argument("--dpi", type=int, default=160)
    return parser


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    try:
        output, report = analyze(**vars(args))
    except (ImuDiagnosticError, RuntimeError, OSError, FileExistsError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    timing = report["timing"]
    print(f"output: {output}")
    print(
        f"samples: {timing['samples']:,}; duration: {timing['duration_s']:.6f} s; "
        f"rate: {timing['effective_rate_hz']:.6f} Hz"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
