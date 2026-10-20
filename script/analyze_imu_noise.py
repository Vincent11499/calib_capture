#!/usr/bin/env python3
"""Assess per-axis Gaussianity and temporal whiteness of stationary IMU data."""
from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path

from calib_capture.imu_diagnostics import (
    AXES,
    UNITS,
    AnalysisOutput,
    ImuDiagnosticError,
    default_output_directory,
    load_window,
    noise_window,
    report_base,
    scan_imu_csv,
    timing_statistics,
    write_json,
)


GAUSSIAN_SKEW_LIMIT = 0.1
GAUSSIAN_KURTOSIS_LIMIT = 0.2
GAUSSIAN_QQ_R_MIN = 0.995
VERDICT_RANK = {"pass": 0, "approximate": 1, "fail": 2}


def _dependencies():
    try:
        import numpy as np
        from scipy import signal, stats
    except ImportError as exc:
        raise RuntimeError(
            "noise diagnostics require NumPy, SciPy and Matplotlib on the PC"
        ) from exc
    return np, signal, stats


def _pyplot():
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as pyplot
    except ImportError as exc:
        raise RuntimeError(
            "noise diagnostics require NumPy, SciPy and Matplotlib on the PC"
        ) from exc
    return pyplot


def _verdict(gaussian: str, white: str) -> str:
    return max((gaussian, white), key=VERDICT_RANK.__getitem__)


def analyze_axis(
    raw_values,
    *,
    rate_hz: float,
    max_lag_s: float,
    alpha: float,
    acf_limit: float,
):
    np, signal, stats = _dependencies()
    values = np.asarray(raw_values, dtype=np.float64)
    if len(values) < 20:
        raise ImuDiagnosticError("noise analysis requires at least 20 samples")
    mean = float(values.mean())
    noise = values - mean
    stddev = float(noise.std(ddof=1))
    if not math.isfinite(stddev) or stddev <= 0:
        raise ImuDiagnosticError("constant axis cannot be assessed as Gaussian white noise")

    skew = float(stats.skew(noise, bias=False))
    excess_kurtosis = float(stats.kurtosis(noise, fisher=True, bias=False))
    normal_k2, normal_p = (float(value) for value in stats.normaltest(noise))
    anderson = stats.anderson(noise, dist="norm")
    desired_percent = alpha * 100.0
    anderson_index = min(
        range(len(anderson.significance_level)),
        key=lambda index: abs(float(anderson.significance_level[index]) - desired_percent),
    )
    anderson_level = float(anderson.significance_level[anderson_index]) / 100.0
    anderson_critical = float(anderson.critical_values[anderson_index])
    anderson_statistic = float(anderson.statistic)
    (qq_theoretical, qq_ordered), (_, _, qq_r) = stats.probplot(
        noise, dist="norm", fit=True
    )
    qq_r = float(qq_r)
    gaussian_practical = (
        abs(skew) <= GAUSSIAN_SKEW_LIMIT
        and abs(excess_kurtosis) <= GAUSSIAN_KURTOSIS_LIMIT
        and qq_r >= GAUSSIAN_QQ_R_MIN
    )
    gaussian_statistical = (
        normal_p >= alpha and anderson_statistic <= anderson_critical
    )
    gaussian_verdict = (
        "fail"
        if not gaussian_practical
        else ("pass" if gaussian_statistical else "approximate")
    )

    max_lag = min(int(round(max_lag_s * rate_hz)), len(noise) - 2)
    if max_lag < 1:
        raise ImuDiagnosticError("selected duration is too short for --max-lag-s")
    correlation = signal.correlate(noise, noise, mode="full", method="fft")
    center = len(noise) - 1
    acf = np.asarray(
        correlation[center:center + max_lag + 1] / correlation[center],
        dtype=np.float64,
    )
    acf_lags = acf[1:]
    max_abs_acf = float(np.max(np.abs(acf_lags)))
    max_abs_index = int(np.argmax(np.abs(acf_lags))) + 1
    confidence = 1.96 / math.sqrt(len(noise))
    outside_confidence = int(np.count_nonzero(np.abs(acf_lags) > confidence))
    lag_numbers = np.arange(1, max_lag + 1, dtype=np.float64)
    ljung_q = float(
        len(noise) * (len(noise) + 2)
        * np.sum((acf_lags * acf_lags) / (len(noise) - lag_numbers))
    )
    ljung_p = float(stats.chi2.sf(ljung_q, max_lag))
    white_practical = max_abs_acf <= acf_limit
    white_statistical = ljung_p >= alpha
    white_verdict = (
        "fail"
        if not white_practical
        else ("pass" if white_statistical else "approximate")
    )

    nperseg = min(
        len(noise),
        8192,
        max(64, int(round(rate_hz * 8.0))),
    )
    frequency, psd = signal.welch(
        noise, fs=rate_hz, nperseg=nperseg, detrend="constant", scaling="density"
    )
    positive_psd = psd[1:][psd[1:] > 0]
    spectral_flatness = (
        float(math.exp(float(np.mean(np.log(positive_psd)))) / np.mean(positive_psd))
        if len(positive_psd)
        else 0.0
    )
    normalized = noise / stddev
    coverage = {
        "within_1_sigma": float(np.mean(np.abs(normalized) <= 1.0)),
        "within_2_sigma": float(np.mean(np.abs(normalized) <= 2.0)),
        "within_3_sigma": float(np.mean(np.abs(normalized) <= 3.0)),
    }
    result = {
        "unit": None,
        "samples": int(len(values)),
        "raw_mean": mean,
        "noise_stddev": stddev,
        "noise_rms": float(math.sqrt(float(np.mean(noise * noise)))),
        "noise_peak_to_peak": float(noise.max() - noise.min()),
        "gaussian": {
            "verdict": gaussian_verdict,
            "practical_pass": gaussian_practical,
            "statistical_pass": gaussian_statistical,
            "skew": skew,
            "absolute_skew_limit": GAUSSIAN_SKEW_LIMIT,
            "excess_kurtosis": excess_kurtosis,
            "absolute_excess_kurtosis_limit": GAUSSIAN_KURTOSIS_LIMIT,
            "qq_r": qq_r,
            "qq_r_minimum": GAUSSIAN_QQ_R_MIN,
            "dagostino_k2": normal_k2,
            "dagostino_p": normal_p,
            "anderson_statistic": anderson_statistic,
            "anderson_critical": anderson_critical,
            "anderson_significance": anderson_level,
            **coverage,
        },
        "white_noise": {
            "verdict": white_verdict,
            "practical_pass": white_practical,
            "statistical_pass": white_statistical,
            "maximum_lag_samples": max_lag,
            "maximum_lag_s": max_lag / rate_hz,
            "max_abs_acf": max_abs_acf,
            "max_abs_acf_limit": acf_limit,
            "max_abs_acf_lag_samples": max_abs_index,
            "max_abs_acf_lag_s": max_abs_index / rate_hz,
            "acf_95_percent_confidence": confidence,
            "acf_lags_outside_confidence": outside_confidence,
            "ljung_box_q": ljung_q,
            "ljung_box_degrees_of_freedom": max_lag,
            "ljung_box_p": ljung_p,
            "spectral_flatness": spectral_flatness,
            "welch_nperseg": nperseg,
        },
        "overall_verdict": _verdict(gaussian_verdict, white_verdict),
    }
    curves = {
        "noise": noise,
        "qq_theoretical": np.asarray(qq_theoretical),
        "qq_ordered": np.asarray(qq_ordered),
        "acf": acf,
        "frequency": frequency,
        "psd": psd,
    }
    return result, curves


def _histogram_edges(values, np):
    edges = np.histogram_bin_edges(values, bins="fd")
    if len(edges) < 21:
        edges = np.linspace(float(values.min()), float(values.max()), 21)
    elif len(edges) > 201:
        edges = np.linspace(float(values.min()), float(values.max()), 201)
    return edges


def render_plots(curves, results, output: Path, dpi: int, acf_limit: float) -> None:
    np, _, stats = _dependencies()
    pyplot = _pyplot()
    colors = ("#4472C4", "#ED7D31", "#70AD47", "#5B9BD5", "#FFC000", "#A5A5A5")

    distribution, panels = pyplot.subplots(2, 3, figsize=(16, 9))
    qq_figure, qq_panels = pyplot.subplots(2, 3, figsize=(16, 9))
    acf_figure, acf_panels = pyplot.subplots(2, 3, figsize=(16, 9))
    psd_figure, psd_panels = pyplot.subplots(2, 3, figsize=(16, 9))

    for index, name in enumerate(AXES):
        result = results[name]
        curve = curves[name]
        noise = curve["noise"]
        row, column = divmod(index, 3)
        color = colors[index]

        axis = panels[row, column]
        edges = _histogram_edges(noise, np)
        axis.hist(noise, bins=edges, density=True, color=color, alpha=0.7)
        x_values = np.linspace(float(noise.min()), float(noise.max()), 500)
        axis.plot(
            x_values,
            stats.norm.pdf(x_values, 0.0, result["noise_stddev"]),
            color="#C00000",
            linewidth=1.2,
            label="fitted normal PDF",
        )
        axis.set_title(
            f"{name}: Gaussian {result['gaussian']['verdict']}"
        )
        axis.set_xlabel(f"Mean-centered noise ({UNITS[name]})")
        axis.set_ylabel("Density")
        axis.grid(True, alpha=0.25)
        axis.legend(loc="best")

        axis = qq_panels[row, column]
        theoretical = curve["qq_theoretical"]
        ordered = curve["qq_ordered"]
        plot_indices = np.linspace(
            0, len(theoretical) - 1, min(5000, len(theoretical)), dtype=np.int64
        )
        axis.scatter(
            theoretical[plot_indices], ordered[plot_indices],
            s=3, alpha=0.45, color=color,
        )
        slope, intercept = np.polyfit(theoretical, ordered, 1)
        axis.plot(
            (theoretical[0], theoretical[-1]),
            (slope * theoretical[0] + intercept, slope * theoretical[-1] + intercept),
            color="#C00000", linewidth=1.0,
        )
        axis.set_title(f"{name}: Q-Q r={result['gaussian']['qq_r']:.6f}")
        axis.set_xlabel("Theoretical normal quantile")
        axis.set_ylabel(f"Observed noise ({UNITS[name]})")
        axis.grid(True, alpha=0.25)

        axis = acf_panels[row, column]
        lag_s = np.arange(len(curve["acf"])) / (
            result["white_noise"]["maximum_lag_samples"]
            / result["white_noise"]["maximum_lag_s"]
        )
        axis.plot(lag_s[1:], curve["acf"][1:], color=color, linewidth=0.75)
        confidence = result["white_noise"]["acf_95_percent_confidence"]
        axis.axhline(confidence, color="#777777", linestyle="--", linewidth=0.8)
        axis.axhline(-confidence, color="#777777", linestyle="--", linewidth=0.8)
        axis.axhline(acf_limit, color="#C00000", linestyle=":", linewidth=0.9)
        axis.axhline(-acf_limit, color="#C00000", linestyle=":", linewidth=0.9)
        axis.set_title(
            f"{name}: white {result['white_noise']['verdict']}; "
            f"max |ACF|={result['white_noise']['max_abs_acf']:.4f}"
        )
        axis.set_xlabel("Lag (s)")
        axis.set_ylabel("Autocorrelation")
        axis.grid(True, alpha=0.25)

        axis = psd_panels[row, column]
        mask = (curve["frequency"] > 0) & (curve["psd"] > 0)
        axis.semilogy(
            curve["frequency"][mask], curve["psd"][mask],
            color=color, linewidth=0.8,
        )
        axis.set_title(
            f"{name}: spectral flatness="
            f"{result['white_noise']['spectral_flatness']:.4f}"
        )
        axis.set_xlabel("Frequency (Hz)")
        axis.set_ylabel(f"PSD ({UNITS[name]}²/Hz)")
        axis.grid(True, which="both", alpha=0.25)

    for figure, filename in (
        (distribution, "noise_distribution.png"),
        (qq_figure, "noise_qq.png"),
        (acf_figure, "noise_acf.png"),
        (psd_figure, "noise_psd.png"),
    ):
        figure.tight_layout()
        figure.savefig(output / filename, dpi=dpi)
        pyplot.close(figure)


def write_axis_csv(path: Path, axes: dict) -> None:
    fields = (
        "axis", "unit", "samples", "raw_mean", "noise_stddev",
        "noise_rms", "noise_peak_to_peak", "gaussian_verdict",
        "skew", "excess_kurtosis", "qq_r", "dagostino_p",
        "anderson_statistic", "anderson_critical", "white_noise_verdict",
        "max_abs_acf", "max_abs_acf_lag_s", "ljung_box_p",
        "spectral_flatness", "overall_verdict",
    )
    with path.open("x", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for name in AXES:
            value = axes[name]
            writer.writerow({
                "axis": name,
                "unit": value["unit"],
                "samples": value["samples"],
                "raw_mean": value["raw_mean"],
                "noise_stddev": value["noise_stddev"],
                "noise_rms": value["noise_rms"],
                "noise_peak_to_peak": value["noise_peak_to_peak"],
                "gaussian_verdict": value["gaussian"]["verdict"],
                "skew": value["gaussian"]["skew"],
                "excess_kurtosis": value["gaussian"]["excess_kurtosis"],
                "qq_r": value["gaussian"]["qq_r"],
                "dagostino_p": value["gaussian"]["dagostino_p"],
                "anderson_statistic": value["gaussian"]["anderson_statistic"],
                "anderson_critical": value["gaussian"]["anderson_critical"],
                "white_noise_verdict": value["white_noise"]["verdict"],
                "max_abs_acf": value["white_noise"]["max_abs_acf"],
                "max_abs_acf_lag_s": value["white_noise"]["max_abs_acf_lag_s"],
                "ljung_box_p": value["white_noise"]["ljung_box_p"],
                "spectral_flatness": value["white_noise"]["spectral_flatness"],
                "overall_verdict": value["overall_verdict"],
            })


def analyze(
    dataset: str | Path,
    *,
    output: str | Path | None = None,
    start_s: float | None = None,
    end_s: float | None = None,
    duration_s: float | None = None,
    max_lag_s: float = 10.0,
    alpha: float = 0.05,
    acf_limit: float = 0.05,
    dpi: int = 160,
) -> tuple[Path, dict]:
    for value, name in (
        (max_lag_s, "--max-lag-s"),
        (alpha, "--alpha"),
        (acf_limit, "--acf-limit"),
    ):
        if not math.isfinite(value) or value <= 0:
            raise ImuDiagnosticError(f"{name} must be finite and positive")
    if alpha >= 1 or acf_limit > 1:
        raise ImuDiagnosticError("--alpha must be < 1 and --acf-limit must be <= 1")
    if dpi <= 0:
        raise ImuDiagnosticError("--dpi must be greater than zero")

    scan = scan_imu_csv(dataset)
    window = noise_window(scan, start_s, end_s, duration_s)
    selection = load_window(scan, window, minimum_samples=20)
    timing, _ = timing_statistics(selection.timestamps_ns)
    rate_hz = timing["effective_rate_hz"]
    axes = {}
    curves = {}
    for index, name in enumerate(AXES):
        result, curve = analyze_axis(
            selection.values[:, index],
            rate_hz=rate_hz,
            max_lag_s=max_lag_s,
            alpha=alpha,
            acf_limit=acf_limit,
        )
        result["unit"] = UNITS[name]
        axes[name] = result
        curves[name] = curve
    overall = max(
        (value["overall_verdict"] for value in axes.values()),
        key=VERDICT_RANK.__getitem__,
    )
    warnings = []
    if timing["intervals_outside_10_percent"]:
        warnings.append(
            "sampling is not perfectly uniform; ACF uses sample order and Welch PSD "
            "uses the measured average rate; no interpolation was performed"
        )
    final = (
        Path(output).expanduser().resolve()
        if output
        else default_output_directory(scan, "noise", window)
    )
    report = report_base("imu_noise_diagnostics", scan, selection)
    report.update(
        status="completed",
        output=str(final),
        stationary_input_asserted_by_user=True,
        preprocessing={
            "method": "per-axis mean subtraction only",
            "linear_detrend": False,
            "filtered": False,
            "resampled": False,
        },
        thresholds={
            "alpha": alpha,
            "absolute_skew_limit": GAUSSIAN_SKEW_LIMIT,
            "absolute_excess_kurtosis_limit": GAUSSIAN_KURTOSIS_LIMIT,
            "qq_r_minimum": GAUSSIAN_QQ_R_MIN,
            "max_abs_acf_limit": acf_limit,
            "max_lag_s": max_lag_s,
        },
        timing=timing,
        axes=axes,
        overall_verdict=overall,
        warnings=warnings,
        interpretation=(
            "pass/approximate/fail are diagnostic thresholds only and do not "
            "certify the final Kalibr noise model"
        ),
        plots={
            "distribution": "noise_distribution.png",
            "qq": "noise_qq.png",
            "acf": "noise_acf.png",
            "psd": "noise_psd.png",
        },
    )
    with AnalysisOutput(final) as staging:
        render_plots(curves, axes, staging, dpi, acf_limit)
        write_axis_csv(staging / "axis_verdicts.csv", axes)
        write_json(staging / "noise_report.json", report)
    return final, report


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dataset", help="IMU session directory or <session>/imu/imu0.csv")
    parser.add_argument("--output", help="exact new result directory")
    parser.add_argument("--start-s", type=float, help="inclusive seconds from first timestamp")
    parser.add_argument("--end-s", type=float, help="exclusive seconds from first timestamp")
    parser.add_argument(
        "--duration-s", type=float,
        help="window duration; default 1800 s, centered when no boundary is given",
    )
    parser.add_argument("--max-lag-s", type=float, default=10.0)
    parser.add_argument("--alpha", type=float, default=0.05)
    parser.add_argument("--acf-limit", type=float, default=0.05)
    parser.add_argument("--dpi", type=int, default=160)
    return parser


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    try:
        output, report = analyze(**vars(args))
    except (ImuDiagnosticError, RuntimeError, OSError, FileExistsError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    print(f"output: {output}")
    for name in AXES:
        axis = report["axes"][name]
        print(
            f"{name}: Gaussian={axis['gaussian']['verdict']}, "
            f"white={axis['white_noise']['verdict']}, "
            f"overall={axis['overall_verdict']}"
        )
    print(f"overall: {report['overall_verdict']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
