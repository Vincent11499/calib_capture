"""Offline overlapping Allan deviation/variance for imu-continuous datasets."""
from __future__ import annotations

import csv
import json
import math
from decimal import Decimal, ROUND_CEILING, ROUND_HALF_UP
from pathlib import Path

from .config import APP_VERSION
from .imu_only import validate_imu_only

AXES = ("wx", "wy", "wz", "ax", "ay", "az")
DEFAULT_OUTPUT_ROOT = Path(__file__).resolve().parents[2] / "data/allan_variance/allantools_output"


def add_options(parser):
    parser.add_argument("dataset", help="imu-continuous session directory")
    parser.add_argument("--output", help="exact result directory; default project data/allan_variance/allantools_output/<input-name>[_<start>h-<end>h]")
    parser.add_argument("--rate-hz", type=float, help="explicit sample rate; default measured from timestamps")
    parser.add_argument("--start-s", type=float, help="inclusive start, seconds from first IMU stamp (default 0)")
    parser.add_argument("--end-s", type=float, help="exclusive end, seconds from first IMU stamp (default end of recording)")
    parser.add_argument("--taus", choices=("decade", "octave"), default="decade")
    parser.add_argument("--kalibr-noise", action="store_true", help="fit noise regions; export complete candidate kalibr_imu.yaml only")
    parser.add_argument("--kalibr-imu-topic", default="/imu/data", help="rostopic in exported Kalibr YAML (default /imu/data)")
    parser.add_argument("--allow-irregular", action="store_true",
                        help="explicitly approximate irregular data as equally spaced; no interpolation")


def timing_summary(stamps, rate_hz=None):
    import numpy as np
    if len(stamps) < 4:
        raise ValueError("Allan analysis requires at least four samples")
    gaps = np.diff(stamps)  # int64 subtraction before any float conversion.
    if np.any(gaps <= 0):
        raise ValueError("timestamps must be strictly increasing")
    measured = (len(stamps) - 1) * 1e9 / int(stamps[-1] - stamps[0])
    rate = measured if rate_hz is None else rate_hz
    if not math.isfinite(rate) or rate <= 0:
        raise ValueError("rate-hz must be finite and positive")
    relative = np.abs(gaps * (rate / 1e9) - 1)
    return {
        "samples": len(stamps), "measured_rate_hz": measured, "used_rate_hz": rate,
        "duration_s": int(stamps[-1] - stamps[0]) / 1e9,
        "minimum_interval_ms": int(gaps.min()) / 1e6,
        "maximum_interval_ms": int(gaps.max()) / 1e6,
        "interval_stddev_ms": float(gaps.std()) / 1e6,
        "maximum_relative_interval_error": float(relative.max()),
        "intervals_outside_10_percent": int(np.count_nonzero(relative > 0.1)),
    }


def select_time_window(data, start_s=0.0, end_s=None):
    """Slice [start, end) relative to the first stamp, without float timestamp rounding."""
    import numpy as np
    if not math.isfinite(start_s) or start_s < 0:
        raise ValueError("start-s must be finite and nonnegative")
    if end_s is not None and (not math.isfinite(end_s) or end_s <= start_s):
        raise ValueError("end-s must be finite and greater than start-s")
    if not len(data):
        raise ValueError("empty IMU recording")
    origin = int(data["timestamp_ns"][0])
    offsets = data["timestamp_ns"] - origin
    def boundary(seconds):
        ns = int((Decimal(str(seconds)) * 1_000_000_000).to_integral_value(rounding=ROUND_CEILING))
        return min(ns, int(offsets[-1]) + 1)
    first = int(np.searchsorted(offsets, boundary(start_s), side="left"))
    stop = len(data) if end_s is None else int(np.searchsorted(offsets, boundary(end_s), side="left"))
    selected = data[first:stop]
    if len(selected) < 4:
        raise ValueError("selected time interval requires at least four samples")
    return selected, {
        "reference_timestamp_ns": origin, "requested_start_s": start_s, "requested_end_s": end_s,
        "interval": "[start, end)", "source_samples": len(data), "selected_samples": len(selected),
        "excluded_before": first, "excluded_after": len(data) - stop,
        "first_timestamp_ns": int(selected["timestamp_ns"][0]),
        "last_timestamp_ns": int(selected["timestamp_ns"][-1]),
        "actual_start_s": int(offsets[first]) / 1e9, "actual_last_s": int(offsets[stop - 1]) / 1e9,
    }


def analysis_directory_name(input_name, start_s=None, end_s=None):
    """Rounded hours label only; exact selection remains in the report."""
    if start_s is None and end_s is None:
        return input_name
    def hours(seconds):
        value = Decimal(str(seconds)) / 3600
        label = str(int(value)) if value == value.to_integral_value() else str(value.quantize(Decimal("0.1"), rounding=ROUND_HALF_UP))
        return label + "h"
    start = hours(0 if start_s is None else start_s)
    end = "end" if end_s is None else hours(end_s)
    return f"{input_name}_{start}-{end}"


def analyze_allan(dataset, *, output=None, rate_hz=None, taus="decade", allow_irregular=False,
                  start_s=None, end_s=None, kalibr_noise=False, kalibr_imu_topic="/imu/data"):
    try:
        import numpy as np
        import allantools
        from matplotlib.figure import Figure
        from matplotlib.backends.backend_agg import FigureCanvasAgg
    except ImportError as exc:
        raise RuntimeError("Allan analysis needs allantools, numpy and matplotlib in the PC Python environment; "
                           "no packages were installed automatically") from exc
    if taus not in {"decade", "octave"}:
        raise ValueError("taus must be decade or octave")
    if not isinstance(kalibr_imu_topic, str) or not kalibr_imu_topic.strip():
        raise ValueError("kalibr-imu-topic must not be empty")
    source = Path(dataset).resolve()
    if source.name.endswith((".inprogress", ".failed")):
        raise ValueError("use a completed IMU recording, not .inprogress or .failed")
    validation = validate_imu_only(source, write_report=False)
    if validation.errors:
        raise ValueError("invalid IMU dataset: " + "; ".join(validation.errors))
    # Typed load keeps nanosecond timestamps integral and all sensor values float64.
    with (source / "imu/imu0.csv").open(encoding="utf-8", newline="") as stream:
        header = tuple(next(csv.reader(stream), ()))
    has_temperature = header == ("timestamp_ns", *AXES, "temperature")
    if not has_temperature and header != ("timestamp_ns", *AXES):
        raise ValueError("unsupported IMU CSV header")
    dtype = [("timestamp_ns", "i8")] + [(name, "f8") for name in AXES]
    if has_temperature:
        dtype.append(("temperature", "f8"))
    data = np.loadtxt(source / "imu/imu0.csv", delimiter=",", skiprows=1, dtype=dtype, ndmin=1)
    data, selection = select_time_window(data, 0.0 if start_s is None else start_s, end_s)
    timing = timing_summary(data["timestamp_ns"], rate_hz)
    if timing["intervals_outside_10_percent"] and not allow_irregular:
        raise ValueError("sampling intervals deviate by more than 10% from the analysis period; "
                         "inspect timestamps or explicitly use --allow-irregular for an approximation")
    destination = Path(output).resolve() if output else DEFAULT_OUTPUT_ROOT / analysis_directory_name(source.name, start_s, end_s)
    staging = destination.with_name(destination.name + ".inprogress")
    failed = destination.with_name(destination.name + ".failed")
    if destination == source or source in destination.parents or destination in source.parents:
        raise ValueError("analysis output must be separate from the source dataset")
    if any(p.exists() for p in (destination, staging, failed)):
        raise FileExistsError(f"analysis output already exists: {destination}")
    warnings = list(validation.warnings)
    if timing["intervals_outside_10_percent"]:
        warnings.append("irregular samples treated as equally spaced; Allan results are approximate")
    warnings.append("stationarity and physical outliers are not automatically certified; no samples removed within selected interval")
    report = {
        "schema_version": APP_VERSION, "kind": "imu_allan_analysis",
        "status": "warning", "source": str(source), "output": str(destination),
        "method": "allantools.oadev", "allantools_version": getattr(allantools, "__version__", "unknown"),
        "data_type": "freq", "taus": taus, "timing": timing, "warnings": warnings,
        "selection": selection,
        "allow_irregular": allow_irregular, "resampled": False, "noise_fit": "not_performed",
        "temperature": ({"available": True, "unit": "degC",
                         "minimum": float(data["temperature"].min()),
                         "maximum": float(data["temperature"].max()),
                         "mean": float(data["temperature"].mean())}
                        if has_temperature else {"available": False}), "axes": {},
    }
    staging.mkdir(parents=True)
    try:
        figures = {}
        curves = {}
        for quantity in ("adev", "avar"):
            fig = Figure(figsize=(12, 5))
            FigureCanvasAgg(fig)
            figures[quantity] = (fig, fig.subplots(1, 2))
        for index, name in enumerate(AXES):
            values = data[name]
            tau, adev, error, count = allantools.oadev(
                values, rate=timing["used_rate_hz"], data_type="freq", taus=taus)
            tau, adev, error, count = (np.asarray(a) for a in (tau, adev, error, count))
            curves[name] = (tau, adev)
            avar = adev**2
            if not len(tau) or not all(np.all(np.isfinite(a)) for a in (tau, adev, avar, error, count)):
                raise ValueError(f"invalid Allan output for {name}")
            with (staging / f"allan_{name}.csv").open("x", newline="", encoding="utf-8") as stream:
                writer = csv.writer(stream)
                writer.writerow(("tau_s", "adev", "avar", "adev_error", "n"))
                writer.writerows((float(t), float(d), float(v), float(e), int(n))
                                 for t, d, v, e, n in zip(tau, adev, avar, error, count))
            unit = "rad/s" if index < 3 else "m/s^2"
            report["axes"][name] = {"unit": unit, "mean": float(values.mean()),
                                    "stddev": float(values.std()), "zero_count": int(np.count_nonzero(values == 0)),
                                    "tau_points": len(tau), "maximum_tau_s": float(tau[-1])}
            for quantity, curve in (("adev", adev), ("avar", avar)):
                ax = figures[quantity][1][index // 3]
                mask = (tau > 0) & (curve > 0)
                ax.loglog(tau[mask], curve[mask], ".-", label=name)
            if np.any(adev == 0):
                warnings.append(f"{name}: zero deviation values retained in CSV but omitted from log plot")
        for quantity, (fig, panels) in figures.items():
            for ax, title, unit in zip(panels, ("Gyroscope", "Accelerometer"), ("rad/s", "m/s^2")):
                ax.set_title(title)
                ax.set_xlabel("Averaging time tau (s)")
                label = f"Allan deviation ({unit})" if quantity == "adev" else f"Allan variance (({unit})^2)"
                ax.set_ylabel(label)
                ax.grid(True, which="both", alpha=0.3)
                ax.legend()
            fig.tight_layout()
            fig.savefig(staging / ("allan_deviation.png" if quantity == "adev" else "allan_variance.png"), dpi=160)
            fig.clear()
        if kalibr_noise:
            from .kalibr_noise import fit_kalibr_noise, write_kalibr_yaml
            fitted = fit_kalibr_noise(curves, timing["duration_s"], timing["used_rate_hz"])
            report["noise_fit"] = fitted
            if fitted["status"] == "candidate":
                write_kalibr_yaml(staging / "kalibr_imu.yaml", fitted, kalibr_imu_topic)
                fitted["yaml_file"] = "kalibr_imu.yaml"
                warnings.append("Kalibr noise YAML is a candidate fit, not calibration quality approval")
            else:
                warnings.append("Kalibr YAML not generated; missing fits: " + ", ".join(fitted["missing"]))
        (staging / "analysis.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8")
        if destination.exists():
            raise FileExistsError(destination)
        staging.rename(destination)
    except BaseException:
        if staging.exists() and not failed.exists():
            staging.rename(failed)
        raise
    return report
