"""Conservative slope-region fitting for the Kalibr continuous-time noise model.

The region checks are engineering heuristics, not statistical confidence bounds.
Reference: https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model
"""
import json
import math

AXES = ("wx", "wy", "wz", "ax", "ay", "az")


def fit_region(tau, adev, duration_s, slope):
    """Find a contiguous near-power-law region; never substitute a curve minimum."""
    import numpy as np
    tau, adev = np.asarray(tau, dtype=float), np.asarray(adev, dtype=float)
    if (tau.ndim != 1 or tau.shape != adev.shape or not math.isfinite(duration_s)
            or duration_s <= 0 or slope not in (-0.5, 0.5)):
        raise ValueError("invalid noise fitting inputs")
    if not np.all(np.isfinite(tau)) or np.any(tau <= 0) or np.any(np.diff(tau) <= 0):
        raise ValueError("fit tau must be finite, positive and strictly increasing")
    valid = np.isfinite(adev) & (adev > 0) & (tau <= duration_s / 10)
    candidates = []
    for first in range(len(tau)):
        for stop in range(first + 4, len(tau) + 1):
            if not np.all(valid[first:stop]):
                continue
            x, y = np.log10(tau[first:stop]), np.log10(adev[first:stop])
            span = float(x[-1] - x[0])
            if span < 0.7:
                continue
            measured_slope = float(np.polyfit(x, y, 1)[0])
            intercept = float(np.mean(y - slope * x))
            residual = float(np.sqrt(np.mean((y - (slope * x + intercept)) ** 2)))
            local_slopes = np.diff(y) / np.diff(x)
            if (abs(measured_slope - slope) > 0.15 or residual > 0.08
                    or np.any(np.abs(local_slopes - slope) > 0.3)):
                continue
            # sigma(tau)=N/sqrt(tau), or sigma(tau)=K*sqrt(tau/3).
            coefficient = 10 ** intercept * (math.sqrt(3) if slope > 0 else 1)
            if not math.isfinite(coefficient) or coefficient <= 0:
                continue
            candidates.append({
                "value": coefficient, "tau_start_s": float(tau[first]),
                "tau_end_s": float(tau[stop - 1]), "points": stop - first,
                "span_decades": span, "measured_slope": measured_slope,
                "fixed_slope": slope, "log10_rmse": residual,
                "minimum_duration_over_tau": duration_s / float(tau[stop - 1]),
                "evaluation_tau_s": 3 if slope > 0 else 1,
            })
    if not candidates:
        return {"status": "unavailable", "reason": "no sufficiently long, supported slope region"}
    best = max(candidates, key=lambda fit: (fit["span_decades"], -fit["log10_rmse"]))
    return {"status": "fitted", **best}


def fit_kalibr_noise(curves, duration_s, rate_hz):
    if not math.isfinite(rate_hz) or rate_hz <= 0:
        raise ValueError("Kalibr update rate must be finite and positive")
    axes, missing = {}, []
    for axis in AXES:
        tau, adev = curves[axis]
        axes[axis] = {}
        for name, slope in (("noise_density", -0.5), ("random_walk", 0.5)):
            fitted = fit_region(tau, adev, duration_s, slope)
            axes[axis][name] = fitted
            if fitted["status"] != "fitted":
                missing.append(f"{axis}.{name}")
    parameters = {"update_rate": rate_hz}
    for prefix, group in (("gyroscope", AXES[:3]), ("accelerometer", AXES[3:])):
        for name in ("noise_density", "random_walk"):
            fits = [axes[axis][name] for axis in group]
            if all(fit["status"] == "fitted" for fit in fits):
                parameters[f"{prefix}_{name}"] = max(fit["value"] for fit in fits)
    return {
        "status": "incomplete" if missing else "candidate",
        "method": "fixed_slope_loglog_regions", "axis_aggregation": "maximum",
        "criteria": {"minimum_points": 4, "minimum_span_decades": 0.7,
                     "maximum_slope_error": 0.15, "maximum_local_slope_error": 0.3,
                     "maximum_log10_rmse": 0.08, "minimum_duration_over_tau": 10},
        "axes": axes, "missing": missing, "parameters": parameters,
        "yaml_file": None,
        "warning": "heuristic fit; stationary, temperature-stable input and independent validation required",
    }


def write_kalibr_yaml(path, fit, topic):
    """Write only complete candidates; keep project metadata outside standard YAML."""
    if fit["status"] != "candidate":
        raise ValueError("cannot export incomplete Kalibr noise parameters")
    keys = ("gyroscope_noise_density", "gyroscope_random_walk",
            "accelerometer_noise_density", "accelerometer_random_walk", "update_rate")
    values = fit["parameters"]
    if any(not math.isfinite(values[key]) or values[key] <= 0 for key in keys):
        raise ValueError("Kalibr noise parameters must be finite and positive")
    lines = ["# Candidate from Allan slope fitting; review analysis.json before calibration.",
             "# Three-axis aggregation: maximum. No empirical safety multiplier applied.",
             "# Density units: rad/s/sqrt(Hz), m/s^2/sqrt(Hz).",
             "# Bias random walk units: rad/s^2/sqrt(Hz), m/s^3/sqrt(Hz)."]
    lines.extend(f"{key}: {values[key]:.17g}" for key in keys)
    lines.append("rostopic: " + json.dumps(topic))
    with path.open("x", encoding="utf-8") as stream:
        stream.write("\n".join(lines) + "\n")
