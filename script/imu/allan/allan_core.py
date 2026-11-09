"""Pure NumPy reproduction of AllanVarianceComputor::allanVariance()."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Callable


UPSTREAM_REVISION = "1d54b602ee7f2ba0427865d63afe4945d913ed24"
AXES = ["acc_x", "acc_y", "acc_z", "gyr_x", "gyr_y", "gyr_z"]
REFERENCE_INDICES = (0, 9, 99, 999, 9998)


def _deviation(prefix, sample_count: int, bin_size: int):
    import numpy as np

    bins = (sample_count - 1) // bin_size
    if bins < 2:
        raise ValueError(f"fewer than two bins for bin size {bin_size}")
    ends = np.arange(1, bins + 1, dtype=np.int64) * bin_size
    means = (prefix[ends] - prefix[ends - bin_size]) / bin_size
    differences = np.diff(means)
    return math.sqrt(float(np.mean(np.square(differences))) / 2.0)


def compute_curves(
    samples_path: Path,
    curve_path: Path,
    measure_rate_hz: int,
    progress: Callable[[str], None] | None = None,
):
    """Compute all 9,999 scales with the upstream non-overlapping bin boundary."""
    import numpy as np

    samples = np.load(samples_path, mmap_mode="r")
    if samples.ndim != 2 or samples.shape[1] != 6:
        raise ValueError("expected selected sample array shaped (N, 6)")
    count = int(samples.shape[0])
    curves = np.empty((9999, 7), dtype=np.float64)
    curves[:, 0] = np.arange(1, 10000, dtype=np.float64) * 0.1
    for axis_index, axis_name in enumerate(AXES):
        if progress:
            progress(f"Computing {axis_name} ({axis_index + 1}/6)")
        prefix = np.empty(count + 1, dtype=np.float64)
        prefix[0] = 0.0
        np.cumsum(samples[:, axis_index], dtype=np.float64, out=prefix[1:])
        for period in range(1, 10000):
            tau = period * 0.1
            bin_size = int(tau * measure_rate_hz)
            curves[period - 1, axis_index + 1] = _deviation(prefix, count, bin_size)

    if not np.all(np.isfinite(curves)) or not np.all(curves > 0):
        raise RuntimeError("Allan output contains a non-finite or non-positive value")
    with curve_path.open("w", encoding="ascii", newline="\n") as stream:
        for row in curves:
            stream.write(
                f"{row[0]:.19g} "
                + " ".join(f"{value:.7g}" for value in row[1:])
                + " \n"
            )
    return curves


def validate_curves(
    samples_path: Path,
    curve_path: Path,
    measure_rate_hz: int,
) -> dict:
    """Validate shape/grid and independently recompute five scales."""
    import numpy as np

    samples = np.load(samples_path, mmap_mode="r")
    curves = np.loadtxt(curve_path)
    if curves.shape != (9999, 7) or not np.all(np.isfinite(curves)):
        raise RuntimeError("expected finite 9999 by 7 Allan deviation output")
    expected_tau = np.arange(1, 10000, dtype=np.float64) * 0.1
    if not np.allclose(curves[:, 0], expected_tau, rtol=0, atol=1e-10):
        raise RuntimeError("unexpected tau grid")
    if not np.all(curves[:, 1:] > 0):
        raise RuntimeError("Allan deviation must be positive")

    count = int(samples.shape[0])
    checks = []
    for index in REFERENCE_INDICES:
        tau = float(curves[index, 0])
        bin_size = int(tau * measure_rate_hz)
        bins = (count - 1) // bin_size
        means = samples[: bins * bin_size].reshape(bins, bin_size, 6).mean(axis=1)
        reference = np.sqrt(np.mean(np.diff(means, axis=0) ** 2, axis=0) / 2.0)
        if not np.allclose(curves[index, 1:], reference, rtol=2e-5, atol=1e-10):
            raise RuntimeError(f"independent Allan check failed at tau={tau}")
        checks.append(
            {"tau_s": tau, "bin_samples": bin_size, "bins": bins, "status": "pass"}
        )
    return {
        "schema_version": "1.0.0",
        "kind": "allan_variance_python_validation",
        "status": "pass",
        "input_messages": count,
        "curve_points": 9999,
        "axes": AXES,
        "units": {"acc": "m/s^2", "gyr": "deg/s"},
        "reference_checks": checks,
        "parameter_status": "upstream_compatible_candidate_not_calibration_acceptance",
    }
