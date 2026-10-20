"""Shared PC-only helpers for IMU time-series and noise diagnostics."""
from __future__ import annotations

import csv
import json
import math
from dataclasses import dataclass
from decimal import Decimal, ROUND_CEILING
from pathlib import Path
from typing import Any, Iterable

from .config import APP_VERSION


COLUMNS = ("timestamp_ns", "wx", "wy", "wz", "ax", "ay", "az", "temperature")
AXES = COLUMNS[1:7]
UNITS = {
    "wx": "rad/s", "wy": "rad/s", "wz": "rad/s",
    "ax": "m/s^2", "ay": "m/s^2", "az": "m/s^2",
    "temperature": "degC",
}


class ImuDiagnosticError(ValueError):
    """Input or analysis contract error suitable for a concise CLI message."""


@dataclass(frozen=True)
class SourceScan:
    session_root: Path
    csv_path: Path
    rows: int
    first_timestamp_ns: int
    last_timestamp_ns: int

    @property
    def duration_ns(self) -> int:
        return self.last_timestamp_ns - self.first_timestamp_ns

    @property
    def duration_s(self) -> float:
        return self.duration_ns / 1e9

    @property
    def default_output_root(self) -> Path:
        return self.session_root.parent / "analysis" / self.session_root.name


@dataclass(frozen=True)
class TimeWindow:
    start_offset_ns: int
    end_offset_ns: int | None
    mode: str

    @property
    def start_s(self) -> float:
        return self.start_offset_ns / 1e9

    @property
    def end_s(self) -> float | None:
        return None if self.end_offset_ns is None else self.end_offset_ns / 1e9


@dataclass(frozen=True)
class ImuSelection:
    timestamps_ns: Any
    values: Any
    metadata: dict[str, Any]


def _finite_seconds(value: float | None, name: str) -> float | None:
    if value is None:
        return None
    result = float(value)
    if not math.isfinite(result) or result < 0:
        raise ImuDiagnosticError(f"{name} must be finite and nonnegative")
    return result


def seconds_to_ns(value: float, name: str) -> int:
    checked = _finite_seconds(value, name)
    assert checked is not None
    return int(
        (Decimal(str(checked)) * 1_000_000_000).to_integral_value(
            rounding=ROUND_CEILING
        )
    )


def resolve_imu_source(dataset: str | Path) -> tuple[Path, Path]:
    source = Path(dataset).expanduser().resolve()
    if source.is_dir():
        root = source
        csv_path = root / "imu" / "imu0.csv"
    elif source.is_file():
        if source.name != "imu0.csv" or source.parent.name != "imu":
            raise ImuDiagnosticError(
                "CSV input must be named <session>/imu/imu0.csv"
            )
        csv_path = source
        root = source.parent.parent
    else:
        raise ImuDiagnosticError(f"input does not exist: {source}")
    if root.name.endswith((".inprogress", ".failed")):
        raise ImuDiagnosticError("use a completed IMU session")
    if not csv_path.is_file():
        raise ImuDiagnosticError(f"missing IMU CSV: {csv_path}")
    return root, csv_path


def _parse_row(line: str, line_number: int) -> tuple[int, tuple[float, ...]]:
    parts = line.rstrip("\r\n").split(",")
    if len(parts) != len(COLUMNS):
        raise ImuDiagnosticError(
            f"line {line_number}: expected {len(COLUMNS)} columns, got {len(parts)}"
        )
    try:
        timestamp = int(parts[0])
        values = tuple(float(value) for value in parts[1:])
    except ValueError as exc:
        raise ImuDiagnosticError(f"line {line_number}: invalid numeric value") from exc
    if timestamp < 0:
        raise ImuDiagnosticError(f"line {line_number}: negative timestamp")
    if not all(math.isfinite(value) for value in values):
        raise ImuDiagnosticError(f"line {line_number}: non-finite sensor value")
    return timestamp, values


def scan_imu_csv(dataset: str | Path) -> SourceScan:
    root, csv_path = resolve_imu_source(dataset)
    count = 0
    first = None
    previous = None
    with csv_path.open("r", encoding="utf-8", newline="") as stream:
        header = stream.readline().rstrip("\r\n").split(",")
        if tuple(header) != COLUMNS:
            raise ImuDiagnosticError(
                "IMU CSV header must be exactly " + ",".join(COLUMNS)
            )
        for line_number, line in enumerate(stream, start=2):
            if not line.strip():
                raise ImuDiagnosticError(f"line {line_number}: empty row")
            timestamp, _ = _parse_row(line, line_number)
            if previous is not None and timestamp <= previous:
                raise ImuDiagnosticError(
                    f"line {line_number}: timestamps must be strictly increasing"
                )
            if first is None:
                first = timestamp
            previous = timestamp
            count += 1
    if count < 2 or first is None or previous is None:
        raise ImuDiagnosticError("IMU CSV requires at least two samples")
    return SourceScan(root, csv_path, count, first, previous)


def timeseries_window(
    scan: SourceScan, start_s: float | None, end_s: float | None
) -> TimeWindow:
    start = 0 if start_s is None else seconds_to_ns(start_s, "--start-s")
    end = None if end_s is None else seconds_to_ns(end_s, "--end-s")
    if end is not None and end <= start:
        raise ImuDiagnosticError("--end-s must be greater than --start-s")
    if start > scan.duration_ns or (
        end is not None and start == scan.duration_ns
    ):
        raise ImuDiagnosticError("selected time range starts after the recording")
    if end is not None and end > scan.duration_ns + 1:
        raise ImuDiagnosticError("selected time range exceeds the recording")
    return TimeWindow(start, end, "all" if start == 0 and end is None else "explicit")


def noise_window(
    scan: SourceScan,
    start_s: float | None,
    end_s: float | None,
    duration_s: float | None,
) -> TimeWindow:
    start_value = _finite_seconds(start_s, "--start-s")
    end_value = _finite_seconds(end_s, "--end-s")
    duration_value = 1800.0 if duration_s is None else _finite_seconds(
        duration_s, "--duration-s"
    )
    assert duration_value is not None
    if duration_value <= 0:
        raise ImuDiagnosticError("--duration-s must be greater than zero")
    if start_value is not None and end_value is not None:
        if duration_s is not None:
            raise ImuDiagnosticError(
                "--duration-s cannot be combined with both --start-s and --end-s"
            )
        start = seconds_to_ns(start_value, "--start-s")
        end = seconds_to_ns(end_value, "--end-s")
        mode = "explicit"
    else:
        duration = seconds_to_ns(duration_value, "--duration-s")
        if duration > scan.duration_ns:
            raise ImuDiagnosticError(
                "requested noise duration is longer than the recording"
            )
        if start_value is None and end_value is None:
            start = (scan.duration_ns - duration) // 2
            end = start + duration
            mode = "centered"
        elif start_value is not None:
            start = seconds_to_ns(start_value, "--start-s")
            end = start + duration
            mode = "start_duration"
        else:
            end = seconds_to_ns(end_value, "--end-s")
            start = end - duration
            mode = "end_duration"
            if start < 0:
                raise ImuDiagnosticError(
                    "--end-s is smaller than the requested duration"
                )
    if end <= start:
        raise ImuDiagnosticError("selected time range must have positive duration")
    if start >= scan.duration_ns or end > scan.duration_ns + 1:
        raise ImuDiagnosticError("selected time range exceeds the recording")
    return TimeWindow(start, end, mode)


def load_window(
    scan: SourceScan, window: TimeWindow, *, minimum_samples: int = 2
) -> ImuSelection:
    try:
        import numpy as np
    except ImportError as exc:
        raise RuntimeError("IMU diagnostics require NumPy on the PC") from exc

    first_absolute = scan.first_timestamp_ns + window.start_offset_ns
    last_absolute = (
        None
        if window.end_offset_ns is None
        else scan.first_timestamp_ns + window.end_offset_ns
    )
    timestamp_chunks = []
    value_chunks = []
    timestamp_buffer: list[int] = []
    value_buffer: list[tuple[float, ...]] = []
    chunk_size = 100_000

    def flush() -> None:
        if timestamp_buffer:
            timestamp_chunks.append(np.asarray(timestamp_buffer, dtype=np.int64))
            value_chunks.append(np.asarray(value_buffer, dtype=np.float64))
            timestamp_buffer.clear()
            value_buffer.clear()

    with scan.csv_path.open("r", encoding="utf-8", newline="") as stream:
        stream.readline()
        for line_number, line in enumerate(stream, start=2):
            timestamp, values = _parse_row(line, line_number)
            if timestamp < first_absolute:
                continue
            if last_absolute is not None and timestamp >= last_absolute:
                break
            timestamp_buffer.append(timestamp)
            value_buffer.append(values)
            if len(timestamp_buffer) >= chunk_size:
                flush()
    flush()
    if not timestamp_chunks:
        raise ImuDiagnosticError("selected time range contains no samples")
    timestamps = (
        timestamp_chunks[0]
        if len(timestamp_chunks) == 1
        else np.concatenate(timestamp_chunks)
    )
    values = (
        value_chunks[0] if len(value_chunks) == 1 else np.concatenate(value_chunks)
    )
    if len(timestamps) < minimum_samples:
        raise ImuDiagnosticError(
            f"selected time range requires at least {minimum_samples} samples"
        )
    offsets = timestamps - scan.first_timestamp_ns
    metadata = {
        "interval": "[start, end)",
        "window_mode": window.mode,
        "source_samples": scan.rows,
        "selected_samples": int(len(timestamps)),
        "source_first_timestamp_ns": scan.first_timestamp_ns,
        "source_last_timestamp_ns": scan.last_timestamp_ns,
        "requested_start_s": window.start_s,
        "requested_end_s": window.end_s,
        "first_timestamp_ns": int(timestamps[0]),
        "last_timestamp_ns": int(timestamps[-1]),
        "actual_start_s": int(offsets[0]) / 1e9,
        "actual_last_s": int(offsets[-1]) / 1e9,
    }
    return ImuSelection(timestamps, values, metadata)


def timing_statistics(timestamps_ns: Any) -> tuple[dict[str, Any], Any]:
    import numpy as np

    gaps_ns = np.diff(timestamps_ns)
    if len(gaps_ns) == 0 or np.any(gaps_ns <= 0):
        raise ImuDiagnosticError("selected timestamps must be strictly increasing")
    gaps_ms = gaps_ns.astype(np.float64) / 1e6
    duration_s = int(timestamps_ns[-1] - timestamps_ns[0]) / 1e9
    rate_hz = (len(timestamps_ns) - 1) / duration_s
    expected_ms = 1000.0 / rate_hz
    relative = np.abs(gaps_ms / expected_ms - 1.0)
    median = float(np.median(gaps_ms))
    stats = {
        "samples": int(len(timestamps_ns)),
        "intervals": int(len(gaps_ms)),
        "duration_s": duration_s,
        "effective_rate_hz": rate_hz,
        "interval_ms": {
            "minimum": float(gaps_ms.min()),
            "p01": float(np.percentile(gaps_ms, 1)),
            "mean": float(gaps_ms.mean()),
            "median": median,
            "p99": float(np.percentile(gaps_ms, 99)),
            "maximum": float(gaps_ms.max()),
            "stddev": float(gaps_ms.std()),
        },
        "intervals_outside_10_percent": int(np.count_nonzero(relative > 0.1)),
        "gaps_over_1_5x_median": int(np.count_nonzero(gaps_ms > 1.5 * median)),
    }
    return stats, gaps_ms


def interval_histogram(
    gaps_ms: Any,
    minimum_ms: float = 7.5,
    maximum_ms: float = 20.0,
    width_ms: float = 1.0,
) -> dict[str, Any]:
    """Build right-closed timing bins and explicit tail ratios."""
    import numpy as np

    for value, name in (
        (minimum_ms, "--hist-min-ms"),
        (maximum_ms, "--hist-max-ms"),
        (width_ms, "--hist-bin-width-ms"),
    ):
        if not math.isfinite(value):
            raise ImuDiagnosticError(f"{name} must be finite")
    if maximum_ms <= minimum_ms:
        raise ImuDiagnosticError("--hist-max-ms must be greater than --hist-min-ms")
    if width_ms <= 0:
        raise ImuDiagnosticError("--hist-bin-width-ms must be greater than zero")

    edges = [float(minimum_ms)]
    current = minimum_ms + width_ms
    tolerance = max(abs(minimum_ms), abs(maximum_ms), 1.0) * 1e-12
    while current < maximum_ms - tolerance:
        edges.append(float(current))
        current += width_ms
    edges.append(float(maximum_ms))
    edge_array = np.asarray(edges, dtype=np.float64)
    bin_indices = np.searchsorted(edge_array, gaps_ms, side="left")
    counts = np.bincount(bin_indices, minlength=len(edges) + 1)
    total = int(len(gaps_ms))

    def ratio(count: int) -> float:
        return count / total if total else 0.0

    def number(value: float) -> str:
        return f"{value:g}"

    labels = [f"<= {number(edges[0])}"]
    labels.extend(
        f"({number(lower)}, {number(upper)}]"
        for lower, upper in zip(edges[:-1], edges[1:])
    )
    labels.append(f"> {number(edges[-1])}")
    bins = [
        {"label": label, "count": int(count), "ratio": ratio(int(count))}
        for label, count in zip(labels, counts)
    ]
    return {
        "right_closed_bins": True,
        "minimum_ms": float(minimum_ms),
        "maximum_ms": float(maximum_ms),
        "bin_width_ms": float(width_ms),
        "total_intervals": total,
        "bins": bins,
        "at_or_below_minimum": bins[0],
        "above_maximum": bins[-1],
    }


def column_statistics(values: Any) -> dict[str, dict[str, float]]:
    import numpy as np

    result = {}
    for index, name in enumerate((*AXES, "temperature")):
        column = values[:, index]
        result[name] = {
            "unit": UNITS[name],
            "minimum": float(column.min()),
            "p01": float(np.percentile(column, 1)),
            "mean": float(column.mean()),
            "median": float(np.median(column)),
            "p99": float(np.percentile(column, 99)),
            "maximum": float(column.max()),
            "stddev": float(column.std()),
        }
    return result


def extrema_downsample(x_values: Any, y_values: Any, max_points: int) -> tuple[Any, Any]:
    import numpy as np

    count = len(y_values)
    if max_points < 4:
        raise ImuDiagnosticError("--max-plot-points must be at least 4")
    if count <= max_points:
        return np.asarray(x_values), np.asarray(y_values)
    bucket_count = max(1, (max_points - 2) // 2)
    edges = np.linspace(1, count - 1, bucket_count + 1, dtype=np.int64)
    selected = [0]
    for start, stop in zip(edges[:-1], edges[1:]):
        if stop <= start:
            continue
        bucket = y_values[start:stop]
        low = int(start + np.argmin(bucket))
        high = int(start + np.argmax(bucket))
        selected.extend((low, high) if low < high else (high, low))
    selected.append(count - 1)
    indices = np.asarray(sorted(set(selected)), dtype=np.int64)
    return np.asarray(x_values)[indices], np.asarray(y_values)[indices]


def time_axis(timestamps_ns: Any) -> tuple[Any, str, float]:
    elapsed_s = (timestamps_ns - int(timestamps_ns[0])).astype("float64") / 1e9
    duration = float(elapsed_s[-1])
    if duration >= 3600:
        return elapsed_s / 3600.0, "Elapsed time (h)", 3600.0
    if duration >= 120:
        return elapsed_s / 60.0, "Elapsed time (min)", 60.0
    return elapsed_s, "Elapsed time (s)", 1.0


def format_seconds(value: float | None) -> str:
    if value is None:
        return "end"
    decimal = Decimal(str(value)).normalize()
    text = format(decimal, "f")
    return (text.rstrip("0").rstrip(".") if "." in text else text) or "0"


def default_output_directory(
    scan: SourceScan, kind: str, window: TimeWindow
) -> Path:
    if kind == "timeseries":
        suffix = "all" if window.mode == "all" else (
            f"{format_seconds(window.start_s)}s-"
            f"{format_seconds(window.end_s)}s"
        )
        name = f"timeseries_{suffix}"
    elif kind == "noise":
        if window.mode == "centered" and window.end_offset_ns is not None:
            minutes = (window.end_offset_ns - window.start_offset_ns) / 60e9
            label = format_seconds(minutes)
            name = f"noise_mid_{label}min"
        else:
            name = (
                f"noise_{format_seconds(window.start_s)}s-"
                f"{format_seconds(window.end_s)}s"
            )
    else:
        raise ValueError(kind)
    return scan.default_output_root / name


class AnalysisOutput:
    """No-overwrite staging directory with failed-result preservation."""

    def __init__(self, final: str | Path) -> None:
        self.final = Path(final).expanduser().resolve()
        self.staging = self.final.with_name(self.final.name + ".inprogress")
        self.failed = self.final.with_name(self.final.name + ".failed")

    def __enter__(self) -> Path:
        if any(path.exists() for path in (self.final, self.staging, self.failed)):
            raise FileExistsError(f"analysis output already exists: {self.final}")
        self.staging.mkdir(parents=True)
        return self.staging

    def __exit__(self, exc_type, exc, traceback) -> bool:
        if exc_type is None:
            self.staging.replace(self.final)
        elif self.staging.exists() and not self.failed.exists():
            self.staging.replace(self.failed)
        return False


def write_json(path: Path, value: dict[str, Any]) -> None:
    path.write_text(
        json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False) + "\n",
        encoding="utf-8",
    )


def write_statistics_csv(
    path: Path,
    timing: dict[str, Any],
    columns: dict[str, dict[str, float]],
    histogram: dict[str, Any] | None = None,
) -> None:
    with path.open("x", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(("section", "name", "metric", "value", "unit"))
        for name, value in timing.items():
            if isinstance(value, dict):
                for metric, nested in value.items():
                    writer.writerow(("timing", "timestamp", metric, nested, "ms"))
            else:
                unit = "Hz" if name.endswith("_hz") else ("s" if name.endswith("_s") else "")
                writer.writerow(("timing", "timestamp", name, value, unit))
        for name, metrics in columns.items():
            unit = metrics["unit"]
            for metric, value in metrics.items():
                if metric != "unit":
                    writer.writerow(("sensor", name, metric, value, unit))
        if histogram is not None:
            for item in histogram["bins"]:
                writer.writerow(
                    ("timestamp_histogram", item["label"], "count", item["count"], "")
                )
                writer.writerow(
                    ("timestamp_histogram", item["label"], "ratio", item["ratio"], "")
                )


def report_base(kind: str, scan: SourceScan, selection: ImuSelection) -> dict[str, Any]:
    return {
        "schema_version": APP_VERSION,
        "kind": kind,
        "source_session": str(scan.session_root),
        "source_csv": str(scan.csv_path),
        "source_read_only": True,
        "selection": selection.metadata,
    }
