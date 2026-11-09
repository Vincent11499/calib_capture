"""Validate the immutable IMU CSV and select a relative half-open range."""

from __future__ import annotations

import csv
from decimal import Decimal, ROUND_HALF_UP
import math
from pathlib import Path
from typing import Iterator


COLUMNS = ["timestamp_ns", "wx", "wy", "wz", "ax", "ay", "az", "temperature"]
ROS1_MAX_TIMESTAMP_NS = (2**32) * 1_000_000_000


def add_time_arguments(parser) -> None:
    start = parser.add_mutually_exclusive_group()
    start.add_argument("--start-s", help="inclusive offset from the first source sample, seconds")
    start.add_argument("--start-h", help="inclusive offset from the first source sample, hours")
    end = parser.add_mutually_exclusive_group()
    end.add_argument("--end-s", help="exclusive offset from the first source sample, seconds")
    end.add_argument("--end-h", help="exclusive offset from the first source sample, hours")


def time_bounds(args) -> tuple[int, int | None]:
    def convert(seconds: str | None, hours: str | None) -> int | None:
        value = hours if hours is not None else seconds
        if value is None:
            return None
        number = Decimal(value)
        if not number.is_finite() or number < 0:
            raise ValueError("time bounds must be finite and non-negative")
        nanoseconds = number * (3600 if hours is not None else 1) * 1_000_000_000
        if nanoseconds != nanoseconds.to_integral_value():
            raise ValueError("time bounds must have nanosecond precision or coarser")
        return int(nanoseconds)

    start = convert(args.start_s, args.start_h)
    end = convert(args.end_s, args.end_h)
    start = 0 if start is None else start
    if end is not None and end <= start:
        raise ValueError("end must be later than start")
    return start, end


def time_suffix(start_ns: int, end_ns: int | None) -> str:
    if start_ns == 0 and end_ns is None:
        return ""

    def hours(value: int) -> str:
        result = (Decimal(value) / 3_600_000_000_000).quantize(
            Decimal("0.1"), rounding=ROUND_HALF_UP
        )
        return format(result, "f").rstrip("0").rstrip(".")

    end = hours(end_ns) + "h" if end_ns is not None else "end"
    return f"_{hours(start_ns)}h-{end}"


def resolve_dataset(path: Path) -> tuple[Path, Path]:
    source = path.resolve()
    if source.is_file():
        if source.name != "imu0.csv" or source.parent.name != "imu":
            raise ValueError("CSV input must be <dataset>/imu/imu0.csv")
        return source.parent.parent, source
    csv_path = source / "imu" / "imu0.csv"
    if not csv_path.is_file():
        raise FileNotFoundError(csv_path)
    return source, csv_path


def rows(csv_path: Path) -> Iterator[tuple[int, tuple[float, ...]]]:
    with csv_path.open(encoding="utf-8", newline="") as stream:
        reader = csv.reader(stream)
        if next(reader, None) != COLUMNS:
            raise ValueError("expected eight-column IMU CSV: " + ",".join(COLUMNS))
        previous = None
        for line_number, row in enumerate(reader, 2):
            if len(row) != len(COLUMNS):
                raise ValueError(f"wrong column count at line {line_number}")
            try:
                stamp = int(row[0])
                values = tuple(float(value) for value in row[1:])
            except ValueError as exc:
                raise ValueError(f"invalid numeric value at line {line_number}") from exc
            if not 0 <= stamp < ROS1_MAX_TIMESTAMP_NS:
                raise ValueError(f"timestamp outside ROS1 range at line {line_number}")
            if previous is not None and stamp <= previous:
                raise ValueError(f"non-increasing timestamp at line {line_number}")
            if not all(math.isfinite(value) for value in values):
                raise ValueError(f"non-finite value at line {line_number}")
            previous = stamp
            yield stamp, values


def selected_rows(
    csv_path: Path, start_ns: int = 0, end_ns: int | None = None
) -> Iterator[tuple[int, tuple[float, ...]]]:
    origin = None
    for stamp, values in rows(csv_path):
        if origin is None:
            origin = stamp
        offset = stamp - origin
        if end_ns is not None and offset >= end_ns:
            break
        if offset >= start_ns:
            yield stamp, values


def validate_input(
    dataset: Path, csv_path: Path, start_ns: int = 0, end_ns: int | None = None
) -> dict:
    """Validate the complete source, including samples outside the selected interval."""
    if start_ns < 0 or (end_ns is not None and end_ns <= start_ns):
        raise ValueError("invalid selection range")
    first = last = origin = source_last = None
    count = total = before = after = 0
    minimum_interval_ns = math.inf
    maximum_interval_ns = 0
    for stamp, _values in rows(csv_path):
        if origin is None:
            origin = stamp
        total += 1
        source_last = stamp
        offset = stamp - origin
        if offset < start_ns:
            before += 1
            continue
        if end_ns is not None and offset >= end_ns:
            after += 1
            continue
        if first is None:
            first = stamp
        if last is not None:
            interval = stamp - last
            minimum_interval_ns = min(minimum_interval_ns, interval)
            maximum_interval_ns = max(maximum_interval_ns, interval)
        last = stamp
        count += 1
    if count < 2:
        raise ValueError("at least two samples are required in the selected interval")
    duration_s = (last - first) / 1e9
    return {
        "schema_version": "1.0.0",
        "kind": "allan_variance_python_input_validation",
        "status": "pass",
        "source": str(csv_path),
        "dataset_id": dataset.name,
        "messages": count,
        "first_timestamp_ns": first,
        "last_timestamp_ns": last,
        "duration_s": duration_s,
        "effective_rate_hz": (count - 1) / duration_s,
        "minimum_interval_ns": int(minimum_interval_ns),
        "maximum_interval_ns": int(maximum_interval_ns),
        "selection": {
            "origin_timestamp_ns": origin,
            "start_offset_ns": start_ns,
            "end_offset_ns": end_ns,
            "start_s": start_ns / 1e9,
            "end_s": None if end_ns is None else end_ns / 1e9,
            "interval": "[start,end)",
            "source_messages": total,
            "excluded_before": before,
            "excluded_after": after,
            "source_last_timestamp_ns": source_last,
            "source_duration_s": (source_last - origin) / 1e9,
            "complete_source_checked": True,
            "timestamps_rebased": False,
        },
        "sampling_note": (
            "The original algorithm bins by sample index at floor(effective_rate_hz); "
            "timestamps are not resampled or interpolated."
        ),
        "temperature_note": "temperature is validated but is not used in the Allan calculation",
    }


def materialize_selected_samples(
    csv_path: Path,
    target: Path,
    count: int,
    start_ns: int,
    end_ns: int | None,
):
    """Write [ax, ay, az, wx_deg, wy_deg, wz_deg] to a disk-backed NumPy array."""
    import numpy as np

    data = np.lib.format.open_memmap(target, mode="w+", dtype=np.float64, shape=(count, 6))
    radians_to_degrees = 180.0 / np.pi
    written = 0
    for _stamp, values in selected_rows(csv_path, start_ns, end_ns):
        wx, wy, wz, ax, ay, az, _temperature = values
        data[written] = (ax, ay, az, wx * radians_to_degrees, wy * radians_to_degrees, wz * radians_to_degrees)
        written += 1
    data.flush()
    del data
    if written != count:
        raise RuntimeError(f"selected sample count changed: expected {count}, wrote {written}")
