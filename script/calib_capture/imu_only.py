"""Streaming IMU-only CSV capture for later Allan analysis; no camera SDK calls."""
from __future__ import annotations

import csv
import json
import math
from pathlib import Path
import threading
import time

from .capture import ImuSubscriber, _CaptureInterrupts
from .config import APP_VERSION, DATASET_SCHEMA_VERSION, CaptureConfig
from .dataset import ImuRecord, _atomic_write_text, utc_now_text
from .validator import ValidationReport

CSV_FIELDS = ("timestamp_ns", "wx", "wy", "wz", "ax", "ay", "az", "temperature")
LAYOUT = "imu_allan_csv"


class ImuCsvWriter:
    """One bounded file buffer, guarded against concurrent flush and callback."""

    def __init__(self, path: Path, fallback_frame_id: str) -> None:
        self.stream = path.open("x", encoding="utf-8", newline="")
        self.writer = csv.writer(self.stream)
        self.writer.writerow(CSV_FIELDS)
        self.lock = threading.Lock()
        self.fallback = fallback_frame_id
        self.frame_id: str | None = None
        self.empty_ids = 0
        self.count = 0
        self.last_stamp = 0
        self.last_arrival = time.monotonic()
        self.temperature = None
        self.temperature_updates = 0
        self.temperature_arrival = None
        self.temperature_max_age_s = 0.0
        self.imu_before_temperature = 0

    def update_temperature(self, message) -> None:
        # Temperature is degrees Celsius; variance (including NaN) is not used.
        value = float(message.temperature)
        if not math.isfinite(value):
            raise ValueError("IMU temperature must be finite")
        with self.lock:
            self.temperature = value
            self.temperature_updates += 1
            self.temperature_arrival = time.monotonic()

    def write(self, record: ImuRecord) -> bool | None:
        with self.lock:
            if self.temperature is None:
                self.imu_before_temperature += 1
                return False
            values = (*record.angular_velocity, *record.linear_acceleration)
            if len(values) != 6 or not all(math.isfinite(v) for v in values):
                raise ValueError("IMU six-axis values must be finite")
            if not self.last_stamp < record.timestamp_ns < 2**63:
                raise ValueError("IMU timestamp must be positive, increasing and < 2^63")
            effective_id = record.frame_id or self.fallback
            if not effective_id:
                raise ValueError("IMU frame_id and fallback are both empty")
            if self.frame_id is not None and effective_id != self.frame_id:
                raise ValueError("IMU effective frame_id changed")
            self.writer.writerow((record.timestamp_ns, *values, self.temperature))
            self.temperature_max_age_s = max(
                self.temperature_max_age_s, time.monotonic() - self.temperature_arrival)
            self.frame_id = effective_id
            self.empty_ids += int(not record.frame_id)
            self.count += 1
            self.last_stamp = record.timestamp_ns
            self.last_arrival = time.monotonic()

    def flush_and_check(self, timeout_s: float) -> None:
        with self.lock:
            self.stream.flush()
            if time.monotonic() - self.last_arrival > timeout_s:
                raise RuntimeError(f"IMU stream stalled for more than {timeout_s:g} s")

    def close(self) -> None:
        with self.lock:
            self.stream.close()


def is_imu_only(path: str | Path) -> bool:
    session = Path(path) / "meta/session.json"
    if not session.is_file():
        return False
    return json.loads(session.read_text(encoding="utf-8")).get("capture_layout") == LAYOUT


def validate_imu_only(path: str | Path, *, write_report: bool = False) -> ValidationReport:
    """Read rows incrementally, even for multi-hour recordings."""
    root = Path(path)
    report = ValidationReport()
    count = first = last = maximum_gap = long_gaps = 0
    try:
        session = json.loads((root / "meta/session.json").read_text(encoding="utf-8"))
        if session.get("schema_version") != DATASET_SCHEMA_VERSION or session.get("kind") != "imu_capture_session":
            raise ValueError("IMU session requires schema_version 1.0.0 and kind imu_capture_session")
        if session.get("capture_layout") != LAYOUT or session.get("mode") != "imu-continuous":
            raise ValueError("not an IMU-only recording")
        expected_rate = float(session["config"]["imu_expected_rate_hz"])
        if not math.isfinite(expected_rate) or expected_rate <= 0:
            raise ValueError("invalid expected IMU rate")
        with (root / "imu/imu0.csv").open(encoding="utf-8", newline="") as stream:
            reader = csv.reader(stream)
            if tuple(next(reader, ())) != CSV_FIELDS:
                raise ValueError("invalid IMU CSV header")
            for row in reader:
                if len(row) != len(CSV_FIELDS):
                    raise ValueError(f"invalid IMU CSV row {count + 2}")
                stamp = int(row[0])
                if not last < stamp < 2**63:
                    raise ValueError(f"IMU timestamp not strictly increasing at row {count + 2}")
                if not all(math.isfinite(float(v)) for v in row[1:]):
                    raise ValueError(f"non-finite IMU value at row {count + 2}")
                if count:
                    gap = stamp - last
                    maximum_gap = max(maximum_gap, gap)
                    long_gaps += int(gap > 1.5e9 / expected_rate)
                else:
                    first = stamp
                last = stamp
                count += 1
        if count < 2:
            raise ValueError("at least two IMU samples are required")
        if count != session["imu_count"]:
            raise ValueError("IMU CSV count differs from session")
        if not session.get("imu_frame_id"):
            raise ValueError("missing IMU frame_id")
        rate = (count - 1) * 1e9 / (last - first)
        report.stats.update(imu_count=count, imu_effective_rate_hz=rate,
                            imu_first_timestamp_ns=first, imu_last_timestamp_ns=last,
                            imu_duration_s=(last - first) / 1e9,
                            imu_maximum_gap_ns=maximum_gap, imu_long_gap_count=long_gaps,
                            imu_frame_id=session["imu_frame_id"])
        if abs(rate / expected_rate - 1) > 0.1:
            report.warning("IMU effective rate differs from expected by more than 10%")
        if long_gaps:
            report.warning(f"IMU intervals above 1.5 expected periods: {long_gaps}")
        if session.get("imu_empty_frame_id_count"):
            report.warning("empty IMU frame_id used configured fallback")
        temperature = session.get("temperature", {})
        if temperature.get("unit") != "degC" or temperature.get("fill") != "latest_received":
            raise ValueError("missing or invalid temperature metadata")
        report.stats["temperature_updates"] = temperature["updates"]
        report.stats["temperature_max_age_s"] = temperature["maximum_age_s"]
        if temperature["maximum_age_s"] > 2.0:
            report.warning("temperature reused for more than 2 seconds; inspect temperature publisher")
        if session.get("capture_status") not in {"completed", "operator_stopped"}:
            report.error("IMU recording did not complete successfully")
    except (OSError, ValueError, TypeError, KeyError, OverflowError) as exc:
        report.error(str(exc))
    if write_report:
        _atomic_write_text(root / "meta/validation.json", json.dumps(report.to_dict(), indent=2) + "\n")
    return report


def record_imu_only(config: CaptureConfig) -> tuple[Path, ValidationReport]:
    final = Path(config.output)
    staging = Path(str(final) + ".inprogress")
    failed = Path(str(final) + ".failed")
    if (not config.output or final.name.endswith((".inprogress", ".failed"))
            or any(p.exists() for p in (final, staging, failed))
            or any(final.parent.glob(final.name + ".failed.*"))):
        raise ValueError("IMU output is empty or session directory already exists")
    staging.mkdir(parents=True)
    writer = subscriber = None
    session = {
        "schema_version": DATASET_SCHEMA_VERSION, "kind": "imu_capture_session",
        "dataset_id": final.name,
        "app_version": APP_VERSION, "mode": "imu-continuous", "capture_layout": LAYOUT,
        "config": config.to_dict(), "started_utc": utc_now_text(),
        "timestamp_unit": "ns", "angular_velocity_unit": "rad/s",
        "linear_acceleration_unit": "m/s^2", "capture_status": "failed",
        "imu_qos": {"reliability": "best_effort", "durability": "volatile",
                    "history": "keep_last", "depth": config.imu_qos_depth},
    }

    def save_session() -> None:
        session.update(imu_count=writer.count if writer else 0,
                       imu_frame_id=writer.frame_id if writer else None,
                       imu_empty_frame_id_count=writer.empty_ids if writer else 0,
                       finished_utc=utc_now_text(), output=str(final))
        session["temperature"] = {
            "topic": config.imu_temperature_topic, "message_type": "sensor_msgs/msg/Temperature",
            "unit": "degC", "fill": "latest_received",
            "updates": writer.temperature_updates if writer else 0,
            "maximum_age_s": writer.temperature_max_age_s if writer else 0,
            "imu_warmup_skipped": writer.imu_before_temperature if writer else 0,
        }
        _atomic_write_text(staging / "meta/session.json", json.dumps(session, indent=2) + "\n")

    try:
        (staging / "imu").mkdir()
        (staging / "meta").mkdir()
        writer = ImuCsvWriter(staging / "imu/imu0.csv", config.imu_frame_id)
        subscriber = ImuSubscriber(config, writer.write, temperature_callback=writer.update_temperature)
        print(f"IMU-only recording: {staging}; all received messages, no camera access")
        with _CaptureInterrupts() as interrupts:
            subscriber.start()
            subscriber.wait_first(config.startup_timeout_s,
                                  lambda: interrupts.check(before_camera=True))
            started = time.monotonic()
            deadline = started + config.duration_s
            while time.monotonic() < deadline:
                interrupts.check()
                if interrupts.count:
                    break
                subscriber.raise_if_failed()
                writer.flush_and_check(config.startup_timeout_s)
                time.sleep(min(0.5, max(0, deadline - time.monotonic())))
            subscriber.stop()  # Join the callback before closing its stream.
            subscriber.raise_if_failed()
            writer.flush_and_check(config.startup_timeout_s)
            writer.close()
            session["capture_status"] = "operator_stopped" if interrupts.count else "completed"
            session["capture_wall_s"] = time.monotonic() - started
            save_session()
            report = validate_imu_only(staging, write_report=True)
            if report.errors:
                raise RuntimeError("; ".join(report.errors))
            interrupts.check()
            if final.exists():
                raise FileExistsError(final)
            staging.rename(final)
            return final, report
    except BaseException as exc:
        cleanup_errors = []
        if subscriber is not None:
            try:
                subscriber.stop()
            except BaseException as cleanup:
                cleanup_errors.append(str(cleanup))
        if writer is not None:
            try:
                writer.close()
            except BaseException as cleanup:
                cleanup_errors.append(str(cleanup))
        session.update(capture_status="failed", error=str(exc), cleanup_errors=cleanup_errors)
        try:
            save_session()
            validate_imu_only(staging, write_report=True)
            if not failed.exists():
                staging.rename(failed)
        except BaseException as cleanup:
            raise RuntimeError(f"IMU capture failed; retained {staging}: {exc}; cleanup: {cleanup}") from exc
        raise RuntimeError(f"IMU capture failed; retained {failed}: {exc}") from exc
