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

IMU_FIELDS = ("timestamp_ns", "wx", "wy", "wz", "ax", "ay", "az")
CSV_FIELDS = (*IMU_FIELDS, "temperature")
LAYOUT = "imu_allan_csv"


class ImuCsvWriter:
    """One bounded file buffer, guarded against concurrent flush and callback."""

    def __init__(
        self,
        path: Path,
        fallback_frame_id: str,
        *,
        use_temperature: bool = True,
        anomaly_path: Path | None = None,
    ) -> None:
        self.stream = path.open("x", encoding="utf-8", newline="")
        self.writer = csv.writer(self.stream)
        self.anomaly_stream = (
            anomaly_path.open("x", encoding="utf-8", newline="")
            if anomaly_path is not None else None
        )
        self.use_temperature = use_temperature
        self.writer.writerow(CSV_FIELDS if use_temperature else IMU_FIELDS)
        self.lock = threading.Lock()
        self.fallback = fallback_frame_id
        self.frame_id: str | None = None
        self.empty_ids = 0
        self.count = 0
        self.last_stamp = 0
        self.last_values: tuple[float, ...] | None = None
        self.last_temperature: float | None = None
        self.timestamp_duplicate_count = 0
        self.timestamp_regression_count = 0
        self.timestamp_max_regression_ns = 0
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
            if self.use_temperature and self.temperature is None:
                self.imu_before_temperature += 1
                return False
            values = (*record.angular_velocity, *record.linear_acceleration)
            if len(values) != 6 or not all(math.isfinite(v) for v in values):
                raise ValueError("IMU six-axis values must be finite")
            if not 0 < record.timestamp_ns < 2**63:
                raise ValueError("IMU timestamp must be positive and < 2^63")
            effective_id = record.frame_id or self.fallback
            if not effective_id:
                raise ValueError("IMU frame_id and fallback are both empty")
            if self.frame_id is not None and effective_id != self.frame_id:
                raise ValueError("IMU effective frame_id changed")
            if self.count and record.timestamp_ns <= self.last_stamp:
                delta_ns = record.timestamp_ns - self.last_stamp
                if delta_ns == 0:
                    self.timestamp_duplicate_count += 1
                    classification = "duplicate"
                else:
                    self.timestamp_regression_count += 1
                    self.timestamp_max_regression_ns = max(
                        self.timestamp_max_regression_ns, -delta_ns
                    )
                    classification = "regression"
                if self.anomaly_stream is not None:
                    payload = {
                        "arrival_index": self.count + 1,
                        "classification": classification,
                        "delta_ns": delta_ns,
                        "previous": {
                            "timestamp_ns": self.last_stamp,
                            "angular_velocity": self.last_values[:3],
                            "linear_acceleration": self.last_values[3:],
                            "temperature": self.last_temperature,
                        },
                        "current": {
                            "timestamp_ns": record.timestamp_ns,
                            "angular_velocity": record.angular_velocity,
                            "linear_acceleration": record.linear_acceleration,
                            "temperature": self.temperature,
                            "arrival_monotonic_ns": record.arrival_monotonic_ns,
                        },
                    }
                    self.anomaly_stream.write(
                        json.dumps(payload, separators=(",", ":")) + "\n"
                    )
            row = (record.timestamp_ns, *values)
            self.writer.writerow((*row, self.temperature) if self.use_temperature else row)
            if self.use_temperature:
                self.temperature_max_age_s = max(
                    self.temperature_max_age_s, time.monotonic() - self.temperature_arrival)
            self.frame_id = effective_id
            self.empty_ids += int(not record.frame_id)
            self.count += 1
            self.last_stamp = record.timestamp_ns
            self.last_values = values
            self.last_temperature = self.temperature
            self.last_arrival = time.monotonic()

    def flush_and_check(self, timeout_s: float) -> None:
        with self.lock:
            self.stream.flush()
            if self.anomaly_stream is not None:
                self.anomaly_stream.flush()
            if time.monotonic() - self.last_arrival > timeout_s:
                raise RuntimeError(f"IMU stream stalled for more than {timeout_s:g} s")

    def close(self) -> None:
        with self.lock:
            self.stream.close()
            if self.anomaly_stream is not None:
                self.anomaly_stream.close()


def is_imu_only(path: str | Path) -> bool:
    session = Path(path) / "meta/session.json"
    if not session.is_file():
        return False
    return json.loads(session.read_text(encoding="utf-8")).get("capture_layout") == LAYOUT


def validate_imu_only(path: str | Path, *, write_report: bool = False) -> ValidationReport:
    """Read rows incrementally, even for multi-hour recordings."""
    root = Path(path)
    report = ValidationReport()
    count = first = last = minimum_stamp = maximum_stamp = maximum_gap = long_gaps = 0
    duplicate_count = regression_count = maximum_regression_ns = 0
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
            temperature = session.get("temperature", {})
            use_temperature = temperature.get("enabled", True)
            expected_fields = CSV_FIELDS if use_temperature else IMU_FIELDS
            if tuple(next(reader, ())) != expected_fields:
                raise ValueError("invalid IMU CSV header")
            for row in reader:
                if len(row) != len(expected_fields):
                    raise ValueError(f"invalid IMU CSV row {count + 2}")
                stamp = int(row[0])
                if not 0 < stamp < 2**63:
                    raise ValueError(f"invalid IMU timestamp at row {count + 2}")
                if not all(math.isfinite(float(v)) for v in row[1:]):
                    raise ValueError(f"non-finite IMU value at row {count + 2}")
                if count:
                    gap = stamp - last
                    if gap == 0:
                        duplicate_count += 1
                    elif gap < 0:
                        regression_count += 1
                        maximum_regression_ns = max(maximum_regression_ns, -gap)
                    else:
                        maximum_gap = max(maximum_gap, gap)
                        long_gaps += int(gap > 1.5e9 / expected_rate)
                else:
                    first = stamp
                    minimum_stamp = maximum_stamp = stamp
                minimum_stamp = min(minimum_stamp, stamp)
                maximum_stamp = max(maximum_stamp, stamp)
                last = stamp
                count += 1
        if count < 2:
            raise ValueError("at least two IMU samples are required")
        if count != session["imu_count"]:
            raise ValueError("IMU CSV count differs from session")
        if not session.get("imu_frame_id"):
            raise ValueError("missing IMU frame_id")
        timestamp_span_ns = maximum_stamp - minimum_stamp
        if timestamp_span_ns <= 0:
            raise ValueError("IMU timestamp range has no positive duration")
        rate = (count - 1) * 1e9 / timestamp_span_ns
        report.stats.update(imu_count=count, imu_effective_rate_hz=rate,
                            imu_first_timestamp_ns=first, imu_last_timestamp_ns=last,
                            imu_timestamp_min_ns=minimum_stamp,
                            imu_timestamp_max_ns=maximum_stamp,
                            imu_duration_s=timestamp_span_ns / 1e9,
                            imu_maximum_gap_ns=maximum_gap, imu_long_gap_count=long_gaps,
                            imu_timestamp_duplicate_count=duplicate_count,
                            imu_timestamp_regression_count=regression_count,
                            imu_timestamp_max_regression_ns=maximum_regression_ns,
                            imu_frame_id=session["imu_frame_id"])
        expected_quality = session.get("timestamp_quality", {})
        actual_anomalies = duplicate_count + regression_count
        if expected_quality != {
            "strictly_increasing": actual_anomalies == 0,
            "anomaly_count": actual_anomalies,
            "duplicate_count": duplicate_count,
            "regression_count": regression_count,
            "maximum_regression_ns": maximum_regression_ns,
        }:
            raise ValueError("IMU timestamp quality metadata disagrees with CSV")
        anomaly_path = root / "meta/imu_timestamp_anomalies.jsonl"
        if not anomaly_path.is_file():
            raise ValueError("missing IMU timestamp anomaly journal")
        with anomaly_path.open(encoding="utf-8") as anomaly_stream:
            anomaly_lines = sum(1 for line in anomaly_stream if line.strip())
        if anomaly_lines != actual_anomalies:
            raise ValueError("IMU timestamp anomaly journal count disagrees with CSV")
        if actual_anomalies:
            report.warning(
                "IMU header timestamps are not strictly increasing: "
                f"duplicates={duplicate_count}, regressions={regression_count}, "
                f"maximum_regression_ns={maximum_regression_ns}; all arrival-order rows were retained"
            )
        if abs(rate / expected_rate - 1) > 0.1:
            report.warning("IMU effective rate differs from expected by more than 10%")
        if long_gaps:
            report.warning(f"IMU intervals above 1.5 expected periods: {long_gaps}")
        if session.get("imu_empty_frame_id_count"):
            report.warning("empty IMU frame_id used configured fallback")
        temperature = session.get("temperature", {})
        if temperature.get("enabled", True):
            if temperature.get("unit") != "degC" or temperature.get("fill") != "latest_received":
                raise ValueError("missing or invalid temperature metadata")
            report.stats["temperature_updates"] = temperature["updates"]
            report.stats["temperature_max_age_s"] = temperature["maximum_age_s"]
            if temperature["maximum_age_s"] > 2.0:
                report.warning("temperature reused for more than 2 seconds; inspect temperature publisher")
        else:
            report.stats["temperature_enabled"] = False
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
        duplicate_count = writer.timestamp_duplicate_count if writer else 0
        regression_count = writer.timestamp_regression_count if writer else 0
        session["timestamp_quality"] = {
            "strictly_increasing": duplicate_count + regression_count == 0,
            "anomaly_count": duplicate_count + regression_count,
            "duplicate_count": duplicate_count,
            "regression_count": regression_count,
            "maximum_regression_ns": writer.timestamp_max_regression_ns if writer else 0,
        }
        session["temperature"] = {
            "enabled": config.use_temperature,
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
        writer = ImuCsvWriter(
            staging / "imu/imu0.csv",
            config.imu_frame_id,
            use_temperature=config.use_temperature,
            anomaly_path=staging / "meta/imu_timestamp_anomalies.jsonl",
        )
        subscriber = ImuSubscriber(
            config, writer.write,
            temperature_callback=writer.update_temperature if config.use_temperature else None,
            allow_non_increasing_timestamps=True,
        )
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
