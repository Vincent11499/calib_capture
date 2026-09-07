from __future__ import annotations

import csv
import json
import os
import threading
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Mapping

from .axera import (
    AX_FORMAT_YUV420_SEMIPLANAR,
    AX_IMG_INFO_BASE_SIZE,
    AX_VIDEO_FRAME_BASE_SIZE,
    FORMAT_NAMES,
    AxImgInfo,
    AxVideoFrame,
    GrayPair,
)
from .bmp import write_mono8_bmp
from .config import APP_VERSION, DATASET_SCHEMA_VERSION, CaptureConfig, SDK_HEADER_SHA256


def utc_now_text() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def _atomic_write_text(path: Path, text: str) -> None:
    temporary = path.with_name("." + path.name + ".tmp")
    temporary.write_text(text, encoding="utf-8")
    os.replace(temporary, path)


@dataclass(frozen=True)
class ImuRecord:
    timestamp_ns: int
    frame_id: str
    orientation: tuple[float, float, float, float]
    orientation_covariance: tuple[float, ...]
    angular_velocity: tuple[float, float, float]
    angular_velocity_covariance: tuple[float, ...]
    linear_acceleration: tuple[float, float, float]
    linear_acceleration_covariance: tuple[float, ...]
    arrival_monotonic_ns: int

    def to_json_dict(self) -> dict[str, Any]:
        return {
            "timestamp_ns": self.timestamp_ns,
            "frame_id": self.frame_id,
            "orientation": {
                "x": self.orientation[0],
                "y": self.orientation[1],
                "z": self.orientation[2],
                "w": self.orientation[3],
            },
            "orientation_covariance": list(self.orientation_covariance),
            "angular_velocity": {
                "x": self.angular_velocity[0],
                "y": self.angular_velocity[1],
                "z": self.angular_velocity[2],
            },
            "angular_velocity_covariance": list(self.angular_velocity_covariance),
            "linear_acceleration": {
                "x": self.linear_acceleration[0],
                "y": self.linear_acceleration[1],
                "z": self.linear_acceleration[2],
            },
            "linear_acceleration_covariance": list(self.linear_acceleration_covariance),
            "arrival_monotonic_ns": self.arrival_monotonic_ns,
        }


def _yaml_scalar(value: Any) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, (int, float)):
        return str(value)
    return json.dumps(str(value), ensure_ascii=False)


def write_dataset_yaml(path: Path, config: CaptureConfig, include_imu: bool) -> None:
    lines = [
        f"schema_version: {DATASET_SCHEMA_VERSION}",
        "data_integrity: structural",
        "dataset_type: calibration",
        "timestamp_unit: ns",
        f"mode: {_yaml_scalar(config.mode)}",
        "capture_api: AX_VIN_GetYuvFrame",
        "source_pixel_format: NV12",
        "cameras:",
        "  cam0:",
        "    side: left",
        f"    vin: {{pipe: {config.left_pipe}, chn: {config.left_chn}}}",
        f"    topic: {_yaml_scalar(config.left_topic)}",
        f"    frame_id: {_yaml_scalar(config.left_frame_id)}",
        "    encoding: mono8",
        f"    resolution: [{config.width}, {config.height}]",
        "    timestamps: cameras/cam0/timestamps.csv",
        "  cam1:",
        "    side: right",
        f"    vin: {{pipe: {config.right_pipe}, chn: {config.right_chn}}}",
        f"    topic: {_yaml_scalar(config.right_topic)}",
        f"    frame_id: {_yaml_scalar(config.right_frame_id)}",
        "    encoding: mono8",
        f"    resolution: [{config.width}, {config.height}]",
        "    timestamps: cameras/cam1/timestamps.csv",
        "imu:",
        f"  enabled: {_yaml_scalar(include_imu)}",
        f"  topic: {_yaml_scalar(config.imu_topic)}",
        f"  frame_id: {_yaml_scalar(config.imu_frame_id)}",
        "  measurements: imu/imu0.csv",
        "  full_messages: meta/imu_full.jsonl",
    ]
    _atomic_write_text(path, "\n".join(lines) + "\n")


class DatasetWriter:
    """Owns one recoverable directory-dataset staging area."""

    def __init__(self, output: str | Path, config: CaptureConfig, include_imu: bool) -> None:
        final_path = Path(output).expanduser().resolve()
        if final_path.name.endswith((".inprogress", ".failed")):
            raise ValueError("output must be the desired final dataset directory")
        self.final_path = final_path
        self.staging_path = final_path.with_name(final_path.name + ".inprogress")
        failed_path = final_path.with_name(final_path.name + ".failed")
        failed_variants = list(final_path.parent.glob(final_path.name + ".failed.*"))
        if (
            self.final_path.exists()
            or self.staging_path.exists()
            or failed_path.exists()
            or failed_variants
        ):
            raise FileExistsError(f"dataset output already exists: {self.final_path}")
        self.staging_path.parent.mkdir(parents=True, exist_ok=True)
        for relative in ("cameras/cam0", "cameras/cam1", "imu", "meta"):
            (self.staging_path / relative).mkdir(parents=True, exist_ok=False)

        self.config = config
        self.include_imu = include_imu
        self.created_utc = utc_now_text()
        self._closed = False
        self._pair_count = 0
        self._imu_count = 0
        self._last_left_timestamp = -1
        self._last_right_timestamp = -1
        self._last_imu_timestamp = -1
        self._pair_write_total_ns = 0
        self._pair_write_max_ns = 0
        self._imu_lock = threading.Lock()

        self._cam0_stream = (self.staging_path / "cameras/cam0/timestamps.csv").open(
            "w", encoding="utf-8", newline=""
        )
        self._cam1_stream = (self.staging_path / "cameras/cam1/timestamps.csv").open(
            "w", encoding="utf-8", newline=""
        )
        self._pairs_stream = (self.staging_path / "meta/pairs.csv").open(
            "w", encoding="utf-8", newline=""
        )
        self._imu_stream = (self.staging_path / "imu/imu0.csv").open(
            "w", encoding="utf-8", newline=""
        )
        self._imu_full_stream = (self.staging_path / "meta/imu_full.jsonl").open(
            "w", encoding="utf-8", newline="\n"
        )
        self._cam0_writer = csv.writer(self._cam0_stream)
        self._cam1_writer = csv.writer(self._cam1_stream)
        self._pairs_writer = csv.writer(self._pairs_stream)
        self._imu_writer = csv.writer(self._imu_stream)
        self._cam0_writer.writerow(("timestamp_ns", "filename"))
        self._cam1_writer.writerow(("timestamp_ns", "filename"))
        self._pairs_writer.writerow(
            (
                "pair_index",
                "left_timestamp_ns",
                "right_timestamp_ns",
                "left_sequence",
                "right_sequence",
                "delta_us",
                "left_format",
                "right_format",
            )
        )
        self._imu_writer.writerow(
            ("timestamp_ns", "gyro_x", "gyro_y", "gyro_z", "accel_x", "accel_y", "accel_z")
        )

    @property
    def pair_count(self) -> int:
        return self._pair_count

    @property
    def imu_count(self) -> int:
        return self._imu_count

    def write_pair(self, pair: GrayPair) -> None:
        start_ns = time.perf_counter_ns()
        if self._closed:
            raise RuntimeError("dataset writer is closed")
        if pair.width != self.config.width or pair.height != self.config.height:
            raise ValueError("pair resolution differs from dataset resolution")
        if (
            pair.left_format != AX_FORMAT_YUV420_SEMIPLANAR
            or pair.right_format != AX_FORMAT_YUV420_SEMIPLANAR
        ):
            raise ValueError("calibration dataset source frames must both be NV12")
        if pair.left_timestamp_ns <= self._last_left_timestamp:
            raise ValueError("left camera timestamp is not strictly increasing")
        if pair.right_timestamp_ns <= self._last_right_timestamp:
            raise ValueError("right camera timestamp is not strictly increasing")

        left_name = f"{pair.left_timestamp_ns}.bmp"
        right_name = f"{pair.right_timestamp_ns}.bmp"
        left_final = self.staging_path / "cameras/cam0" / left_name
        right_final = self.staging_path / "cameras/cam1" / right_name
        left_temp = left_final.with_name("." + left_name + ".tmp")
        right_temp = right_final.with_name("." + right_name + ".tmp")
        try:
            write_mono8_bmp(
                left_temp, pair.width, pair.height, pair.left_pixels
            )
            write_mono8_bmp(
                right_temp, pair.width, pair.height, pair.right_pixels
            )
            os.replace(left_temp, left_final)
            os.replace(right_temp, right_final)
        except BaseException:
            for temporary in (left_temp, right_temp):
                try:
                    temporary.unlink(missing_ok=True)
                except OSError:
                    pass
            raise

        self._pair_count += 1
        self._last_left_timestamp = pair.left_timestamp_ns
        self._last_right_timestamp = pair.right_timestamp_ns
        self._cam0_writer.writerow((pair.left_timestamp_ns, left_name))
        self._cam1_writer.writerow((pair.right_timestamp_ns, right_name))
        self._pairs_writer.writerow(
            (
                self._pair_count,
                pair.left_timestamp_ns,
                pair.right_timestamp_ns,
                pair.left_sequence,
                pair.right_sequence,
                f"{pair.delta_us:.3f}",
                FORMAT_NAMES.get(pair.left_format, str(pair.left_format)),
                FORMAT_NAMES.get(pair.right_format, str(pair.right_format)),
            )
        )
        for stream in (self._cam0_stream, self._cam1_stream, self._pairs_stream):
            stream.flush()
        elapsed_ns = time.perf_counter_ns() - start_ns
        self._pair_write_total_ns += elapsed_ns
        self._pair_write_max_ns = max(self._pair_write_max_ns, elapsed_ns)

    def write_imu(self, record: ImuRecord) -> None:
        if self._closed:
            raise RuntimeError("dataset writer is closed")
        with self._imu_lock:
            if record.timestamp_ns <= self._last_imu_timestamp:
                raise ValueError("IMU timestamp is not strictly increasing")
            self._last_imu_timestamp = record.timestamp_ns
            self._imu_count += 1
            self._imu_writer.writerow(
                (
                    record.timestamp_ns,
                    *record.angular_velocity,
                    *record.linear_acceleration,
                )
            )
            self._imu_full_stream.write(
                json.dumps(record.to_json_dict(), separators=(",", ":"), ensure_ascii=False)
                + "\n"
            )
            self._imu_stream.flush()
            self._imu_full_stream.flush()

    def _close_streams(self) -> None:
        if self._closed:
            return
        self._closed = True
        for stream in (
            self._cam0_stream,
            self._cam1_stream,
            self._pairs_stream,
            self._imu_stream,
            self._imu_full_stream,
        ):
            stream.flush()
            stream.close()

    def seal_payload(self, session: Mapping[str, Any]) -> dict[str, Any]:
        self._close_streams()
        write_dataset_yaml(self.staging_path / "dataset.yaml", self.config, self.include_imu)
        session_data = {
            "schema_version": DATASET_SCHEMA_VERSION,
            "data_integrity": "structural",
            "app_version": APP_VERSION,
            "created_utc": self.created_utc,
            "sealed_utc": utc_now_text(),
            "pair_count": self._pair_count,
            "imu_count": self._imu_count,
            "config": self.config.to_dict(),
            "ax_video_frame_abi": {
                "base_size": AX_VIDEO_FRAME_BASE_SIZE,
                "pts_offset": AxVideoFrame.u64PTS.offset,
                "sequence_offset": AxVideoFrame.u64SeqNum.offset,
            },
            "ax_img_info_abi": {
                "base_size": AX_IMG_INFO_BASE_SIZE,
                "frame_info_offset": AxImgInfo.tFrameInfo.offset,
                "isp_info_offset": AxImgInfo.tIspInfo.offset,
            },
            "pinned_sdk_header_sha256": SDK_HEADER_SHA256,
            **dict(session),
            "write_performance": {
                "pair_count": self._pair_count,
                "mean_pair_ms": (
                    self._pair_write_total_ns / self._pair_count / 1e6
                    if self._pair_count else None
                ),
                "max_pair_ms": self._pair_write_max_ns / 1e6 if self._pair_count else None,
                "total_pair_write_s": self._pair_write_total_ns / 1e9,
            },
        }
        self.write_session(session_data)
        return session_data

    def write_session(self, session: Mapping[str, Any]) -> None:
        path = self.staging_path / "meta/session.json"
        _atomic_write_text(
            path,
            json.dumps(dict(session), indent=2, sort_keys=True, ensure_ascii=False) + "\n",
        )

    def commit(self, success: bool) -> Path:
        if not self._closed:
            raise RuntimeError("seal_payload must be called before commit")
        if success:
            destination = self.final_path
        else:
            destination = self.final_path.with_name(self.final_path.name + ".failed")
            counter = 1
            while destination.exists():
                destination = self.final_path.with_name(
                    f"{self.final_path.name}.failed.{counter}"
                )
                counter += 1
        if destination.exists():
            raise FileExistsError(f"commit destination already exists: {destination}")
        self.staging_path.replace(destination)
        return destination

    def abandon(self, error: BaseException, session: Mapping[str, Any] | None = None) -> Path:
        session_data: dict[str, Any]
        try:
            session_data = self.seal_payload(
                {
                    **dict(session or {}),
                    "capture_status": "failed",
                    "error": f"{type(error).__name__}: {error}",
                }
            )
        except Exception as seal_error:
            self._close_streams()
            session_data = {
                "capture_status": "failed",
                "error": f"{type(error).__name__}: {error}",
                "seal_error": f"{type(seal_error).__name__}: {seal_error}",
            }
            self.write_session(session_data)

        # A failed capture is still a first-class, inspectable dataset.  Import
        # lazily to avoid a module cycle and make a best effort even when the
        # original failure happened while sealing metadata.
        try:
            from .validator import validate_dataset

            report = validate_dataset(self.staging_path, write_report=True)
            session_data["validation_status"] = report.status
            session_data["validation_errors"] = report.errors
            session_data["validation_warnings"] = report.warnings
            session_data["validation_stats"] = report.stats
            self.write_session(session_data)
        except Exception as validation_error:
            session_data["validation_status"] = "fail"
            session_data["validation_error"] = (
                f"{type(validation_error).__name__}: {validation_error}"
            )
            self.write_session(session_data)
        return self.commit(False)
