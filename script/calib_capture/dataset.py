from __future__ import annotations

import csv
import json
import os
import re
import threading
import time
from concurrent.futures import Future, ThreadPoolExecutor, wait
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
    GrayFrame,
    GrayPair,
)
from .image_io import ensure_image_support, image_suffix, write_mono8_image
from .config import (
    APP_VERSION, CAPTURE_LAYOUT, DATASET_SCHEMA_VERSION, KALIBR_MANIFEST_MARKER,
    KALIBR_MANIFEST_VERSION, CaptureConfig, SDK_HEADER_SHA256,
)
from .performance import Timings, peak_rss_bytes
from .modes import is_usb_mode
from .config import USB_IVPS_CHN, USB_IVPS_GRP


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
            "schema_version": DATASET_SCHEMA_VERSION,
            "kind": "imu_message",
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


def kalibr_manifest(
    config: Mapping[str, Any], include_imu: bool, *, dataset_id: str | None = None,
    imu_frame_id: str | None = None,
) -> dict[str, Any]:
    """Shared v1.0.0 contract; frame_id is the effective sensor frame."""
    usb = is_usb_mode(config.get("mode"))
    cameras = (
        [{"id": "cam0", "timestamps": "cameras/cam0/timestamps.csv",
          "images": "cameras/cam0", "frame_id": "usb_camera",
          "resolution": [config["width"], config["height"]],
          "clock_source": "axera_pts_us"}]
        if usb else
        [
            {"id": camera,
             "timestamps": f"cameras/{camera}/timestamps.csv", "images": f"cameras/{camera}",
             "frame_id": config[f"{side}_frame_id"],
             "resolution": [config["width"], config["height"]], "clock_source": "axera_pts_us"}
            for camera, side in (("cam0", "left"), ("cam1", "right"))
        ]
    )
    return {
        "schema_version": KALIBR_MANIFEST_VERSION,
        "type": "kalibr_directory_dataset",
        "dataset_id": dataset_id or Path(config["output"]).name,
        "cameras": cameras,
        "imus": ([{"id": "imu0", "data": "imu/imu0.csv",
                   "frame_id": imu_frame_id if imu_frame_id is not None else config["imu_frame_id"],
                   "clock_source": "ros_header_stamp", "angular_velocity_unit": "rad/s",
                   "linear_acceleration_unit": "m/s^2"}]
                 if include_imu else []),
    }


def write_dataset_yaml(
    path: Path, config: CaptureConfig, include_imu: bool, *, dataset_id: str,
    imu_frame_id: str,
) -> None:
    # JSON is a YAML subset accepted by Kalibr. Using it keeps strict parsing
    # dependency-free and avoids a second, partial YAML parser in this project.
    _atomic_write_text(path, json.dumps(kalibr_manifest(config.to_dict(), include_imu,
                                       dataset_id=dataset_id, imu_frame_id=imu_frame_id),
                                       indent=2, ensure_ascii=False) + "\n")


def write_capture_dataset_yaml(
    path: Path, config: CaptureConfig, include_imu: bool, imu_frame_id: str,
) -> None:
    _atomic_write_text(path, capture_dataset_yaml(config, include_imu, imu_frame_id))


def capture_dataset_yaml(
    config: CaptureConfig, include_imu: bool, imu_frame_id: str,
) -> str:
    if is_usb_mode(config.mode):
        lines = [
            f"schema_version: {DATASET_SCHEMA_VERSION}",
            "kind: capture_dataset",
            "data_integrity: structural",
            "dataset_type: calibration",
            "timestamp_unit: ns",
            f"mode: {_yaml_scalar(config.mode)}",
            "capture_api: AX_IVPS_GetChnFrame",
            f"ivps: {{grp: {USB_IVPS_GRP}, chn: {USB_IVPS_CHN}}}",
            "source_pixel_format: runtime",
            f"image_format: {config.image_format}",
            f"jpeg_quality: {config.jpeg_quality}",
            f"lossy: {_yaml_scalar(config.image_format == 'jpeg')}",
            "cameras:",
            "  cam0:",
            "    side: usb",
            "    frame_id: \"usb_camera\"",
            "    encoding: mono8",
            f"    resolution: [{config.width}, {config.height}]",
            "    timestamps: cameras/cam0/timestamps.csv",
            "imu:",
            "  enabled: false",
            "  measurements: null",
            "  full_messages: null",
        ]
        return "\n".join(lines) + "\n"

    lines = [
        f"schema_version: {DATASET_SCHEMA_VERSION}",
        "kind: capture_dataset",
        "data_integrity: structural",
        "dataset_type: calibration",
        "timestamp_unit: ns",
        f"mode: {_yaml_scalar(config.mode)}",
        "capture_api: AX_VIN_GetYuvFrame",
        "source_pixel_format: NV12",
        f"image_format: {config.image_format}",
        f"jpeg_quality: {config.jpeg_quality}",
        f"lossy: {_yaml_scalar(config.image_format == 'jpeg')}",
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
        f"  frame_id: {_yaml_scalar(imu_frame_id)}",
        "  measurements: imu/imu0.csv" if include_imu else "  measurements: null",
        "  full_messages: meta/imu_full.jsonl" if include_imu else "  full_messages: null",
    ]
    return "\n".join(lines) + "\n"


@dataclass
class PendingPair:
    pair: GrayPair
    temporary: tuple[Path, Path]
    final: tuple[Path, Path]
    futures: list[Future]
    started_ns: int

    def done(self) -> bool:
        return all(future.done() for future in self.futures)


@dataclass
class PendingFrame:
    frame: GrayFrame
    temporary: Path
    final: Path
    future: Future
    started_ns: int

    def done(self) -> bool:
        return self.future.done()


class DatasetWriter:
    """Owns one recoverable directory-dataset staging area."""

    def __init__(self, output: str | Path, config: CaptureConfig, include_imu: bool) -> None:
        ensure_image_support(config.image_format)
        if include_imu and is_usb_mode(config.mode):
            raise ValueError("USB capture modes do not record IMU")
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
        self.single_camera = is_usb_mode(config.mode)
        directories = ["cameras/cam0", "meta"]
        if not self.single_camera:
            directories.append("cameras/cam1")
        if include_imu:
            directories.append("imu")
        for relative in directories:
            (self.staging_path / relative).mkdir(parents=True, exist_ok=False)

        self.config = config
        self.include_imu = include_imu
        self.created_utc = utc_now_text()
        self._closed = False
        self._pair_count = 0
        self._frame_count = 0
        self._imu_count = 0
        self._last_left_timestamp = -1
        self._last_right_timestamp = -1
        self._last_frame_timestamp = -1
        self._last_imu_timestamp = -1
        self._first_imu_timestamp: int | None = None
        self._first_camera_timestamp: int | None = None
        self._effective_imu_frame_id: str | None = None
        self._imu_empty_frame_id_count = 0
        self._last_submitted_left = -1
        self._last_submitted_right = -1
        self._last_submitted_frame = -1
        self._pair_write_total_ns = 0
        self._pair_write_max_ns = 0
        self._frame_write_total_ns = 0
        self._frame_write_max_ns = 0
        self._imu_lock = threading.Lock()
        self._camera_lock = threading.Lock()
        self.timings = Timings()
        self._flush_stop = threading.Event()
        self._flush_error: BaseException | None = None
        self._camera_dirty = False
        self._imu_dirty = False

        self._cam0_stream = (self.staging_path / "cameras/cam0/timestamps.csv").open(
            "w", encoding="utf-8", newline=""
        )
        self._cam1_stream = ((self.staging_path / "cameras/cam1/timestamps.csv").open(
            "w", encoding="utf-8", newline=""
        ) if not self.single_camera else None)
        self._pairs_stream = ((self.staging_path / "meta/pairs.csv").open(
            "w", encoding="utf-8", newline=""
        ) if not self.single_camera else None)
        self._frames_stream = ((self.staging_path / "meta/frames.csv").open(
            "w", encoding="utf-8", newline=""
        ) if self.single_camera else None)
        self._imu_stream = (self.staging_path / "imu/imu0.csv").open(
            "w", encoding="utf-8", newline=""
        ) if include_imu else None
        self._imu_full_stream = (self.staging_path / "meta/imu_full.jsonl").open(
            "w", encoding="utf-8", newline="\n"
        ) if include_imu else None
        self._cam0_writer = csv.writer(self._cam0_stream)
        self._cam1_writer = csv.writer(self._cam1_stream) if self._cam1_stream else None
        self._pairs_writer = csv.writer(self._pairs_stream) if self._pairs_stream else None
        self._frames_writer = csv.writer(self._frames_stream) if self._frames_stream else None
        self._imu_writer = csv.writer(self._imu_stream) if include_imu else None
        self._cam0_writer.writerow(("timestamp_ns", "filename"))
        if self._cam1_writer is not None:
            self._cam1_writer.writerow(("timestamp_ns", "filename"))
        if self._pairs_writer is not None:
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
        if self._frames_writer is not None:
            self._frames_writer.writerow(
                ("frame_index", "timestamp_ns", "sequence", "source_format")
            )
        if self._imu_writer is not None:
            self._imu_writer.writerow(("timestamp_ns", "wx", "wy", "wz", "ax", "ay", "az"))
        self._image_executor = ThreadPoolExecutor(
            max_workers=config.image_workers, thread_name_prefix="image")
        self._flush_thread = threading.Thread(target=self._flush_loop,
                                               name="metadata-flush", daemon=True)
        self._flush_thread.start()

    @property
    def pair_count(self) -> int:
        return self._pair_count

    @property
    def frame_count(self) -> int:
        return self._frame_count

    @property
    def imu_count(self) -> int:
        return self._imu_count

    @property
    def first_camera_timestamp_ns(self) -> int | None:
        return self._first_camera_timestamp

    @property
    def last_camera_timestamp_ns(self) -> int | None:
        if self.single_camera:
            return self._last_frame_timestamp if self._frame_count else None
        if not self._pair_count:
            return None
        return max(self._last_left_timestamp, self._last_right_timestamp)

    @property
    def effective_imu_frame_id(self) -> str:
        return self._effective_imu_frame_id or self.config.imu_frame_id

    def write_pair(self, pair: GrayPair, *, filename_stem: str | None = None) -> None:
        """Synchronous convenience path for triggered capture and offline tests."""
        pending = (
            self.start_pair(pair)
            if filename_stem is None
            else self.start_pair(pair, filename_stem=filename_stem)
        )
        self.finish_pair(pending)

    def start_pair(self, pair: GrayPair, *, filename_stem: str | None = None) -> PendingPair:
        """Schedule only image tasks; the coordinator owns ordering and commits."""
        start_ns = time.perf_counter_ns()
        self.raise_if_failed()
        if self._closed:
            raise RuntimeError("dataset writer is closed")
        if pair.width != self.config.width or pair.height != self.config.height:
            raise ValueError("pair resolution differs from dataset resolution")
        if (
            pair.left_format != AX_FORMAT_YUV420_SEMIPLANAR
            or pair.right_format != AX_FORMAT_YUV420_SEMIPLANAR
        ):
            raise ValueError("calibration dataset source frames must both be NV12")
        if pair.left_timestamp_ns <= self._last_submitted_left:
            raise ValueError("left camera timestamp is not strictly increasing")
        if pair.right_timestamp_ns <= self._last_submitted_right:
            raise ValueError("right camera timestamp is not strictly increasing")

        suffix = image_suffix(self.config.image_format)
        if filename_stem is None:
            left_name = f"{pair.left_timestamp_ns}{suffix}"
            right_name = f"{pair.right_timestamp_ns}{suffix}"
        else:
            if not re.fullmatch(r"[A-Za-z0-9_-]{1,128}", filename_stem):
                raise ValueError(f"unsafe image filename stem: {filename_stem!r}")
            left_name = right_name = f"{filename_stem}{suffix}"
        left_final = self.staging_path / "cameras/cam0" / left_name
        right_final = self.staging_path / "cameras/cam1" / right_name
        if left_final.exists() or right_final.exists():
            raise FileExistsError(f"image filename already exists: {filename_stem}")
        left_temp = left_final.with_name("." + left_name + ".tmp")
        right_temp = right_final.with_name("." + right_name + ".tmp")
        futures: list[Future] = []
        pending = PendingPair(pair, (left_temp, right_temp), (left_final, right_final),
                              futures, start_ns)
        try:
            jobs = ((left_temp, pair.left_pixels), (right_temp, pair.right_pixels))
            for path, pixels in jobs:
                futures.append(self._image_executor.submit(
                    write_mono8_image, path, pair.width, pair.height, pixels,
                    self.config.image_format, self.config.jpeg_quality, pair.bottom_up))
        except BaseException:
            self.cancel_pair(pending)
            raise
        self._last_submitted_left = pair.left_timestamp_ns
        self._last_submitted_right = pair.right_timestamp_ns
        return pending

    def cancel_pair(self, pending: PendingPair) -> None:
        # Workers may still be writing. Never remove files or recycle their
        # source buffers before every successfully submitted task has exited.
        wait(pending.futures)
        for path in pending.temporary:
            path.unlink(missing_ok=True)

    def finish_pair(self, pending: PendingPair) -> None:
        wait(pending.futures)
        try:
            self.raise_if_failed()
            for future in pending.futures:
                for stage, elapsed in (future.result() or {}).items():
                    self.timings.add(stage, elapsed)
            start = time.perf_counter_ns()
            with self._camera_lock:
                self._commit_pair(pending)
            self.timings.add("pair_commit", time.perf_counter_ns() - start)
        except BaseException:
            self.cancel_pair(pending)
            raise
        elapsed_ns = time.perf_counter_ns() - pending.started_ns
        self._pair_write_total_ns += elapsed_ns
        self._pair_write_max_ns = max(self._pair_write_max_ns, elapsed_ns)

    def _commit_pair(self, pending: PendingPair) -> None:
        pair = pending.pair
        if (pair.left_timestamp_ns <= self._last_left_timestamp or
                pair.right_timestamp_ns <= self._last_right_timestamp):
            raise ValueError("pair commit timestamps are not strictly increasing")
        for temporary, final in zip(pending.temporary, pending.final):
            os.replace(temporary, final)
        left_name, right_name = (path.name for path in pending.final)
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
        self._camera_dirty = True
        if self._first_camera_timestamp is None:
            self._first_camera_timestamp = min(pair.left_timestamp_ns, pair.right_timestamp_ns)

    def write_frame(self, frame: GrayFrame, *, filename_stem: str | None = None) -> None:
        """Synchronous single-camera path used by USB triggered capture."""
        self.finish_frame(self.start_frame(frame, filename_stem=filename_stem))

    def start_frame(
        self, frame: GrayFrame, *, filename_stem: str | None = None,
    ) -> PendingFrame:
        if not self.single_camera:
            raise RuntimeError("single-camera writes require a USB dataset")
        started_ns = time.perf_counter_ns()
        self.raise_if_failed()
        if self._closed:
            raise RuntimeError("dataset writer is closed")
        if (frame.width, frame.height) != (self.config.width, self.config.height):
            raise ValueError("frame resolution differs from dataset resolution")
        if frame.source_format not in FORMAT_NAMES:
            raise ValueError(f"unsupported USB source format: 0x{frame.source_format:x}")
        if frame.timestamp_ns <= self._last_submitted_frame:
            raise ValueError("USB camera timestamp is not strictly increasing")
        if filename_stem is None:
            filename_stem = str(frame.timestamp_ns)
        elif not re.fullmatch(r"[A-Za-z0-9_-]{1,128}", filename_stem):
            raise ValueError(f"unsafe image filename stem: {filename_stem!r}")
        name = filename_stem + image_suffix(self.config.image_format)
        final = self.staging_path / "cameras/cam0" / name
        if final.exists():
            raise FileExistsError(f"image filename already exists: {name}")
        temporary = final.with_name("." + name + ".tmp")
        future = self._image_executor.submit(
            write_mono8_image, temporary, frame.width, frame.height, frame.pixels,
            self.config.image_format, self.config.jpeg_quality, frame.bottom_up,
        )
        self._last_submitted_frame = frame.timestamp_ns
        return PendingFrame(frame, temporary, final, future, started_ns)

    def cancel_frame(self, pending: PendingFrame) -> None:
        wait([pending.future])
        pending.temporary.unlink(missing_ok=True)

    def finish_frame(self, pending: PendingFrame) -> None:
        wait([pending.future])
        try:
            self.raise_if_failed()
            for stage, elapsed in (pending.future.result() or {}).items():
                self.timings.add(stage, elapsed)
            started = time.perf_counter_ns()
            with self._camera_lock:
                frame = pending.frame
                if frame.timestamp_ns <= self._last_frame_timestamp:
                    raise ValueError("USB frame commit timestamp is not strictly increasing")
                os.replace(pending.temporary, pending.final)
                self._frame_count += 1
                self._last_frame_timestamp = frame.timestamp_ns
                self._cam0_writer.writerow((frame.timestamp_ns, pending.final.name))
                self._frames_writer.writerow((
                    self._frame_count, frame.timestamp_ns, frame.sequence,
                    FORMAT_NAMES.get(frame.source_format, str(frame.source_format)),
                ))
                self._camera_dirty = True
                if self._first_camera_timestamp is None:
                    self._first_camera_timestamp = frame.timestamp_ns
            self.timings.add("frame_commit", time.perf_counter_ns() - started)
        except BaseException:
            self.cancel_frame(pending)
            raise
        elapsed_ns = time.perf_counter_ns() - pending.started_ns
        self._frame_write_total_ns += elapsed_ns
        self._frame_write_max_ns = max(self._frame_write_max_ns, elapsed_ns)

    def write_imu(self, record: ImuRecord) -> None:
        self.raise_if_failed()
        if not self.include_imu:
            raise RuntimeError("cannot write IMU messages to a stereo-only dataset")
        if self._closed:
            raise RuntimeError("dataset writer is closed")
        with self._imu_lock:
            if record.timestamp_ns <= self._last_imu_timestamp:
                raise ValueError("IMU timestamp is not strictly increasing")
            if not isinstance(record.frame_id, str):
                raise ValueError("IMU frame_id must be a string")
            effective_id = record.frame_id or self.config.imu_frame_id
            if self._effective_imu_frame_id is not None and effective_id != self._effective_imu_frame_id:
                raise ValueError(
                    f"IMU effective frame_id changed: {self._effective_imu_frame_id!r} -> {effective_id!r}"
                )
            self._effective_imu_frame_id = effective_id
            self._imu_empty_frame_id_count += int(not record.frame_id)
            if self._first_imu_timestamp is None:
                self._first_imu_timestamp = record.timestamp_ns
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
            self._imu_dirty = True

    def raise_if_failed(self) -> None:
        if self._flush_error is not None:
            raise RuntimeError(f"metadata flush failed: {self._flush_error}") from self._flush_error

    def flush_metadata(self) -> None:
        start = time.perf_counter_ns()
        with self._camera_lock:
            if self._camera_dirty:
                for stream in (
                    self._cam0_stream, self._cam1_stream, self._pairs_stream,
                    self._frames_stream,
                ):
                    if stream is not None:
                        stream.flush()
                self._camera_dirty = False
        with self._imu_lock:
            if self._imu_dirty:
                self._imu_stream.flush()
                self._imu_full_stream.flush()
                self._imu_dirty = False
        self.timings.add("metadata_flush", time.perf_counter_ns() - start)

    def _flush_loop(self) -> None:
        try:
            while not self._flush_stop.wait(0.5):
                self.flush_metadata()
        except BaseException as exc:
            self._flush_error = exc

    def _close_streams(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._image_executor.shutdown(wait=True)
        self._flush_stop.set()
        self._flush_thread.join()
        close_error: BaseException | None = None
        for stream in (
            self._cam0_stream,
            self._cam1_stream,
            self._pairs_stream,
            self._frames_stream,
            self._imu_stream,
            self._imu_full_stream,
        ):
            if stream is None:
                continue
            try:
                stream.close()  # close flushes; still attempt every stream on error.
            except BaseException as exc:
                close_error = close_error or exc
        if close_error is not None:
            raise close_error
        self.raise_if_failed()

    def seal_payload(self, session: Mapping[str, Any]) -> dict[str, Any]:
        self._close_streams()
        write_capture_dataset_yaml(self.staging_path / "meta/capture_dataset.yaml", self.config,
                                   self.include_imu, self.effective_imu_frame_id)
        write_dataset_yaml(self.staging_path / "dataset.yaml", self.config, self.include_imu,
                           dataset_id=self.final_path.name, imu_frame_id=self.effective_imu_frame_id)
        session_data = {
            "schema_version": DATASET_SCHEMA_VERSION,
            "kind": "capture_session",
            "dataset_id": self.final_path.name,
            "data_integrity": "structural",
            "app_version": APP_VERSION,
            "created_utc": self.created_utc,
            "sealed_utc": utc_now_text(),
            "pair_count": self._pair_count,
            "frame_count": self._frame_count,
            "imu_count": self._imu_count,
            "config": self.config.to_dict(),
            "image_writer_workers": self.config.image_workers,
            "metadata_flush_interval_s": 0.5,
            "image_stages": self.timings.summary(),
            "peak_rss_bytes": peak_rss_bytes(),
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
            "dataset_manifest": KALIBR_MANIFEST_MARKER,
            "capture_layout": CAPTURE_LAYOUT,
            "imu_frame_id": self.effective_imu_frame_id,
            "imu_empty_frame_id_count": self._imu_empty_frame_id_count,
            "camera_first_timestamp_ns": self.first_camera_timestamp_ns,
            "camera_last_timestamp_ns": self.last_camera_timestamp_ns,
            "imu_preroll_actual_s": (
                (self.first_camera_timestamp_ns - self._first_imu_timestamp) / 1e9
                if self.first_camera_timestamp_ns is not None and self._first_imu_timestamp is not None
                else None
            ),
            "imu_postroll_actual_s": (
                (self._last_imu_timestamp - self.last_camera_timestamp_ns) / 1e9
                if self.last_camera_timestamp_ns is not None and self._first_imu_timestamp is not None
                else None
            ),
            "write_performance": {
                "pair_count": self._pair_count,
                "mean_pair_ms": (
                    self._pair_write_total_ns / self._pair_count / 1e6
                    if self._pair_count else None
                ),
                "max_pair_ms": self._pair_write_max_ns / 1e6 if self._pair_count else None,
                "total_pair_write_s": self._pair_write_total_ns / 1e9,
                "frame_count": self._frame_count,
                "mean_frame_ms": (
                    self._frame_write_total_ns / self._frame_count / 1e6
                    if self._frame_count else None
                ),
                "max_frame_ms": (
                    self._frame_write_max_ns / 1e6 if self._frame_count else None
                ),
                "total_frame_write_s": self._frame_write_total_ns / 1e9,
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
                "schema_version": DATASET_SCHEMA_VERSION,
                "kind": "capture_session",
                "dataset_id": self.final_path.name,
                "app_version": APP_VERSION,
                "config": self.config.to_dict(),
                "capture_layout": CAPTURE_LAYOUT,
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
