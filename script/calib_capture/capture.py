from __future__ import annotations

import json
import math
import statistics
import threading
import time
from fractions import Fraction
from pathlib import Path
from typing import Any, Callable

from .axera import (
    AX_FORMAT_YUV420_SEMIPLANAR,
    AxVinApi,
    StereoGrabber,
    assert_ctypes_abi,
)
from .bmp import BMP_PIXEL_OFFSET
from .config import CaptureConfig, NS_PER_SECOND
from .dataset import DatasetWriter, ImuRecord, utc_now_text
from .validator import ValidationReport, validate_dataset
from .pipeline import PairDiskWriter
from .performance import peak_rss_bytes


def monotonic_raw_ns() -> int:
    clock_id = getattr(
        time, "CLOCK_MONOTONIC_RAW", getattr(time, "CLOCK_MONOTONIC", None)
    )
    if clock_id is None:
        return time.monotonic_ns()
    return time.clock_gettime_ns(clock_id)


class CaptureFailed(RuntimeError):
    pass


class UserAbort(RuntimeError):
    pass


def _estimated_stereo_gb(config: CaptureConfig, pair_count: float) -> float:
    row_stride = (config.width + 3) & ~3
    bytes_per_image = BMP_PIXEL_OFFSET + row_stride * config.height
    return 2.0 * bytes_per_image * pair_count / 1_000_000_000.0


class RateLimiter:
    """Select the first pair at/after each sensor-time deadline, without bursts."""

    def __init__(self, rate_hz: float) -> None:
        if not math.isfinite(rate_hz) or rate_hz <= 0:
            raise ValueError("rate_hz must be finite and positive")
        # Keep the period as an integer ratio: rounding 1/6 second to whole
        # nanoseconds would incorrectly move the 500 ms deadline past its frame.
        rate = Fraction(str(rate_hz))
        self._rate_numerator = rate.numerator
        self._period_numerator_ns = NS_PER_SECOND * rate.denominator
        self._origin_ns: int | None = None
        self._next_slot = 1

    def accept(self, timestamp_ns: int) -> bool:
        if self._origin_ns is None:
            self._origin_ns = timestamp_ns
            return True
        elapsed_scaled = (timestamp_ns - self._origin_ns) * self._rate_numerator
        if elapsed_scaled < self._next_slot * self._period_numerator_ns:
            return False
        self._next_slot = elapsed_scaled // self._period_numerator_ns + 1
        return True


class FrameDivider:
    """Select pairs after both AX source sequences advance by ``divisor``."""

    def __init__(self, divisor: int) -> None:
        if divisor <= 0:
            raise ValueError("divisor must be positive")
        self.divisor = divisor
        self.last_left_sequence: int | None = None
        self.last_right_sequence: int | None = None

    def accept(self, left_sequence: int, right_sequence: int) -> bool:
        if left_sequence < 0 or right_sequence < 0:
            raise ValueError("source sequences must be nonnegative")
        if self.last_left_sequence is None or self.last_right_sequence is None:
            self.last_left_sequence = left_sequence
            self.last_right_sequence = right_sequence
            return True
        # A source restart begins a new division epoch. The capture loop records
        # the sequence regression as a quality warning independently.
        if (
            left_sequence < self.last_left_sequence
            or right_sequence < self.last_right_sequence
        ):
            self.last_left_sequence = left_sequence
            self.last_right_sequence = right_sequence
            return True
        if (
            left_sequence - self.last_left_sequence < self.divisor
            or right_sequence - self.last_right_sequence < self.divisor
        ):
            return False
        self.last_left_sequence = left_sequence
        self.last_right_sequence = right_sequence
        return True


class ImuSubscriber:
    def __init__(
        self,
        config: CaptureConfig,
        callback: Callable[[ImuRecord], None] | None = None,
    ) -> None:
        self.config = config
        self.callback = callback
        self.records: list[ImuRecord] = []
        self.count = 0
        self.first_timestamp_ns: int | None = None
        self.last_timestamp_ns: int | None = None
        self.first_event = threading.Event()
        self.stop_event = threading.Event()
        self.error: BaseException | None = None
        self._thread: threading.Thread | None = None
        self._rclpy: Any = None
        self._node: Any = None

    def start(self) -> None:
        try:
            import rclpy
            from rclpy.node import Node
            from rclpy.qos import (
                DurabilityPolicy,
                HistoryPolicy,
                QoSProfile,
                ReliabilityPolicy,
            )
            from sensor_msgs.msg import Imu
        except ImportError as exc:
            raise RuntimeError(
                "ROS 2 Python modules are unavailable; source the ROS 2 environment first"
            ) from exc

        owner = self

        class CaptureImuNode(Node):
            def __init__(self) -> None:
                super().__init__("calibration_data_capture")
                qos = QoSProfile(
                    history=HistoryPolicy.KEEP_LAST,
                    depth=owner.config.imu_qos_depth,
                    reliability=ReliabilityPolicy.BEST_EFFORT,
                    durability=DurabilityPolicy.VOLATILE,
                )
                self.subscription = self.create_subscription(
                    Imu, owner.config.imu_topic, self.on_imu, qos
                )

            def on_imu(self, message: Any) -> None:
                try:
                    timestamp_ns = int(message.header.stamp.sec) * NS_PER_SECOND + int(
                        message.header.stamp.nanosec
                    )
                    record = ImuRecord(
                        timestamp_ns=timestamp_ns,
                        frame_id=str(message.header.frame_id or owner.config.imu_frame_id),
                        orientation=(
                            float(message.orientation.x),
                            float(message.orientation.y),
                            float(message.orientation.z),
                            float(message.orientation.w),
                        ),
                        orientation_covariance=tuple(
                            float(value) for value in message.orientation_covariance
                        ),
                        angular_velocity=(
                            float(message.angular_velocity.x),
                            float(message.angular_velocity.y),
                            float(message.angular_velocity.z),
                        ),
                        angular_velocity_covariance=tuple(
                            float(value) for value in message.angular_velocity_covariance
                        ),
                        linear_acceleration=(
                            float(message.linear_acceleration.x),
                            float(message.linear_acceleration.y),
                            float(message.linear_acceleration.z),
                        ),
                        linear_acceleration_covariance=tuple(
                            float(value) for value in message.linear_acceleration_covariance
                        ),
                        arrival_monotonic_ns=monotonic_raw_ns(),
                    )
                    if record.timestamp_ns <= 0:
                        raise ValueError("IMU header timestamp is zero or negative")
                    if owner.last_timestamp_ns is not None and record.timestamp_ns <= owner.last_timestamp_ns:
                        raise ValueError("IMU header timestamp is not strictly increasing")
                    if owner.callback is not None:
                        owner.callback(record)
                    else:
                        owner.records.append(record)
                    owner.count += 1
                    owner.last_timestamp_ns = record.timestamp_ns
                    if owner.first_timestamp_ns is None:
                        owner.first_timestamp_ns = record.timestamp_ns
                        owner.first_event.set()
                except BaseException as exc:
                    owner.error = exc
                    owner.stop_event.set()
                    owner.first_event.set()

        self._rclpy = rclpy
        # Do not let rclpy parse this tool's non-ROS command-line arguments.
        rclpy.init(args=[])
        self._node = CaptureImuNode()
        self._thread = threading.Thread(target=self._spin, name="calibration-imu-subscriber")
        self._thread.start()

    def _spin(self) -> None:
        if self._rclpy is None or self._node is None:
            self.error = RuntimeError("IMU subscriber was not initialized")
            self.first_event.set()
            return
        try:
            while not self.stop_event.is_set() and self._rclpy.ok():
                self._rclpy.spin_once(self._node, timeout_sec=0.1)
        except BaseException as exc:
            self.error = exc
            self.first_event.set()

    def wait_first(self, timeout_s: float) -> int:
        if not self.first_event.wait(timeout_s):
            raise RuntimeError(
                f"startup timeout: no IMU data from {self.config.imu_topic} in {timeout_s:g} s"
            )
        self.raise_if_failed()
        if self.first_timestamp_ns is None:
            raise RuntimeError("IMU subscriber stopped before receiving a sample")
        return self.first_timestamp_ns

    def raise_if_failed(self) -> None:
        if self.error is not None:
            raise RuntimeError(f"IMU subscriber failed: {self.error}") from self.error

    def stop(self) -> None:
        self.stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=3.0)
            if self._thread.is_alive():
                raise RuntimeError("IMU subscriber thread did not stop")
        if self._node is not None:
            self._node.destroy_node()
            self._node = None
        if self._rclpy is not None and self._rclpy.ok():
            self._rclpy.shutdown()
        # A callback may fail after the capture loop's last error check.
        self.raise_if_failed()


def _make_grabber(api: AxVinApi, config: CaptureConfig) -> StereoGrabber:
    return StereoGrabber(
        api,
        left_pipe=config.left_pipe,
        left_chn=config.left_chn,
        right_pipe=config.right_pipe,
        right_chn=config.right_chn,
        timeout_ms=config.capture_timeout_ms,
        tolerance_us=config.sync_tolerance_us,
        max_retries=config.max_sync_retries,
        expected_resolution=config.resolution,
        expected_format=AX_FORMAT_YUV420_SEMIPLANAR,
    )


def _warmup_pair(grabber: StereoGrabber, *, bottom_up: bool = False) -> tuple[dict[str, Any], int]:
    frames = grabber.grab_frames()
    try:
        description = grabber.describe_frames(frames)
        pair = grabber.copy_gray_pair(frames, release_after_copy=True, bottom_up=bottom_up)
        if len(pair.left_pixels) != pair.width * pair.height:
            raise RuntimeError("left grayscale conversion returned the wrong size")
        if len(pair.right_pixels) != pair.width * pair.height:
            raise RuntimeError("right grayscale conversion returned the wrong size")
        return description, pair.pair_timestamp_ns
    finally:
        if not frames.released:
            grabber.release_frames(frames)


def _finish_dataset(
    dataset: DatasetWriter,
    session: dict[str, Any],
    *,
    requested_success: bool,
) -> tuple[Path, ValidationReport]:
    session_data = dataset.seal_payload(session)
    validation_start = time.perf_counter()
    report = validate_dataset(dataset.staging_path, write_report=True)
    session_data["validation_duration_s"] = time.perf_counter() - validation_start
    session_data["peak_rss_bytes"] = peak_rss_bytes()
    session_data["capture_status"] = (
        session.get("capture_status", "completed") if requested_success else "failed"
    )
    session_data["validation_status"] = report.status
    session_data["validation_errors"] = report.errors
    session_data["validation_warnings"] = report.warnings
    session_data["validation_stats"] = report.stats
    dataset.write_session(session_data)
    success = requested_success and not report.errors
    return dataset.commit(success), report


def _capture_session(
    config: CaptureConfig,
    api: AxVinApi | None,
    grabber: StereoGrabber | None,
    warmup: dict[str, Any],
    stats: dict[str, Any],
    deinit_result: int | None,
    *,
    failed: bool = False,
) -> dict[str, Any]:
    # DatasetWriter adds configuration and ABI metadata for every outcome.
    session: dict[str, Any] = {
        "mode": config.mode,
        "capture_status": "failed" if failed else stats.get("capture_status", "completed"),
        "finished_utc": utc_now_text(),
        "ax_deinit_result": deinit_result,
        "warmup_frame": warmup,
        "capture_stats": stats,
        "imu_qos": {
            "reliability": "best_effort",
            "durability": "volatile",
            "history": "keep_last",
            "depth": config.imu_qos_depth,
        },
    }
    if api is not None:
        session["ax_library_dir"] = api.library_dir
        session["sdk_header_contract"] = api.header_contract
        session["ax_return_errors"] = list(api.ax_errors)
    if grabber is not None:
        session["grabber_stats"] = grabber.stats
        session["capture_stages"] = grabber.timings.summary()
    return session


def record_continuous(config: CaptureConfig, *, include_imu: bool) -> tuple[Path, ValidationReport]:
    if not config.output:
        raise ValueError("--output is required")
    estimated_pairs = config.duration_s * config.target_save_rate_hz
    rate_description = (
        f"one pair per {config.frame_divisor} synchronized source pairs "
        f"(~{config.target_save_rate_hz:g} Hz at the expected source rate)"
        if config.frame_divisor > 0
        else f"PTS rate limit {config.save_rate_hz:g} Hz"
    )
    print(
        f"Uncompressed stereo BMP equivalent using {rate_description}: "
        f"{_estimated_stereo_gb(config, estimated_pairs):.2f} GB. "
        "No free-space or write-speed preflight is performed."
    )
    if config.image_format == "jpeg":
        print(f"Saving grayscale JPEG quality={config.jpeg_quality} (lossy); actual size depends on image content.")
    dataset = DatasetWriter(config.output, config, include_imu)
    api: AxVinApi | None = None
    grabber: StereoGrabber | None = None
    imu: ImuSubscriber | None = None
    pair_writer: PairDiskWriter | None = None
    warmup: dict[str, Any] = {}
    stats: dict[str, Any] = {
        "source_pairs": 0,
        "source_left_sequence_gaps": 0,
        "source_right_sequence_gaps": 0,
        "source_sequence_regressions": 0,
        "frame_divisor_skipped_pairs": 0,
        "rate_limiter_skipped_pairs": 0,
        "writer_queue_drop_pairs": 0,
        "writer_saved_pairs": 0,
        "operator_interrupted": False,
        "capture_status": "completed",
    }
    deinit_result: int | None = None
    try:
        api = AxVinApi(config.ax_lib_dir, config.sdk_include_dir)
        grabber = _make_grabber(api, config)
        if include_imu:
            imu = ImuSubscriber(config, dataset.write_imu)
            imu.start()
            first_imu_timestamp = imu.wait_first(config.startup_timeout_s)
        else:
            first_imu_timestamp = None

        warmup, warmup_timestamp = _warmup_pair(grabber, bottom_up=config.image_format == "bmp")
        if first_imu_timestamp is not None:
            offset_ns = abs(warmup_timestamp - first_imu_timestamp)
            stats["startup_camera_imu_offset_ns"] = offset_ns
            if offset_ns > round(config.clock_domain_max_offset_s * NS_PER_SECOND):
                raise RuntimeError(
                    f"camera/IMU timestamps are in different clock domains: offset={offset_ns / 1e9:.6f} s"
                )

        pair_writer = PairDiskWriter(dataset, config.writer_queue_pairs)
        divider = FrameDivider(config.frame_divisor) if config.frame_divisor > 0 else None
        limiter = None if divider is not None else RateLimiter(config.save_rate_hz)
        capture_started = time.monotonic()
        capture_cpu_started = time.process_time()
        deadline = capture_started + config.duration_s
        previous_left_ns = int(warmup["left"]["timestamp_us"]) * 1000
        previous_right_ns = int(warmup["right"]["timestamp_us"]) * 1000
        # The warmup performs one full-resolution gray copy. Do not count the
        # source frames produced during that startup check as recording gaps.
        previous_left_sequence: int | None = None
        previous_right_sequence: int | None = None
        try:
            while time.monotonic() < deadline:
                if imu is not None:
                    imu.raise_if_failed()
                pair_writer.raise_if_failed()
                frames = grabber.grab_frames()
                try:
                    left_ns = int(frames.left.u64PTS) * 1000
                    right_ns = int(frames.right.u64PTS) * 1000
                    if left_ns <= previous_left_ns or right_ns <= previous_right_ns:
                        raise RuntimeError("camera source PTS is not strictly increasing")
                    previous_left_ns, previous_right_ns = left_ns, right_ns
                    left_sequence = int(frames.left.u64SeqNum)
                    right_sequence = int(frames.right.u64SeqNum)
                    if previous_left_sequence is not None:
                        if left_sequence > previous_left_sequence + 1:
                            stats["source_left_sequence_gaps"] += (
                                left_sequence - previous_left_sequence - 1
                            )
                        elif left_sequence <= previous_left_sequence:
                            stats["source_sequence_regressions"] += 1
                    if previous_right_sequence is not None:
                        if right_sequence > previous_right_sequence + 1:
                            stats["source_right_sequence_gaps"] += (
                                right_sequence - previous_right_sequence - 1
                            )
                        elif right_sequence <= previous_right_sequence:
                            stats["source_sequence_regressions"] += 1
                    previous_left_sequence = left_sequence
                    previous_right_sequence = right_sequence
                    stats["source_pairs"] += 1
                    pair_timestamp_ns = (left_ns + right_ns) // 2
                    stats.setdefault("source_first_timestamp_ns", pair_timestamp_ns)
                    stats["source_last_timestamp_ns"] = pair_timestamp_ns
                    if divider is not None and not divider.accept(
                        left_sequence, right_sequence
                    ):
                        stats["frame_divisor_skipped_pairs"] += 1
                        continue
                    if limiter is not None and not limiter.accept(pair_timestamp_ns):
                        stats["rate_limiter_skipped_pairs"] += 1
                        continue
                    slot = pair_writer.reserve()
                    if slot is None:
                        stats["writer_queue_drop_pairs"] += 1
                        continue
                    try:
                        gray_pair = grabber.copy_gray_pair(
                            frames, release_after_copy=True, buffers=slot,
                            bottom_up=config.image_format == "bmp")
                        if pair_writer.submit(gray_pair, slot):
                            slot = None  # Ownership transferred to the writer.
                        else:
                            stats["writer_queue_drop_pairs"] += 1
                    finally:
                        if slot is not None:
                            pair_writer.pool.release(slot)
                finally:
                    if not frames.released:
                        grabber.release_frames(frames)
        except KeyboardInterrupt:
            stats["operator_interrupted"] = True
            stats["capture_status"] = "operator_stopped"

        if imu is not None:
            imu.stop()
            imu = None
        stats["capture_wall_s"] = time.monotonic() - capture_started
        pair_writer.finish()
        stats["capture_and_drain_cpu_s"] = time.process_time() - capture_cpu_started
        stats["capture_and_drain_wall_s"] = time.monotonic() - capture_started
        stats["pipeline"] = pair_writer.summary()
        stats["capture_peak_rss_bytes"] = peak_rss_bytes()
        pair_writer = None
        stats["writer_saved_pairs"] = dataset.pair_count
        stats["imu_saved_samples"] = dataset.imu_count
        if stats["source_pairs"] > 1:
            source_span_ns = (
                int(stats["source_last_timestamp_ns"])
                - int(stats["source_first_timestamp_ns"])
            )
            if source_span_ns > 0:
                stats["source_effective_rate_hz"] = (
                    (int(stats["source_pairs"]) - 1) * NS_PER_SECOND / source_span_ns
                )
        deinit_result = api.close()
        session = _capture_session(config, api, grabber, warmup, stats, deinit_result)
        return _finish_dataset(dataset, session, requested_success=True)
    except BaseException as exc:
        cleanup_errors: list[str] = []
        if imu is not None:
            try:
                imu.stop()
            except BaseException as cleanup_error:
                cleanup_errors.append(f"IMU stop: {cleanup_error}")
        if pair_writer is not None:
            try:
                pair_writer.finish()
            except BaseException as cleanup_error:
                cleanup_errors.append(f"pair writer stop: {cleanup_error}")
            stats["pipeline"] = pair_writer.summary()
        stats["writer_saved_pairs"] = dataset.pair_count
        stats["imu_saved_samples"] = dataset.imu_count
        if api is not None and deinit_result is None:
            try:
                deinit_result = api.close()
            except BaseException as cleanup_error:
                cleanup_errors.append(f"AX deinit: {cleanup_error}")
        if cleanup_errors:
            stats["cleanup_errors"] = cleanup_errors
        failed_path = dataset.abandon(
            exc,
            _capture_session(config, api, grabber, warmup, stats, deinit_result, failed=True),
        )
        raise CaptureFailed(
            f"capture failed; recoverable data kept at {failed_path}: {exc}"
        ) from exc


def record_triggered(
    config: CaptureConfig,
    *,
    input_function: Callable[[str], str] = input,
) -> tuple[Path, ValidationReport]:
    if not config.output:
        raise ValueError("--output is required")
    print(
        f"Uncompressed stereo BMP equivalent for {config.poses} poses: "
        f"{_estimated_stereo_gb(config, config.poses):.2f} GB. "
        "No free-space or write-speed preflight is performed."
    )
    dataset = DatasetWriter(config.output, config, include_imu=False)
    api: AxVinApi | None = None
    grabber: StereoGrabber | None = None
    warmup: dict[str, Any] = {}
    stats: dict[str, Any] = {
        "source_pairs": 0,
        "writer_queue_drop_pairs": 0,
        "capture_status": "completed",
    }
    deinit_result: int | None = None
    try:
        api = AxVinApi(config.ax_lib_dir, config.sdk_include_dir)
        grabber = _make_grabber(api, config)
        warmup, _ = _warmup_pair(grabber, bottom_up=config.image_format == "bmp")
        storage = (f"JPEG quality={config.jpeg_quality} (lossy)"
                   if config.image_format == "jpeg" else "mono8 BMP")
        print(
            f"Ready: {config.width}x{config.height} {storage}, "
            f"VIN left pipe {config.left_pipe}/chn {config.left_chn}, "
            f"right pipe {config.right_pipe}/chn {config.right_chn}."
        )
        buffers = (bytearray(config.width * config.height), bytearray(config.width * config.height))
        for pose_index in range(1, config.poses + 1):
            command = input_function(
                f"[{pose_index}/{config.poses}] Place target, press Enter to save (q abort): "
            ).strip().lower()
            if command in {"q", "quit", "exit"}:
                raise UserAbort(
                    f"operator aborted after {dataset.pair_count}/{config.poses} stereo pairs"
                )
            frames = grabber.grab_frames()
            try:
                stats["source_pairs"] += 1
                pair = grabber.copy_gray_pair(frames, release_after_copy=True,
                                              buffers=buffers, bottom_up=config.image_format == "bmp")
            finally:
                if not frames.released:
                    grabber.release_frames(frames)
            dataset.write_pair(pair)
            print(
                f"  saved pair {pose_index}: L={pair.left_timestamp_ns}, "
                f"R={pair.right_timestamp_ns}, delta={pair.delta_us:.3f} us"
            )
        deinit_result = api.close()
        stats["writer_saved_pairs"] = dataset.pair_count
        session = _capture_session(config, api, grabber, warmup, stats, deinit_result)
        return _finish_dataset(dataset, session, requested_success=True)
    except (KeyboardInterrupt, UserAbort) as exc:
        stats["capture_status"] = "failed"
        stats["writer_saved_pairs"] = dataset.pair_count
        if api is not None and deinit_result is None:
            try:
                deinit_result = api.close()
            except BaseException:
                pass
        failed_path = dataset.abandon(
            exc,
            _capture_session(config, api, grabber, warmup, stats, deinit_result, failed=True),
        )
        raise CaptureFailed(
            f"triggered capture incomplete; data kept at {failed_path}: {exc}"
        ) from exc
    except BaseException as exc:
        stats["writer_saved_pairs"] = dataset.pair_count
        if api is not None and deinit_result is None:
            try:
                deinit_result = api.close()
            except BaseException:
                pass
        failed_path = dataset.abandon(
            exc,
            _capture_session(config, api, grabber, warmup, stats, deinit_result, failed=True),
        )
        raise CaptureFailed(f"capture failed; data kept at {failed_path}: {exc}") from exc


def probe_hardware(
    config: CaptureConfig,
    *,
    samples: int = 50,
    with_imu: bool = False,
) -> dict[str, Any]:
    if samples < 2:
        raise ValueError("probe samples must be >= 2")
    assert_ctypes_abi()
    api = AxVinApi(config.ax_lib_dir, config.sdk_include_dir)
    grabber = _make_grabber(api, config)
    imu: ImuSubscriber | None = None
    imu_summary: dict[str, Any] | None = None
    descriptions: list[dict[str, Any]] = []
    first_frame: dict[str, Any] = {}
    left_timestamps: list[int] = []
    right_timestamps: list[int] = []
    imu_timestamps: list[int] = []
    try:
        if with_imu:
            imu = ImuSubscriber(config)
            imu.start()
            imu.wait_first(config.startup_timeout_s)
        # Validate metadata, mapping and grayscale copying before starting the
        # timing window. A full-resolution first-frame copy can take long enough
        # to bias a short source-rate probe and create an artificial seq gap.
        frames = grabber.grab_frames()
        try:
            first_frame = grabber.describe_frames(frames)
            pair = grabber.copy_gray_pair(frames, release_after_copy=True)
            first_frame["gray_copy_bytes_per_camera"] = len(pair.left_pixels)
        finally:
            if not frames.released:
                grabber.release_frames(frames)
        for _ in range(samples):
            frames = grabber.grab_frames()
            try:
                description = grabber.describe_frames(frames)
                descriptions.append(description)
                left_timestamps.append(int(frames.left.u64PTS) * 1000)
                right_timestamps.append(int(frames.right.u64PTS) * 1000)
            finally:
                if not frames.released:
                    grabber.release_frames(frames)
        if imu is not None:
            time.sleep(0.2)
            imu.stop()
            imu_summary = {
                "topic": config.imu_topic,
                "received_samples": imu.count,
                "first_timestamp_ns": imu.first_timestamp_ns,
                "last_timestamp_ns": imu.last_timestamp_ns,
                "qos": {
                    "reliability": "best_effort",
                    "durability": "volatile",
                    "history": "keep_last",
                    "depth": config.imu_qos_depth,
                },
            }
            imu_timestamps = [record.timestamp_ns for record in imu.records]
            imu = None
        deinit_result = api.close()
    finally:
        try:
            if imu is not None:
                imu.stop()
        finally:
            api.close()

    def rate(values: list[int]) -> float | None:
        if len(values) < 2 or any(
            current <= previous for previous, current in zip(values, values[1:])
        ):
            return None
        return (len(values) - 1) * NS_PER_SECOND / (values[-1] - values[0])

    deltas = [float(item["delta_us"]) for item in descriptions]
    left_rate = rate(left_timestamps)
    right_rate = rate(right_timestamps)
    warnings: list[str] = []
    errors: list[str] = []
    sequence_summary: dict[str, dict[str, int]] = {}
    for side in ("left", "right"):
        sequences = [int(item[side]["sequence"]) for item in descriptions]
        gaps = sum(
            max(0, current - previous - 1)
            for previous, current in zip(sequences, sequences[1:])
        )
        regressions = sum(
            current <= previous
            for previous, current in zip(sequences, sequences[1:])
        )
        sequence_summary[side] = {"gaps": gaps, "regressions": regressions}
        if gaps:
            warnings.append(f"{side} camera source sequence gaps observed: {gaps}")
        if regressions:
            warnings.append(
                f"{side} camera source sequence regressions observed: {regressions}"
            )
    for name, value in (("left", left_rate), ("right", right_rate)):
        if value is None:
            errors.append(f"{name} camera PTS is not strictly increasing")
            continue
        ratio = value / config.expected_source_rate_hz
        if ratio < 0.1 or ratio > 10.0:
            errors.append(
                f"{name} PTS rate {value:.3f} Hz is inconsistent with microsecond PTS units"
            )
        elif ratio < 0.9 or ratio > 1.1:
            warnings.append(
                f"{name} source rate {value:.3f} Hz differs from expected "
                f"{config.expected_source_rate_hz:g} Hz"
            )

    if with_imu:
        imu_rate = rate(imu_timestamps)
        if imu_summary is None:
            raise RuntimeError("internal error: IMU probe summary is missing")
        imu_summary["effective_rate_hz"] = imu_rate
        if len(imu_timestamps) > 1:
            gaps = [
                current - previous
                for previous, current in zip(imu_timestamps, imu_timestamps[1:])
            ]
            imu_summary["maximum_gap_ns"] = max(gaps)
        if imu_rate is None:
            errors.append("not enough increasing IMU timestamps to measure frequency")
        elif imu_rate < config.imu_expected_rate_hz * 0.9:
            warnings.append(
                f"IMU rate {imu_rate:.3f} Hz is below 90% of expected "
                f"{config.imu_expected_rate_hz:g} Hz"
            )
        if imu_timestamps:
            first_camera_ns = (left_timestamps[0] + right_timestamps[0]) // 2
            clock_offset_ns = abs(first_camera_ns - imu_timestamps[0])
            imu_summary["startup_camera_offset_ns"] = clock_offset_ns
            if clock_offset_ns > round(
                config.clock_domain_max_offset_s * NS_PER_SECOND
            ):
                errors.append(
                    "camera/IMU timestamps are in different clock domains: "
                    f"offset={clock_offset_ns / NS_PER_SECOND:.6f} s"
                )
            common_start = max(left_timestamps[0], imu_timestamps[0])
            common_end = min(left_timestamps[-1], imu_timestamps[-1])
            imu_summary["common_time_window_ns"] = max(0, common_end - common_start)
            if common_end < common_start:
                errors.append("camera and IMU timestamps do not overlap during probe")

    result = {
        "status": "fail" if errors else ("warning" if warnings else "pass"),
        "errors": errors,
        "warnings": warnings,
        "samples": samples,
        "mapping": {
            "api": "AX_VIN_GetYuvFrame",
            "cam0_left": {"pipe": config.left_pipe, "chn": config.left_chn},
            "cam1_right": {"pipe": config.right_pipe, "chn": config.right_chn},
        },
        "expected_resolution": [config.width, config.height],
        "actual_resolution": [
            int(first_frame["left"]["width"]),
            int(first_frame["left"]["height"]),
        ],
        "camera_timestamp_contract": "AX u64PTS is microseconds; stored timestamp is u64PTS * 1000 ns",
        "first_frame": first_frame,
        "observed_formats": {
            side: sorted(
                {
                    f"{int(item[side]['format'])}:{item[side]['format_name']}"
                    for item in descriptions
                }
            )
            for side in ("left", "right")
        },
        "source_rate_hz": {"left": left_rate, "right": right_rate},
        "source_sequence": sequence_summary,
        "stereo_delta_us": {
            "minimum": min(deltas),
            "maximum": max(deltas),
            "mean": statistics.fmean(deltas),
        },
        "grabber_stats": grabber.stats,
        "ax_library_dir": api.library_dir,
        "sdk_header_contract": api.header_contract,
        "ax_deinit_result": deinit_result,
        "ax_return_errors": list(api.ax_errors),
    }
    if deinit_result != 0:
        result["status"] = "fail"
        result["errors"].append(f"AX_SYS_Deinit returned {deinit_result}")
    if with_imu:
        result["imu"] = imu_summary
    return result


def format_probe_result(result: dict[str, Any]) -> str:
    return json.dumps(result, indent=2, sort_keys=True, ensure_ascii=False)
