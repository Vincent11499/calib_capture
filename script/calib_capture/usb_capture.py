from __future__ import annotations

import time
from pathlib import Path
from typing import Any, Callable

from .axera import AxIvpsApi, IvpsGrabber
from .base_link import BASE_LINK_POSES
from .bmp import BMP_PIXEL_OFFSET
from .capture import CaptureFailed, RateLimiter, UserAbort, _finish_dataset
from .config import CaptureConfig, NS_PER_SECOND, USB_IVPS_CHN, USB_IVPS_GRP
from .dataset import DatasetWriter, utc_now_text
from .performance import peak_rss_bytes
from .pipeline import FrameDiskWriter
from .validator import ValidationReport


def _estimated_usb_gb(config: CaptureConfig, frame_count: float) -> float:
    row_stride = (config.width + 3) & ~3
    return (BMP_PIXEL_OFFSET + row_stride * config.height) * frame_count / 1_000_000_000.0


def _make_grabber(api: AxIvpsApi, config: CaptureConfig) -> IvpsGrabber:
    return IvpsGrabber(
        api,
        grp=USB_IVPS_GRP,
        chn=USB_IVPS_CHN,
        timeout_ms=config.capture_timeout_ms,
        expected_resolution=(config.width, config.height),
    )


def _session(
    config: CaptureConfig,
    api: AxIvpsApi | None,
    grabber: IvpsGrabber | None,
    warmup: dict[str, Any],
    stats: dict[str, Any],
    deinit_result: int | None,
    *,
    failed: bool = False,
) -> dict[str, Any]:
    result: dict[str, Any] = {
        "mode": config.mode,
        "capture_status": "failed" if failed else stats.get("capture_status", "completed"),
        "finished_utc": utc_now_text(),
        "ax_deinit_result": deinit_result,
        "warmup_frame": warmup,
        "capture_stats": stats,
        "capture_source": {
            "api": "AX_IVPS_GetChnFrame",
            "grp": USB_IVPS_GRP,
            "chn": USB_IVPS_CHN,
        },
    }
    if api is not None:
        result["ax_library_dir"] = api.library_dir
        result["sdk_header_contract"] = api.header_contract
        result["ax_return_errors"] = list(api.ax_errors)
    if grabber is not None:
        result["grabber_stats"] = dict(grabber.stats)
        result["capture_stages"] = grabber.timings.summary()
    return result


def _warmup(grabber: IvpsGrabber, config: CaptureConfig) -> dict[str, Any]:
    frame = grabber.grab_frame()
    try:
        description = grabber.describe_frame(frame)
        gray = grabber.copy_gray_frame(
            frame,
            release_after_copy=True,
            bottom_up=config.image_format == "bmp",
        )
        description["gray_copy_bytes"] = len(gray.pixels)
        return description
    finally:
        if getattr(frame, "_ivps_owned", False):
            grabber.release_frame(frame)


def record_usb_continuous(config: CaptureConfig) -> tuple[Path, ValidationReport]:
    if not config.output:
        raise ValueError("--output is required")
    estimated_frames = config.duration_s * config.save_rate_hz
    print(
        f"Uncompressed USB BMP equivalent at {config.save_rate_hz:g} Hz: "
        f"{_estimated_usb_gb(config, estimated_frames):.2f} GB. "
        "No free-space or write-speed preflight is performed."
    )
    dataset = DatasetWriter(config.output, config, include_imu=False)
    api: AxIvpsApi | None = None
    grabber: IvpsGrabber | None = None
    writer: FrameDiskWriter | None = None
    warmup: dict[str, Any] = {}
    stats: dict[str, Any] = {
        "source_frames": 0,
        "rate_limiter_skipped_frames": 0,
        "writer_queue_drop_frames": 0,
        "writer_saved_frames": 0,
        "capture_status": "completed",
    }
    deinit_result: int | None = None
    try:
        api = AxIvpsApi(config.ax_lib_dir, config.sdk_include_dir)
        grabber = _make_grabber(api, config)
        warmup = _warmup(grabber, config)
        previous_timestamp_ns = int(warmup["timestamp_us"]) * 1000
        print(
            f"Ready: fixed {config.width}x{config.height}, "
            f"IVPS grp {USB_IVPS_GRP}/chn {USB_IVPS_CHN}, "
            f"{config.image_format.upper()}, target {config.save_rate_hz:g} Hz."
        )
        writer = FrameDiskWriter(dataset, config.writer_queue_pairs)
        limiter = RateLimiter(config.save_rate_hz)
        started = time.monotonic()
        cpu_started = time.process_time()
        deadline = started + config.duration_s
        grabber.start_sequence_window()
        while time.monotonic() < deadline:
            writer.raise_if_failed()
            frame = grabber.grab_frame()
            try:
                timestamp_ns = int(frame.u64PTS) * 1000
                if timestamp_ns <= previous_timestamp_ns:
                    raise RuntimeError("USB IVPS PTS is not strictly increasing")
                previous_timestamp_ns = timestamp_ns
                stats["source_frames"] += 1
                stats.setdefault("source_first_timestamp_ns", timestamp_ns)
                stats["source_last_timestamp_ns"] = timestamp_ns
                if not limiter.accept(timestamp_ns):
                    stats["rate_limiter_skipped_frames"] += 1
                    continue
                slot = writer.reserve()
                if slot is None:
                    stats["writer_queue_drop_frames"] += 1
                    continue
                try:
                    gray = grabber.copy_gray_frame(
                        frame,
                        release_after_copy=True,
                        buffer=slot,
                        bottom_up=config.image_format == "bmp",
                    )
                    if writer.submit(gray, slot):
                        slot = None
                    else:
                        stats["writer_queue_drop_frames"] += 1
                finally:
                    if slot is not None:
                        writer.pool.release(slot)
            finally:
                if getattr(frame, "_ivps_owned", False):
                    grabber.release_frame(frame)
        stats["capture_wall_s"] = time.monotonic() - started
        writer.finish()
        stats["capture_and_drain_cpu_s"] = time.process_time() - cpu_started
        stats["capture_and_drain_wall_s"] = time.monotonic() - started
        stats["pipeline"] = writer.summary()
        stats["capture_peak_rss_bytes"] = peak_rss_bytes()
        writer = None
        stats["writer_saved_frames"] = dataset.frame_count
        if stats["source_frames"] > 1:
            span_ns = stats["source_last_timestamp_ns"] - stats["source_first_timestamp_ns"]
            if span_ns > 0:
                stats["source_effective_rate_hz"] = (
                    (stats["source_frames"] - 1) * NS_PER_SECOND / span_ns
                )
        stats.update({
            "source_sequence_accounting": grabber.stats["source_sequence_accounting"],
            "source_sequence_gaps": grabber.stats["source_sequence_gaps"],
            "source_sequence_regressions": grabber.stats["source_sequence_regressions"],
        })
        deinit_result = api.close()
        return _finish_dataset(
            dataset,
            _session(config, api, grabber, warmup, stats, deinit_result),
            requested_success=True,
        )
    except BaseException as exc:
        cleanup_errors: list[str] = []
        if writer is not None:
            try:
                writer.finish()
            except BaseException as cleanup_error:
                cleanup_errors.append(f"frame writer stop: {cleanup_error}")
            stats["pipeline"] = writer.summary()
        stats["writer_saved_frames"] = dataset.frame_count
        if api is not None and deinit_result is None:
            try:
                deinit_result = api.close()
            except BaseException as cleanup_error:
                cleanup_errors.append(f"AX deinit: {cleanup_error}")
        if cleanup_errors:
            stats["cleanup_errors"] = cleanup_errors
        failed_path = dataset.abandon(
            exc, _session(config, api, grabber, warmup, stats, deinit_result, failed=True)
        )
        raise CaptureFailed(f"USB capture failed; data kept at {failed_path}: {exc}") from exc


def record_usb_baselink_triggered(
    config: CaptureConfig,
    *,
    input_function: Callable[[str], str] = input,
) -> tuple[Path, ValidationReport]:
    if not config.output:
        raise ValueError("--output is required")
    dataset = DatasetWriter(config.output, config, include_imu=False)
    api: AxIvpsApi | None = None
    grabber: IvpsGrabber | None = None
    warmup: dict[str, Any] = {}
    stats: dict[str, Any] = {
        "source_frames": 0,
        "writer_saved_frames": 0,
        "capture_status": "completed",
    }
    deinit_result: int | None = None
    try:
        api = AxIvpsApi(config.ax_lib_dir, config.sdk_include_dir)
        grabber = _make_grabber(api, config)
        warmup = _warmup(grabber, config)
        storage = (
            f"JPEG quality={config.jpeg_quality} (lossy)"
            if config.image_format == "jpeg" else "mono8 BMP"
        )
        print(
            f"Ready: fixed {config.width}x{config.height} {storage}, "
            f"IVPS grp {USB_IVPS_GRP}/chn {USB_IVPS_CHN}."
        )
        buffer = bytearray(config.width * config.height)
        for pose_index, pose_name in enumerate(BASE_LINK_POSES, 1):
            prompt = (
                "\n"
                "============================================================\n"
                f"[{pose_index}/{len(BASE_LINK_POSES)}] 请调整到以下姿态 / TARGET POSE\n"
                f">>>                 {pose_name}                 <<<\n"
                "============================================================\n"
                "调整完成后按 Enter 保存（输入 q 退出）: "
            )
            if input_function(prompt).strip().lower() in {"q", "quit", "exit"}:
                raise UserAbort(
                    f"operator aborted after {dataset.frame_count}/{len(BASE_LINK_POSES)} USB images"
                )
            frame = grabber.grab_frame()
            try:
                stats["source_frames"] += 1
                gray = grabber.copy_gray_frame(
                    frame,
                    release_after_copy=True,
                    buffer=buffer,
                    bottom_up=config.image_format == "bmp",
                )
            finally:
                if getattr(frame, "_ivps_owned", False):
                    grabber.release_frame(frame)
            dataset.write_frame(gray, filename_stem=pose_name)
            print(
                f"  saved USB image {pose_index} ({pose_name}): "
                f"timestamp={gray.timestamp_ns}, sequence={gray.sequence}"
            )
        stats["writer_saved_frames"] = dataset.frame_count
        stats.update({
            # Interactive pauses intentionally let the producer advance; this
            # is not a continuously observed source-loss window.
            "source_sequence_accounting": "triggered_gets_not_continuous",
            "sequence_advance_between_triggers": grabber.stats["source_sequence_gaps"],
            "source_sequence_regressions": grabber.stats["source_sequence_regressions"],
        })
        deinit_result = api.close()
        return _finish_dataset(
            dataset,
            _session(config, api, grabber, warmup, stats, deinit_result),
            requested_success=True,
        )
    except BaseException as exc:
        stats["writer_saved_frames"] = dataset.frame_count
        if api is not None and deinit_result is None:
            try:
                deinit_result = api.close()
            except BaseException:
                pass
        failed_path = dataset.abandon(
            exc, _session(config, api, grabber, warmup, stats, deinit_result, failed=True)
        )
        prefix = "USB base_link capture incomplete" if isinstance(
            exc, (KeyboardInterrupt, UserAbort)
        ) else "USB base_link capture failed"
        raise CaptureFailed(f"{prefix}; data kept at {failed_path}: {exc}") from exc
