from __future__ import annotations

import csv
import json
import math
from dataclasses import asdict, dataclass, field, fields
from pathlib import Path
from typing import Any, Iterable

from .image_io import image_suffix, inspect_mono8_image
from .base_link import BASE_LINK_POSES
from .config import (CAPTURE_LAYOUT, DATASET_SCHEMA_VERSION, DEFAULTS, KALIBR_MANIFEST_MARKER,
                     CaptureConfig, SDK_HEADER_SHA256)
from .dataset import capture_dataset_yaml, kalibr_manifest
from .modes import (
    STEREO_BASELINK_MODE,
    USB_BASELINK_MODE,
    USB_CONTINUOUS_MODE,
    is_stereo_baselink_mode,
    is_stereo_imu_mode,
    is_usb_mode,
)
from .config import USB_IVPS_CHN, USB_IVPS_GRP, USB_RESOLUTION


@dataclass
class ValidationReport:
    status: str = "pass"
    errors: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)
    stats: dict[str, Any] = field(default_factory=dict)

    def error(self, message: str) -> None:
        self.errors.append(message)
        self.status = "fail"

    def warning(self, message: str) -> None:
        self.warnings.append(message)
        if self.status == "pass":
            self.status = "warning"

    def to_dict(self) -> dict[str, Any]:
        return {"schema_version": DATASET_SCHEMA_VERSION, "kind": "capture_validation", **asdict(self)}


def _write_validation_report(dataset: Path, report: ValidationReport, *, compact: bool = False) -> None:
    directory = dataset / "meta"
    directory.mkdir(exist_ok=True)
    destination = directory / "validation.json"
    temporary = directory / ".validation.json.tmp"
    temporary.write_text(
        json.dumps(report.to_dict(), indent=2, sort_keys=True, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    temporary.replace(destination)


def _read_csv(path: Path, expected_fields: tuple[str, ...]) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        if tuple(reader.fieldnames or ()) != expected_fields:
            raise ValueError(
                f"{path.name} columns are {reader.fieldnames}; expected {list(expected_fields)}"
            )
        return list(reader)


def _timestamps(rows: Iterable[dict[str, str]], key: str = "timestamp_ns") -> list[int]:
    return [int(row[key]) for row in rows]


def _strictly_increasing(values: list[int]) -> bool:
    return all(current > previous for previous, current in zip(values, values[1:]))


def _effective_rate(values: list[int]) -> float | None:
    if len(values) < 2 or values[-1] <= values[0]:
        return None
    return (len(values) - 1) * 1_000_000_000.0 / (values[-1] - values[0])


def _unique_object(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate manifest key: {key}")
        result[key] = value
    return result


def _validate_kalibr_layout(
    dataset: Path, session: dict[str, Any], config: dict[str, Any], report: ValidationReport,
) -> bool:
    """Validate only the shared v1.0.0 layout without rewriting input files."""
    root_text = (dataset / "dataset.yaml").read_text(encoding="utf-8")
    marker = session.get("dataset_manifest")
    compact = session.get("capture_layout") == CAPTURE_LAYOUT
    if "capture_layout" in session and not compact:
        report.error(f"unsupported capture_layout: {session['capture_layout']!r}")
    if marker != KALIBR_MANIFEST_MARKER or session.get("schema_version") != DATASET_SCHEMA_VERSION:
        report.error("layout requires dataset_manifest=kalibr_directory_dataset and schema_version 1.0.0")
    try:
        manifest = json.loads(root_text, object_pairs_hook=_unique_object)
        if not isinstance(manifest, dict) or manifest.get("schema_version") != DATASET_SCHEMA_VERSION:
            raise ValueError("manifest must be an object with schema_version 1.0.0")
        expected = kalibr_manifest(config, is_stereo_imu_mode(config.get("mode")),
                                   dataset_id=session["dataset_id"], imu_frame_id=session["imu_frame_id"])
        # Optional ROS export metadata is checked when present; directory
        # stream selection itself uses sensor IDs and relative paths.
        for group, keys in (("cameras", ("left_topic", "right_topic")),
                            ("imus", ("imu_topic",))):
            for sensor, key in zip(manifest.get(group, []), keys):
                if "topic" in sensor and sensor.pop("topic") != config[key]:
                    raise ValueError("optional topic disagrees with session metadata")
        if manifest != expected:
            raise ValueError("strict Kalibr fields/topics/relative paths disagree with session metadata")
    except (ValueError, TypeError, KeyError) as exc:
        report.error(f"invalid Kalibr dataset.yaml: {exc}")
    required = ["meta/capture_dataset.yaml"]
    if (dataset / "imu/imu0_kalibr.csv").exists():
        report.error("compact layout must not contain duplicate imu0_kalibr.csv")
    if compact and not is_stereo_imu_mode(config.get("mode")):
        if (dataset / "imu").exists() or (dataset / "meta/imu_full.jsonl").exists():
            report.error("stereo-only compact layout must not contain IMU files")
    for relative in required:
        path = dataset / relative
        if not path.is_file():
            report.error(f"missing required new-layout file: {relative}")
        elif not path.resolve().is_relative_to(dataset):
            report.error(f"unsafe new-layout file: {relative}")
    try:
        capture_config = CaptureConfig(**{item.name: config[item.name] for item in fields(CaptureConfig)})
        effective_id = session["imu_frame_id"]
        if not isinstance(effective_id, str):
            raise ValueError("session imu_frame_id must be a string")
        expected_capture = capture_dataset_yaml(capture_config, is_stereo_imu_mode(config.get("mode")),
                                                effective_id)
        actual_capture = (dataset / "meta/capture_dataset.yaml").read_text(encoding="utf-8")
        if actual_capture != expected_capture:
            raise ValueError("capture manifest disagrees with session metadata")
    except (OSError, UnicodeError, KeyError, TypeError, ValueError) as exc:
        report.error(f"invalid meta/capture_dataset.yaml: {exc}")
    return True


def _validate_usb_dataset(
    dataset: Path,
    session: dict[str, Any],
    config: dict[str, Any],
    report: ValidationReport,
    *,
    write_report: bool,
) -> ValidationReport:
    """Validate the one-camera IVPS layout without pretending it is stereo."""
    compact = session.get("capture_layout") == CAPTURE_LAYOUT
    mode = str(config.get("mode", session.get("mode", "")))
    if mode not in {USB_CONTINUOUS_MODE, USB_BASELINK_MODE}:
        report.error(f"unsupported USB capture mode: {mode!r}")
    if session.get("schema_version") != DATASET_SCHEMA_VERSION:
        report.error("unsupported dataset schema_version; expected 1.0.0")
    if session.get("kind") != "capture_session" or not compact:
        report.error("session requires kind capture_session and capture_layout compact")
    if session.get("data_integrity") != "structural":
        report.error("session data_integrity must be structural")
    if not isinstance(session.get("dataset_id"), str) or not session.get("dataset_id"):
        report.error("session requires a nonempty dataset_id")
    if config.get("schema_version") != DATASET_SCHEMA_VERSION:
        report.error("capture configuration schema_version must be 1.0.0")
    required = (
        "dataset.yaml", "cameras/cam0/timestamps.csv", "meta/frames.csv",
        "meta/capture_dataset.yaml", "meta/session.json",
    )
    for relative in required:
        path = dataset / relative
        if not path.is_file():
            report.error(f"missing required file: {relative}")
        elif not path.resolve().is_relative_to(dataset):
            report.error(f"unsafe required file: {relative}")
    for relative in ("cameras/cam1", "meta/pairs.csv", "imu", "meta/imu_full.jsonl"):
        if (dataset / relative).exists():
            report.error(f"single-camera USB dataset must not contain: {relative}")
    if report.errors:
        if write_report and dataset.is_dir():
            _write_validation_report(dataset, report, compact=compact)
        return report

    try:
        _validate_kalibr_layout(dataset, session, config, report)
    except (OSError, UnicodeError) as exc:
        report.error(f"cannot read dataset manifest: {exc}")
    try:
        width, height = int(config["width"]), int(config["height"])
    except (KeyError, TypeError, ValueError, OverflowError):
        width = height = 0
        report.error("invalid USB resolution in session config")
    if (width, height) != USB_RESOLUTION:
        report.error(
            f"USB resolution must be fixed at {USB_RESOLUTION[0]}x{USB_RESOLUTION[1]}"
        )
    stored_format = str(config.get("image_format", "bmp"))
    try:
        suffix = image_suffix(stored_format)
    except ValueError as exc:
        report.error(str(exc))
        suffix = ".bmp"
    if stored_format == "jpeg":
        report.warning("JPEG storage is lossy; decoded pixels differ from the IVPS source")
    expected_abi = {"base_size": 232, "pts_offset": 192, "sequence_offset": 200}
    if session.get("ax_video_frame_abi") != expected_abi:
        report.error("session AX_VIDEO_FRAME_T ABI metadata does not match the pinned contract")
    if session.get("pinned_sdk_header_sha256") != SDK_HEADER_SHA256:
        report.error("session pinned SDK header hashes do not match this tool's contract")
    header_contract = session.get("sdk_header_contract")
    if not isinstance(header_contract, dict):
        report.error("session SDK header inspection result is invalid")
    elif header_contract.get("present") and not header_contract.get("contract_match"):
        report.error("runtime SDK header contract did not match during capture")
    elif header_contract.get("present") and header_contract.get("hashes") != SDK_HEADER_SHA256:
        report.error("runtime SDK header hashes differ from the pinned contract")
    elif not header_contract.get("present"):
        report.warning("SDK headers were unavailable at runtime; ctypes ABI was checked")
    if session.get("capture_source") != {
        "api": "AX_IVPS_GetChnFrame", "grp": USB_IVPS_GRP, "chn": USB_IVPS_CHN,
    }:
        report.error("session USB capture source must be AX_IVPS_GetChnFrame grp 3/chn 0")
    if not isinstance(session.get("ax_return_errors", []), list):
        report.error("session ax_return_errors is not a list")
    elif session.get("ax_return_errors"):
        report.warning("AX SDK returned nonzero/error results during USB capture")
    try:
        if session.get("ax_deinit_result") is not None and int(session["ax_deinit_result"]) != 0:
            report.warning(f"AX SYS deinitialization returned {int(session['ax_deinit_result'])}")
    except (TypeError, ValueError, OverflowError):
        report.error("invalid ax_deinit_result in USB session metadata")
    if str(session.get("capture_status", "")) == "failed":
        report.error("session metadata marks the capture as failed")

    try:
        camera_rows = _read_csv(
            dataset / "cameras/cam0/timestamps.csv", ("timestamp_ns", "filename")
        )
        frame_rows = _read_csv(
            dataset / "meta/frames.csv",
            ("frame_index", "timestamp_ns", "sequence", "source_format"),
        )
    except (OSError, UnicodeError, csv.Error, ValueError) as exc:
        report.error(f"invalid USB CSV data: {exc}")
        camera_rows, frame_rows = [], []
    try:
        timestamps = _timestamps(camera_rows)
    except (KeyError, TypeError, ValueError) as exc:
        report.error(f"invalid timestamp value in USB CSV: {exc}")
        timestamps = []
    if not camera_rows:
        report.error("USB dataset contains no images")
    if len(camera_rows) != len(frame_rows):
        report.error(
            f"USB count mismatch: cam0={len(camera_rows)}, frames={len(frame_rows)}"
        )
    if timestamps and not _strictly_increasing(timestamps):
        report.error("cam0 timestamps are not strictly increasing")
    if any(value < 0 or value > 2**63 - 1 for value in timestamps):
        report.error("cam0 timestamps are outside Kalibr nonnegative int64 range")
    previous_sequence: int | None = None
    saved_sequence_gaps = 0
    for index, row in enumerate(frame_rows, 1):
        try:
            if int(row["frame_index"]) != index:
                raise ValueError("frame_index is not contiguous")
            if index <= len(timestamps) and int(row["timestamp_ns"]) != timestamps[index - 1]:
                raise ValueError("frame timestamp disagrees with cam0 timestamps.csv")
            sequence = int(row["sequence"])
            if previous_sequence is not None:
                if sequence <= previous_sequence:
                    report.warning("saved USB source sequence regressed")
                else:
                    saved_sequence_gaps += max(0, sequence - previous_sequence - 1)
            previous_sequence = sequence
            if not row["source_format"]:
                raise ValueError("source_format is empty")
        except (KeyError, TypeError, ValueError) as exc:
            report.error(f"invalid meta/frames.csv line {index + 1}: {exc}")

    referenced: set[str] = set()
    for index, row in enumerate(camera_rows):
        filename = row.get("filename", "")
        if not filename or Path(filename).name != filename or not filename.endswith(suffix):
            report.error(f"unsafe or invalid cam0 filename: {filename!r}")
            continue
        expected_stem = (
            BASE_LINK_POSES[index]
            if mode == USB_BASELINK_MODE and index < len(BASE_LINK_POSES)
            else str(timestamps[index])
        )
        if Path(filename).stem != expected_stem:
            report.error(
                f"cam0 filename does not match expected stem {expected_stem!r}: {filename}"
            )
        referenced.add(filename)
        path = dataset / "cameras/cam0" / filename
        if not path.is_file():
            report.error(f"missing image: cameras/cam0/{filename}")
            continue
        try:
            info = inspect_mono8_image(path)
            if (info.width, info.height) != USB_RESOLUTION:
                report.error(
                    f"wrong USB image resolution {filename}: {info.width}x{info.height}"
                )
        except (OSError, ValueError, RuntimeError) as exc:
            report.error(f"invalid image cameras/cam0/{filename}: {exc}")
    extras = {
        path.name for path in (dataset / "cameras/cam0").iterdir()
        if path.suffix.lower() in {".bmp", ".jpg", ".jpeg"}
    } - referenced
    if extras:
        report.error("unreferenced cam0 image files: " + ", ".join(sorted(extras)[:5]))
    if mode == USB_BASELINK_MODE and len(camera_rows) != len(BASE_LINK_POSES):
        report.error(
            f"USB base_link dataset has {len(camera_rows)} images; "
            f"expected {len(BASE_LINK_POSES)}"
        )
    temporary_files = [
        path.relative_to(dataset).as_posix() for path in dataset.rglob("*.tmp") if path.is_file()
    ]
    if temporary_files:
        report.error("temporary files remain in dataset: " + ", ".join(temporary_files[:5]))
    try:
        if int(session.get("frame_count", -1)) != len(frame_rows):
            report.error("session frame_count disagrees with frames.csv")
        if int(session.get("pair_count", -1)) != 0 or int(session.get("imu_count", -1)) != 0:
            report.error("USB session pair_count and imu_count must both be zero")
    except (TypeError, ValueError, OverflowError):
        report.error("invalid frame/pair/IMU counts in session metadata")
    first_timestamp = timestamps[0] if timestamps else None
    last_timestamp = timestamps[-1] if timestamps else None
    if session.get("camera_first_timestamp_ns") != first_timestamp:
        report.error("session camera_first_timestamp_ns disagrees with USB timestamps")
    if session.get("camera_last_timestamp_ns") != last_timestamp:
        report.error("session camera_last_timestamp_ns disagrees with USB timestamps")

    capture_stats = session.get("capture_stats", {})
    if not isinstance(capture_stats, dict):
        report.error("session capture_stats is not an object")
        capture_stats = {}

    def capture_stat_int(name: str) -> int:
        try:
            return int(capture_stats.get(name, 0))
        except (TypeError, ValueError, OverflowError):
            report.error(f"invalid {name} in USB session metadata")
            return 0

    rate = _effective_rate(timestamps)
    try:
        target_rate = float(config.get("save_rate_hz", 0.0))
    except (TypeError, ValueError, OverflowError):
        target_rate = 0.0
        report.error("invalid USB save_rate_hz")
    if mode == USB_CONTINUOUS_MODE and rate is not None and target_rate > 0:
        if rate < target_rate * 0.9:
            report.warning(
                f"cam0 effective rate {rate:.3f} Hz is below 90% of target {target_rate:g} Hz"
            )
    queue_drops = capture_stat_int("writer_queue_drop_frames")
    if queue_drops:
        report.warning(f"writer queue dropped USB frames: {queue_drops}")
    source_gaps = capture_stat_int("source_sequence_gaps")
    source_regressions = capture_stat_int("source_sequence_regressions")
    if source_gaps:
        report.warning(f"USB source sequence gaps between successful IVPS gets: {source_gaps}")
    if source_regressions:
        report.warning(f"USB source sequence regressions observed: {source_regressions}")
    report.stats = {
        "mode": mode,
        "cam0_count": len(camera_rows),
        "cam1_count": 0,
        "imu_count": 0,
        "cam0_effective_rate_hz": rate,
        "camera_sequence_gap_count": saved_sequence_gaps,
        "camera_source_sequence_gap_count": source_gaps,
        "camera_source_sequence_regression_count": source_regressions,
        "rate_limiter_skipped_frames": capture_stat_int("rate_limiter_skipped_frames"),
        "writer_queue_drop_frames": queue_drops,
        "ivps_grp": USB_IVPS_GRP,
        "ivps_chn": USB_IVPS_CHN,
        "capture_schema_version": DATASET_SCHEMA_VERSION,
        "data_integrity": "structural",
        "checksums_verified": False,
    }
    if write_report:
        _write_validation_report(dataset, report, compact=compact)
    return report


def validate_dataset(
    root: str | Path,
    *,
    write_report: bool = True,
) -> ValidationReport:
    dataset = Path(root).expanduser().resolve()
    report = ValidationReport()
    try:
        session = json.loads((dataset / "meta/session.json").read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        report.error(f"invalid meta/session.json: {exc}")
        session = {}
    if not isinstance(session, dict):
        report.error("meta/session.json root is not an object")
        session = {}
    compact = session.get("capture_layout") == CAPTURE_LAYOUT
    config = session.get("config", {})
    if not isinstance(config, dict):
        report.error("meta/session.json config is not an object")
        config = {}
    read_imu = is_stereo_imu_mode(config.get("mode"))
    if is_usb_mode(config.get("mode")):
        return _validate_usb_dataset(
            dataset, session, config, report, write_report=write_report
        )
    if session.get("schema_version") != DATASET_SCHEMA_VERSION:
        report.error("unsupported dataset schema_version; expected 1.0.0 (no legacy compatibility)")
    if session.get("kind") != "capture_session" or not compact:
        report.error("session requires kind capture_session and capture_layout compact")
    if not isinstance(session.get("dataset_id"), str) or not session["dataset_id"]:
        report.error("session requires a nonempty dataset_id")
    if config.get("schema_version") != DATASET_SCHEMA_VERSION:
        report.error("capture configuration schema_version must be 1.0.0")
    required = [
        "dataset.yaml",
        "cameras/cam0/timestamps.csv",
        "cameras/cam1/timestamps.csv",
        "meta/pairs.csv",
        "meta/session.json",
    ]
    if read_imu:
        required.extend(("imu/imu0.csv", "meta/imu_full.jsonl"))
    for relative in required:
        if not (dataset / relative).is_file():
            report.error(f"missing required file: {relative}")
        elif not (dataset / relative).resolve().is_relative_to(dataset):
            report.error(f"unsafe required file: {relative}")
    if report.errors:
        if write_report and dataset.is_dir():
            _write_validation_report(dataset, report, compact=compact)
        return report

    schema_version = session.get("schema_version")
    if session.get("data_integrity") != "structural":
        report.error("session data_integrity must be structural")
    try:
        new_layout = _validate_kalibr_layout(dataset, session, config, report)
    except (OSError, UnicodeError) as exc:
        report.error(f"cannot read dataset manifest: {exc}")
        new_layout = bool(session.get("dataset_manifest"))
    stored_format = config.get("image_format", "bmp")
    try:
        suffix = image_suffix(stored_format)
    except ValueError as exc:
        report.error(str(exc))
        suffix = ".bmp"
    if stored_format == "jpeg":
        report.warning("JPEG storage is lossy; exported mono8 pixels are decoded JPEG, not original pixels")
    if schema_version == DATASET_SCHEMA_VERSION:
        if "image_format" not in config:
            report.error("missing image_format in session config")
        try:
            if not 1 <= int(config["jpeg_quality"]) <= 100:
                raise ValueError("out of range")
        except (KeyError, TypeError, ValueError, OverflowError):
            report.error("invalid or missing jpeg_quality in session config")
    expected_abi = {"base_size": 232, "pts_offset": 192, "sequence_offset": 200}
    if session.get("ax_video_frame_abi") != expected_abi:
        report.error("session AX_VIDEO_FRAME_T ABI metadata does not match the pinned contract")
    expected_img_abi = {
        "base_size": 576,
        "frame_info_offset": 0,
        "isp_info_offset": 240,
    }
    if session.get("ax_img_info_abi") != expected_img_abi:
        report.error("session AX_IMG_INFO_T ABI metadata does not match the pinned contract")
    pinned_hashes = session.get("pinned_sdk_header_sha256")
    if not isinstance(pinned_hashes, dict) or {
        str(key): str(value).lower() for key, value in pinned_hashes.items()
    } != SDK_HEADER_SHA256:
        report.error("session pinned SDK header hashes do not match this tool's contract")
    header_contract = session.get("sdk_header_contract")
    if header_contract is None:
        report.warning("session has no runtime SDK header inspection result")
    elif not isinstance(header_contract, dict):
        report.error("session SDK header inspection result is invalid")
    elif header_contract.get("present"):
        if not header_contract.get("contract_match"):
            report.error("runtime SDK header contract did not match during capture")
        actual_hashes = header_contract.get("hashes", {})
        if not isinstance(actual_hashes, dict) or {
            str(key): str(value).lower() for key, value in actual_hashes.items()
        } != SDK_HEADER_SHA256:
            report.error("runtime SDK header hashes differ from the pinned contract")
    else:
        report.warning("SDK headers were unavailable at runtime; ctypes ABI was checked")
    ax_return_errors = session.get("ax_return_errors", [])
    if not isinstance(ax_return_errors, list):
        report.error("session ax_return_errors is not a list")
    elif ax_return_errors:
        report.warning(f"AX SDK returned nonzero/error results: {len(ax_return_errors)}")
    try:
        deinit_result = session.get("ax_deinit_result", session.get("ax_sys_deinit_result"))
        if deinit_result is not None and int(deinit_result) != 0:
            report.warning(f"AX VIN/SYS deinitialization returned {int(deinit_result)}")
    except (TypeError, ValueError, OverflowError):
        report.error("invalid ax_deinit_result in session metadata")
    if str(session.get("capture_status", "")) == "failed":
        report.error("session metadata marks the capture as failed")
    mode = str(config.get("mode", session.get("mode", "")))
    triggered_modes = {"stereo-triggered", STEREO_BASELINK_MODE, "stereo_base_link_triggered"}
    if mode not in triggered_modes | {"stereo-continuous", "stereo-imu", "camera-imu"}:
        report.error(f"unsupported or missing capture mode: {mode!r}")
    try:
        expected_width = int(config.get("width", 0))
        expected_height = int(config.get("height", 0))
        expected_source_rate = float(config.get("expected_source_rate_hz", 0.0))
        frame_divisor = int(config.get("frame_divisor", 0))
        expected_camera_rate = (
            expected_source_rate / frame_divisor
            if frame_divisor > 0
            else float(config.get("save_rate_hz", 0.0))
        )
        expected_imu_rate = float(config.get("imu_expected_rate_hz", 0.0))
        tolerance_us = float(config.get("sync_tolerance_us", DEFAULTS["sync_tolerance_us"]))
    except (TypeError, ValueError, OverflowError) as exc:
        report.error(f"invalid numeric configuration in meta/session.json: {exc}")
        expected_width = expected_height = 0
        expected_camera_rate = expected_source_rate = expected_imu_rate = 0.0
        tolerance_us = float(DEFAULTS["sync_tolerance_us"])
    if expected_width <= 0 or expected_height <= 0:
        report.error("session metadata has invalid camera resolution")
    if not math.isfinite(tolerance_us) or tolerance_us <= 0:
        report.error("session metadata has invalid stereo synchronization tolerance")
    for name, rate in (("camera save rate", expected_camera_rate),
                       ("camera source rate", expected_source_rate), ("IMU rate", expected_imu_rate)):
        if not math.isfinite(rate):
            report.error(f"session metadata has a non-finite {name}")

    try:
        metadata_file = "meta/capture_dataset.yaml" if new_layout else "dataset.yaml"
        dataset_yaml = (dataset / metadata_file).read_text(encoding="utf-8")
        required_yaml_lines = (
            f"schema_version: {schema_version}",
            "timestamp_unit: ns",
            "capture_api: AX_VIN_GetYuvFrame",
            "source_pixel_format: NV12",
            "    encoding: mono8",
            "    timestamps: cameras/cam0/timestamps.csv",
            "    timestamps: cameras/cam1/timestamps.csv",
            "  measurements: imu/imu0.csv" if read_imu else "  measurements: null",
            "  full_messages: meta/imu_full.jsonl" if read_imu else "  full_messages: null",
        )
        for required_line in required_yaml_lines:
            if required_line not in dataset_yaml:
                report.error(f"{metadata_file} is missing contract entry: {required_line.strip()}")
        if "data_integrity: structural" not in dataset_yaml:
            report.error(f"{metadata_file} is missing data_integrity: structural")
        if schema_version == DATASET_SCHEMA_VERSION:
            for line in (f"image_format: {stored_format}",
                         f"jpeg_quality: {config.get('jpeg_quality')}",
                         f"lossy: {'true' if stored_format == 'jpeg' else 'false'}"):
                if line not in dataset_yaml.splitlines():
                    report.error(f"{metadata_file} is missing contract entry: {line}")
        resolution_line = f"    resolution: [{expected_width}, {expected_height}]"
        if expected_width and expected_height and dataset_yaml.count(resolution_line) != 2:
            report.error(f"{metadata_file} camera resolutions disagree with session metadata")
        if f"mode: \"{mode}\"" not in dataset_yaml:
            report.error(f"{metadata_file} mode disagrees with session metadata")
        for mapping in (
            f"    vin: {{pipe: {config.get('left_pipe')}, chn: {config.get('left_chn')}}}",
            f"    vin: {{pipe: {config.get('right_pipe')}, chn: {config.get('right_chn')}}}",
        ):
            if mapping not in dataset_yaml:
                report.error(f"{metadata_file} is missing VIN mapping: {mapping.strip()}")
        for key, prefix in (
            ("left_topic", "    topic: "),
            ("right_topic", "    topic: "),
            ("imu_topic", "  topic: "),
        ):
            expected_topic_line = prefix + json.dumps(
                str(config.get(key, "")), ensure_ascii=False
            )
            if expected_topic_line not in dataset_yaml:
                report.error(f"{metadata_file} topic disagrees with session field {key}")
        expected_imu_enabled = "true" if is_stereo_imu_mode(mode) else "false"
        if f"  enabled: {expected_imu_enabled}" not in dataset_yaml:
            report.error(f"{metadata_file} IMU enabled flag disagrees with capture mode")
    except (OSError, UnicodeError) as exc:
        report.error(f"cannot read {metadata_file}: {exc}")

    try:
        camera_fields = ("timestamp_ns", "filename")
        cam0_rows = _read_csv(dataset / "cameras/cam0/timestamps.csv", camera_fields)
        cam1_rows = _read_csv(dataset / "cameras/cam1/timestamps.csv", camera_fields)
        pair_rows = _read_csv(
            dataset / "meta/pairs.csv",
            (
                "pair_index",
                "left_timestamp_ns",
                "right_timestamp_ns",
                "left_sequence",
                "right_sequence",
                "delta_us",
                "left_format",
                "right_format",
            ),
        )
        imu_fields = ("timestamp_ns", "wx", "wy", "wz", "ax", "ay", "az")
        imu_rows = _read_csv(
            dataset / "imu/imu0.csv",
            imu_fields,
        ) if read_imu else []
    except (OSError, UnicodeError, csv.Error, ValueError, KeyError) as exc:
        report.error(f"invalid CSV data: {exc}")
        cam0_rows, cam1_rows, pair_rows, imu_rows = [], [], [], []

    try:
        cam0_ts = _timestamps(cam0_rows) if cam0_rows else []
        cam1_ts = _timestamps(cam1_rows) if cam1_rows else []
        imu_ts = _timestamps(imu_rows) if imu_rows else []
    except (KeyError, TypeError, ValueError) as exc:
        report.error(f"invalid timestamp value in CSV: {exc}")
        cam0_ts, cam1_ts, imu_ts = [], [], []
    if not cam0_rows or not cam1_rows:
        report.error("dataset contains no stereo images")
    if len(cam0_rows) != len(cam1_rows) or len(cam0_rows) != len(pair_rows):
        report.error(
            f"stereo count mismatch: cam0={len(cam0_rows)}, cam1={len(cam1_rows)}, pairs={len(pair_rows)}"
        )
    for name, values in (("cam0", cam0_ts), ("cam1", cam1_ts), ("imu", imu_ts)):
        if values and not _strictly_increasing(values):
            report.error(f"{name} timestamps are not strictly increasing")
        if new_layout and any(value < 0 or value > 2**63 - 1 for value in values):
            report.error(f"{name} timestamps are outside Kalibr nonnegative int64 range")

    if mode in triggered_modes:
        try:
            required_poses = int(config.get("poses", 20))
        except (TypeError, ValueError, OverflowError):
            report.error("invalid poses value in session metadata")
            required_poses = 20
        if len(pair_rows) != required_poses:
            report.error(f"triggered dataset has {len(pair_rows)} pairs; expected {required_poses}")
        if is_stereo_baselink_mode(mode) and required_poses != len(BASE_LINK_POSES):
            report.error(
                f"base_link pose count is {required_poses}; expected {len(BASE_LINK_POSES)}"
            )
    if is_stereo_imu_mode(mode) and not imu_rows:
        report.error("stereo-imu dataset contains no IMU samples")

    maximum_delta = 0.0
    sequence_gaps = 0
    sequence_regressions = 0
    previous_left_seq: int | None = None
    previous_right_seq: int | None = None
    for pair_index, row in enumerate(pair_rows):
        try:
            recorded_pair_index = int(row["pair_index"])
            delta = float(row["delta_us"])
            left_seq = int(row["left_sequence"])
            right_seq = int(row["right_sequence"])
        except (KeyError, TypeError, ValueError) as exc:
            report.error(f"invalid pair metadata: {exc}")
            continue
        if row.get("left_format") != "NV12" or row.get("right_format") != "NV12":
            report.error(
                f"pair {pair_index + 1} source format is not NV12: "
                f"left={row.get('left_format')!r}, right={row.get('right_format')!r}"
            )
        if recorded_pair_index != pair_index + 1:
            report.error(
                f"pair_index is not consecutive at row {pair_index + 1}: "
                f"{recorded_pair_index}"
            )
        maximum_delta = max(maximum_delta, delta)
        if not math.isfinite(delta):
            report.error("stereo pair has a non-finite delta")
        if delta < 0:
            report.error(f"stereo pair has a negative delta: {delta:g} us")
        if delta > tolerance_us:
            report.error(f"stereo pair exceeds {tolerance_us:g} us: {delta:g} us")
        if pair_index < len(cam0_ts) and pair_index < len(cam1_ts):
            try:
                metadata_left = int(row["left_timestamp_ns"])
                metadata_right = int(row["right_timestamp_ns"])
            except (KeyError, TypeError, ValueError) as exc:
                report.error(f"invalid pair timestamps: {exc}")
            else:
                if metadata_left != cam0_ts[pair_index] or metadata_right != cam1_ts[pair_index]:
                    report.error(f"pair {pair_index + 1} timestamps disagree with camera CSV files")
                timestamp_delta_us = abs(metadata_left - metadata_right) / 1000.0
                if abs(timestamp_delta_us - delta) > 0.0005:
                    report.error(
                        f"pair {pair_index + 1} delta_us disagrees with its timestamps"
                    )
                if timestamp_delta_us > tolerance_us:
                    report.error(
                        f"stereo pair exceeds {tolerance_us:g} us by timestamps: "
                        f"{timestamp_delta_us:g} us"
                    )
        if previous_left_seq is not None and left_seq > previous_left_seq + 1:
            sequence_gaps += left_seq - previous_left_seq - 1
        elif previous_left_seq is not None and left_seq <= previous_left_seq:
            sequence_regressions += 1
        if previous_right_seq is not None and right_seq > previous_right_seq + 1:
            sequence_gaps += right_seq - previous_right_seq - 1
        elif previous_right_seq is not None and right_seq <= previous_right_seq:
            sequence_regressions += 1
        previous_left_seq, previous_right_seq = left_seq, right_seq
    if sequence_regressions:
        report.warning(
            f"saved camera sequence regressions observed: {sequence_regressions}"
        )

    referenced: dict[str, set[str]] = {"cam0": set(), "cam1": set()}
    for camera, rows in (("cam0", cam0_rows), ("cam1", cam1_rows)):
        for row_index, row in enumerate(rows):
            filename = row.get("filename", "")
            if (
                not filename
                or "\x00" in filename
                or Path(filename).name != filename
                or not filename.endswith(suffix)
            ):
                report.error(f"unsafe or invalid {camera} filename: {filename!r}")
                continue
            try:
                expected_stem = (
                    BASE_LINK_POSES[row_index]
                    if is_stereo_baselink_mode(mode) and row_index < len(BASE_LINK_POSES)
                    else str(int(row["timestamp_ns"]))
                )
                if Path(filename).stem != expected_stem:
                    report.error(
                        f"{camera} filename does not match expected stem "
                        f"{expected_stem!r}: {filename}"
                    )
            except (KeyError, TypeError, ValueError):
                report.error(f"invalid {camera} timestamp for {filename}")
                continue
            referenced[camera].add(filename)
            path = dataset / "cameras" / camera / filename
            try:
                path.resolve().relative_to((dataset / "cameras" / camera).resolve())
            except (OSError, ValueError):
                report.error(f"unsafe image path: cameras/{camera}/{filename}")
                continue
            if not path.is_file():
                report.error(f"missing image: cameras/{camera}/{filename}")
                continue
            try:
                info = inspect_mono8_image(path)
            except (OSError, ValueError, RuntimeError) as exc:
                report.error(f"invalid image cameras/{camera}/{filename}: {exc}")
                continue
            if expected_width and expected_height and (
                info.width != expected_width or info.height != expected_height
            ):
                report.error(
                    f"wrong image resolution cameras/{camera}/{filename}: "
                    f"{info.width}x{info.height}"
                )
        actual = {path.name for path in (dataset / "cameras" / camera).iterdir()
                  if path.suffix.lower() in {".bmp", ".jpg", ".jpeg"}}
        extras = sorted(actual - referenced[camera])
        if extras:
            report.error(f"unreferenced {camera} image files: {', '.join(extras[:5])}")

    temporary_files = [
        path.relative_to(dataset).as_posix()
        for path in dataset.rglob("*.tmp")
        if path.is_file()
    ]
    if temporary_files:
        report.error("temporary files remain in dataset: " + ", ".join(temporary_files[:5]))

    csv_imu_measurements: list[tuple[float, ...] | None] = []
    for line_number, row in enumerate(imu_rows, 2):
        try:
            values = tuple(
                float(row[key])
                for key in imu_fields[1:]
            )
            if not all(math.isfinite(value) for value in values):
                raise ValueError("non-finite measurement")
            csv_imu_measurements.append(values)
        except (KeyError, TypeError, ValueError) as exc:
            report.error(f"invalid IMU CSV line {line_number}: {exc}")
            csv_imu_measurements.append(None)

    full_imu_timestamps: list[int] = []
    full_imu_measurements: list[tuple[float, ...] | None] = []
    effective_imu_frame_ids: set[str] = set()
    empty_imu_frame_id_count = 0
    try:
        full_lines = ((dataset / "meta/imu_full.jsonl").read_text(encoding="utf-8").splitlines()
                      if read_imu else [])
        for line_number, raw in enumerate(
            full_lines, 1
        ):
            if not raw.strip():
                continue
            try:
                payload = json.loads(raw)
                if not isinstance(payload, dict):
                    raise ValueError("record is not an object")
                if payload.get("schema_version") != DATASET_SCHEMA_VERSION or payload.get("kind") != "imu_message":
                    raise ValueError("IMU message requires schema_version 1.0.0 and kind imu_message")
                if not isinstance(payload.get("frame_id"), str):
                    raise ValueError("frame_id is not a string")
                empty_imu_frame_id_count += int(not payload["frame_id"])
                effective_imu_frame_ids.add(payload["frame_id"] or str(config.get("imu_frame_id", "")))
                if int(payload.get("arrival_monotonic_ns", 0)) <= 0:
                    raise ValueError("arrival_monotonic_ns is missing or invalid")
                full_imu_timestamps.append(int(payload["timestamp_ns"]))
                orientation = payload.get("orientation", {})
                angular = payload.get("angular_velocity", {})
                linear = payload.get("linear_acceleration", {})
                orientation_values = tuple(
                    float(orientation[key]) for key in ("x", "y", "z", "w")
                )
                measurement_values = tuple(
                    float(angular[key]) for key in ("x", "y", "z")
                ) + tuple(float(linear[key]) for key in ("x", "y", "z"))
                if not all(
                    math.isfinite(value)
                    for value in orientation_values + measurement_values
                ):
                    raise ValueError("non-finite IMU value")
                for key in (
                    "orientation_covariance",
                    "angular_velocity_covariance",
                    "linear_acceleration_covariance",
                ):
                    raw_covariance = payload.get(key)
                    if not isinstance(raw_covariance, list) or len(raw_covariance) != 9:
                        raise ValueError(f"invalid {key}: expected an array of 9 values")
                    covariance = tuple(float(value) for value in raw_covariance)
                    if len(covariance) != 9 or not all(
                        math.isfinite(value) for value in covariance
                    ):
                        raise ValueError(f"invalid {key}")
                full_imu_measurements.append(measurement_values)
            except (json.JSONDecodeError, KeyError, TypeError, ValueError, OverflowError) as exc:
                report.error(f"invalid IMU JSONL line {line_number}: {exc}")
                full_imu_measurements.append(None)
    except (OSError, UnicodeError) as exc:
        report.error(f"invalid meta/imu_full.jsonl: {exc}")
    if full_imu_timestamps != imu_ts:
        report.error("imu0.csv and imu_full.jsonl timestamps/counts disagree")
    if full_imu_measurements != csv_imu_measurements:
        report.error("imu0.csv measurements disagree with imu_full.jsonl")
    if new_layout:
        if len(effective_imu_frame_ids) > 1:
            report.error("IMU effective frame_id changed during the session")
        if effective_imu_frame_ids and (
            not isinstance(session.get("imu_frame_id"), str)
            or effective_imu_frame_ids != {session["imu_frame_id"]}
        ):
            report.error("IMU frame_id in session metadata disagrees with raw messages/fallback")
        if session.get("imu_empty_frame_id_count") != empty_imu_frame_id_count:
            report.error("session imu_empty_frame_id_count disagrees with raw messages")
        if empty_imu_frame_id_count:
            report.warning(f"{empty_imu_frame_id_count} IMU messages have empty frame_id; "
                           f"using configured fallback {config.get('imu_frame_id')!r}")

    try:
        if int(session.get("pair_count", -1)) != len(pair_rows):
            report.error("session pair_count disagrees with pairs.csv")
        if int(session.get("imu_count", -1)) != len(imu_rows):
            report.error("session imu_count disagrees with imu0.csv")
    except (TypeError, ValueError, OverflowError):
        report.error("invalid pair_count or imu_count in session metadata")

    camera_rate0 = _effective_rate(cam0_ts)
    camera_rate1 = _effective_rate(cam1_ts)
    imu_rate = _effective_rate(imu_ts)
    if mode not in triggered_modes and expected_camera_rate > 0:
        for name, rate in (("cam0", camera_rate0), ("cam1", camera_rate1)):
            if rate is not None and rate < expected_camera_rate * 0.9:
                report.warning(
                    f"{name} effective rate {rate:.3f} Hz is below 90% of target {expected_camera_rate:g} Hz"
                )
    if is_stereo_imu_mode(mode) and expected_imu_rate > 0 and imu_rate is not None:
        if imu_rate < expected_imu_rate * 0.9:
            report.warning(
                f"IMU effective rate {imu_rate:.3f} Hz is below 90% of target {expected_imu_rate:g} Hz"
            )
    imu_max_gap_ns: int | None = None
    imu_long_gap_count = 0
    camera_imu_first_offset_ns: int | None = None
    camera_imu_common_window_ns: int | None = None
    startup_offset_ns: int | None = None
    preroll_actual_s: float | None = None
    postroll_actual_s: float | None = None
    first_camera_ns = min(cam0_ts[0], cam1_ts[0]) if cam0_ts and cam1_ts else None
    last_camera_ns = max(cam0_ts[-1], cam1_ts[-1]) if cam0_ts and cam1_ts else None
    capture_stats = session.get("capture_stats", {})
    if not isinstance(capture_stats, dict):
        report.error("session capture_stats is not an object")
        capture_stats = {}
    if new_layout:
        for key, actual in (("camera_first_timestamp_ns", first_camera_ns),
                            ("camera_last_timestamp_ns", last_camera_ns)):
            if key not in session or session[key] != actual:
                report.error(f"session {key} disagrees with saved stereo timestamps")
    if len(imu_ts) > 1:
        imu_gaps = [current - previous for previous, current in zip(imu_ts, imu_ts[1:])]
        imu_max_gap_ns = max(imu_gaps)
        if is_stereo_imu_mode(mode) and expected_imu_rate > 0 and math.isfinite(expected_imu_rate):
            threshold = 2.5 * 1_000_000_000 / expected_imu_rate
            # Extreme finite metadata can still overflow division; such a rate
            # has no representable gap threshold and must not crash validation.
            if not math.isfinite(threshold):
                report.error("IMU rate is outside the representable gap range")
                threshold = 2**63 - 1
            long_gap_threshold_ns = round(threshold)
            imu_long_gap_count = sum(gap > long_gap_threshold_ns for gap in imu_gaps)
            if imu_long_gap_count:
                report.warning(
                    f"IMU has {imu_long_gap_count} intervals longer than "
                    f"{long_gap_threshold_ns / 1e6:.3f} ms; maximum is "
                    f"{imu_max_gap_ns / 1e6:.3f} ms"
                )
    if is_stereo_imu_mode(mode) and first_camera_ns is not None and imu_ts:
        try:
            max_offset_s = float(config.get("clock_domain_max_offset_s", 5.0))
            if not math.isfinite(max_offset_s) or max_offset_s <= 0:
                raise ValueError("must be finite and positive")
            max_offset_ns = int(max_offset_s * 1e9)
        except (TypeError, ValueError, OverflowError):
            report.error("invalid clock_domain_max_offset_s in session metadata")
            max_offset_ns = 5_000_000_000
        first_offset = abs(first_camera_ns - imu_ts[0])
        camera_imu_first_offset_ns = first_offset
        common_start = max(cam0_ts[0], cam1_ts[0] if cam1_ts else cam0_ts[0], imu_ts[0])
        common_end = min(cam0_ts[-1], cam1_ts[-1] if cam1_ts else cam0_ts[-1], imu_ts[-1])
        camera_imu_common_window_ns = max(0, common_end - common_start)
        clock_offset = first_offset
        if new_layout:
            # Pre-roll intentionally separates the first saved image from the
            # first IMU. Only the contemporaneous startup references test clocks.
            clock_offset = None
            try:
                camera_ref = capture_stats["startup_camera_timestamp_ns"]
                imu_ref = capture_stats["startup_imu_timestamp_ns"]
                if any(type(value) is not int or not 0 <= value <= 2**63 - 1
                       for value in (camera_ref, imu_ref)):
                    raise ValueError("startup references must be nonnegative int64 timestamps")
                startup_offset_ns = abs(camera_ref - imu_ref)
                if capture_stats.get("startup_camera_imu_offset_ns") != startup_offset_ns:
                    raise ValueError("startup reference offset disagrees with timestamps")
                if imu_ref not in imu_ts or camera_ref > first_camera_ns:
                    raise ValueError("startup reference is outside the recorded IMU / pre-image window")
                clock_offset = startup_offset_ns
            except (KeyError, TypeError, ValueError) as exc:
                report.error(f"invalid or missing startup clock references: {exc}")
        if clock_offset is not None and clock_offset > max_offset_ns:
            report.error(f"camera/IMU clock-domain offset exceeds limit: {clock_offset / 1e9:.6f} s")
        if common_end < common_start:
            report.error("camera and IMU timestamp ranges do not overlap")
        preroll_actual_s = (first_camera_ns - imu_ts[0]) / 1e9
        postroll_actual_s = (imu_ts[-1] - last_camera_ns) / 1e9
        if new_layout:
            for name, actual, coverage_ns in (
                ("imu_preroll", preroll_actual_s, first_camera_ns - imu_ts[0]),
                ("imu_postroll", postroll_actual_s, imu_ts[-1] - last_camera_ns),
            ):
                try:
                    requested = float(config[name + "_s"])
                    if not math.isfinite(requested) or requested < 0:
                        raise ValueError("must be finite and nonnegative")
                    if requested > 0 and coverage_ns < round(requested * 1e9):
                        report.error(f"{name} coverage {actual:.9f} s is below required {requested:g} s")
                    stored_actual = float(session[name + "_actual_s"])
                    if not math.isfinite(stored_actual) or abs(stored_actual - actual) > 1e-9:
                        raise ValueError("actual coverage metadata disagrees with sensor timestamps")
                except (KeyError, TypeError, ValueError, OverflowError) as exc:
                    report.error(f"invalid {name} metadata: {exc}")

    def capture_stat_int(name: str) -> int:
        try:
            return int(capture_stats.get(name, 0))
        except (TypeError, ValueError, OverflowError):
            report.error(f"invalid {name} in session metadata")
            return 0

    queue_drops = capture_stat_int("writer_queue_drop_pairs")
    if queue_drops:
        report.warning(f"writer queue dropped complete stereo pairs: {queue_drops}")
    source_left_gaps = capture_stat_int("source_left_sequence_gaps")
    source_right_gaps = capture_stat_int("source_right_sequence_gaps")
    source_regressions = capture_stat_int("source_sequence_regressions")
    source_gaps = source_left_gaps + source_right_gaps
    sequence_accounting = capture_stats.get("source_sequence_accounting", "legacy_post_sync")
    sync_left = capture_stat_int("sync_left_discarded_frames")
    sync_right = capture_stat_int("sync_right_discarded_frames")
    rate_skips = capture_stat_int("rate_limiter_skipped_pairs")
    divisor_skips = capture_stat_int("frame_divisor_skipped_pairs")
    source_gap_metadata_present = any(
        name in capture_stats
        for name in (
            "source_left_sequence_gaps",
            "source_right_sequence_gaps",
            "source_sequence_regressions",
        )
    )
    if source_gaps:
        if sequence_accounting == "successful_vin_get_v1":
            report.warning(f"camera source sequence gaps between successful VIN gets: {source_gaps} frames")
        else:
            report.warning(
                f"legacy camera source sequence gaps observed: {source_gaps}; "
                "may include synchronization discards, not a true VIN loss count"
            )
    elif mode not in triggered_modes and sequence_gaps and not source_gap_metadata_present:
        report.warning(
            f"saved camera sequence gaps observed without source-gap metadata: {sequence_gaps}"
        )
    if source_regressions:
        report.warning(f"camera source sequence regressions observed: {source_regressions}")
    source_rate: float | None = None
    try:
        if capture_stats.get("source_effective_rate_hz") is not None:
            source_rate = float(capture_stats["source_effective_rate_hz"])
    except (TypeError, ValueError):
        report.error("invalid source_effective_rate_hz in session metadata")
    if (
        mode not in triggered_modes
        and source_rate is not None
        and expected_source_rate > 0
        and source_rate < expected_source_rate * 0.9
    ):
        report.warning(
            f"camera source rate {source_rate:.3f} Hz is below 90% of expected "
            f"{expected_source_rate:g} Hz"
        )

    report.stats = {
        "mode": mode,
        "cam0_count": len(cam0_rows),
        "cam1_count": len(cam1_rows),
        "imu_count": len(imu_rows),
        "maximum_stereo_delta_us": maximum_delta,
        "camera_sequence_gap_count": sequence_gaps,
        "camera_sequence_regression_count": sequence_regressions,
        "camera_source_sequence_gap_count": source_gaps,
        "source_sequence_accounting": sequence_accounting,
        "source_left_sequence_gaps": source_left_gaps,
        "source_right_sequence_gaps": source_right_gaps,
        "sync_left_discarded_frames": sync_left if sequence_accounting == "successful_vin_get_v1" else None,
        "sync_right_discarded_frames": sync_right if sequence_accounting == "successful_vin_get_v1" else None,
        "sync_discarded_frames": sync_left + sync_right if sequence_accounting == "successful_vin_get_v1" else None,
        "rate_limiter_skipped_pairs": rate_skips,
        "frame_divisor_skipped_pairs": divisor_skips,
        "writer_queue_drop_pairs": queue_drops,
        "camera_source_sequence_regression_count": source_regressions,
        "camera_source_effective_rate_hz": source_rate,
        "cam0_effective_rate_hz": camera_rate0,
        "cam1_effective_rate_hz": camera_rate1,
        "imu_effective_rate_hz": imu_rate,
        "imu_maximum_gap_ns": imu_max_gap_ns,
        "imu_long_gap_count": imu_long_gap_count,
        "camera_imu_first_offset_ns": camera_imu_first_offset_ns,
        "camera_imu_common_window_ns": camera_imu_common_window_ns,
        "startup_camera_imu_offset_ns": startup_offset_ns,
        "imu_preroll_actual_s": preroll_actual_s,
        "imu_postroll_actual_s": postroll_actual_s,
        "imu_frame_id": session.get("imu_frame_id") if new_layout else None,
        "kalibr_manifest_version": DATASET_SCHEMA_VERSION if new_layout else None,
        "capture_schema_version": schema_version,
        "data_integrity": "structural",
        "checksums_verified": False,
    }
    if write_report:
        _write_validation_report(dataset, report, compact=compact)
    return report
