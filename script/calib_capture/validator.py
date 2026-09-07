from __future__ import annotations

import csv
import hashlib
import json
import math
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Iterable

from .bmp import inspect_mono8_bmp
from .config import SDK_HEADER_SHA256


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
        return asdict(self)


def _write_validation_report(dataset: Path, report: ValidationReport) -> None:
    destination = dataset / "validation.json"
    temporary = dataset / ".validation.json.tmp"
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


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while block := stream.read(1024 * 1024):
            digest.update(block)
    return digest.hexdigest()


def _load_manifest(path: Path, report: ValidationReport) -> dict[str, str]:
    entries: dict[str, str] = {}
    if not path.is_file():
        report.error("missing meta/checksums.sha256")
        return entries
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except (OSError, UnicodeError) as exc:
        report.error(f"cannot read checksum manifest: {exc}")
        return entries
    for line_number, raw in enumerate(lines, 1):
        if not raw.strip():
            continue
        try:
            digest, relative = raw.split("  ", 1)
        except ValueError:
            report.error(f"invalid checksum line {line_number}")
            continue
        if (
            len(digest) != 64
            or any(character not in "0123456789abcdefABCDEF" for character in digest)
            or relative in entries
        ):
            report.error(f"invalid or duplicate checksum entry: {relative}")
            continue
        entries[relative] = digest.lower()
    return entries


def validate_dataset(
    root: str | Path,
    *,
    verify_checksums: bool = False,
    write_report: bool = True,
) -> ValidationReport:
    dataset = Path(root).expanduser().resolve()
    report = ValidationReport()
    required = [
        "dataset.yaml",
        "cameras/cam0/timestamps.csv",
        "cameras/cam1/timestamps.csv",
        "imu/imu0.csv",
        "meta/pairs.csv",
        "meta/imu_full.jsonl",
        "meta/session.json",
        "meta/checksums.sha256",
    ]
    for relative in required:
        if not (dataset / relative).is_file():
            report.error(f"missing required file: {relative}")
    if report.errors:
        if write_report and dataset.is_dir():
            _write_validation_report(dataset, report)
        return report

    try:
        session = json.loads((dataset / "meta/session.json").read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        report.error(f"invalid meta/session.json: {exc}")
        session = {}
    if not isinstance(session, dict):
        report.error("meta/session.json root is not an object")
        session = {}
    config = session.get("config", {})
    if not isinstance(config, dict):
        report.error("meta/session.json config is not an object")
        config = {}
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
    except (TypeError, ValueError):
        report.error("invalid ax_deinit_result in session metadata")
    if str(session.get("capture_status", "")) == "failed":
        report.error("session metadata marks the capture as failed")
    mode = str(config.get("mode", session.get("mode", "")))
    if mode not in {"stereo-triggered", "stereo-continuous", "camera-imu"}:
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
        tolerance_us = float(config.get("sync_tolerance_us", 2000.0))
    except (TypeError, ValueError) as exc:
        report.error(f"invalid numeric configuration in meta/session.json: {exc}")
        expected_width = expected_height = 0
        expected_camera_rate = expected_source_rate = expected_imu_rate = 0.0
        tolerance_us = 2000.0
    if expected_width <= 0 or expected_height <= 0:
        report.error("session metadata has invalid camera resolution")
    if tolerance_us <= 0:
        report.error("session metadata has invalid stereo synchronization tolerance")

    try:
        dataset_yaml = (dataset / "dataset.yaml").read_text(encoding="utf-8")
        required_yaml_lines = (
            "schema_version: 1",
            "timestamp_unit: ns",
            "capture_api: AX_VIN_GetYuvFrame",
            "source_pixel_format: NV12",
            "    encoding: mono8",
            "    timestamps: cameras/cam0/timestamps.csv",
            "    timestamps: cameras/cam1/timestamps.csv",
            "  measurements: imu/imu0.csv",
            "  full_messages: meta/imu_full.jsonl",
        )
        for required_line in required_yaml_lines:
            if required_line not in dataset_yaml:
                report.error(f"dataset.yaml is missing contract entry: {required_line.strip()}")
        resolution_line = f"    resolution: [{expected_width}, {expected_height}]"
        if expected_width and expected_height and dataset_yaml.count(resolution_line) != 2:
            report.error("dataset.yaml camera resolutions disagree with session metadata")
        if f"mode: \"{mode}\"" not in dataset_yaml:
            report.error("dataset.yaml mode disagrees with session metadata")
        for mapping in (
            f"    vin: {{pipe: {config.get('left_pipe')}, chn: {config.get('left_chn')}}}",
            f"    vin: {{pipe: {config.get('right_pipe')}, chn: {config.get('right_chn')}}}",
        ):
            if mapping not in dataset_yaml:
                report.error(f"dataset.yaml is missing VIN mapping: {mapping.strip()}")
        for key, prefix in (
            ("left_topic", "    topic: "),
            ("right_topic", "    topic: "),
            ("imu_topic", "  topic: "),
        ):
            expected_topic_line = prefix + json.dumps(
                str(config.get(key, "")), ensure_ascii=False
            )
            if expected_topic_line not in dataset_yaml:
                report.error(f"dataset.yaml topic disagrees with session field {key}")
        expected_imu_enabled = "true" if mode == "camera-imu" else "false"
        if f"  enabled: {expected_imu_enabled}" not in dataset_yaml:
            report.error("dataset.yaml IMU enabled flag disagrees with capture mode")
    except (OSError, UnicodeError) as exc:
        report.error(f"cannot read dataset.yaml: {exc}")

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
        imu_rows = _read_csv(
            dataset / "imu/imu0.csv",
            (
                "timestamp_ns",
                "gyro_x",
                "gyro_y",
                "gyro_z",
                "accel_x",
                "accel_y",
                "accel_z",
            ),
        )
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

    if mode == "stereo-triggered":
        try:
            required_poses = int(config.get("poses", 20))
        except (TypeError, ValueError):
            report.error("invalid poses value in session metadata")
            required_poses = 20
        if len(pair_rows) != required_poses:
            report.error(f"triggered dataset has {len(pair_rows)} pairs; expected {required_poses}")
    if mode == "camera-imu" and not imu_rows:
        report.error("camera-imu dataset contains no IMU samples")

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
        for row in rows:
            filename = row.get("filename", "")
            if (
                not filename
                or "\x00" in filename
                or Path(filename).name != filename
                or not filename.endswith(".bmp")
            ):
                report.error(f"unsafe or invalid {camera} filename: {filename!r}")
                continue
            try:
                if Path(filename).stem != str(int(row["timestamp_ns"])):
                    report.error(f"{camera} filename does not match its timestamp: {filename}")
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
                info = inspect_mono8_bmp(path)
            except (OSError, ValueError) as exc:
                report.error(f"invalid image cameras/{camera}/{filename}: {exc}")
                continue
            if expected_width and expected_height and (
                info.width != expected_width or info.height != expected_height
            ):
                report.error(
                    f"wrong image resolution cameras/{camera}/{filename}: "
                    f"{info.width}x{info.height}"
                )
        actual = {path.name for path in (dataset / "cameras" / camera).glob("*.bmp")}
        extras = sorted(actual - referenced[camera])
        if extras:
            report.error(f"unreferenced {camera} BMP files: {', '.join(extras[:5])}")

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
                for key in (
                    "gyro_x",
                    "gyro_y",
                    "gyro_z",
                    "accel_x",
                    "accel_y",
                    "accel_z",
                )
            )
            if not all(math.isfinite(value) for value in values):
                raise ValueError("non-finite measurement")
            csv_imu_measurements.append(values)
        except (KeyError, TypeError, ValueError) as exc:
            report.error(f"invalid IMU CSV line {line_number}: {exc}")
            csv_imu_measurements.append(None)

    full_imu_timestamps: list[int] = []
    full_imu_measurements: list[tuple[float, ...] | None] = []
    try:
        for line_number, raw in enumerate(
            (dataset / "meta/imu_full.jsonl").read_text(encoding="utf-8").splitlines(), 1
        ):
            if not raw.strip():
                continue
            try:
                payload = json.loads(raw)
                if not isinstance(payload, dict):
                    raise ValueError("record is not an object")
                if not isinstance(payload.get("frame_id"), str):
                    raise ValueError("frame_id is not a string")
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
                    covariance = tuple(float(value) for value in payload.get(key, []))
                    if len(covariance) != 9 or not all(
                        math.isfinite(value) for value in covariance
                    ):
                        raise ValueError(f"invalid {key}")
                full_imu_measurements.append(measurement_values)
            except (json.JSONDecodeError, KeyError, TypeError, ValueError) as exc:
                report.error(f"invalid IMU JSONL line {line_number}: {exc}")
                full_imu_measurements.append(None)
    except (OSError, UnicodeError) as exc:
        report.error(f"invalid meta/imu_full.jsonl: {exc}")
    if full_imu_timestamps != imu_ts:
        report.error("imu0.csv and imu_full.jsonl timestamps/counts disagree")
    if full_imu_measurements != csv_imu_measurements:
        report.error("imu0.csv measurements disagree with imu_full.jsonl")

    try:
        if int(session.get("pair_count", -1)) != len(pair_rows):
            report.error("session pair_count disagrees with pairs.csv")
        if int(session.get("imu_count", -1)) != len(imu_rows):
            report.error("session imu_count disagrees with imu0.csv")
    except (TypeError, ValueError):
        report.error("invalid pair_count or imu_count in session metadata")

    manifest = _load_manifest(dataset / "meta/checksums.sha256", report)
    referenced_payloads = {
        f"cameras/cam0/{row.get('filename', '')}" for row in cam0_rows
    } | {
        f"cameras/cam1/{row.get('filename', '')}" for row in cam1_rows
    }
    referenced_payloads.update(
        {
            "dataset.yaml",
            "cameras/cam0/timestamps.csv",
            "cameras/cam1/timestamps.csv",
            "meta/pairs.csv",
            "imu/imu0.csv",
            "meta/imu_full.jsonl",
        }
    )
    for relative in referenced_payloads:
        if relative not in manifest:
            report.error(f"payload missing from checksum manifest: {relative}")
    unexpected_manifest_entries = sorted(set(manifest) - referenced_payloads)
    if unexpected_manifest_entries:
        report.error(
            "checksum manifest has unreferenced entries: "
            + ", ".join(unexpected_manifest_entries[:5])
        )
    for relative, expected_hash in manifest.items():
        path = dataset / Path(relative)
        try:
            resolved = path.resolve()
            resolved.relative_to(dataset)
        except (OSError, ValueError):
            report.error(f"unsafe checksum path: {relative}")
            continue
        if not path.is_file():
            report.error(f"checksum target is missing: {relative}")
        elif verify_checksums:
            try:
                actual_hash = _sha256(path)
            except OSError as exc:
                report.error(f"cannot hash {relative}: {exc}")
            else:
                if actual_hash != expected_hash:
                    report.error(f"checksum mismatch: {relative}")

    camera_rate0 = _effective_rate(cam0_ts)
    camera_rate1 = _effective_rate(cam1_ts)
    imu_rate = _effective_rate(imu_ts)
    if mode != "stereo-triggered" and expected_camera_rate > 0:
        for name, rate in (("cam0", camera_rate0), ("cam1", camera_rate1)):
            if rate is not None and rate < expected_camera_rate * 0.9:
                report.warning(
                    f"{name} effective rate {rate:.3f} Hz is below 90% of target {expected_camera_rate:g} Hz"
                )
    if mode == "camera-imu" and expected_imu_rate > 0 and imu_rate is not None:
        if imu_rate < expected_imu_rate * 0.9:
            report.warning(
                f"IMU effective rate {imu_rate:.3f} Hz is below 90% of target {expected_imu_rate:g} Hz"
            )
    imu_max_gap_ns: int | None = None
    imu_long_gap_count = 0
    camera_imu_first_offset_ns: int | None = None
    camera_imu_common_window_ns: int | None = None
    if len(imu_ts) > 1:
        imu_gaps = [current - previous for previous, current in zip(imu_ts, imu_ts[1:])]
        imu_max_gap_ns = max(imu_gaps)
        if mode == "camera-imu" and expected_imu_rate > 0:
            long_gap_threshold_ns = round(2.5 * 1_000_000_000 / expected_imu_rate)
            imu_long_gap_count = sum(gap > long_gap_threshold_ns for gap in imu_gaps)
            if imu_long_gap_count:
                report.warning(
                    f"IMU has {imu_long_gap_count} intervals longer than "
                    f"{long_gap_threshold_ns / 1e6:.3f} ms; maximum is "
                    f"{imu_max_gap_ns / 1e6:.3f} ms"
                )
    if mode == "camera-imu" and cam0_ts and imu_ts:
        try:
            max_offset_ns = int(float(config.get("clock_domain_max_offset_s", 5.0)) * 1e9)
        except (TypeError, ValueError):
            report.error("invalid clock_domain_max_offset_s in session metadata")
            max_offset_ns = 5_000_000_000
        first_camera_ns = min(cam0_ts[0], cam1_ts[0]) if cam1_ts else cam0_ts[0]
        first_offset = abs(first_camera_ns - imu_ts[0])
        camera_imu_first_offset_ns = first_offset
        common_start = max(cam0_ts[0], cam1_ts[0] if cam1_ts else cam0_ts[0], imu_ts[0])
        common_end = min(cam0_ts[-1], cam1_ts[-1] if cam1_ts else cam0_ts[-1], imu_ts[-1])
        camera_imu_common_window_ns = max(0, common_end - common_start)
        if first_offset > max_offset_ns:
            report.error(f"camera/IMU clock-domain offset exceeds limit: {first_offset / 1e9:.6f} s")
        if common_end < common_start:
            report.error("camera and IMU timestamp ranges do not overlap")

    capture_stats = session.get("capture_stats", {}) if isinstance(session, dict) else {}
    if not isinstance(capture_stats, dict):
        report.error("session capture_stats is not an object")
        capture_stats = {}
    def capture_stat_int(name: str) -> int:
        try:
            return int(capture_stats.get(name, 0))
        except (TypeError, ValueError):
            report.error(f"invalid {name} in session metadata")
            return 0

    queue_drops = capture_stat_int("writer_queue_drop_pairs")
    if queue_drops:
        report.warning(f"writer queue dropped complete stereo pairs: {queue_drops}")
    source_left_gaps = capture_stat_int("source_left_sequence_gaps")
    source_right_gaps = capture_stat_int("source_right_sequence_gaps")
    source_regressions = capture_stat_int("source_sequence_regressions")
    source_gaps = source_left_gaps + source_right_gaps
    source_gap_metadata_present = any(
        name in capture_stats
        for name in (
            "source_left_sequence_gaps",
            "source_right_sequence_gaps",
            "source_sequence_regressions",
        )
    )
    if source_gaps:
        report.warning(f"camera source sequence gaps observed: {source_gaps}")
    elif mode != "stereo-triggered" and sequence_gaps and not source_gap_metadata_present:
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
        mode != "stereo-triggered"
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
        "camera_source_sequence_regression_count": source_regressions,
        "camera_source_effective_rate_hz": source_rate,
        "cam0_effective_rate_hz": camera_rate0,
        "cam1_effective_rate_hz": camera_rate1,
        "imu_effective_rate_hz": imu_rate,
        "imu_maximum_gap_ns": imu_max_gap_ns,
        "imu_long_gap_count": imu_long_gap_count,
        "camera_imu_first_offset_ns": camera_imu_first_offset_ns,
        "camera_imu_common_window_ns": camera_imu_common_window_ns,
        "checksum_entries": len(manifest),
        "checksums_verified": verify_checksums,
    }
    if write_report:
        _write_validation_report(dataset, report)
    return report
