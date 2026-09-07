from __future__ import annotations

import configparser
import math
import os
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Mapping


APP_VERSION = "1.6.0"
DATASET_SCHEMA_VERSION = 2
NS_PER_SECOND = 1_000_000_000
SESSION_DIRECTORY_FORMAT = "%Y-%m-%d_%H-%M-%S"

AUTHORITATIVE_SDK_INCLUDE = Path(
    r"D:\tool\mipi_tools\sensor_timing_check_tool\bsp_sdk\include"
)

SDK_HEADER_SHA256 = {
    "ax_base_type.h": "1ade71f0acfdbaf7e8856479c272a5fc4aa747c61b6c0bc23c127c3da238ed2c",
    "ax_global_type.h": "52513b3b1ff5d6ebed00cdc5fb166c98ba5c34ecd48001de15918bbf70092419",
    "ax_ivps_api.h": "f070f8fde31ac2619aed659c069f5619e09930c7e4730cd2d4a260309c88fc84",
    "ax_ivps_type.h": "7bb0736d57e4b4b8cdc82f23f8e1809d40260ec7bbf7cf47fb983b34d578868a",
    "ax_vin_api.h": "ef5c2186ba36ff91e81bc7af083f028640a860a104d845f7a8e287c7118e7b0b",
    "ax_isp_common.h": "3decc8719c3b971cc2711652c3f911bc55aff2e5105dab4dca9c7c791b206e23",
    "ax_sys_api.h": "24a468e5582dbabdfeeb8bce98951babb85ca0b409933ca70fce12411e34c8f3",
    "ax_pool_type.h": "ba96dbab982f2737b29999a62bf29987260584fa01ea37045b7a6c3d5b655c9d",
}


DEFAULTS: dict[str, Any] = {
    "ax_lib_dir": "/soc/lib",
    "sdk_include_dir": "",
    "left_pipe": 2,
    "left_chn": 0,
    "right_pipe": 0,
    "right_chn": 0,
    "resolution": "3840x2160",
    "expected_source_rate_hz": 50.0,
    "save_rate_hz": 5.0,
    "frame_divisor": 0,
    "duration_s": 60.0,
    "poses": 20,
    "sync_tolerance_us": 2000,
    "max_sync_retries": 10,
    "capture_timeout_ms": 200,
    "writer_queue_pairs": 2,
    "startup_timeout_s": 15.0,
    "imu_topic": "/imu/data",
    "imu_expected_rate_hz": 100.0,
    "imu_qos_depth": 2000,
    "clock_domain_max_offset_s": 5.0,
    "left_topic": "/stereo/left/image_raw",
    "right_topic": "/stereo/right/image_raw",
    "left_frame_id": "cam0",
    "right_frame_id": "cam1",
    "imu_frame_id": "imu0",
}


def parse_resolution(value: str) -> tuple[int, int]:
    normalized = value.lower().replace(" ", "").replace("*", "x").replace("×", "x")
    try:
        width_text, height_text = normalized.split("x", 1)
        width, height = int(width_text), int(height_text)
    except (ValueError, TypeError) as exc:
        raise ValueError(f"invalid resolution {value!r}; use WIDTHxHEIGHT") from exc
    if (width, height) not in {(3840, 2160), (1920, 1080)}:
        raise ValueError("resolution must be 3840x2160 or 1920x1080")
    return width, height


def session_output_path(
    output_root: str | Path = "./data", *, now: datetime | None = None
) -> str:
    """Return <output_root>/YYYY-MM-DD_HH-MM-SS using local wall-clock time."""
    root_text = str(output_root).strip() or "./data"
    timestamp = (now or datetime.now()).strftime(SESSION_DIRECTORY_FORMAT)
    return str(Path(root_text).expanduser() / timestamp)


def detect_sdk_include_dir(explicit: str = "") -> str:
    if explicit:
        return str(Path(explicit).expanduser())
    candidates: list[Path] = []
    env_value = os.environ.get("AX_SDK_INCLUDE_DIR", "")
    if env_value:
        candidates.append(Path(env_value).expanduser())
    candidates.append(AUTHORITATIVE_SDK_INCLUDE)
    project_reference = (
        Path(__file__).resolve().parents[2]
        / "tmp"
        / "slam"
        / "ms_fusion"
        / "third_party_axera"
        / "include"
    )
    candidates.append(project_reference)
    for candidate in candidates:
        if all((candidate / name).is_file() for name in SDK_HEADER_SHA256):
            return str(candidate.resolve())
    return ""


def load_ini_mapping(path: str | Path | None) -> dict[str, str]:
    """Read literal values from [capture]; build_capture_config converts types."""
    if not path:
        return {}
    config_path = Path(path).expanduser()
    if config_path.suffix.lower() != ".ini":
        raise ValueError(
            "--config requires an .ini file with [capture]; convert old YAML/JSON to INI "
            "instead of only renaming the file"
        )
    parser = configparser.ConfigParser(
        interpolation=None,
        delimiters=("=",),
        strict=True,
        empty_lines_in_values=False,
    )
    # Keep misspelled/case-mismatched keys visible instead of normalizing them.
    parser.optionxform = str
    try:
        # utf-8-sig accepts both plain UTF-8 and Windows editors' UTF-8 BOM.
        with config_path.open("r", encoding="utf-8-sig") as stream:
            parser.read_file(stream)
    except configparser.Error as exc:
        raise ValueError(f"invalid INI configuration {config_path}: {exc}") from exc
    if parser.defaults():
        raise ValueError("INI [DEFAULT] settings are not supported; use [capture]")
    if not parser.has_section("capture"):
        raise ValueError("INI configuration requires a [capture] section")
    unknown_sections = sorted(set(parser.sections()) - {"capture"})
    if unknown_sections:
        raise ValueError("unknown configuration sections: " + ", ".join(unknown_sections))
    values = dict(parser.items("capture"))
    unknown = sorted(set(values) - set(DEFAULTS))
    if unknown:
        raise ValueError("unknown configuration keys: " + ", ".join(unknown))
    for key, value in values.items():
        if "\n" in value:
            raise ValueError(f"INI value must be on one line: {key}")
    return values


@dataclass(frozen=True)
class CaptureConfig:
    mode: str
    output: str
    ax_lib_dir: str
    sdk_include_dir: str
    left_pipe: int
    left_chn: int
    right_pipe: int
    right_chn: int
    width: int
    height: int
    expected_source_rate_hz: float
    save_rate_hz: float
    frame_divisor: int
    duration_s: float
    poses: int
    sync_tolerance_us: int
    max_sync_retries: int
    capture_timeout_ms: int
    writer_queue_pairs: int
    startup_timeout_s: float
    imu_topic: str
    imu_expected_rate_hz: float
    imu_qos_depth: int
    clock_domain_max_offset_s: float
    left_topic: str
    right_topic: str
    left_frame_id: str
    right_frame_id: str
    imu_frame_id: str

    @property
    def resolution(self) -> tuple[int, int]:
        return self.width, self.height

    @property
    def target_save_rate_hz(self) -> float:
        if self.frame_divisor > 0:
            return self.expected_source_rate_hz / self.frame_divisor
        return self.save_rate_hz

    def to_dict(self) -> dict[str, Any]:
        result = asdict(self)
        result["resolution"] = f"{self.width}x{self.height}"
        return result


def build_capture_config(
    *,
    mode: str,
    output: str = "",
    config_path: str | None = None,
    overrides: Mapping[str, Any] | None = None,
) -> CaptureConfig:
    file_values = load_ini_mapping(config_path)
    override_values = {
        key: value
        for key, value in (overrides or {}).items()
        if key in DEFAULTS and value is not None
    }
    if (
        "save_rate_hz" in override_values
        and int(override_values.get("frame_divisor", 0)) > 0
    ):
        raise ValueError("save_rate_hz and frame_divisor are mutually exclusive")
    if (
        "save_rate_hz" in file_values
        and int(file_values.get("frame_divisor", 0)) > 0
        and "save_rate_hz" not in override_values
        and "frame_divisor" not in override_values
    ):
        raise ValueError(
            "INI save_rate_hz and frame_divisor are mutually exclusive; choose one"
        )
    values = dict(DEFAULTS)
    values.update(file_values)
    # Selecting a save rate at the higher-priority layer disables a divisor
    # inherited from INI. Selecting a divisor naturally makes save_rate_hz
    # inactive while retaining it as the fallback for frame_divisor=0.
    if "save_rate_hz" in override_values:
        values["frame_divisor"] = 0
    values.update(override_values)

    width, height = parse_resolution(str(values.pop("resolution")))
    values["sdk_include_dir"] = detect_sdk_include_dir(str(values["sdk_include_dir"]))

    integer_nonnegative = ("left_pipe", "left_chn", "right_pipe", "right_chn")
    for key in integer_nonnegative:
        if int(values[key]) < 0:
            raise ValueError(f"{key} must be >= 0")
        values[key] = int(values[key])
    values["frame_divisor"] = int(values["frame_divisor"])
    if values["frame_divisor"] < 0:
        raise ValueError("frame_divisor must be >= 0")
    if (values["left_pipe"], values["left_chn"]) == (
        values["right_pipe"],
        values["right_chn"],
    ):
        raise ValueError("left and right VIN pipe/chn mappings must be different")
    for key in (
        "sync_tolerance_us",
        "max_sync_retries",
        "capture_timeout_ms",
        "writer_queue_pairs",
        "imu_qos_depth",
        "poses",
    ):
        if int(values[key]) <= 0:
            raise ValueError(f"{key} must be > 0")
        values[key] = int(values[key])
    for key in (
        "expected_source_rate_hz",
        "save_rate_hz",
        "duration_s",
        "startup_timeout_s",
        "imu_expected_rate_hz",
        "clock_domain_max_offset_s",
    ):
        numeric_value = float(values[key])
        if not math.isfinite(numeric_value) or numeric_value <= 0:
            raise ValueError(f"{key} must be > 0")
        values[key] = numeric_value
    for key in (
        "ax_lib_dir",
        "sdk_include_dir",
        "imu_topic",
        "left_topic",
        "right_topic",
        "left_frame_id",
        "right_frame_id",
        "imu_frame_id",
    ):
        values[key] = str(values[key])
    if not values["left_topic"] or not values["right_topic"] or not values["imu_topic"]:
        raise ValueError("camera and IMU topic names must not be empty")
    if values["left_topic"] == values["right_topic"]:
        raise ValueError("left and right camera topics must be different")
    if mode == "camera-imu" and values["imu_topic"] in {
        values["left_topic"],
        values["right_topic"],
    }:
        raise ValueError("IMU and camera topics must be different")

    return CaptureConfig(
        mode=mode,
        output=output,
        width=width,
        height=height,
        **values,
    )
