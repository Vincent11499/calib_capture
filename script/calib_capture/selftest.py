from __future__ import annotations

import io
import tempfile
import unittest
from pathlib import Path
from typing import Any

from .axera import (
    AX_FORMAT_BGR888,
    AX_FORMAT_RGB888,
    AX_FORMAT_YUV420_PLANAR,
    AX_FORMAT_YUV420_PLANAR_VU,
    AX_FORMAT_YUV420_SEMIPLANAR,
    AX_FORMAT_YUV420_SEMIPLANAR_VU,
    AX_FORMAT_YUV400,
    AX_IMG_INFO_BASE_SIZE,
    AX_VIDEO_FRAME_BASE_SIZE,
    AxImgInfo,
    AxVideoFrame,
    GrayPair,
    assert_ctypes_abi,
    inspect_sdk_headers,
    packed_frame_to_gray,
)
from .bmp import read_mono8_bmp, write_mono8_bmp
from .capture import FrameDivider, RateLimiter
from .config import build_capture_config
from .dataset import DatasetWriter, ImuRecord
from .validator import validate_dataset


def _check(condition: bool, message: str) -> None:
    if not condition:
        raise RuntimeError(message)


def run_self_test(sdk_include_dir: str = "") -> dict[str, Any]:
    tests: list[str] = []
    assert_ctypes_abi()
    _check(AX_VIDEO_FRAME_BASE_SIZE == 232, "unexpected AX frame base size")
    _check(AxVideoFrame.u64PTS.offset == 192, "unexpected AX PTS offset")
    _check(AxVideoFrame.u64SeqNum.offset == 200, "unexpected AX sequence offset")
    _check(AX_IMG_INFO_BASE_SIZE == 576, "unexpected AX_IMG_INFO_T size")
    _check(AxImgInfo.tIspInfo.offset == 240, "unexpected AX ISP info offset")
    tests.append("ctypes_abi")

    header_contract = inspect_sdk_headers(sdk_include_dir) if sdk_include_dir else None
    if header_contract is not None:
        _check(
            bool(header_contract["contract_match"]),
            "SDK header contract failed: " + "; ".join(header_contract["errors"]),
        )
        tests.append("sdk_header_contract")

    with tempfile.TemporaryDirectory(prefix="calibration_capture_selftest_") as raw_temp:
        temp = Path(raw_temp)
        pixels = bytes(range(15))
        bmp_path = temp / "small.bmp"
        digest = write_mono8_bmp(bmp_path, 5, 3, pixels)
        info, decoded = read_mono8_bmp(bmp_path)
        _check((info.width, info.height) == (5, 3), "BMP dimensions changed")
        _check(decoded == pixels, "BMP pixels did not round-trip")
        _check(len(digest) == 64, "BMP SHA-256 length is invalid")
        tests.append("mono8_bmp_roundtrip")

        yuv = bytes((1, 2, 3, 9, 4, 5, 6, 9))
        for image_format in (
            AX_FORMAT_YUV400,
            AX_FORMAT_YUV420_PLANAR,
            AX_FORMAT_YUV420_PLANAR_VU,
            AX_FORMAT_YUV420_SEMIPLANAR,
            AX_FORMAT_YUV420_SEMIPLANAR_VU,
        ):
            _check(
                packed_frame_to_gray(yuv, 3, 2, 4, image_format)
                == bytes((1, 2, 3, 4, 5, 6)),
                f"Y-plane conversion failed for format {image_format}",
            )
        red_blue = bytes((255, 0, 0, 0, 0, 255))
        _check(
            packed_frame_to_gray(red_blue, 2, 1, 6, AX_FORMAT_RGB888)
            == bytes((29, 77)),
            "RGB888 grayscale conversion failed",
        )
        _check(
            packed_frame_to_gray(red_blue, 2, 1, 6, AX_FORMAT_BGR888)
            == bytes((77, 29)),
            "BGR888 grayscale conversion failed",
        )
        tests.append("pixel_format_conversion")

        limiter = RateLimiter(10.0)
        _check(
            [
                limiter.accept(value)
                for value in (0, 20_000_000, 100_000_000, 199_000_000)
            ]
            == [True, False, True, False],
            "PTS rate limiter behavior changed",
        )
        tests.append("pts_rate_limiter")

        divider = FrameDivider(3)
        _check(
            [divider.accept(value, value) for value in range(100, 107)]
            == [True, False, False, True, False, False, True],
            "source frame divider behavior changed",
        )
        gap_divider = FrameDivider(3)
        _check(
            [gap_divider.accept(value, value) for value in (200, 203, 206)]
            == [True, True, True],
            "source sequence gaps were not included in frame division",
        )
        tests.append("source_frame_divider")

        output = temp / "dataset"
        config = build_capture_config(
            mode="camera-imu",
            output=str(output),
            overrides={"resolution": "1920x1080", "save_rate_hz": 10.0},
        )
        # Use a compact test frame while retaining the same data contract.
        object.__setattr__(config, "width", 5)
        object.__setattr__(config, "height", 3)
        dataset = DatasetWriter(output, config, include_imu=True)
        for index in range(2):
            timestamp = 1_000_000_000 + index * 100_000_000
            pair = GrayPair(
                left_timestamp_ns=timestamp,
                right_timestamp_ns=timestamp + 1_000,
                left_sequence=index + 1,
                right_sequence=index + 1,
                width=5,
                height=3,
                left_pixels=pixels,
                right_pixels=pixels,
                left_format=AX_FORMAT_YUV420_SEMIPLANAR,
                right_format=AX_FORMAT_YUV420_SEMIPLANAR,
            )
            dataset.write_pair(pair)
        for index in range(12):
            timestamp = 1_000_000_000 + index * 10_000_000
            dataset.write_imu(
                ImuRecord(
                    timestamp_ns=timestamp,
                    frame_id="imu0",
                    orientation=(0.0, 0.0, 0.0, 1.0),
                    orientation_covariance=(0.0,) * 9,
                    angular_velocity=(0.1, 0.2, 0.3),
                    angular_velocity_covariance=(0.0,) * 9,
                    linear_acceleration=(1.0, 2.0, 3.0),
                    linear_acceleration_covariance=(0.0,) * 9,
                    arrival_monotonic_ns=timestamp,
                )
            )
        session = dataset.seal_payload(
            {
                "mode": "camera-imu",
                "capture_status": "completed",
                "capture_stats": {},
                "sdk_header_contract": header_contract,
            }
        )
        report = validate_dataset(dataset.staging_path, verify_checksums=True)
        _check(not report.errors, "dataset validation failed: " + "; ".join(report.errors))
        session["validation_status"] = report.status
        dataset.write_session(session)
        committed = dataset.commit(True)
        _check(committed == output, "dataset did not commit to the requested path")
        tests.append("directory_dataset_roundtrip")

    unit_test_summary: dict[str, Any] | None = None
    tests_directory = Path(__file__).resolve().parents[1] / "tests"
    if tests_directory.is_dir():
        stream = io.StringIO()
        suite = unittest.defaultTestLoader.discover(str(tests_directory))
        result = unittest.TextTestRunner(stream=stream, verbosity=1).run(suite)
        unit_test_summary = {
            "run": result.testsRun,
            "failures": len(result.failures),
            "errors": len(result.errors),
            "skipped": len(result.skipped),
        }
        if not result.wasSuccessful():
            raise RuntimeError("no-hardware unit tests failed:\n" + stream.getvalue())
        tests.append("no_hardware_unittest_suite")

    return {
        "status": "pass",
        "tests": tests,
        "ctypes": {
            "base_size": AX_VIDEO_FRAME_BASE_SIZE,
            "pts_offset": AxVideoFrame.u64PTS.offset,
            "sequence_offset": AxVideoFrame.u64SeqNum.offset,
            "img_info_base_size": AX_IMG_INFO_BASE_SIZE,
            "img_info_isp_offset": AxImgInfo.tIspInfo.offset,
        },
        "sdk_header_contract": header_contract,
        "unit_tests": unit_test_summary,
    }
