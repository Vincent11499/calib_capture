from __future__ import annotations

import ctypes
import contextlib
import importlib.util
import io
import json
import tempfile
import threading
import unittest
from datetime import datetime
from pathlib import Path
from unittest import mock

from calib_capture.axera import (
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
    AxVinApi,
    AxVideoFrame,
    GrayPair,
    StereoGrabber,
    _load_ax_libraries,
    packed_frame_to_gray,
    validate_frame_metadata,
)
from calib_capture.bmp import BMP_PIXEL_OFFSET, read_mono8_bmp, write_mono8_bmp
from calib_capture.capture import FrameDivider, ImuSubscriber, PairDiskWriter, RateLimiter
from calib_capture.cli import _capture_config, build_parser
from calib_capture.config import (
    DEFAULTS,
    build_capture_config,
    load_ini_mapping,
    session_output_path,
)
from calib_capture.dataset import DatasetWriter, ImuRecord
from calib_capture.ros1_export import export_ros1_bag
from calib_capture.validator import validate_dataset


def make_frame(timestamp_us: int, sequence: int, width: int = 8, height: int = 4) -> AxVideoFrame:
    frame = AxVideoFrame()
    frame.u32Width = width
    frame.u32Height = height
    frame.enImgFormat = AX_FORMAT_YUV420_SEMIPLANAR
    frame.u32PicStride[0] = width
    frame.u32FrameSize = width * height
    frame.u64PhyAddr[0] = 1
    frame.u64PTS = timestamp_us
    frame.u64SeqNum = sequence
    return frame


def make_pair(index: int, width: int = 5, height: int = 3) -> GrayPair:
    timestamp = 1_000_000_000 + index * 100_000_000
    pixels = bytes((value + index) % 256 for value in range(width * height))
    return GrayPair(
        left_timestamp_ns=timestamp,
        right_timestamp_ns=timestamp + 1_000,
        left_sequence=index + 1,
        right_sequence=index + 1,
        width=width,
        height=height,
        left_pixels=pixels,
        right_pixels=pixels,
        left_format=AX_FORMAT_YUV420_SEMIPLANAR,
        right_format=AX_FORMAT_YUV420_SEMIPLANAR,
    )


class FakeVinApi:
    def __init__(self, queues: dict[int, list[AxVideoFrame | int]]) -> None:
        self.queues = queues
        self.releases: list[tuple[int, int]] = []

    def get_frame(self, pipe: int, chn: int, timeout_ms: int, destination: AxVideoFrame) -> int:
        item = self.queues[pipe].pop(0)
        if isinstance(item, int):
            return item
        ctypes.memmove(
            ctypes.addressof(destination), ctypes.addressof(item), AX_VIDEO_FRAME_BASE_SIZE
        )
        return 0

    def release_frame(self, pipe: int, chn: int, frame: AxVideoFrame) -> int:
        self.releases.append((pipe, int(frame.u64SeqNum)))
        return 0


def make_grabber(api: FakeVinApi, tolerance_us: int = 5) -> StereoGrabber:
    return StereoGrabber(
        api,  # type: ignore[arg-type]
        left_pipe=2,
        left_chn=0,
        right_pipe=0,
        right_chn=0,
        timeout_ms=200,
        tolerance_us=tolerance_us,
        max_retries=10,
        expected_resolution=(8, 4),
    )


def create_dataset(root: Path, mode: str = "camera-imu") -> Path:
    output = root / "dataset"
    config = build_capture_config(
        mode=mode,
        output=str(output),
        overrides={"resolution": "1920x1080", "save_rate_hz": 10.0},
    )
    object.__setattr__(config, "width", 5)
    object.__setattr__(config, "height", 3)
    writer = DatasetWriter(output, config, include_imu=mode == "camera-imu")
    writer.write_pair(make_pair(0))
    writer.write_pair(make_pair(1))
    if mode == "camera-imu":
        for index in range(12):
            timestamp = 1_000_000_000 + index * 10_000_000
            writer.write_imu(
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
    session = writer.seal_payload(
        {"mode": mode, "capture_status": "completed", "capture_stats": {}}
    )
    report = validate_dataset(writer.staging_path)
    session["validation_status"] = report.status
    writer.write_session(session)
    return writer.commit(not report.errors)


class BmpTests(unittest.TestCase):
    def test_roundtrip_with_row_padding(self) -> None:
        with tempfile.TemporaryDirectory() as raw:
            path = Path(raw) / "image.bmp"
            pixels = bytes(range(21))
            write_mono8_bmp(path, 7, 3, pixels)
            info, decoded = read_mono8_bmp(path)
            self.assertEqual((info.width, info.height, info.row_stride), (7, 3, 8))
            self.assertEqual(decoded, pixels)

    def test_large_block_write_preserves_bottom_up_bytes_without_hashing(self) -> None:
        with tempfile.TemporaryDirectory() as raw:
            for width in (4, 7, 1920, 3840):
                with self.subTest(width=width):
                    path = Path(raw) / f"{width}.bmp"
                    height = 3
                    pixels = bytes(value % 256 for value in range(width * height))
                    with mock.patch("hashlib.sha256", side_effect=AssertionError("unexpected hash")):
                        self.assertIsNone(write_mono8_bmp(path, width, height, pixels))
                    padding = bytes((-width) % 4)
                    expected = b"".join(
                        pixels[row * width:(row + 1) * width] + padding
                        for row in range(height - 1, -1, -1)
                    )
                    self.assertEqual(path.read_bytes()[BMP_PIXEL_OFFSET:], expected)
                    self.assertEqual(read_mono8_bmp(path)[1], pixels)

    def test_all_supported_formats_convert_to_gray(self) -> None:
        y_plane = bytes((1, 2, 3, 99, 4, 5, 6, 99))
        for image_format in (
            AX_FORMAT_YUV400,
            AX_FORMAT_YUV420_PLANAR,
            AX_FORMAT_YUV420_PLANAR_VU,
            AX_FORMAT_YUV420_SEMIPLANAR,
            AX_FORMAT_YUV420_SEMIPLANAR_VU,
        ):
            with self.subTest(image_format=image_format):
                self.assertEqual(
                    packed_frame_to_gray(y_plane, 3, 2, 4, image_format),
                    bytes((1, 2, 3, 4, 5, 6)),
                )
        self.assertEqual(
            packed_frame_to_gray(
                bytes((255, 0, 0, 0, 0, 255)), 2, 1, 6, AX_FORMAT_RGB888
            ),
            bytes((29, 77)),
        )
        self.assertEqual(
            packed_frame_to_gray(
                bytes((255, 0, 0, 0, 0, 255)), 2, 1, 6, AX_FORMAT_BGR888
            ),
            bytes((77, 29)),
        )

    def test_frame_metadata_rejects_short_mapping(self) -> None:
        frame = make_frame(100, 1)
        frame.u32FrameSize = 31
        with self.assertRaisesRegex(RuntimeError, "smaller than its first plane"):
            validate_frame_metadata(frame, (8, 4))


class AxVinAbiTests(unittest.TestCase):
    def test_vin_dependencies_load_before_proton(self) -> None:
        with tempfile.TemporaryDirectory() as raw:
            library_dir = Path(raw)
            names = (
                "libax_sys.so",
                "libax_interpreter.so",
                "libax_engine.so",
                "libax_proton.so",
            )
            for name in names:
                (library_dir / name).touch()
            with mock.patch("calib_capture.axera.ctypes.CDLL") as load:
                load.side_effect = lambda name, mode=0: name
                sys_library, dependencies, vin_library = _load_ax_libraries(
                    library_dir
                )
            self.assertEqual(sys_library, str(library_dir / names[0]))
            self.assertEqual(
                dependencies,
                [str(library_dir / names[1]), str(library_dir / names[2])],
            )
            self.assertEqual(vin_library, str(library_dir / names[3]))
            self.assertEqual(
                [Path(call.args[0]).name for call in load.call_args_list],
                list(names),
            )

    def test_img_info_layout_and_exact_owner_release(self) -> None:
        self.assertEqual(AX_VIDEO_FRAME_BASE_SIZE, 232)
        self.assertEqual(AX_IMG_INFO_BASE_SIZE, 576)
        self.assertEqual(AxImgInfo.tIspInfo.offset, 240)

        class FakeVinModule:
            def __init__(self) -> None:
                self.owner_address: int | None = None
                self.release_calls: list[tuple[int, int, int]] = []

            def AX_VIN_GetYuvFrame(
                self, pipe: int, chn: int, pointer: object, timeout_ms: int
            ) -> int:
                info = ctypes.cast(pointer, ctypes.POINTER(AxImgInfo)).contents
                self.owner_address = ctypes.addressof(info)
                video = info.tFrameInfo.stVFrame
                video.u32Width = 3840
                video.u32Height = 2160
                video.enImgFormat = AX_FORMAT_YUV420_SEMIPLANAR
                video.u32PicStride[0] = 3840
                video.u64PhyAddr[0] = 1234
                video.u64PTS = 987654
                video.u64SeqNum = 42
                video.u32FrameSize = 3840 * 2160 * 3 // 2
                return 0

            def AX_VIN_ReleaseYuvFrame(
                self, pipe: int, chn: int, pointer: object
            ) -> int:
                info = ctypes.cast(pointer, ctypes.POINTER(AxImgInfo)).contents
                self.release_calls.append((pipe, chn, ctypes.addressof(info)))
                return 0

        module = FakeVinModule()
        api = AxVinApi.__new__(AxVinApi)
        api._vin = module
        api.ax_errors = []
        frame = AxVideoFrame()
        self.assertEqual(api.get_frame(2, 0, 200, frame), 0)
        self.assertEqual((frame.u32Width, frame.u32Height), (3840, 2160))
        self.assertEqual(frame.u64SeqNum, 42)
        self.assertEqual(api.release_frame(2, 0, frame), 0)
        self.assertEqual(module.release_calls, [(2, 0, module.owner_address)])
        with self.assertRaisesRegex(RuntimeError, "already released"):
            api.release_frame(2, 0, frame)

    def test_img_info_tail_overwrite_is_released_and_rejected(self) -> None:
        class CorruptingVinModule:
            def __init__(self) -> None:
                self.release_count = 0

            def AX_VIN_GetYuvFrame(
                self, pipe: int, chn: int, pointer: object, timeout_ms: int
            ) -> int:
                info = ctypes.cast(pointer, ctypes.POINTER(AxImgInfo)).contents
                info.tFrameInfo.stVFrame.u64SeqNum = 7
                info._abi_tail_guard[0] = 0
                return 0

            def AX_VIN_ReleaseYuvFrame(
                self, pipe: int, chn: int, pointer: object
            ) -> int:
                self.release_count += 1
                return 0

        module = CorruptingVinModule()
        api = AxVinApi.__new__(AxVinApi)
        api._vin = module
        api.ax_errors = []
        with self.assertRaisesRegex(RuntimeError, "beyond the pinned 576-byte"):
            api.get_frame(2, 0, 200, AxVideoFrame())
        self.assertEqual(module.release_count, 1)


class ConfigTests(unittest.TestCase):
    def test_default_cali_mapping_is_full_resolution_vin(self) -> None:
        config = build_capture_config(mode="probe")
        self.assertEqual((config.left_pipe, config.left_chn), (2, 0))
        self.assertEqual((config.right_pipe, config.right_chn), (0, 0))
        self.assertEqual(config.resolution, (3840, 2160))
        self.assertEqual(config.frame_divisor, 0)
        self.assertEqual(config.target_save_rate_hz, 5.0)

    def test_continuous_cli_defaults_to_five_hz_stereo(self) -> None:
        for command in ("stereo-continuous", "camera-imu"):
            with self.subTest(command=command):
                args = build_parser().parse_args([command])
                config = _capture_config(args, command)
                self.assertEqual(config.save_rate_hz, 5.0)
                self.assertEqual(config.target_save_rate_hz, 5.0)
                self.assertEqual(config.frame_divisor, 0)
                self.assertEqual(config.imu_expected_rate_hz, 100.0)

    def test_default_ini_matches_builtin_defaults(self) -> None:
        path = Path(__file__).resolve().parents[1] / "default_config.ini"
        self.assertEqual(set(load_ini_mapping(path)), set(DEFAULTS))
        self.assertEqual(
            build_capture_config(mode="camera-imu", config_path=str(path)),
            build_capture_config(mode="camera-imu"),
        )

    def test_ini_accepts_comments_and_utf8_bom(self) -> None:
        document = (
            "; stereo settings\n"
            "[capture]\n"
            "# full resolution VIN\n"
            "left_pipe = 4\n"
            "left_frame_id = 左目\n"
        )
        with tempfile.TemporaryDirectory() as raw:
            path = Path(raw) / "capture.ini"
            for encoding in ("utf-8", "utf-8-sig"):
                with self.subTest(encoding=encoding):
                    path.write_text(document, encoding=encoding)
                    self.assertEqual(
                        load_ini_mapping(path),
                        {"left_pipe": "4", "left_frame_id": "左目"},
                    )

    def test_ini_preserves_literal_values_without_interpolation(self) -> None:
        value = "left%100/${camera} # still literal;also literal"
        with tempfile.TemporaryDirectory() as raw:
            path = Path(raw) / "capture.ini"
            path.write_text(
                f"[capture]\nleft_frame_id = {value}\n", encoding="utf-8"
            )
            self.assertEqual(load_ini_mapping(path), {"left_frame_id": value})
            config = build_capture_config(mode="probe", config_path=str(path))
            self.assertEqual(config.left_frame_id, value)

    def test_ini_empty_capture_and_no_config_use_defaults(self) -> None:
        self.assertEqual(load_ini_mapping(None), {})
        with tempfile.TemporaryDirectory() as raw:
            path = Path(raw) / "capture.ini"
            for document in ("[capture]\n", "[DEFAULT]\n[capture]\n"):
                with self.subTest(document=document):
                    path.write_text(document, encoding="utf-8")
                    self.assertEqual(
                        build_capture_config(mode="probe", config_path=str(path)),
                        build_capture_config(mode="probe"),
                    )

    def test_ini_rejects_missing_capture_and_malformed_content(self) -> None:
        documents = (
            "",
            "; comments only\n",
            "left_pipe = 4\n",
            "[capture\nleft_pipe = 4\n",
            "[capture]\nleft_pipe\n",
            "[capture]\nleft_pipe: 4\n",
            "[capture]\nleft_frame_id = left\n  continued\n",
            "[DEFAULT]\n",
            "[Capture]\nleft_pipe = 4\n",
        )
        with tempfile.TemporaryDirectory() as raw:
            path = Path(raw) / "capture.ini"
            for document in documents:
                with self.subTest(document=document):
                    path.write_text(document, encoding="utf-8")
                    with self.assertRaises(ValueError):
                        load_ini_mapping(path)

    def test_ini_rejects_unknown_or_wrong_case_keys(self) -> None:
        with tempfile.TemporaryDirectory() as raw:
            path = Path(raw) / "capture.ini"
            for key in ("save_rate", "Left_pipe", "LEFT_PIPE"):
                with self.subTest(key=key):
                    path.write_text(f"[capture]\n{key} = 4\n", encoding="utf-8")
                    with self.assertRaisesRegex(ValueError, "unknown configuration"):
                        load_ini_mapping(path)

    def test_ini_rejects_extra_sections_and_default_inheritance(self) -> None:
        documents = (
            "[capture]\n[hardware_layout]\n",
            "[capture]\n[other]\nleft_pipe = 4\n",
            "[DEFAULT]\nleft_pipe = 4\n[capture]\n",
        )
        with tempfile.TemporaryDirectory() as raw:
            path = Path(raw) / "capture.ini"
            for document in documents:
                with self.subTest(document=document):
                    path.write_text(document, encoding="utf-8")
                    with self.assertRaises(ValueError):
                        load_ini_mapping(path)

    def test_ini_rejects_duplicate_keys_and_sections(self) -> None:
        documents = (
            "[capture]\nleft_pipe = 2\nleft_pipe = 4\n",
            "[capture]\n[capture]\n",
        )
        with tempfile.TemporaryDirectory() as raw:
            path = Path(raw) / "capture.ini"
            for document in documents:
                with self.subTest(document=document):
                    path.write_text(document, encoding="utf-8")
                    with self.assertRaises(ValueError):
                        load_ini_mapping(path)

    def test_ini_numeric_values_are_validated_after_loading(self) -> None:
        with tempfile.TemporaryDirectory() as raw:
            path = Path(raw) / "capture.ini"
            for value in ("0", "-1", "NaN", "Infinity", "-Infinity", "fast"):
                with self.subTest(value=value):
                    path.write_text(
                        f"[capture]\nsave_rate_hz = {value}\n", encoding="utf-8"
                    )
                    self.assertEqual(load_ini_mapping(path), {"save_rate_hz": value})
                    with self.assertRaises(ValueError):
                        build_capture_config(mode="camera-imu", config_path=str(path))

    def test_old_config_extensions_explain_ini_migration(self) -> None:
        with tempfile.TemporaryDirectory() as raw:
            for suffix in (".yaml", ".yml", ".json", ".YAML", ".JSON"):
                with self.subTest(suffix=suffix):
                    path = Path(raw) / ("capture" + suffix)
                    path.write_text("[capture]\nleft_pipe = 4\n", encoding="utf-8")
                    with self.assertRaisesRegex(ValueError, "INI"):
                        load_ini_mapping(path)

    def test_cli_overrides_ini_and_ini_overrides_defaults(self) -> None:
        with tempfile.TemporaryDirectory() as raw:
            path = Path(raw) / "capture.ini"
            path.write_text(
                "[capture]\n"
                "left_pipe = 4\n"
                "resolution = 1920x1080\n"
                "save_rate_hz = 7.5\n",
                encoding="utf-8",
            )
            config = build_capture_config(
                mode="stereo-continuous",
                config_path=str(path),
                overrides={"left_pipe": 5},
            )
            self.assertEqual(config.left_pipe, 5)
            self.assertEqual(config.resolution, (1920, 1080))
            self.assertEqual(config.save_rate_hz, 7.5)
            self.assertEqual(config.frame_divisor, 0)
            self.assertAlmostEqual(config.target_save_rate_hz, 7.5)
            self.assertEqual(config.right_pipe, 0)

    def test_negative_frame_divisor_is_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "frame_divisor"):
            build_capture_config(
                mode="stereo-continuous", overrides={"frame_divisor": -1}
            )

    def test_rate_controls_are_rejected_together_in_one_config_layer(self) -> None:
        with self.assertRaisesRegex(ValueError, "mutually exclusive"):
            build_capture_config(
                mode="camera-imu",
                overrides={"save_rate_hz": 7.5, "frame_divisor": 5},
            )

    def test_cli_rate_strategy_overrides_ini(self) -> None:
        cases = (
            ("frame_divisor = 3", ["--save-rate-hz", "7"], 0, 7.0),
            ("save_rate_hz = 7", ["--frame-divisor", "5"], 5, 10.0),
        )
        with tempfile.TemporaryDirectory() as raw:
            path = Path(raw) / "capture.ini"
            for values, arguments, divisor, rate_hz in cases:
                with self.subTest(values=values, arguments=arguments):
                    path.write_text(f"[capture]\n{values}\n", encoding="utf-8")
                    args = build_parser().parse_args(
                        ["--config", str(path), "stereo-continuous", *arguments]
                    )
                    config = _capture_config(args, "stereo-continuous")
                    self.assertEqual(config.frame_divisor, divisor)
                    self.assertEqual(config.target_save_rate_hz, rate_hz)

    def test_ini_rate_controls_are_rejected_together(self) -> None:
        with tempfile.TemporaryDirectory() as raw:
            path = Path(raw) / "capture.ini"
            path.write_text(
                "[capture]\nsave_rate_hz = 7.5\nframe_divisor = 3\n",
                encoding="utf-8",
            )
            with self.assertRaisesRegex(ValueError, "mutually exclusive"):
                build_capture_config(mode="camera-imu", config_path=str(path))

    def test_session_output_uses_timestamp_subdirectory(self) -> None:
        path = session_output_path(
            "./data", now=datetime(2026, 9, 7, 15, 4, 5)
        )
        self.assertEqual(Path(path), Path("data") / "2026-09-07_15-04-05")

    def test_recording_cli_defaults_to_data_root(self) -> None:
        for command in ("stereo-triggered", "stereo-continuous", "camera-imu"):
            with self.subTest(command=command):
                args = build_parser().parse_args([command])
                self.assertEqual(args.output, "./data")
                config = _capture_config(args, command)
                output = Path(config.output)
                self.assertEqual(output.parent, Path("data"))
                datetime.strptime(output.name, "%Y-%m-%d_%H-%M-%S")

    def test_continuous_rate_controls_are_mutually_exclusive(self) -> None:
        parser = build_parser()
        with contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit):
                parser.parse_args(
                    [
                        "stereo-continuous",
                        "--save-rate-hz",
                        "10",
                        "--frame-divisor",
                        "5",
                    ]
                )

    def test_probe_samples_are_not_recording_rate_controls(self) -> None:
        parser = build_parser()
        probe = parser.parse_args(["probe", "--samples", "75"])
        self.assertEqual(probe.samples, 75)
        with contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit):
                parser.parse_args(["probe", "--save-rate-hz", "10"])

    def test_same_left_and_right_mapping_is_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "must be different"):
            build_capture_config(
                mode="probe",
                overrides={
                    "left_pipe": 1,
                    "left_chn": 0,
                    "right_pipe": 1,
                    "right_chn": 0,
                },
            )


class StereoGrabberTests(unittest.TestCase):
    def test_resynchronizes_by_replacing_older_frame(self) -> None:
        api = FakeVinApi(
            {
                2: [make_frame(100, 1), make_frame(200, 2)],
                0: [make_frame(200, 1)],
            }
        )
        grabber = make_grabber(api)
        frames = grabber.grab_frames()
        self.assertEqual(frames.retries, 1)
        self.assertEqual(int(frames.left.u64SeqNum), 2)
        self.assertEqual(api.releases, [(2, 1)])
        grabber.release_frames(frames)
        self.assertEqual(api.releases, [(2, 1), (2, 2), (0, 1)])
        with self.assertRaises(RuntimeError):
            grabber.release_frames(frames)

    def test_right_get_failure_releases_left_exactly_once(self) -> None:
        api = FakeVinApi({2: [make_frame(100, 1)], 0: [-1]})
        with self.assertRaises(RuntimeError):
            make_grabber(api).grab_frames()
        self.assertEqual(api.releases, [(2, 1)])

    def test_metadata_failure_releases_both_frames_exactly_once(self) -> None:
        left = make_frame(100, 1)
        right = make_frame(100, 2)
        right.u32Width = 7
        api = FakeVinApi({2: [left], 0: [right]})
        with self.assertRaisesRegex(RuntimeError, "resolution"):
            make_grabber(api).grab_frames()
        self.assertEqual(api.releases, [(2, 1), (0, 2)])

    def test_non_nv12_source_is_rejected_and_released(self) -> None:
        left = make_frame(100, 1)
        right = make_frame(100, 2)
        right.enImgFormat = AX_FORMAT_YUV400
        api = FakeVinApi({2: [left], 0: [right]})
        with self.assertRaisesRegex(RuntimeError, "expected NV12"):
            make_grabber(api).grab_frames()
        self.assertEqual(api.releases, [(2, 1), (0, 2)])

    def test_copy_unmaps_and_releases_before_returning_pixels(self) -> None:
        class MappedFakeVinApi(FakeVinApi):
            def __init__(self) -> None:
                super().__init__(
                    {2: [make_frame(100, 1)], 0: [make_frame(100, 2)]}
                )
                self.buffers = {
                    1: ctypes.create_string_buffer(bytes(range(32))),
                    2: ctypes.create_string_buffer(bytes(reversed(range(32)))),
                }
                self.unmaps: list[tuple[int, int, bool]] = []

            def map_frame(self, frame: AxVideoFrame) -> tuple[int, int, bool]:
                buffer = self.buffers[int(frame.u64SeqNum)]
                return ctypes.addressof(buffer), 32, True

            def unmap_frame(
                self, pointer: int, map_size: int, mapped_here: bool
            ) -> None:
                self.unmaps.append((pointer, map_size, mapped_here))

        api = MappedFakeVinApi()
        grabber = make_grabber(api)
        frames = grabber.grab_frames()
        pair = grabber.copy_gray_pair(frames, release_after_copy=True)
        self.assertTrue(frames.released)
        self.assertEqual(len(api.unmaps), 2)
        self.assertEqual(api.releases, [(2, 1), (0, 2)])
        self.assertEqual(pair.left_pixels, bytes(range(32)))
        self.assertEqual(pair.right_pixels, bytes(reversed(range(32))))


class QueueTests(unittest.TestCase):
    def test_full_queue_rejects_complete_pair(self) -> None:
        entered = threading.Event()
        release = threading.Event()

        class SlowDataset:
            def __init__(self) -> None:
                self.items: list[GrayPair] = []

            def write_pair(self, pair: GrayPair) -> None:
                entered.set()
                release.wait(2.0)
                self.items.append(pair)

        dataset = SlowDataset()
        writer = PairDiskWriter(dataset, capacity=1)  # type: ignore[arg-type]
        self.assertTrue(writer.submit(make_pair(0)))
        self.assertTrue(entered.wait(1.0))
        self.assertTrue(writer.submit(make_pair(1)))
        self.assertFalse(writer.submit(make_pair(2)))
        release.set()
        writer.finish()
        self.assertEqual(len(dataset.items), 2)

    def test_failed_writer_finish_is_idempotent(self) -> None:
        class BrokenDataset:
            def write_pair(self, pair: GrayPair) -> None:
                raise OSError("simulated disk failure")

        writer = PairDiskWriter(BrokenDataset(), capacity=1)  # type: ignore[arg-type]
        self.assertTrue(writer.submit(make_pair(0)))
        with self.assertRaisesRegex(RuntimeError, "BMP writer failed"):
            writer.finish()
        with self.assertRaisesRegex(RuntimeError, "BMP writer failed"):
            writer.finish()

    def test_rate_limiter_uses_source_timestamps(self) -> None:
        limiter = RateLimiter(10.0)
        values = [0, 50_000_000, 100_000_000, 150_000_000, 200_000_000]
        self.assertEqual([limiter.accept(value) for value in values], [True, False, True, False, True])

    def test_six_hz_selects_first_source_frame_at_each_exact_deadline(self) -> None:
        # 50 Hz input alternates 180/160/160 ms gaps without accumulating rounding.
        expected_ms = [0, 180, 340, 500, 680, 840, 1000]
        for origin_ns in (0, 1_789_000_000_000_000_001):
            with self.subTest(origin_ns=origin_ns):
                limiter = RateLimiter(6.0)
                selected_ms = [
                    elapsed_ms
                    for elapsed_ms in range(0, 1001, 20)
                    if limiter.accept(origin_ns + elapsed_ms * 1_000_000)
                ]
                self.assertEqual(selected_ms, expected_ms)

    def test_rate_limiter_skips_elapsed_deadlines_after_source_gap(self) -> None:
        limiter = RateLimiter(5.0)
        source_ms = [0, 20, 1050, 1070, 1090, 1199, 1200, 1220, 1400]
        selected_ms = [
            value for value in source_ms if limiter.accept(value * 1_000_000)
        ]
        self.assertEqual(selected_ms, [0, 1050, 1200, 1400])

    def test_rate_limiter_rejects_nonpositive_and_nonfinite_rates(self) -> None:
        for rate_hz in (0.0, -1.0, float("nan"), float("inf"), float("-inf")):
            with self.subTest(rate_hz=rate_hz), self.assertRaises(ValueError):
                RateLimiter(rate_hz)

    def test_frame_divider_accepts_one_pair_out_of_n(self) -> None:
        divider = FrameDivider(3)
        self.assertEqual(
            [divider.accept(value, value) for value in range(100, 108)],
            [True, False, False, True, False, False, True, False],
        )

    def test_frame_divider_uses_source_sequence_gaps(self) -> None:
        divider = FrameDivider(3)
        self.assertEqual(
            [divider.accept(value, value) for value in (100, 101, 104, 107)],
            [True, False, True, True],
        )

    def test_frame_divider_rejects_nonpositive_divisor(self) -> None:
        with self.assertRaises(ValueError):
            FrameDivider(0)


class ImuSubscriberTests(unittest.TestCase):
    def test_stop_reports_last_callback_failure(self) -> None:
        subscriber = ImuSubscriber(build_capture_config(mode="camera-imu"))
        subscriber.error = ValueError("late callback")
        with self.assertRaisesRegex(RuntimeError, "late callback"):
            subscriber.stop()
        self.assertTrue(subscriber.stop_event.is_set())


class DatasetTests(unittest.TestCase):
    def test_existing_failed_timestamp_directory_is_not_reused(self) -> None:
        with tempfile.TemporaryDirectory() as raw:
            root = Path(raw)
            output = root / "2026-09-07_15-30-00"
            output.with_name(output.name + ".failed").mkdir()
            config = build_capture_config(
                mode="stereo-continuous", output=str(output)
            )
            with self.assertRaises(FileExistsError):
                DatasetWriter(output, config, include_imu=False)

    def test_dataset_validates_without_payload_hashes(self) -> None:
        with tempfile.TemporaryDirectory() as raw:
            with mock.patch("hashlib.sha256", side_effect=AssertionError("unexpected hash")):
                dataset = create_dataset(Path(raw))
                report = validate_dataset(dataset)
            self.assertFalse(report.errors)
            self.assertFalse((dataset / "meta/checksums.sha256").exists())
            self.assertEqual(report.stats["data_integrity"], "structural")
            self.assertFalse(report.stats["checksums_verified"])
            self.assertEqual(report.stats["cam0_count"], 2)
            self.assertEqual(report.stats["imu_count"], 12)
            contract = (dataset / "dataset.yaml").read_text(encoding="utf-8")
            self.assertIn("schema_version: 2", contract)
            self.assertIn("data_integrity: structural", contract)
            self.assertIn("capture_api: AX_VIN_GetYuvFrame", contract)
            self.assertIn("vin: {pipe: 2, chn: 0}", contract)
            self.assertIn("vin: {pipe: 0, chn: 0}", contract)
            self.assertNotIn("ivps:", contract)
            performance = json.loads((dataset / "meta/session.json").read_text(encoding="utf-8"))["write_performance"]
            self.assertEqual(performance["pair_count"], 2)
            self.assertGreaterEqual(performance["max_pair_ms"], performance["mean_pair_ms"])

    def test_missing_one_camera_image_is_hard_failure(self) -> None:
        with tempfile.TemporaryDirectory() as raw:
            dataset = create_dataset(Path(raw))
            row = next(csv_row(dataset / "cameras/cam1/timestamps.csv"))
            (dataset / "cameras/cam1" / row["filename"]).unlink()
            report = validate_dataset(dataset)
            self.assertTrue(any("missing image" in error for error in report.errors))

    def test_structural_validation_accepts_same_size_pixel_change(self) -> None:
        with tempfile.TemporaryDirectory() as raw:
            dataset = create_dataset(Path(raw))
            row = next(csv_row(dataset / "cameras/cam0/timestamps.csv"))
            image = dataset / "cameras/cam0" / row["filename"]
            content = bytearray(image.read_bytes())
            content[BMP_PIXEL_OFFSET] ^= 0x01
            image.write_bytes(content)
            report = validate_dataset(dataset)
            self.assertFalse(report.errors)
            self.assertFalse(report.stats["checksums_verified"])

    def test_structural_validation_still_rejects_truncated_image(self) -> None:
        with tempfile.TemporaryDirectory() as raw:
            dataset = create_dataset(Path(raw))
            row = next(csv_row(dataset / "cameras/cam0/timestamps.csv"))
            image = dataset / "cameras/cam0" / row["filename"]
            image.write_bytes(image.read_bytes()[:-1])
            report = validate_dataset(dataset)
            self.assertTrue(any("invalid image" in error for error in report.errors))

    def test_legacy_dataset_is_readable_without_using_manifest(self) -> None:
        with tempfile.TemporaryDirectory() as raw:
            dataset = create_dataset(Path(raw))
            session_path = dataset / "meta/session.json"
            session = json.loads(session_path.read_text(encoding="utf-8"))
            session["schema_version"] = 1
            session.pop("data_integrity")
            session_path.write_text(json.dumps(session), encoding="utf-8")
            contract_path = dataset / "dataset.yaml"
            contract_path.write_text(
                contract_path.read_text(encoding="utf-8")
                .replace("schema_version: 2", "schema_version: 1")
                .replace("data_integrity: structural\n", ""),
                encoding="utf-8",
            )
            manifest = dataset / "meta/checksums.sha256"
            manifest.write_text("legacy manifest is ignored\n", encoding="utf-8")
            with mock.patch("hashlib.sha256", side_effect=AssertionError("unexpected hash")):
                self.assertFalse(validate_dataset(dataset).errors)
                manifest.unlink()
                self.assertFalse(validate_dataset(dataset).errors)

    def test_abandoned_dataset_is_failed_and_has_validation_report(self) -> None:
        with tempfile.TemporaryDirectory() as raw:
            root = Path(raw)
            output = root / "interrupted"
            config = build_capture_config(
                mode="stereo-triggered",
                output=str(output),
                overrides={"resolution": "1920x1080", "poses": 20},
            )
            object.__setattr__(config, "width", 5)
            object.__setattr__(config, "height", 3)
            writer = DatasetWriter(output, config, include_imu=False)
            writer.write_pair(make_pair(0))
            failed = writer.abandon(RuntimeError("simulated capture failure"))
            self.assertTrue(failed.name.endswith(".failed"))
            self.assertTrue((failed / "validation.json").is_file())
            report = json.loads((failed / "validation.json").read_text(encoding="utf-8"))
            self.assertEqual(report["status"], "fail")
            session = json.loads(
                (failed / "meta/session.json").read_text(encoding="utf-8")
            )
            self.assertEqual(session["capture_status"], "failed")
            self.assertEqual(session["validation_status"], "fail")


def csv_row(path: Path):
    import csv

    with path.open("r", encoding="utf-8", newline="") as stream:
        yield from csv.DictReader(stream)


@unittest.skipUnless(importlib.util.find_spec("rosbags"), "rosbags is not installed")
class RosbagExportTests(unittest.TestCase):
    def test_export_and_reopen_ros1_bag(self) -> None:
        with tempfile.TemporaryDirectory() as raw:
            root = Path(raw)
            dataset = create_dataset(root)
            for compression in ("none", "lz4", "bz2"):
                with self.subTest(compression=compression):
                    bag = root / f"calibration_{compression}.bag"
                    result = export_ros1_bag(dataset, bag, compression=compression)
                    self.assertEqual(result["status"], "pass")
                    self.assertTrue(bag.is_file())
                    self.assertEqual(result["counts"]["/stereo/left/image_raw"], 2)
                    self.assertEqual(result["counts"]["/imu/data"], 12)
                    self.assertEqual(
                        result["first_timestamp_ns"]["/stereo/left/image_raw"],
                        1_000_000_000,
                    )

    def test_stereo_only_bag_has_no_imu_connection(self) -> None:
        with tempfile.TemporaryDirectory() as raw:
            root = Path(raw)
            dataset = create_dataset(root, mode="stereo-continuous")
            bag = root / "stereo.bag"
            result = export_ros1_bag(dataset, bag)
            self.assertEqual(
                set(result["counts"]),
                {"/stereo/left/image_raw", "/stereo/right/image_raw"},
            )


if __name__ == "__main__":
    unittest.main()
