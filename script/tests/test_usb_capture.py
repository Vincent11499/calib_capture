from __future__ import annotations

import contextlib
import ctypes
import io
import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

from calib_capture.axera import (
    AX_FORMAT_BGR888,
    AX_FORMAT_RGB888,
    AX_FORMAT_YUV400,
    AX_FORMAT_YUV420_SEMIPLANAR,
    AX_FORMAT_YUV420_SEMIPLANAR_VU,
    AX_VIDEO_FRAME_BASE_SIZE,
    AxIvpsApi,
    AxVideoFrame,
    GrayFrame,
    IvpsGetFrameError,
    IvpsGrabber,
    copy_packed_frame_to_gray,
    copy_packed_frame_to_rgb,
    copy_y_plane_to_rgb,
    copy_yuv420_semiplanar_to_rgb,
)
from calib_capture.base_link import BASE_LINK_POSES
from calib_capture.bmp import inspect_rgb8_bmp, read_rgb8_bmp, write_rgb8_bmp
from calib_capture.capture import RateLimiter, UserAbort, _CaptureInterrupts
from calib_capture.cli import _capture_config, build_parser
from calib_capture.config import (
    USB_IVPS_CHN,
    USB_IVPS_GRP,
    USB_RESOLUTION,
    USB_SOURCE_RATE_HZ,
    build_capture_config,
)
from calib_capture.dataset import DatasetWriter
from calib_capture.image_io import read_rgb8_image
from calib_capture.pipeline import FrameDiskWriter
from calib_capture.validator import validate_dataset
from calib_capture.usb_capture import _grab_frame_or_stop, record_usb_baselink_triggered


def make_frame(timestamp_ns: int = 100, sequence: int = 1) -> AxVideoFrame:
    frame = AxVideoFrame()
    frame.u32Width = 8
    frame.u32Height = 4
    frame.enImgFormat = AX_FORMAT_YUV400
    frame.u32PicStride[0] = 8
    frame.u32FrameSize = 32
    frame.u64PhyAddr[0] = 1
    frame.u64PTS = timestamp_ns
    frame.u64SeqNum = sequence
    return frame


class FakeIvpsApi:
    def __init__(self, frames: list[AxVideoFrame]) -> None:
        self.frames = frames
        self.releases: list[tuple[int, int, int]] = []
        self.buffer = ctypes.create_string_buffer(bytes(range(32)))
        self.unmaps = 0

    def get_frame(self, grp, chn, timeout_ms, destination):
        source = self.frames.pop(0)
        ctypes.memmove(
            ctypes.addressof(destination), ctypes.addressof(source), AX_VIDEO_FRAME_BASE_SIZE
        )
        destination._ivps_owned = True
        return 0

    def release_frame(self, grp, chn, frame):
        if not getattr(frame, "_ivps_owned", False):
            raise RuntimeError("released twice")
        frame._ivps_owned = False
        self.releases.append((grp, chn, int(frame.u64SeqNum)))
        return 0

    def map_frame(self, frame):
        return ctypes.addressof(self.buffer), 32, True

    def unmap_frame(self, pointer, map_size, mapped_here):
        self.unmaps += 1


class UsbCaptureTests(unittest.TestCase):
    def test_ivps_get_error_preserves_sdk_diagnostics(self) -> None:
        result = ctypes.c_int32(0x800D0104).value
        error = IvpsGetFrameError(3, 0, result)
        self.assertIsInstance(error, RuntimeError)
        self.assertEqual((error.grp, error.chn, error.result), (3, 0, result))
        self.assertEqual(error.result_hex, "0x800d0104")
        self.assertIn("grp=3, chn=0", str(error))

    def test_ivps_get_error_without_sigint_remains_fatal(self) -> None:
        result = ctypes.c_int32(0x800D0104).value

        class FailingGrabber:
            def grab_frame(self):
                raise IvpsGetFrameError(3, 0, result)

        interrupts = _CaptureInterrupts()
        with self.assertRaises(IvpsGetFrameError):
            _grab_frame_or_stop(FailingGrabber(), interrupts, {})

    def test_first_sigint_turns_blocking_ivps_get_error_into_clean_stop(self) -> None:
        result = ctypes.c_int32(0x800D0104).value

        class FailingGrabber:
            def grab_frame(self):
                raise IvpsGetFrameError(3, 0, result)

        interrupts = _CaptureInterrupts()
        interrupts.count = 1
        stats: dict[str, object] = {}
        self.assertIsNone(_grab_frame_or_stop(FailingGrabber(), interrupts, stats))
        self.assertEqual(
            stats["operator_stop_get_error"],
            {
                "call": "AX_IVPS_GetChnFrame",
                "grp": 3,
                "chn": 0,
                "result": result,
                "result_hex": "0x800d0104",
            },
        )

    def test_second_sigint_still_aborts_ivps_get_cleanup(self) -> None:
        result = ctypes.c_int32(0x800D0104).value

        class FailingGrabber:
            def grab_frame(self):
                raise IvpsGetFrameError(3, 0, result)

        interrupts = _CaptureInterrupts()
        interrupts.count = 2
        with self.assertRaises(UserAbort):
            _grab_frame_or_stop(FailingGrabber(), interrupts, {})

    def test_ivps_binding_uses_guarded_frame_and_exact_release(self) -> None:
        class Module:
            def __init__(self) -> None:
                self.gets: list[tuple[int, int, int]] = []
                self.releases: list[tuple[int, int, int]] = []

            def AX_IVPS_GetChnFrame(self, grp, chn, pointer, timeout_ms):
                frame = ctypes.cast(pointer, ctypes.POINTER(AxVideoFrame)).contents
                frame.u32Width = USB_RESOLUTION[0]
                frame.u32Height = USB_RESOLUTION[1]
                frame.enImgFormat = AX_FORMAT_YUV400
                frame.u32PicStride[0] = USB_RESOLUTION[0]
                frame.u32FrameSize = USB_RESOLUTION[0] * USB_RESOLUTION[1]
                frame.u64PhyAddr[0] = 1
                frame.u64PTS = 123
                frame.u64SeqNum = 9
                self.gets.append((grp, chn, timeout_ms))
                return 0

            def AX_IVPS_ReleaseChnFrame(self, grp, chn, pointer):
                frame = ctypes.cast(pointer, ctypes.POINTER(AxVideoFrame)).contents
                self.releases.append((grp, chn, int(frame.u64SeqNum)))
                return 0

        module = Module()
        api = AxIvpsApi.__new__(AxIvpsApi)
        api._ivps = module
        api.ax_errors = []
        frame = AxVideoFrame()
        self.assertEqual(api.get_frame(3, 0, 200, frame), 0)
        self.assertEqual(module.gets, [(3, 0, 200)])
        self.assertEqual(api.release_frame(3, 0, frame), 0)
        self.assertEqual(module.releases, [(3, 0, 9)])
        with self.assertRaisesRegex(RuntimeError, "already released"):
            api.release_frame(3, 0, frame)

    def test_cli_names_fixed_source_resolution_and_rate_alias(self) -> None:
        parser = build_parser()
        args = parser.parse_args(["usb-continuous", "--rate-hz", "8"])
        config = _capture_config(args, args.command)
        self.assertEqual(config.mode, "usb-continuous")
        self.assertEqual(config.resolution, USB_RESOLUTION)
        self.assertEqual(config.expected_source_rate_hz, USB_SOURCE_RATE_HZ)
        self.assertEqual(config.save_rate_hz, 8.0)
        self.assertIsNone(config.duration_s)
        self.assertEqual((config.pixel_format, config.image_format), ("rgb", "bmp"))
        self.assertEqual(config.frame_divisor, 0)
        self.assertEqual((USB_IVPS_GRP, USB_IVPS_CHN), (3, 0))
        base = _capture_config(
            parser.parse_args(["usb-baselink-triggered"]), "usb-baselink-triggered"
        )
        self.assertEqual(base.poses, len(BASE_LINK_POSES))
        self.assertEqual(base.dataset_name, "usb_base_link")
        self.assertTrue(Path(base.output).name.startswith("usb_base_link_"))
        mono = _capture_config(
            parser.parse_args(["usb-continuous", "--pixel-format", "mono8"]),
            "usb-continuous",
        )
        self.assertEqual(mono.pixel_format, "mono8")
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            parser.parse_args(["usb-continuous", "--image-format", "jpeg"])
        with contextlib.redirect_stderr(io.StringIO()):
            for old_name in ("camera-imu", "stereo_base_link_triggered"):
                with self.subTest(old_name=old_name), self.assertRaises(SystemExit):
                    parser.parse_args([old_name])
            with self.assertRaises(SystemExit):
                parser.parse_args(["usb-continuous", "--resolution", "3840x2160"])

    def test_ivps_grabber_copies_then_unmaps_and_releases_once(self) -> None:
        api = FakeIvpsApi([make_frame()])
        grabber = IvpsGrabber(
            api, grp=3, chn=0, timeout_ms=200, expected_resolution=(8, 4)
        )
        frame = grabber.grab_frame()
        gray = grabber.copy_gray_frame(frame, release_after_copy=True)
        self.assertEqual(gray.pixels, bytes(range(32)))
        self.assertEqual(gray.timestamp_ns, 100)
        self.assertEqual(api.unmaps, 1)
        self.assertEqual(api.releases, [(3, 0, 1)])
        with self.assertRaisesRegex(RuntimeError, "released twice"):
            api.release_frame(3, 0, frame)

    def test_mapped_packed_color_converts_directly_into_reusable_buffer(self) -> None:
        # Two RGB pixels per row plus two padding bytes. AX_FORMAT_BGR888 is
        # documented as R,G,B byte order in memory.
        raw = bytes((255, 0, 0, 0, 255, 0, 9, 9,
                     0, 0, 255, 255, 255, 255, 8, 8))
        mapped = ctypes.create_string_buffer(raw)
        destination = bytearray(4)
        copy_packed_frame_to_gray(
            ctypes.addressof(mapped), 2, 2, 8, AX_FORMAT_BGR888, destination
        )
        self.assertEqual(destination, bytes((77, 149, 29, 255)))
        copy_packed_frame_to_gray(
            ctypes.addressof(mapped), 2, 2, 8, AX_FORMAT_BGR888, destination,
            bottom_up=True,
        )
        self.assertEqual(destination, bytes((29, 255, 77, 149)))

        # AX_FORMAT_RGB888 uses B,G,R memory order; swapping red/blue bytes
        # must produce the same luminance values.
        swapped = bytes((0, 0, 255, 0, 255, 0, 9, 9,
                         255, 0, 0, 255, 255, 255, 8, 8))
        mapped_swapped = ctypes.create_string_buffer(swapped)
        copy_packed_frame_to_gray(
            ctypes.addressof(mapped_swapped), 2, 2, 8, AX_FORMAT_RGB888, destination
        )
        self.assertEqual(destination, bytes((77, 149, 29, 255)))

    def test_mapped_packed_color_copies_compact_rgb_with_stride(self) -> None:
        rgb = bytes((255, 0, 0, 0, 255, 0, 9, 9,
                     0, 0, 255, 255, 255, 255, 8, 8))
        expected = bytes((255, 0, 0, 0, 255, 0,
                          0, 0, 255, 255, 255, 255))
        mapped = ctypes.create_string_buffer(rgb)
        destination = bytearray(12)
        copy_packed_frame_to_rgb(
            ctypes.addressof(mapped), 2, 2, 8, AX_FORMAT_BGR888, destination
        )
        self.assertEqual(destination, expected)

        bgr = bytes((0, 0, 255, 0, 255, 0, 9, 9,
                     255, 0, 0, 255, 255, 255, 8, 8))
        mapped_bgr = ctypes.create_string_buffer(bgr)
        copy_packed_frame_to_rgb(
            ctypes.addressof(mapped_bgr), 2, 2, 8, AX_FORMAT_RGB888, destination
        )
        self.assertEqual(destination, expected)
        with self.assertRaisesRegex(ValueError, "requires an AX RGB888/BGR888"):
            copy_packed_frame_to_rgb(
                ctypes.addressof(mapped), 2, 2, 8, AX_FORMAT_YUV400, destination
            )

    def test_nv12_and_nv21_convert_to_compact_rgb(self) -> None:
        # BT.601 limited-range sample representing red. Include row padding to
        # prove that Y and interleaved chroma strides are handled separately.
        y_rows = bytes((82, 82, 9, 9, 82, 82, 8, 8))
        expected = bytes((255, 1, 0)) * 4
        for image_format, chroma in (
            (AX_FORMAT_YUV420_SEMIPLANAR, bytes((90, 240, 7, 7))),
            (AX_FORMAT_YUV420_SEMIPLANAR_VU, bytes((240, 90, 7, 7))),
        ):
            with self.subTest(image_format=image_format):
                mapped = ctypes.create_string_buffer(y_rows + chroma)
                destination = bytearray(12)
                with mock.patch("calib_capture.axera._load_libyuv", return_value=None):
                    backend = copy_yuv420_semiplanar_to_rgb(
                        ctypes.addressof(mapped),
                        12,
                        2,
                        2,
                        4,
                        4,
                        image_format,
                        destination,
                    )
                self.assertEqual(backend, "numpy_or_python")
                self.assertEqual(destination, expected)

    def test_nv12_rgb_conversion_rejects_truncated_frame(self) -> None:
        mapped = ctypes.create_string_buffer(bytes(12))
        with self.assertRaisesRegex(ValueError, "truncated"):
            copy_yuv420_semiplanar_to_rgb(
                ctypes.addressof(mapped),
                11,
                2,
                2,
                4,
                4,
                AX_FORMAT_YUV420_SEMIPLANAR,
                bytearray(12),
            )

    def test_yuv400_expands_to_equal_rgb_channels(self) -> None:
        mapped = ctypes.create_string_buffer(bytes((0, 64, 9, 9, 128, 255, 8, 8)))
        destination = bytearray(12)
        copy_y_plane_to_rgb(ctypes.addressof(mapped), 2, 2, 4, destination)
        self.assertEqual(
            destination,
            bytes((0, 0, 0, 64, 64, 64, 128, 128, 128, 255, 255, 255)),
        )

    def test_usb_nanosecond_pts_drives_ten_hz_limiter_on_thirty_hz_source(self) -> None:
        limiter = RateLimiter(10.0)
        timestamps_ns = [1_000_000_000 + index * 33_333_333 for index in range(31)]
        accepted = [stamp for stamp in timestamps_ns if limiter.accept(stamp)]
        self.assertEqual(len(accepted), 10)
        self.assertEqual(accepted[0], timestamps_ns[0])
        self.assertGreaterEqual(min(b - a for a, b in zip(accepted, accepted[1:])), 99_999_999)

    def test_rgb_bmp_roundtrip_preserves_channels_and_top_down_layout(self) -> None:
        pixels = bytes((255, 0, 0, 0, 255, 0,
                        0, 0, 255, 255, 255, 255))
        with tempfile.TemporaryDirectory() as raw:
            path = Path(raw) / "rgb.bmp"
            write_rgb8_bmp(path, 2, 2, pixels)
            info = inspect_rgb8_bmp(path)
            self.assertEqual((info.bits_per_pixel, info.top_down), (24, True))
            self.assertEqual(read_rgb8_bmp(path)[1], pixels)

    def test_usb_frame_pool_uses_three_bytes_for_rgb_and_one_for_mono8(self) -> None:
        with tempfile.TemporaryDirectory() as raw:
            for index, pixel_format in enumerate(("rgb", "mono8")):
                with self.subTest(pixel_format=pixel_format):
                    config = build_capture_config(
                        mode="usb-continuous",
                        output=str(Path(raw) / f"usb{index}"),
                        overrides={"pixel_format": pixel_format},
                    )
                    dataset = DatasetWriter(config.output, config, include_imu=False)
                    writer = FrameDiskWriter(dataset, 2)
                    expected = config.width * config.height * (3 if pixel_format == "rgb" else 1)
                    self.assertEqual(writer.pool.image_bytes, expected)
                    writer.finish()
                    dataset.abandon(RuntimeError("test cleanup"))

    def test_usb_dataset_is_single_camera_and_validates(self) -> None:
        with tempfile.TemporaryDirectory() as raw:
            output = Path(raw) / "usb"
            config = build_capture_config(mode="usb-continuous", output=str(output))
            writer = DatasetWriter(output, config, include_imu=False)
            pixels = bytes(config.width * config.height * 3)
            writer.write_frame(GrayFrame(
                timestamp_ns=1_000_000_000,
                sequence=7,
                width=config.width,
                height=config.height,
                pixels=pixels,
                source_format=AX_FORMAT_BGR888,
                encoding="rgb8",
            ))
            writer.seal_payload({
                "mode": "usb-continuous",
                "capture_status": "completed",
                "capture_source": {
                    "api": "AX_IVPS_GetChnFrame", "grp": 3, "chn": 0,
                },
                "capture_stats": {
                    "source_sequence_gaps": 0,
                    "source_sequence_regressions": 0,
                    "rate_limiter_skipped_frames": 0,
                    "writer_queue_drop_frames": 0,
                },
                "sdk_header_contract": {
                    "path": "", "present": False, "contract_match": False,
                    "hashes": {}, "errors": ["SDK include directory was not found"],
                },
                "ax_return_errors": [],
                "ax_deinit_result": 0,
            })
            report = validate_dataset(writer.staging_path)
            self.assertFalse(report.errors, report.errors)
            self.assertEqual(report.stats["cam0_count"], 1)
            self.assertEqual(report.stats["cam1_count"], 0)
            self.assertFalse((writer.staging_path / "cameras/cam1").exists())
            self.assertFalse((writer.staging_path / "meta/pairs.csv").exists())
            manifest = json.loads(
                (writer.staging_path / "dataset.yaml").read_text(encoding="utf-8")
            )
            self.assertEqual([camera["id"] for camera in manifest["cameras"]], ["cam0"])
            self.assertEqual(manifest["cameras"][0]["clock_source"], "axera_ivps_pts_ns")
            capture = (writer.staging_path / "meta/capture_dataset.yaml").read_text(
                encoding="utf-8"
            )
            self.assertIn("source_pts_unit: ns\n", capture)
            self.assertIn("pixel_format: rgb\n", capture)
            self.assertIn("    encoding: rgb8\n", capture)
            image = next((writer.staging_path / "cameras/cam0").glob("*.bmp"))
            info, decoded = read_rgb8_image(image)
            self.assertEqual((info.width, info.height), config.resolution)
            self.assertEqual(decoded, pixels)

    def test_usb_baselink_pose_filename_is_preserved(self) -> None:
        with tempfile.TemporaryDirectory() as raw:
            output = Path(raw) / "usb_pose"
            config = build_capture_config(mode="usb-baselink-triggered", output=str(output))
            writer = DatasetWriter(output, config, include_imu=False)
            writer.write_frame(GrayFrame(
                timestamp_ns=2_000_000_000,
                sequence=1,
                width=config.width,
                height=config.height,
                pixels=bytes(config.width * config.height * 3),
                source_format=AX_FORMAT_BGR888,
                encoding="rgb8",
            ), filename_stem=BASE_LINK_POSES[0])
            writer._close_streams()
            rows = (writer.staging_path / "cameras/cam0/timestamps.csv").read_text(
                encoding="utf-8"
            )
            self.assertIn(f"{BASE_LINK_POSES[0]}.bmp", rows)

    def test_usb_baselink_command_uses_all_fixed_poses(self) -> None:
        class FakeDataset:
            def __init__(self, *args, **kwargs):
                self.frame_count = 0
                self.names: list[str] = []

            def write_frame(self, frame, *, filename_stem=None):
                self.frame_count += 1
                self.names.append(filename_stem)

        class FakeApi:
            library_dir = "/soc/lib"
            header_contract = {"present": False}
            ax_errors: list[dict] = []

            def close(self):
                return 0

        class FakeGrabber:
            def __init__(self):
                self.index = 0
                self.stats = {
                    "source_sequence_gaps": 0,
                    "source_sequence_regressions": 0,
                }
                self.timings = SimpleNamespace(summary=lambda: {})

            def grab_frame(self):
                self.index += 1
                return SimpleNamespace(_ivps_owned=True, u64PTS=self.index, u64SeqNum=self.index)

            def describe_frame(self, frame):
                return {
                    "timestamp_ns": frame.u64PTS,
                    "pts_unit": "ns",
                    "sequence": frame.u64SeqNum,
                }

            def copy_gray_frame(self, frame, **kwargs):
                frame._ivps_owned = False
                return GrayFrame(
                    timestamp_ns=frame.u64PTS,
                    sequence=frame.u64SeqNum,
                    width=USB_RESOLUTION[0],
                    height=USB_RESOLUTION[1],
                    pixels=b"",
                    source_format=AX_FORMAT_YUV400,
                )

            def copy_rgb_frame(self, frame, **kwargs):
                frame._ivps_owned = False
                return GrayFrame(
                    timestamp_ns=frame.u64PTS,
                    sequence=frame.u64SeqNum,
                    width=USB_RESOLUTION[0],
                    height=USB_RESOLUTION[1],
                    pixels=b"",
                    source_format=AX_FORMAT_BGR888,
                    encoding="rgb8",
                )

            def release_frame(self, frame):
                frame._ivps_owned = False

        created = FakeDataset()
        prompts: list[str] = []
        config = build_capture_config(
            mode="usb-baselink-triggered", output="unused"
        )
        with (
            mock.patch("calib_capture.usb_capture.DatasetWriter", return_value=created),
            mock.patch("calib_capture.usb_capture.AxIvpsApi", return_value=FakeApi()),
            mock.patch("calib_capture.usb_capture._make_grabber", return_value=FakeGrabber()),
            mock.patch(
                "calib_capture.usb_capture._finish_dataset",
                return_value=(Path("done"), SimpleNamespace(errors=[])),
            ),
        ):
            with contextlib.redirect_stdout(io.StringIO()):
                record_usb_baselink_triggered(
                    config, input_function=lambda prompt: prompts.append(prompt) or ""
                )
        self.assertEqual(created.names, list(BASE_LINK_POSES))
        self.assertEqual(len(prompts), len(BASE_LINK_POSES))
        for pose, prompt in zip(BASE_LINK_POSES, prompts):
            self.assertIn(pose, prompt)


if __name__ == "__main__":
    unittest.main()
