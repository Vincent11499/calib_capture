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
    AX_FORMAT_YUV400,
    AX_VIDEO_FRAME_BASE_SIZE,
    AxIvpsApi,
    AxVideoFrame,
    GrayFrame,
    IvpsGrabber,
)
from calib_capture.base_link import BASE_LINK_POSES
from calib_capture.cli import _capture_config, build_parser
from calib_capture.config import USB_IVPS_CHN, USB_IVPS_GRP, USB_RESOLUTION, build_capture_config
from calib_capture.dataset import DatasetWriter
from calib_capture.validator import validate_dataset
from calib_capture.usb_capture import record_usb_baselink_triggered


def make_frame(timestamp_us: int = 100, sequence: int = 1) -> AxVideoFrame:
    frame = AxVideoFrame()
    frame.u32Width = 8
    frame.u32Height = 4
    frame.enImgFormat = AX_FORMAT_YUV400
    frame.u32PicStride[0] = 8
    frame.u32FrameSize = 32
    frame.u64PhyAddr[0] = 1
    frame.u64PTS = timestamp_us
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
    def test_ivps_binding_uses_guarded_frame_and_exact_release(self) -> None:
        class Module:
            def __init__(self) -> None:
                self.gets: list[tuple[int, int, int]] = []
                self.releases: list[tuple[int, int, int]] = []

            def AX_IVPS_GetChnFrame(self, grp, chn, pointer, timeout_ms):
                frame = ctypes.cast(pointer, ctypes.POINTER(AxVideoFrame)).contents
                frame.u32Width = 1920
                frame.u32Height = 1080
                frame.enImgFormat = AX_FORMAT_YUV400
                frame.u32PicStride[0] = 1920
                frame.u32FrameSize = 1920 * 1080
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
        self.assertEqual(config.save_rate_hz, 8.0)
        self.assertEqual(config.frame_divisor, 0)
        self.assertEqual((USB_IVPS_GRP, USB_IVPS_CHN), (3, 0))
        base = _capture_config(
            parser.parse_args(["usb-baselink-triggered"]), "usb-baselink-triggered"
        )
        self.assertEqual(base.poses, len(BASE_LINK_POSES))
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
        self.assertEqual(api.unmaps, 1)
        self.assertEqual(api.releases, [(3, 0, 1)])
        with self.assertRaisesRegex(RuntimeError, "released twice"):
            api.release_frame(3, 0, frame)

    def test_usb_dataset_is_single_camera_and_validates(self) -> None:
        with tempfile.TemporaryDirectory() as raw:
            output = Path(raw) / "usb"
            config = build_capture_config(mode="usb-continuous", output=str(output))
            writer = DatasetWriter(output, config, include_imu=False)
            pixels = bytes(config.width * config.height)
            writer.write_frame(GrayFrame(
                timestamp_ns=1_000_000_000,
                sequence=7,
                width=config.width,
                height=config.height,
                pixels=pixels,
                source_format=AX_FORMAT_YUV400,
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
                pixels=bytes(config.width * config.height),
                source_format=AX_FORMAT_YUV400,
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
                return {"timestamp_us": frame.u64PTS, "sequence": frame.u64SeqNum}

            def copy_gray_frame(self, frame, **kwargs):
                frame._ivps_owned = False
                return GrayFrame(
                    timestamp_ns=frame.u64PTS * 1000,
                    sequence=frame.u64SeqNum,
                    width=1920,
                    height=1080,
                    pixels=b"",
                    source_format=AX_FORMAT_YUV400,
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
