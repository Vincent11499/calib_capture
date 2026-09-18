import ctypes
import json
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from calib_capture.axera import AxVideoFrame
from calib_capture.cli import build_parser, _capture_config
from calib_capture.config import build_capture_config
from calib_capture.raw_capture import RawApi, RawGrabber, record_raw_triggered


class FakeRaw:
    header_contract = {}
    library_dir = "fake"
    ax_errors = []

    def __init__(self):
        self.counts = {2: 0, 0: 0}
        self.releases = []
        self.buffer = ctypes.create_string_buffer(bytes(range(48)))
        self.map_count = 0
        self.unmap_count = 0

    def get_frame(self, pipe, chn, timeout, frame):
        self.counts[pipe] += 1
        frame.u32Width, frame.u32Height = 8, 4
        frame.enImgFormat = 0x82
        frame.u32PicStride[0] = 12
        frame.u32FrameSize = 48
        frame.u64PTS = self.counts[pipe] * 20000 + 1
        frame.u64SeqNum = self.counts[pipe]
        frame.u64PhyAddr[0] = 123
        return 0

    def release_frame(self, pipe, chn, frame):
        identity = (pipe, frame.u64SeqNum)
        assert identity not in self.releases
        self.releases.append(identity)
        return 0

    def map_frame(self, frame):
        self.map_count += 1
        return ctypes.addressof(self.buffer), 48, True

    def unmap_frame(self, *args):
        self.unmap_count += 1

    def close(self):
        return 0


def grabber(api):
    return RawGrabber(api, left_pipe=2, left_chn=0, right_pipe=0, right_chn=0,
                      timeout_ms=200, tolerance_us=200, max_retries=2,
                      expected_resolution=(8, 4))


class RawTests(unittest.TestCase):
    def test_raw16_raw12_and_lossless_accepted(self):
        for fmt, mode in ((0x84, 0), (0x84, 1), (0x86, 0), (0x82, 1)):
            with self.subTest(fmt=fmt, mode=mode):
                api = FakeRaw()
                original = api.get_frame
                def invalid(*args):
                    original(*args)
                    args[-1].enImgFormat = fmt
                    args[-1].stCompressInfo.enCompressMode = mode
                    return 0
                api.get_frame = invalid
                capture = grabber(api)
                pair = capture.grab_frames()
                try:
                    self.assertEqual(capture.copy_raw(pair.left), bytes(range(48)))
                finally:
                    capture.release_frames(pair)
                self.assertEqual(len(api.releases), 2)

    def test_lossy_unknown_and_invalid_compressed_buffers_rejected(self):
        for field, value in (("mode", 2), ("mode", 3), ("size", 0),
                             ("size", 256 * 1024**2 + 1), ("header", 1), ("ext", 1)):
            with self.subTest(field=field, value=value):
                api = FakeRaw()
                original = api.get_frame
                def invalid(*args):
                    original(*args)
                    frame = args[-1]
                    frame.stCompressInfo.enCompressMode = value if field == "mode" else 0
                    if field == "size": frame.u32FrameSize = value
                    if field == "header": frame.u32HeaderSize[0] = value
                    if field == "ext": frame.u64ExtPhyAddr[0] = value
                    return 0
                api.get_frame = invalid
                with self.assertRaises(RuntimeError):
                    grabber(api).grab_frames()
                self.assertEqual(len(api.releases), 2)

    def test_cli(self):
        args = build_parser().parse_args(["stereo-raw-triggered", "--poses", "3"])
        config = _capture_config(args, args.command)
        self.assertEqual(config.dataset_name, "stereo_raw")
        self.assertEqual(config.poses, 3)
        self.assertEqual(config.sync_tolerance_us, 200)

    def test_binding_exact_owner_and_fixed_parameters(self):
        api = RawApi.__new__(RawApi)
        owners = []
        def get(pipe, node, exposure, pointer, timeout):
            self.assertEqual((pipe, node, exposure, timeout), (2, 0, 0, 200))
            owners.append(ctypes.addressof(pointer._obj))
            return 0
        def release(pipe, node, exposure, pointer):
            self.assertEqual((pipe, node, exposure), (2, 0, 0))
            self.assertEqual(ctypes.addressof(pointer._obj), owners[0])
            return 0
        api._vin = SimpleNamespace(AX_VIN_GetRawFrame=get, AX_VIN_ReleaseRawFrame=release)
        frame = AxVideoFrame()
        api.get_frame(2, 0, 200, frame)
        api.release_frame(2, 0, frame)
        with self.assertRaises(RuntimeError):
            api.release_frame(2, 0, frame)

    def test_copy_preserves_packing_and_padding(self):
        api = FakeRaw()
        capture = grabber(api)
        pair = capture.grab_frames()
        try:
            self.assertEqual(capture.copy_raw(pair.left), bytes(range(48)))
        finally:
            capture.release_frames(pair)
        self.assertEqual((api.map_count, api.unmap_count, len(api.releases)), (1, 1, 2))

    def test_invalid_format_releases_both(self):
        api = FakeRaw()
        original = api.get_frame
        def invalid(*args):
            original(*args)
            args[-1].enImgFormat = 3
            return 0
        api.get_frame = invalid
        with self.assertRaises(RuntimeError):
            grabber(api).grab_frames()
        self.assertEqual(len(api.releases), 2)

    def test_failed_right_get_releases_left(self):
        api = FakeRaw()
        original = api.get_frame
        api.get_frame = lambda *args: -1 if args[0] == 0 else original(*args)
        with self.assertRaises(RuntimeError):
            grabber(api).grab_frames()
        self.assertEqual(api.releases, [(2, 1)])

    def test_sync_reacquires_older_side(self):
        api = FakeRaw()
        api.counts[0] = 1
        capture = grabber(api)
        pair = capture.grab_frames()
        capture.release_frames(pair)
        self.assertEqual(api.releases, [(2, 1), (2, 2), (0, 2)])

    def test_session_and_abort(self):
        for abort in (False, True):
            with self.subTest(abort=abort), tempfile.TemporaryDirectory() as directory:
                config = replace(build_capture_config(mode="stereo-raw-triggered"),
                                 output=str(Path(directory) / "raw"), width=8, height=4, poses=2)
                api = FakeRaw()
                # Deterministic clock skips prompt queue draining in this unit test.
                with patch("calib_capture.raw_capture.time.monotonic", side_effect=range(100)):
                    if abort:
                        with self.assertRaises(RuntimeError):
                            record_raw_triggered(config, input_function=lambda _: "q", api_factory=lambda *a: api)
                    else:
                        record_raw_triggered(config, input_function=lambda _: "", api_factory=lambda *a: api)
                root = Path(config.output + (".failed" if abort else ""))
                session = json.loads((root / "meta/session.json").read_text())
                self.assertEqual(session["saved_pairs"], 0 if abort else 2)
                self.assertFalse((root / "dataset.yaml").exists())
                if not abort:
                    for raw in root.glob("cameras/*/*.raw"):
                        self.assertEqual(raw.read_bytes(), bytes(range(48)))
                    self.assertEqual(len(api.releases), 4)

    def test_copy_failure_unmaps_and_releases(self):
        api = FakeRaw()
        capture = grabber(api)
        frames = capture.grab_frames()
        with patch("calib_capture.raw_capture.ctypes.string_at", side_effect=MemoryError):
            with self.assertRaises(MemoryError):
                try:
                    capture.copy_raw(frames.left)
                finally:
                    capture.release_frames(frames)
        self.assertEqual(api.unmap_count, 1)
        self.assertEqual(len(api.releases), 2)

    def test_write_failure_keeps_failed_and_released_frames(self):
        with tempfile.TemporaryDirectory() as directory:
            config = replace(build_capture_config(mode="stereo-raw-triggered"),
                             output=str(Path(directory) / "raw"), width=8, height=4, poses=1)
            api = FakeRaw()
            with patch("calib_capture.raw_capture.time.monotonic", side_effect=range(100)), \
                    patch.object(Path, "write_bytes", side_effect=OSError("disk full")):
                with self.assertRaisesRegex(RuntimeError, "disk full"):
                    record_raw_triggered(config, input_function=lambda _: "", api_factory=lambda *a: api)
            self.assertTrue(Path(config.output + ".failed/meta/session.json").is_file())
            self.assertEqual(len(api.releases), 2)

    def test_interrupt_retains_failed(self):
        with tempfile.TemporaryDirectory() as directory:
            config = replace(build_capture_config(mode="stereo-raw-triggered"),
                             output=str(Path(directory) / "raw"))
            def interrupt(_):
                raise KeyboardInterrupt
            with self.assertRaises(RuntimeError):
                record_raw_triggered(config, input_function=interrupt, api_factory=lambda *a: FakeRaw())
            self.assertTrue(Path(config.output + ".failed/meta/session.json").is_file())


if __name__ == "__main__":
    unittest.main()
