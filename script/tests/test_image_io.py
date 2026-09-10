from __future__ import annotations

import importlib.util
import json
import tempfile
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

from calib_capture.cli import _capture_config, build_parser
from calib_capture.config import build_capture_config
from calib_capture.dataset import DatasetWriter
from calib_capture.image_io import read_mono8_image, write_mono8_image
from calib_capture.ros1_export import ExportEvent, _make_image, export_ros1_bag
from calib_capture.validator import validate_dataset
from test_calibration_data import create_dataset, make_legacy_fixture, make_pair, rosbag_export_skip_reason


class ImageConfigTests(unittest.TestCase):
    def test_jpeg_cli_and_ini_priority(self):
        with tempfile.TemporaryDirectory() as raw:
            path = Path(raw) / "capture.ini"
            path.write_text("[capture]\nschema_version = 1.0.0\nimage_format = jpeg\njpeg_quality = 90\n", encoding="utf-8")
            args = build_parser().parse_args([
                "--config", str(path), "camera-imu", "--jpeg-quality", "95"])
            config = _capture_config(args, args.command)
            self.assertEqual((config.image_format, config.jpeg_quality), ("jpeg", 95))
            self.assertEqual(config.save_rate_hz, 5)
            self.assertEqual(config.imu_expected_rate_hz, 100)

    def test_bad_format_and_quality_rejected(self):
        for options in ({"image_format": "png"}, {"jpeg_quality": 0}, {"jpeg_quality": 101}):
            with self.subTest(options=options), self.assertRaises(ValueError):
                build_capture_config(mode="camera-imu", overrides=options)

    def test_missing_jpeg_codec_fails_before_creating_dataset(self):
        with tempfile.TemporaryDirectory() as raw:
            path = Path(raw) / "dataset"
            config = build_capture_config(mode="camera-imu", overrides={"image_format": "jpeg"})
            with mock.patch("calib_capture.image_io._jpeg_modules", side_effect=RuntimeError("missing OpenCV")):
                with self.assertRaisesRegex(RuntimeError, "OpenCV"):
                    DatasetWriter(path, config, True)
            self.assertEqual(list(Path(raw).iterdir()), [])

    def test_schema_two_bmp_without_new_config_keys_is_rejected(self):
        with tempfile.TemporaryDirectory() as raw:
            path = create_dataset(Path(raw))
            make_legacy_fixture(path, 2)
            session_path = path / "meta/session.json"
            session = json.loads(session_path.read_text())
            session["schema_version"] = 2
            for key in ("image_format", "jpeg_quality"):
                session["config"].pop(key)
            session_path.write_text(json.dumps(session), encoding="utf-8")
            yaml = path / "dataset.yaml"
            text = yaml.read_text().replace("schema_version: 3", "schema_version: 2")
            yaml.write_text("\n".join(line for line in text.splitlines()
                                      if not line.startswith(("image_format:", "jpeg_quality:", "lossy:"))) + "\n")
            self.assertTrue(validate_dataset(path).errors)

    def test_jpeg_workers_overlap_before_pair_commit(self):
        with tempfile.TemporaryDirectory() as raw:
            config = build_capture_config(mode="stereo-continuous", overrides={"image_format": "jpeg"})
            object.__setattr__(config, "width", 5)
            object.__setattr__(config, "height", 3)
            barrier = threading.Barrier(2, timeout=3)
            def fake_encode(path, *args):
                barrier.wait()
                path.write_bytes(b"test")
            with mock.patch("calib_capture.dataset.ensure_image_support"), \
                 mock.patch("calib_capture.dataset.write_mono8_image", side_effect=fake_encode):
                writer = DatasetWriter(Path(raw)/"dataset", config, False)
                try:
                    writer.write_pair(make_pair(0))
                    self.assertEqual(writer.pair_count, 1)
                    self.assertEqual(len(list(writer.staging_path.rglob("*.jpg"))), 2)
                finally:
                    writer.seal_payload({})


@unittest.skipUnless(importlib.util.find_spec("cv2") and importlib.util.find_spec("numpy"), "OpenCV/NumPy are not installed")
class JpegTests(unittest.TestCase):
    def test_grayscale_roundtrip_and_quality(self):
        pixels = bytes((x * 3 + y * 5) % 256 for y in range(64) for x in range(64))
        with tempfile.TemporaryDirectory() as raw:
            sizes = []
            for quality in (30, 95):
                path = Path(raw) / f"{quality}.jpg"
                write_mono8_image(path, 64, 64, pixels, "jpeg", quality)
                info, decoded = read_mono8_image(path)
                self.assertEqual((info.width, info.height, len(decoded)), (64, 64, len(pixels)))
                sizes.append(path.stat().st_size)
                if quality == 95:
                    self.assertLess(sum(abs(a-b) for a,b in zip(pixels, decoded))/len(pixels), 4)
            self.assertLess(sizes[0], sizes[1])

    def test_dataset_validation_and_bag_image_decoding(self):
        with tempfile.TemporaryDirectory() as raw:
            path = create_dataset(Path(raw), image_format="jpeg")
            report = validate_dataset(path)
            self.assertFalse(report.errors, report.errors)
            self.assertEqual(report.stats["imu_count"], 12)
            self.assertTrue(any("lossy" in warning for warning in report.warnings))
            image_path = next((path / "cameras/cam0").glob("*.jpg"))
            pixels = read_mono8_image(image_path)[1]
            np = SimpleNamespace(uint8=object(), frombuffer=lambda data, dtype:
                                 SimpleNamespace(copy=lambda: data))
            event = ExportEvent(123, 0, "cam0", 0, image_path)
            with mock.patch("calib_capture.ros1_export._type", return_value=SimpleNamespace), \
                 mock.patch("calib_capture.ros1_export._make_header", return_value="header"):
                message = _make_image(None, np, event, "cam0")
            self.assertEqual(message.encoding, "mono8")
            self.assertEqual(message.data, pixels)
            self.assertEqual((message.width, message.height), (5, 3))

    def test_truncated_wrong_mode_and_wrong_extension_rejected(self):
        import cv2
        import numpy as np
        with tempfile.TemporaryDirectory() as raw:
            path = Path(raw) / "image.jpg"
            write_mono8_image(path, 8, 4, bytes(32), "jpeg")
            path.write_bytes(path.read_bytes()[:-2])
            with self.assertRaisesRegex(ValueError, "truncated"):
                read_mono8_image(path)
            self.assertTrue(cv2.imwrite(str(path), np.zeros((4, 8, 3), dtype=np.uint8)))
            with self.assertRaisesRegex(ValueError, "grayscale"):
                read_mono8_image(path)
            path = create_dataset(Path(raw), image_format="jpeg")
            extra = path / "cameras/cam0/999.bmp"
            write_mono8_image(extra, 8, 4, bytes(32), "bmp")
            self.assertTrue(any("unreferenced" in e for e in validate_dataset(path).errors))

    def test_second_image_failure_never_commits_a_half_pair(self):
        with tempfile.TemporaryDirectory() as raw:
            config = build_capture_config(mode="stereo-continuous", overrides={"image_format": "jpeg"})
            object.__setattr__(config, "width", 5)
            object.__setattr__(config, "height", 3)
            writer = DatasetWriter(Path(raw) / "dataset", config, False)
            def write_then_fail(path, *args):
                if "cam1" in path.parts:
                    raise OSError("simulated encode failure")
                write_mono8_image(path, *args)
            with mock.patch("calib_capture.dataset.write_mono8_image", side_effect=write_then_fail):
                with self.assertRaises(OSError):
                    writer.write_pair(make_pair(0))
            self.assertEqual(writer.pair_count, 0)
            self.assertFalse(list(writer.staging_path.rglob("*.jpg")))
            self.assertFalse(list(writer.staging_path.rglob("*.tmp")))
            writer.abandon(RuntimeError("test cleanup"))

    @unittest.skipIf(rosbag_export_skip_reason(), rosbag_export_skip_reason())
    def test_jpeg_ros1_export_and_reopen(self):
        with tempfile.TemporaryDirectory() as raw:
            root = Path(raw)
            path = create_dataset(root, image_format="jpeg")
            result = export_ros1_bag(path, root / "jpeg.bag")
            self.assertEqual(result["counts"]["/stereo/left/image_raw"], 2)
            self.assertEqual(result["counts"]["/imu/data"], 12)
