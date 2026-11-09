from __future__ import annotations

import contextlib
import io
import json
import tempfile
import threading
import unittest
from pathlib import Path
from unittest import mock

from calib_capture.cli import build_parser
from calib_capture.config import build_capture_config
from calib_capture.dataset import DatasetWriter
from calib_capture.image_io import read_mono8_image, write_mono8_image
from calib_capture.validator import validate_dataset
from test_calibration_data import create_dataset, make_legacy_fixture, make_pair


class ImageConfigTests(unittest.TestCase):
    def test_capture_is_bmp_only_and_cli_has_no_jpeg_options(self):
        self.assertEqual(build_capture_config(mode="stereo-imu").image_format, "bmp")
        for option in ("--image-format", "--jpeg-quality"):
            with self.subTest(option=option), contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit):
                    build_parser().parse_args(["stereo-imu", option, "jpeg"])

    def test_non_bmp_ini_and_removed_jpeg_key_are_rejected(self):
        with tempfile.TemporaryDirectory() as raw:
            for text, message in (
                ("[capture]\nschema_version = 1.0.0\nimage_format = jpeg\n", "must be bmp"),
                ("[capture]\nschema_version = 1.0.0\njpeg_quality = 95\n", "unknown configuration keys"),
            ):
                path = Path(raw) / "capture.ini"
                path.write_text(text, encoding="utf-8")
                with self.subTest(text=text), self.assertRaisesRegex(ValueError, message):
                    build_capture_config(mode="stereo-imu", config_path=str(path))

    def test_bmp_roundtrip_requires_no_cv2(self):
        pixels = bytes(range(32))
        with tempfile.TemporaryDirectory() as raw:
            path = Path(raw) / "image.bmp"
            write_mono8_image(path, 8, 4, pixels, "bmp")
            info, decoded = read_mono8_image(path)
            self.assertEqual((info.width, info.height), (8, 4))
            self.assertEqual(decoded, pixels)
            with self.assertRaisesRegex(ValueError, "only BMP"):
                read_mono8_image(Path(raw) / "image.jpg")

    def test_current_schema_requires_fixed_image_format(self):
        with tempfile.TemporaryDirectory() as raw:
            path = create_dataset(Path(raw))
            session_path = path / "meta/session.json"
            session = json.loads(session_path.read_text())
            session["config"].pop("image_format")
            session_path.write_text(json.dumps(session), encoding="utf-8")
            self.assertTrue(validate_dataset(path).errors)

    def test_old_schema_without_current_storage_contract_is_rejected(self):
        with tempfile.TemporaryDirectory() as raw:
            path = create_dataset(Path(raw))
            make_legacy_fixture(path, 2)
            session_path = path / "meta/session.json"
            session = json.loads(session_path.read_text())
            session["schema_version"] = 2
            session["config"].pop("image_format")
            session_path.write_text(json.dumps(session), encoding="utf-8")
            yaml = path / "dataset.yaml"
            text = yaml.read_text().replace("schema_version: 3", "schema_version: 2")
            yaml.write_text("\n".join(
                line for line in text.splitlines() if not line.startswith("image_format:")
            ) + "\n")
            self.assertTrue(validate_dataset(path).errors)

    def test_bmp_workers_overlap_before_pair_commit(self):
        with tempfile.TemporaryDirectory() as raw:
            config = build_capture_config(mode="stereo-continuous")
            object.__setattr__(config, "width", 5)
            object.__setattr__(config, "height", 3)
            barrier = threading.Barrier(2, timeout=3)

            def fake_write(path, *args):
                barrier.wait()
                path.write_bytes(b"test")

            with mock.patch("calib_capture.dataset.write_mono8_image", side_effect=fake_write):
                writer = DatasetWriter(Path(raw) / "dataset", config, False)
                try:
                    writer.write_pair(make_pair(0))
                    self.assertEqual(writer.pair_count, 1)
                    self.assertEqual(len(list(writer.staging_path.rglob("*.bmp"))), 2)
                finally:
                    writer.seal_payload({})

    def test_second_bmp_failure_never_commits_a_half_pair(self):
        with tempfile.TemporaryDirectory() as raw:
            config = build_capture_config(mode="stereo-continuous")
            object.__setattr__(config, "width", 5)
            object.__setattr__(config, "height", 3)
            writer = DatasetWriter(Path(raw) / "dataset", config, False)

            def write_then_fail(path, *args):
                if "cam1" in path.parts:
                    raise OSError("simulated write failure")
                write_mono8_image(path, *args)

            with mock.patch("calib_capture.dataset.write_mono8_image", side_effect=write_then_fail):
                with self.assertRaises(OSError):
                    writer.write_pair(make_pair(0))
            self.assertEqual(writer.pair_count, 0)
            self.assertFalse(list(writer.staging_path.rglob("*.bmp")))
            self.assertFalse(list(writer.staging_path.rglob("*.tmp")))
            writer.abandon(RuntimeError("test cleanup"))


if __name__ == "__main__":
    unittest.main()
