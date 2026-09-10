"""Optional integration with the real, externally provided Kalibr directory reader.

Use an existing kalibr_bag_io installation or add its source/dependency folders
to PYTHONPATH. No hardware, calibration run, downloads, or existing data needed.
"""

from __future__ import annotations

import json
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path

from calib_capture.axera import AX_FORMAT_YUV420_SEMIPLANAR, GrayPair
from calib_capture.config import build_capture_config
from calib_capture.dataset import DatasetWriter, ImuRecord
from calib_capture.image_io import ensure_image_support
from calib_capture.validator import validate_dataset

try:
    from kalibr_bag_io.directory import DirectoryReader
except ImportError as error:
    DirectoryReader = None
    READER_UNAVAILABLE = f"optional Kalibr directory reader unavailable: {error}"
else:
    READER_UNAVAILABLE = ""


@unittest.skipIf(DirectoryReader is None, READER_UNAVAILABLE)
class KalibrReaderIntegrationTest(unittest.TestCase):
    def _dataset(self, parent: Path, image_format: str, *, include_imu: bool):
        output = parent / "capture"
        config = replace(
            build_capture_config(
                mode="camera-imu" if include_imu else "stereo-continuous",
                output=str(output),
                overrides={
                    "image_format": image_format,
                    "jpeg_quality": 95,
                    "imu_preroll_s": 0,
                    "imu_postroll_s": 0,
                    "left_topic": "/fixture/left",
                    "right_topic": "/fixture/right",
                    "imu_topic": "/fixture/imu",
                },
            ),
            width=8,
            height=4,
        )
        first_ns = 1_234_567_890_123_000
        pairs = [
            GrayPair(
                left_timestamp_ns=first_ns + index * 200_000_000,
                right_timestamp_ns=first_ns + index * 200_000_000 + 5_000,
                left_sequence=index * 10 + 1,
                right_sequence=index * 10 + 1,
                width=config.width,
                height=config.height,
                left_pixels=bytes(value * 4 + index * 5 for value in range(32)),
                right_pixels=bytes(180 - value * 3 - index * 4 for value in range(32)),
                left_format=AX_FORMAT_YUV420_SEMIPLANAR,
                right_format=AX_FORMAT_YUV420_SEMIPLANAR,
            )
            for index in range(2)
        ]
        records = [
            ImuRecord(
                timestamp_ns=first_ns - 100_000_000 + index * 10_000_000 + 37,
                frame_id="imu_link",
                orientation=(0.0, 0.0, 0.0, 1.0),
                orientation_covariance=(-1.0,) + (0.0,) * 8,
                angular_velocity=(index / 10, -0.2, 0.3),
                angular_velocity_covariance=(0.0,) * 9,
                linear_acceleration=(-0.5, index / 20, 9.81),
                linear_acceleration_covariance=(0.0,) * 9,
                arrival_monotonic_ns=(index + 1) * 10_000_000,
            )
            for index in range(42)
        ] if include_imu else []
        writer = DatasetWriter(output, config, include_imu)
        try:
            for pair in pairs:
                writer.write_pair(pair)
            for record in records:
                writer.write_imu(record)
            capture_stats = {}
            if records:
                capture_stats = {
                    "startup_camera_timestamp_ns": pairs[0].left_timestamp_ns,
                    "startup_imu_timestamp_ns": records[0].timestamp_ns,
                    "startup_camera_imu_offset_ns": abs(
                        pairs[0].left_timestamp_ns - records[0].timestamp_ns),
                }
            writer.seal_payload({"capture_stats": capture_stats})
            report = validate_dataset(writer.staging_path, write_report=False)
            self.assertEqual(report.errors, [])
            path = writer.commit(True)
        finally:
            writer._close_streams()
        return path, config, pairs, records

    def _check_reader(self, image_format: str, *, include_imu: bool):
        with tempfile.TemporaryDirectory() as temporary:
            path, config, pairs, records = self._dataset(
                Path(temporary), image_format, include_imu=include_imu)
            manifest = json.loads((path / "dataset.yaml").read_text(encoding="utf-8"))
            self.assertEqual(manifest["schema_version"], "1.0.0")
            self.assertEqual(manifest["type"], "kalibr_directory_dataset")
            self.assertEqual(set(manifest), {"schema_version", "type", "dataset_id", "cameras", "imus"})
            self.assertEqual(manifest["cameras"][0]["images"], "cameras/cam0")
            self.assertTrue((path / "meta/capture_dataset.yaml").is_file())
            reader = DirectoryReader(path)
            self.assertEqual(reader.dataset_id, path.name)
            expected_topics = {
                'cam0': ("sensor_msgs/msg/Image", 2),
                'cam1': ("sensor_msgs/msg/Image", 2),
            }
            if include_imu:
                expected_topics['imu0'] = ("sensor_msgs/msg/Imu", len(records))
            self.assertEqual(
                {item.name: (item.msgtype, item.message_count) for item in reader.topics()},
                expected_topics,
            )
            for side, topic in (("left", 'cam0'), ("right", 'cam1')):
                with reader.index_images(topic) as images:
                    self.assertEqual(len(images.index), len(pairs))
                    for index, pair in enumerate(pairs):
                        image = images.get(index)
                        timestamp = getattr(pair, f"{side}_timestamp_ns")
                        self.assertEqual(image.header_timestamp_ns, timestamp)
                        self.assertEqual(image.record_timestamp_ns, timestamp)
                        self.assertEqual(image.frame_id, getattr(config, f"{side}_frame_id"))
                        self.assertEqual(image.image.shape, (config.height, config.width))
                        self.assertEqual(str(image.image.dtype), "uint8")
                        actual = image.image.tobytes()
                        expected = getattr(pair, f"{side}_pixels")
                        if image_format == "bmp":
                            self.assertEqual(actual, expected)
                        else:
                            # JPEG preserves dimensions/stream semantics, not exact pixels.
                            self.assertLessEqual(
                                max(abs(a - b) for a, b in zip(actual, expected)), 5)
            if records:
                actual_imu = reader.read_imu('imu0')
                self.assertEqual(len(actual_imu), len(records))
                for actual, expected in zip(actual_imu, records):
                    self.assertEqual(actual.header_timestamp_ns, expected.timestamp_ns)
                    self.assertEqual(actual.record_timestamp_ns, expected.timestamp_ns)
                    self.assertEqual(actual.frame_id, "imu_link")
                    self.assertEqual(tuple(actual.angular_velocity), expected.angular_velocity)
                    self.assertEqual(tuple(actual.linear_acceleration), expected.linear_acceleration)
                original = (path / "imu/imu0.csv").read_bytes().splitlines()
                self.assertEqual(original[0], b"timestamp_ns,wx,wy,wz,ax,ay,az")
                self.assertFalse((path / "imu/imu0_kalibr.csv").exists())
            else:
                self.assertEqual(manifest["imus"], [])
                self.assertFalse((path / "imu").exists())

    def test_bmp_stereo_imu_custom_topics_and_original_values(self):
        self._check_reader("bmp", include_imu=True)

    def test_bmp_stereo_has_no_imu_topic(self):
        self._check_reader("bmp", include_imu=False)

    def test_jpeg95_stereo_imu_is_decodable(self):
        try:
            ensure_image_support("jpeg")
        except RuntimeError as error:
            self.skipTest(f"optional JPEG codec unavailable: {error}")
        self._check_reader("jpeg", include_imu=True)


if __name__ == "__main__":
    unittest.main()
