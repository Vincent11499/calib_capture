"""Small synthetic datasets exercise the capture/Kalibr contract without ROS."""
from __future__ import annotations

import contextlib
import io
import json
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path

from calib_capture.axera import AX_FORMAT_YUV420_SEMIPLANAR, GrayPair
from calib_capture.cli import _capture_config, build_parser
from calib_capture.config import build_capture_config
from calib_capture.dataset import DatasetWriter, ImuRecord
from calib_capture.validator import validate_dataset
from test_calibration_data import make_legacy_fixture, make_v190_fixture


def imu_record(timestamp: int, frame_id: str = "sensor_imu") -> ImuRecord:
    return ImuRecord(
        timestamp_ns=timestamp,
        frame_id=frame_id,
        orientation=(0.0, 0.0, 0.0, 1.0),
        orientation_covariance=(0.0,) * 9,
        angular_velocity=(-0.125, 2.5e-7, 3.125),
        angular_velocity_covariance=(0.0,) * 9,
        linear_acceleration=(9.80665, -0.0, -12.25),
        linear_acceleration_covariance=(0.0,) * 9,
        arrival_monotonic_ns=timestamp,
    )


def pair_record(timestamp: int, index: int, delta_ns: int = 1_000) -> GrayPair:
    return GrayPair(
        left_timestamp_ns=timestamp,
        right_timestamp_ns=timestamp + delta_ns,
        left_sequence=index + 1,
        right_sequence=index + 1,
        width=5,
        height=3,
        left_pixels=bytes(range(15)),
        right_pixels=bytes(reversed(range(15))),
        left_format=AX_FORMAT_YUV420_SEMIPLANAR,
        right_format=AX_FORMAT_YUV420_SEMIPLANAR,
    )


class DatasetContractTests(unittest.TestCase):
    def setUp(self) -> None:
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.fixture_number = 0

    @staticmethod
    def read_json(path: Path) -> dict:
        return json.loads(path.read_text(encoding="utf-8"))

    @staticmethod
    def write_json(path: Path, value: dict) -> None:
        path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

    def config(self, output: Path, mode: str = "camera-imu", **overrides):
        values = {"imu_preroll_s": 0, "imu_postroll_s": 0, **overrides}
        config = build_capture_config(mode=mode, output=str(output), overrides=values)
        return replace(config, width=5, height=3)

    def dataset(
        self, *, mode: str = "camera-imu", camera_start: int = 2_000_000_000,
        imu_first: int | None = None, imu_last: int | None = None,
        frame_id: str = "sensor_imu", delta_ns: int = 1_000, **overrides,
    ) -> Path:
        self.fixture_number += 1
        output = self.root / f"dataset_{self.fixture_number}"
        config = self.config(output, mode, **overrides)
        writer = DatasetWriter(output, config, include_imu=mode == "camera-imu")
        self.addCleanup(writer._close_streams)
        for index in range(2):
            writer.write_pair(pair_record(camera_start + index * 200_000_000, index, delta_ns))
        stats = {}
        if mode == "camera-imu":
            first = camera_start - 100_000_000 if imu_first is None else imu_first
            last = camera_start + 300_000_000 if imu_last is None else imu_last
            timestamps = list(range(first, last, 10_000_000)) + [last]
            for timestamp in timestamps:
                writer.write_imu(imu_record(timestamp, frame_id))
            camera_reference = min(camera_start, first + 1_000)
            stats = {
                "startup_camera_timestamp_ns": camera_reference,
                "startup_imu_timestamp_ns": first,
                "startup_camera_imu_offset_ns": abs(camera_reference - first),
            }
        writer.seal_payload({"capture_status": "completed", "mode": mode, "capture_stats": stats})
        return writer.commit(True)

    def assert_valid(self, path: Path):
        report = validate_dataset(path, write_report=False)
        self.assertEqual([], report.errors, report.errors)
        return report

    def assert_invalid(self, path: Path):
        report = validate_dataset(path, write_report=False)
        self.assertTrue(report.errors, report.to_dict())
        return report

    def test_root_and_capture_metadata_share_version_with_explicit_sensor_identity(self) -> None:
        path = self.dataset()
        manifest = self.read_json(path / "dataset.yaml")
        self.assertEqual(set(manifest), {"schema_version", "type", "dataset_id", "cameras", "imus"})
        self.assertEqual(manifest["schema_version"], "1.0.0")
        self.assertEqual(manifest["type"], "kalibr_directory_dataset")
        self.assertEqual(manifest["cameras"], [
            {"id": "cam0", "timestamps": "cameras/cam0/timestamps.csv",
             "images": "cameras/cam0", "frame_id": "cam0", "resolution": [5, 3], "clock_source": "axera_pts_us"},
            {"id": "cam1", "timestamps": "cameras/cam1/timestamps.csv",
             "images": "cameras/cam1", "frame_id": "cam1", "resolution": [5, 3], "clock_source": "axera_pts_us"},
        ])
        self.assertEqual(manifest["dataset_id"], path.name)
        self.assertEqual(manifest["imus"], [{"id": "imu0", "data": "imu/imu0.csv",
            "frame_id": "sensor_imu", "clock_source": "ros_header_stamp", "angular_velocity_unit": "rad/s",
            "linear_acceleration_unit": "m/s^2"}])
        capture = (path / "meta/capture_dataset.yaml").read_text(encoding="utf-8")
        self.assertIn("schema_version: 1.0.0\n", capture)
        self.assertIn("capture_api: AX_VIN_GetYuvFrame\n", capture)
        self.assertIn("vin: {pipe: 2, chn: 0}", capture)
        self.assertIn("vin: {pipe: 0, chn: 0}", capture)
        session = self.read_json(path / "meta/session.json")
        self.assertEqual(session["schema_version"], "1.0.0")
        self.assertEqual(session["dataset_manifest"], "kalibr_directory_dataset")
        self.assertFalse(list(path.rglob("*.tmp")))
        self.assertFalse((path / "meta/checksums.sha256").exists())
        self.assert_valid(path)

    def test_stereo_manifest_has_no_imu_and_custom_camera_topics(self) -> None:
        path = self.dataset(mode="stereo-continuous", left_topic="/rig/left", right_topic="/rig/right")
        manifest = self.read_json(path / "dataset.yaml")
        self.assertEqual(manifest["imus"], [])
        self.assertTrue(all('topic' not in camera for camera in manifest['cameras']))
        session = self.read_json(path / 'meta/session.json')['config']
        self.assertEqual([session['left_topic'], session['right_topic']], ['/rig/left', '/rig/right'])
        self.assert_valid(path)

    def test_joint_manifest_preserves_all_custom_topics(self) -> None:
        path = self.dataset(left_topic="/rig/left", right_topic="/rig/right", imu_topic="/rig/imu")
        manifest = self.read_json(path / "dataset.yaml")
        self.assertTrue(all('topic' not in camera for camera in manifest['cameras']))
        self.assertNotIn('topic', manifest['imus'][0])
        session = self.read_json(path / 'meta/session.json')['config']
        self.assertEqual([session['left_topic'], session['right_topic'], session['imu_topic']],
                         ['/rig/left', '/rig/right', '/rig/imu'])
        self.assertEqual(manifest["imus"][0]["data"], "imu/imu0.csv")
        self.assert_valid(path)

    def test_sensor_identity_and_units_must_match_acquisition_metadata(self) -> None:
        mutations = (
            lambda m: m.update(dataset_id="different_session"),
            lambda m: m["cameras"][0].update(id="cam1"),
            lambda m: m["cameras"][0].update(resolution=[10, 10]),
            lambda m: m["imus"][0].update(frame_id="fallback_instead_of_raw_frame"),
            lambda m: m["imus"][0].update(angular_velocity_unit="deg/s"),
            lambda m: m["imus"][0].update(clock_source="wall_clock"),
        )
        for index, mutate in enumerate(mutations):
            with self.subTest(index=index):
                path = self.dataset()
                manifest = self.read_json(path / "dataset.yaml")
                mutate(manifest)
                self.write_json(path / "dataset.yaml", manifest)
                self.assert_invalid(path)

    def test_public_documents_share_string_version_and_keep_raw_frame_id(self) -> None:
        path = self.dataset(frame_id="")
        manifest = self.read_json(path / "dataset.yaml")
        self.assertEqual(manifest["imus"][0]["frame_id"], "imu0")
        report = validate_dataset(path)
        for relative, kind in (("meta/session.json", "capture_session"), ("meta/validation.json", "capture_validation")):
            data = self.read_json(path / relative)
            self.assertEqual(data["schema_version"], "1.0.0")
            self.assertEqual(data["kind"], kind)
        raw = json.loads((path / "meta/imu_full.jsonl").read_text().splitlines()[0])
        self.assertEqual(raw["schema_version"], "1.0.0")
        self.assertEqual(raw["kind"], "imu_message")
        self.assertEqual(raw["frame_id"], "")
        self.assertFalse(report.errors)

    def test_single_imu_csv_has_kalibr_header_and_original_values(self) -> None:
        path = self.dataset()
        header, body = (path / "imu/imu0.csv").read_bytes().split(b"\n", 1)
        self.assertEqual(header.rstrip(b"\r"), b"timestamp_ns,wx,wy,wz,ax,ay,az")
        self.assertIn(b"-0.125,2.5e-07,3.125,9.80665,-0.0,-12.25", body)
        self.assertEqual([item.name for item in (path / "imu").iterdir()], ["imu0.csv"])

    def test_new_layout_missing_companions_never_falls_back_to_legacy(self) -> None:
        for relative in ("dataset.yaml", "meta/capture_dataset.yaml", "meta/imu_full.jsonl",
                         "imu/imu0.csv", "meta/session.json"):
            with self.subTest(relative=relative):
                path = self.dataset()
                (path / relative).unlink()
                self.assert_invalid(path)

    def test_new_layout_missing_or_unknown_session_marker_is_failure(self) -> None:
        for marker in (None, "kalibr_directory_v2", "capture_schema_v3"):
            with self.subTest(marker=marker):
                path = self.dataset()
                session = self.read_json(path / "meta/session.json")
                if marker is None:
                    session.pop("dataset_manifest")
                else:
                    session["dataset_manifest"] = marker
                self.write_json(path / "meta/session.json", session)
                self.assert_invalid(path)

    def test_root_unknown_fields_are_rejected_at_each_level(self) -> None:
        for level in ("root", "camera", "imu"):
            with self.subTest(level=level):
                path = self.dataset()
                manifest = self.read_json(path / "dataset.yaml")
                target = manifest if level == "root" else manifest["cameras" if level == "camera" else "imus"][0]
                target["synchronization_tolerance_s"] = 0.00002
                self.write_json(path / "dataset.yaml", manifest)
                self.assert_invalid(path)

    def test_root_duplicate_keys_and_boolean_version_are_not_schema_one(self) -> None:
        path = self.dataset()
        root_manifest = path / "dataset.yaml"
        original = root_manifest.read_text(encoding="utf-8")
        self.assertIn('"schema_version": "1.0.0",', original)
        root_manifest.write_text(original.replace('"schema_version": "1.0.0",',
                                                  '"schema_version": "1.0.0", "schema_version": "1.0.0",', 1),
                                 encoding="utf-8")
        self.assert_invalid(path)
        manifest = json.loads(original)
        manifest["schema_version"] = True
        self.write_json(root_manifest, manifest)
        self.assert_invalid(path)

    def test_root_paths_topics_and_schema_must_match_capture_session(self) -> None:
        changes = (
            lambda manifest: manifest.update(schema_version=3),
            lambda manifest: manifest.update(type="calibration"),
            lambda manifest: manifest["cameras"][0].update(topic="/wrong/left"),
            lambda manifest: manifest["cameras"][0].update(images="../external"),
            lambda manifest: manifest["cameras"][1].update(timestamps="cameras/cam0/timestamps.csv"),
            lambda manifest: manifest["imus"][0].update(data="imu/imu0_kalibr.csv"),
            lambda manifest: manifest.update(imus=[]),
        )
        for index, change in enumerate(changes):
            with self.subTest(change=index):
                path = self.dataset()
                manifest = self.read_json(path / "dataset.yaml")
                change(manifest)
                self.write_json(path / "dataset.yaml", manifest)
                self.assert_invalid(path)

    def test_capture_sidecar_inconsistent_topic_resolution_or_frame_is_failure(self) -> None:
        for before, after in (("/stereo/left/image_raw", "/wrong/left"),
                              ("resolution: [5, 3]", "resolution: [4, 3]"),
                              ('frame_id: "sensor_imu"', 'frame_id: "other_imu"')):
            with self.subTest(before=before):
                path = self.dataset()
                sidecar = path / "meta/capture_dataset.yaml"
                original = sidecar.read_text(encoding="utf-8")
                self.assertIn(before, original)
                sidecar.write_text(original.replace(before, after), encoding="utf-8")
                self.assert_invalid(path)

    def test_csv_header_and_original_measurements_are_required(self) -> None:
        for before, after in ((b"timestamp_ns,wx", b"timestamp_ns,gyro_x"),
                              (b"-0.125", b"0.125"),
                              (b"-12.25", b"12.25")):
            with self.subTest(before=before):
                path = self.dataset()
                compatible = path / "imu/imu0.csv"
                compatible.write_bytes(compatible.read_bytes().replace(before, after, 1))
                self.assert_invalid(path)

    def test_nonempty_source_frame_id_overrides_config_without_rewriting_raw(self) -> None:
        path = self.dataset(frame_id="physical_imu", imu_frame_id="configured_fallback")
        session = self.read_json(path / "meta/session.json")
        messages = [json.loads(line) for line in (path / "meta/imu_full.jsonl").read_text().splitlines()]
        self.assertEqual(session["imu_frame_id"], "physical_imu")
        self.assertEqual(session["config"]["imu_frame_id"], "configured_fallback")
        self.assertEqual(session["imu_empty_frame_id_count"], 0)
        self.assertTrue(all(message["frame_id"] == "physical_imu" for message in messages))
        self.assertTrue(all("raw_frame_id" not in message for message in messages))
        self.assert_valid(path)

    def test_empty_source_frame_id_is_preserved_and_fallback_is_warned(self) -> None:
        path = self.dataset(frame_id="", imu_frame_id="fallback_imu")
        session = self.read_json(path / "meta/session.json")
        messages = [json.loads(line) for line in (path / "meta/imu_full.jsonl").read_text().splitlines()]
        self.assertEqual(session["imu_frame_id"], "fallback_imu")
        self.assertEqual(session["imu_empty_frame_id_count"], len(messages))
        self.assertTrue(all(message["frame_id"] == "" for message in messages))
        report = self.assert_valid(path)
        self.assertTrue(any("frame" in warning.lower() for warning in report.warnings), report.warnings)

    def test_effective_frame_id_change_rejects_message_without_rewriting_prior_data(self) -> None:
        output = self.root / "frame_change"
        config = self.config(output, imu_frame_id="sensor_imu")
        writer = DatasetWriter(output, config, include_imu=True)
        self.addCleanup(writer._close_streams)
        writer.write_imu(imu_record(1_000_000_000, ""))
        writer.write_imu(imu_record(1_010_000_000, "sensor_imu"))
        with self.assertRaisesRegex(ValueError, "frame_id"):
            writer.write_imu(imu_record(1_020_000_000, "another_sensor"))
        writer.seal_payload({"capture_status": "failed"})
        messages = [json.loads(line) for line in (writer.staging_path / "meta/imu_full.jsonl").read_text().splitlines()]
        self.assertEqual([message["frame_id"] for message in messages], ["", "sensor_imu"])
        self.assertEqual(writer.imu_count, 2)

    def test_inconsistent_frame_id_metadata_or_saved_message_is_failure(self) -> None:
        for field in ("imu_frame_id", "imu_empty_frame_id_count", "raw_message"):
            with self.subTest(field=field):
                path = self.dataset()
                if field == "raw_message":
                    raw = path / "meta/imu_full.jsonl"
                    lines = raw.read_text(encoding="utf-8").splitlines()
                    message = json.loads(lines[-1])
                    message["frame_id"] = "another_frame"
                    lines[-1] = json.dumps(message)
                    raw.write_text("\n".join(lines) + "\n", encoding="utf-8")
                else:
                    session = self.read_json(path / "meta/session.json")
                    session[field] = "another_frame" if field == "imu_frame_id" else 1
                    self.write_json(path / "meta/session.json", session)
                self.assert_invalid(path)

    def test_non_string_effective_frame_id_metadata_is_rejected_without_crashing(self) -> None:
        for value in (None, 42, [], {"frame": "sensor_imu"}):
            with self.subTest(value=value):
                path = self.dataset()
                session = self.read_json(path / "meta/session.json")
                session["imu_frame_id"] = value
                self.write_json(path / "meta/session.json", session)
                self.assert_invalid(path)

    def test_nonfinite_numeric_metadata_is_a_reported_failure_not_an_exception(self) -> None:
        cases = (("config", "jpeg_quality"), ("config", "width"),
                 ("config", "sync_tolerance_us"), ("config", "save_rate_hz"),
                 ("config", "expected_source_rate_hz"), ("config", "imu_expected_rate_hz"),
                 ("session", "pair_count"), ("session", "ax_deinit_result"),
                 ("capture_stats", "writer_queue_drop_pairs"),
                 ("message", "arrival_monotonic_ns"))
        for container, key in cases:
            with self.subTest(container=container, key=key):
                path = self.dataset()
                if container == "message":
                    raw = path / "meta/imu_full.jsonl"
                    lines = raw.read_text(encoding="utf-8").splitlines()
                    message = json.loads(lines[0])
                    message[key] = float("inf")
                    lines[0] = json.dumps(message)
                    raw.write_text("\n".join(lines) + "\n", encoding="utf-8")
                else:
                    session = self.read_json(path / "meta/session.json")
                    target = session if container == "session" else session[container]
                    target[key] = float("inf")
                    self.write_json(path / "meta/session.json", session)
                self.assert_invalid(path)

    def test_new_directory_rejects_timestamps_outside_kalibr_signed_int64(self) -> None:
        path = self.dataset(camera_start=(1 << 63) + 1_000_000_000)
        self.assert_invalid(path)

    def test_nan_pair_delta_cannot_bypass_timestamp_consistency_validation(self) -> None:
        path = self.dataset()
        pairs = path / "meta/pairs.csv"
        original = pairs.read_text(encoding="utf-8")
        self.assertIn(",1.000,", original)
        pairs.write_text(original.replace(",1.000,", ",nan,", 1), encoding="utf-8")
        self.assert_invalid(path)

    def test_covariance_requires_an_array_not_an_iterable_string_or_object(self) -> None:
        for key in ("orientation_covariance", "angular_velocity_covariance", "linear_acceleration_covariance"):
            for value in ("000000000", {str(index): 0 for index in range(9)}):
                with self.subTest(key=key, value=value):
                    path = self.dataset()
                    raw = path / "meta/imu_full.jsonl"
                    lines = raw.read_text(encoding="utf-8").splitlines()
                    message = json.loads(lines[0])
                    message[key] = value
                    lines[0] = json.dumps(message)
                    raw.write_text("\n".join(lines) + "\n", encoding="utf-8")
                    self.assert_invalid(path)

    def test_exact_roll_coverage_uses_earliest_first_and_latest_last_camera(self) -> None:
        path = self.dataset(imu_preroll_s=1, imu_postroll_s=1, imu_first=1_000_000_000,
                            imu_last=3_200_020_000, delta_ns=20_000)
        session = self.read_json(path / "meta/session.json")
        self.assertEqual(session["camera_first_timestamp_ns"], 2_000_000_000)
        self.assertEqual(session["camera_last_timestamp_ns"], 2_200_020_000)
        self.assertEqual(session["imu_preroll_actual_s"], 1.0)
        self.assertEqual(session["imu_postroll_actual_s"], 1.0)
        self.assert_valid(path)

    def test_one_nanosecond_short_of_requested_roll_is_failure(self) -> None:
        for first, last in ((1_000_000_001, 3_200_020_000), (1_000_000_000, 3_200_019_999)):
            with self.subTest(imu_first=first, imu_last=last):
                path = self.dataset(imu_preroll_s=1, imu_postroll_s=1, imu_first=first,
                                    imu_last=last, delta_ns=20_000)
                self.assert_invalid(path)

    def test_zero_rolls_disable_new_boundary_requirements(self) -> None:
        path = self.dataset(imu_first=2_010_000_000, imu_last=2_190_000_000)
        self.assert_valid(path)

    def test_long_preroll_does_not_become_a_false_clock_domain_error(self) -> None:
        path = self.dataset(camera_start=8_000_000_000, imu_first=1_000_000_000,
                            imu_last=9_200_001_000, imu_preroll_s=7, imu_postroll_s=1)
        self.assert_valid(path)

    def test_missing_startup_reference_or_wrong_reference_offset_is_failure(self) -> None:
        for field in ("startup_camera_timestamp_ns", "startup_imu_timestamp_ns", "startup_camera_imu_offset_ns"):
            with self.subTest(field=field):
                path = self.dataset()
                session = self.read_json(path / "meta/session.json")
                del session["capture_stats"][field]
                self.write_json(path / "meta/session.json", session)
                self.assert_invalid(path)
        path = self.dataset()
        session = self.read_json(path / "meta/session.json")
        session["capture_stats"]["startup_camera_imu_offset_ns"] = 0
        self.write_json(path / "meta/session.json", session)
        self.assert_invalid(path)

    def test_large_startup_clock_offset_remains_a_hard_failure(self) -> None:
        path = self.dataset()
        session = self.read_json(path / "meta/session.json")
        reference = session["capture_stats"]
        reference["startup_camera_timestamp_ns"] = reference["startup_imu_timestamp_ns"] + 6_000_000_000
        reference["startup_camera_imu_offset_ns"] = 6_000_000_000
        self.write_json(path / "meta/session.json", session)
        self.assert_invalid(path)

    def test_legacy_capture_schema_versions_are_rejected(self) -> None:
        for schema in (1, 2, 3):
            with self.subTest(schema=schema):
                path = self.dataset()
                make_legacy_fixture(path, schema)
                session = self.read_json(path / "meta/session.json")
                session["schema_version"] = schema
                for key in ("dataset_manifest", "imu_frame_id", "imu_empty_frame_id_count",
                            "camera_first_timestamp_ns", "camera_last_timestamp_ns",
                            "imu_preroll_actual_s", "imu_postroll_actual_s"):
                    session.pop(key, None)
                for key in ("imu_preroll_s", "imu_postroll_s"):
                    session["config"].pop(key, None)
                session["config"]["imu_frame_id"] = "sensor_imu"
                session["capture_stats"] = {}
                self.write_json(path / "meta/session.json", session)
                self.assert_invalid(path)

    def test_stereo_modes_have_no_imu_files_and_reports_stay_in_meta(self) -> None:
        for mode in ("stereo-triggered", "stereo-continuous"):
            with self.subTest(mode=mode):
                path = self.dataset(mode=mode, poses=2)
                report = validate_dataset(path)
                self.assertFalse(report.errors, report.errors)
                self.assertEqual({item.name for item in path.iterdir()}, {"dataset.yaml", "cameras", "meta"})
                self.assertFalse((path / "meta/imu_full.jsonl").exists())
                self.assertTrue((path / "meta/validation.json").is_file())

    def test_previous_dual_csv_layout_is_rejected(self) -> None:
        for mode in ("stereo-continuous", "camera-imu"):
            with self.subTest(mode=mode):
                path = self.dataset(mode=mode)
                make_v190_fixture(path)
                self.assert_invalid(path)
                compatible = path / "imu/imu0_kalibr.csv"
                compatible.write_bytes(compatible.read_bytes() + b"extra\n")
                self.assert_invalid(path)

    def test_compact_marker_is_required_and_duplicate_imu_is_rejected(self) -> None:
        for marker in (None, "unknown"):
            with self.subTest(marker=marker):
                path = self.dataset()
                session = self.read_json(path / "meta/session.json")
                if marker is None:
                    session.pop("capture_layout")
                else:
                    session["capture_layout"] = marker
                self.write_json(path / "meta/session.json", session)
                self.assert_invalid(path)
        path = self.dataset()
        (path / "imu/imu0_kalibr.csv").write_bytes((path / "imu/imu0.csv").read_bytes())
        self.assert_invalid(path)

    def test_roll_config_defaults_and_finite_nonnegative_range(self) -> None:
        config = build_capture_config(mode="camera-imu")
        self.assertEqual(config.imu_preroll_s, 1.0)
        self.assertEqual(config.imu_postroll_s, 1.0)
        for key in ("imu_preroll_s", "imu_postroll_s"):
            for value in (0, 0.25, 6, "1.5"):
                with self.subTest(key=key, accepted=value):
                    configured = build_capture_config(mode="camera-imu", overrides={key: value})
                    self.assertEqual(getattr(configured, key), float(value))
            for value in (-1, "NaN", "Infinity", "-Infinity", "not-a-number"):
                with self.subTest(key=key, rejected=value):
                    with self.assertRaises(ValueError):
                        build_capture_config(mode="camera-imu", overrides={key: value})

    def test_roll_ini_and_cli_obey_precedence_without_changing_image_duration(self) -> None:
        ini = self.root / "capture.ini"
        ini.write_text("[capture]\nschema_version = 1.0.0\nimu_preroll_s = 2.5\nimu_postroll_s = 3\n", encoding="utf-8")
        parser = build_parser()
        args = parser.parse_args(["--config", str(ini), "camera-imu", "--imu-preroll-s", "0"])
        config = _capture_config(args, "camera-imu")
        self.assertEqual(config.imu_preroll_s, 0.0)
        self.assertEqual(config.imu_postroll_s, 3.0)
        self.assertEqual(config.duration_s, 60.0)
        args = parser.parse_args(["camera-imu", "--imu-preroll-s", "1.5", "--imu-postroll-s", "0"])
        config = _capture_config(args, "camera-imu")
        self.assertEqual(config.imu_preroll_s, 1.5)
        self.assertEqual(config.imu_postroll_s, 0.0)
        for key in ("imu_preroll_s", "imu_postroll_s"):
            ini.write_text(f"[capture]\nschema_version = 1.0.0\n{key} = nan\n", encoding="utf-8")
            with self.assertRaises(ValueError):
                build_capture_config(mode="camera-imu", config_path=str(ini))

    def test_roll_cli_options_apply_only_to_camera_imu(self) -> None:
        parser = build_parser()
        for mode in ("probe", "stereo-triggered", "stereo-continuous"):
            for flag in ("--imu-preroll-s", "--imu-postroll-s"):
                with self.subTest(mode=mode, flag=flag), contextlib.redirect_stderr(io.StringIO()):
                    with self.assertRaises(SystemExit):
                        parser.parse_args([mode, flag, "1"])


if __name__ == "__main__":
    unittest.main()
