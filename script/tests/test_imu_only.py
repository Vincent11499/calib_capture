import contextlib
import csv
from dataclasses import replace
import json
import io
import sys
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock

from calib_capture import imu_only
from calib_capture.cli import build_parser, _capture_config, main
from calib_capture.config import build_capture_config
from calib_capture.dataset import ImuRecord


def sample(stamp=1_000_000_000, frame_id="imu_link"):
    return ImuRecord(stamp, frame_id, (0., 0., 0., 1.), (0.,) * 9,
                     (0.1, -0.2, 0.3), (0.,) * 9, (1., 2., 9.8), (0.,) * 9, 123)


class ImuOnlyTests(unittest.TestCase):
    def run_capture(self, output, *, records=None, error=None, stopped=False):
        records = records if records is not None else [sample(), sample(1_010_000_000)]
        subscriber = mock.Mock()

        def construct(config, callback, *, temperature_callback):
            def start():
                temperature_callback(SimpleNamespace(temperature=39.0, variance=float("nan")))
                for record in records:
                    callback(record)
            subscriber.start.side_effect = start
            if error:
                subscriber.wait_first.side_effect = RuntimeError(error)
            return subscriber

        config = build_capture_config(mode="imu-continuous", output=str(output),
                                      overrides={"duration_s": 0.001})
        interrupts = SimpleNamespace(count=int(stopped), check=lambda **kwargs: None)
        with mock.patch.object(imu_only, "ImuSubscriber", side_effect=construct), \
                mock.patch.object(imu_only, "_CaptureInterrupts",
                                  return_value=contextlib.nullcontext(interrupts)), \
                mock.patch("calib_capture.capture.AxVinApi", side_effect=AssertionError("camera accessed")), \
                mock.patch("builtins.print"):
            return imu_only.record_imu_only(config)

    def test_cli_scope_and_ini_precedence(self):
        with tempfile.TemporaryDirectory() as raw:
            ini = Path(raw) / "imu.ini"
            ini.write_text("[capture]\nschema_version = 1.0.0\nduration_s=123\nimu_topic=/custom\n", encoding="utf-8")
            parser = build_parser()
            args = parser.parse_args(["--config", str(ini), "imu-continuous", "--duration-s", "3600"])
            config = _capture_config(args, args.command)
            self.assertEqual(config.duration_s, 3600)
            self.assertEqual(config.imu_topic, "/custom")
            self.assertTrue(config.output)
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                parser.parse_args(["imu-continuous", "--save-rate-hz", "5"])

    def test_capture_raw_values_without_camera_files_and_validate_dispatch(self):
        with tempfile.TemporaryDirectory() as raw:
            output, report = self.run_capture(Path(raw) / "session")
            self.assertFalse(report.errors)
            self.assertFalse((output / "cameras").exists())
            self.assertFalse((output / "dataset.yaml").exists())
            self.assertFalse((output / "meta/imu_full.jsonl").exists())
            with (output / "imu/imu0.csv").open() as stream:
                rows = list(csv.reader(stream))
            self.assertEqual(rows[0], list(imu_only.CSV_FIELDS))
            self.assertEqual(rows[1], ["1000000000", "0.1", "-0.2", "0.3", "1.0", "2.0", "9.8", "39.0"])
            self.assertEqual(report.stats["imu_effective_rate_hz"], 100)
            with mock.patch("builtins.print"):
                self.assertEqual(main(["validate", str(output)]), 0)
                self.assertEqual(main(["export-ros1", str(output), str(output / "x.bag")]), 1)
            self.assertFalse((output / "x.bag").exists())

    def test_operator_stop_preserves_valid_data(self):
        with tempfile.TemporaryDirectory() as raw:
            output, report = self.run_capture(Path(raw) / "session", stopped=True)
            session = json.loads((output / "meta/session.json").read_text())
            self.assertEqual(session["capture_status"], "operator_stopped")
            self.assertFalse(report.errors)

    def test_empty_and_startup_timeout_are_failed_not_committed(self):
        for error in (None, "startup timeout"):
            with self.subTest(error=error), tempfile.TemporaryDirectory() as raw:
                output = Path(raw) / "session"
                with self.assertRaises(RuntimeError):
                    self.run_capture(output, records=[], error=error)
                self.assertFalse(output.exists())
                self.assertTrue(Path(str(output) + ".failed").is_dir())

    def test_invalid_values_stamp_and_frame_change_fail(self):
        invalid = [sample(), sample(999), sample(1_010_000_000, "other"),
                   replace(sample(1_010_000_000), angular_velocity=(float("nan"), 0., 0.))]
        for record in invalid:
            with self.subTest(record=record), tempfile.TemporaryDirectory() as raw:
                with self.assertRaises(RuntimeError):
                    self.run_capture(Path(raw) / "session", records=[sample(), record])

    def test_flush_stall_and_write_failure(self):
        with tempfile.TemporaryDirectory() as raw:
            writer = imu_only.ImuCsvWriter(Path(raw) / "imu.csv", "fallback")
            try:
                writer.update_temperature(SimpleNamespace(temperature=39.0))
                writer.write(sample())
                with mock.patch.object(imu_only.time, "monotonic", return_value=writer.last_arrival + 16):
                    with self.assertRaisesRegex(RuntimeError, "stalled"):
                        writer.flush_and_check(15)
                with mock.patch.object(writer, "stream") as stream:
                    stream.flush.side_effect = OSError("disk full")
                    with self.assertRaises(OSError):
                        writer.flush_and_check(15)
            finally:
                writer.close()

    def test_gap_and_empty_frame_id_warnings(self):
        with tempfile.TemporaryDirectory() as raw:
            _, report = self.run_capture(Path(raw) / "session", records=[sample(frame_id=""),
                                         sample(1_030_000_000, "")])
            self.assertFalse(report.errors)
            self.assertEqual(report.stats["imu_long_gap_count"], 1)
            self.assertEqual(report.stats["imu_frame_id"], "imu0")
            self.assertTrue(any("fallback" in w for w in report.warnings))

    def test_csv_corruption_is_rejected_and_output_not_overwritten(self):
        with tempfile.TemporaryDirectory() as raw:
            output, _ = self.run_capture(Path(raw) / "session")
            with self.assertRaises(ValueError):
                self.run_capture(output)
            with (output / "imu/imu0.csv").open("a") as stream:
                stream.write("truncated\n")
            self.assertTrue(imu_only.validate_imu_only(output).errors)

    def test_temperature_hold_update_and_startup_gate(self):
        with tempfile.TemporaryDirectory() as raw:
            path = Path(raw) / "imu.csv"
            writer = imu_only.ImuCsvWriter(path, "imu0")
            try:
                self.assertIs(writer.write(sample()), False)
                self.assertEqual(writer.count, 0)
                writer.update_temperature(SimpleNamespace(temperature=39.0, variance=float("nan")))
                for i in range(100):
                    writer.write(sample(1_000_000_000 + i * 10_000_000))
                writer.update_temperature(SimpleNamespace(temperature=40.0))
                writer.write(sample(2_000_000_000))
                self.assertEqual(writer.temperature_updates, 2)
                self.assertEqual(writer.imu_before_temperature, 1)
                with self.assertRaisesRegex(ValueError, "temperature must be finite"):
                    writer.update_temperature(SimpleNamespace(temperature=float("nan")))
                self.assertEqual(writer.temperature, 40.0)
            finally:
                writer.close()
            with path.open() as stream:
                rows = list(csv.reader(stream))[1:]
            self.assertEqual(len(rows), 101)
            self.assertTrue(all(row[-1] == "39.0" for row in rows[:100]))
            self.assertEqual(rows[-1][-1], "40.0")

    def test_temperature_topic_cli(self):
        args = build_parser().parse_args(["imu-continuous", "--imu-temperature-topic", "/other/temp"])
        self.assertEqual(_capture_config(args, args.command).imu_temperature_topic, "/other/temp")
        with self.assertRaises(ValueError):
            build_capture_config(mode="imu-continuous", overrides={"imu_temperature_topic": "/imu/data"})

    def test_ros_temperature_subscription_gate_timeout_and_callback_error(self):
        from calib_capture.capture import ImuSubscriber
        subscriptions = {}

        class Node:
            def __init__(self, name):
                pass

            def create_subscription(self, message_type, topic, callback, qos):
                subscriptions[topic] = callback
                return object()

        modules = {
            "rclpy": SimpleNamespace(init=mock.Mock()),
            "rclpy.node": SimpleNamespace(Node=Node),
            "rclpy.signals": SimpleNamespace(SignalHandlerOptions=SimpleNamespace(NO=0)),
            "rclpy.qos": SimpleNamespace(
                DurabilityPolicy=SimpleNamespace(VOLATILE=0),
                HistoryPolicy=SimpleNamespace(KEEP_LAST=0),
                ReliabilityPolicy=SimpleNamespace(BEST_EFFORT=0), QoSProfile=mock.Mock()),
            "sensor_msgs.msg": SimpleNamespace(Imu=object, Temperature=object),
        }
        vector = SimpleNamespace(x=0., y=0., z=0., w=1.)
        message = SimpleNamespace(
            header=SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=0), frame_id="imu_link"),
            orientation=vector, angular_velocity=vector, linear_acceleration=vector,
            orientation_covariance=[0.] * 9, angular_velocity_covariance=[0.] * 9,
            linear_acceleration_covariance=[0.] * 9)
        with tempfile.TemporaryDirectory() as raw:
            writer = imu_only.ImuCsvWriter(Path(raw) / "imu.csv", "imu0")
            try:
                subscriber = ImuSubscriber(build_capture_config(mode="imu-continuous"), writer.write,
                                           temperature_callback=writer.update_temperature)
                with mock.patch.dict(sys.modules, modules), mock.patch("threading.Thread.start"):
                    subscriber.start()
                subscriptions["/imu/data"](message)
                self.assertEqual(subscriber.count, 0)
                self.assertFalse(subscriber.first_event.is_set())
                with self.assertRaisesRegex(RuntimeError, "/imu/temperature"):
                    subscriber.wait_first(0.001)
                subscriptions["/imu/temperature"](SimpleNamespace(temperature=39., variance=float("nan")))
                subscriptions["/imu/data"](message)
                self.assertEqual(subscriber.wait_first(0.001), 1_000_000_000)
                self.assertEqual(subscriber.count, 1)
                self.assertEqual(subscriber.records, [])
                subscriptions["/imu/temperature"](SimpleNamespace(temperature=float("inf")))
                with self.assertRaisesRegex(RuntimeError, "temperature must be finite"):
                    subscriber.raise_if_failed()
                self.assertTrue(subscriber.stop_event.is_set())
            finally:
                writer.close()


if __name__ == "__main__":
    unittest.main()
