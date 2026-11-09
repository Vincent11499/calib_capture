from __future__ import annotations

import signal
import sys
import tempfile
import threading
import unittest
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

from calib_capture import capture
from calib_capture.capture import (
    CaptureFailed, ImuSubscriber, UserAbort, _CaptureInterrupts,
    _check_startup_clock, _grab_frames_or_stop, _wait_imu_postroll,
    _wait_imu_preroll,
)
from calib_capture.config import build_capture_config
from calib_capture.axera import GrayPair, VinGetFrameError
from calib_capture.dataset import DatasetWriter, ImuRecord
from calib_capture.pipeline import PairDiskWriter


NS = 1_000_000_000


class FakeClock:
    def __init__(self):
        self.now = 0.0
        self.on_advance = lambda: None

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds
        self.on_advance()


def roll_config(**values):
    config = dict(imu_preroll_s=1.0, imu_postroll_s=1.0, startup_timeout_s=0.3,
                  clock_domain_max_offset_s=5.0)
    config.update(values)
    return SimpleNamespace(**config)


def fake_imu(first=10 * NS, last=None):
    return SimpleNamespace(first_timestamp_ns=first,
                           last_timestamp_ns=first if last is None else last,
                           raise_if_failed=mock.Mock())


class FakeGrabber:
    def __init__(self, stamps, clock=None):
        self.stamps = iter(stamps)
        self.clock = clock
        self.frames = []
        self.copy_gray_pair = mock.Mock(side_effect=AssertionError("pre-roll must not map"))

    def grab_frames(self):
        if self.clock is not None:
            self.clock.sleep(0.1)
        left, right = next(self.stamps)
        frames = SimpleNamespace(left=SimpleNamespace(u64PTS=left // 1000),
                                 right=SimpleNamespace(u64PTS=right // 1000), released=False)
        self.frames.append(frames)
        return frames

    def release_frames(self, frames):
        if frames.released:
            raise AssertionError("double release")
        frames.released = True


class RollTests(unittest.TestCase):
    def test_preroll_uses_earlier_side_and_releases_without_mapping(self):
        clock = FakeClock()
        imu = fake_imu(last=12 * NS)
        grabber = FakeGrabber([(11 * NS, 11 * NS - 20_000),
                              (11 * NS + 20_000, 11 * NS)], clock)
        with mock.patch.object(capture.time, "monotonic", clock.monotonic):
            result = _wait_imu_preroll(grabber, imu, roll_config(), 10 * NS, 10 * NS,
                                       lambda: None)
        self.assertEqual(result, (11 * NS + 20_000, 11 * NS))
        self.assertTrue(all(frame.released for frame in grabber.frames))
        grabber.copy_gray_pair.assert_not_called()

    def test_preroll_zero_does_not_grab_or_map(self):
        imu = fake_imu()
        grabber = FakeGrabber([])
        result = _wait_imu_preroll(grabber, imu, roll_config(imu_preroll_s=0),
                                   9 * NS, 9 * NS, lambda: None)
        self.assertEqual(result, (9 * NS, 9 * NS))
        self.assertFalse(grabber.frames)
        imu.raise_if_failed.assert_called_once()

    def test_preroll_requires_imu_to_keep_advancing_and_times_out(self):
        clock = FakeClock()
        grabber = FakeGrabber(((11 * NS + index * 100_000_000,
                                11 * NS + index * 100_000_000) for index in range(30)), clock)
        with mock.patch.object(capture.time, "monotonic", clock.monotonic):
            with self.assertRaisesRegex(RuntimeError, "pre-roll timeout"):
                _wait_imu_preroll(grabber, fake_imu(), roll_config(),
                                   10 * NS, 10 * NS, lambda: None)
        self.assertLess(clock.now, 1.5)
        self.assertTrue(all(frame.released for frame in grabber.frames))

    def test_preroll_regression_and_callback_error_release_acquired_pair(self):
        for regression in (True, False):
            imu = fake_imu(last=12 * NS)
            if not regression:
                imu.raise_if_failed.side_effect = [None, None, RuntimeError("IMU failed")]
            stamp = 10 * NS if regression else 11 * NS
            grabber = FakeGrabber([(stamp, stamp)])
            with self.assertRaisesRegex(RuntimeError, "strictly increasing|IMU failed"):
                _wait_imu_preroll(grabber, imu, roll_config(), 10 * NS, 10 * NS,
                                   lambda: None)
            self.assertTrue(grabber.frames[0].released)

    def test_preroll_first_interrupt_releases_the_acquired_pair(self):
        interrupts = _CaptureInterrupts()
        clock = FakeClock()
        clock.on_advance = lambda: interrupts._on_signal(signal.SIGINT, None)
        grabber = FakeGrabber([(11 * NS, 11 * NS)], clock)
        with self.assertRaisesRegex(UserAbort, "before the camera"):
            _wait_imu_preroll(grabber, fake_imu(last=12 * NS), roll_config(),
                               10 * NS, 10 * NS,
                               lambda: interrupts.check(before_camera=True))
        self.assertTrue(grabber.frames[0].released)

    def test_startup_clock_uses_latest_not_deliberately_early_imu(self):
        stats = {}
        _check_startup_clock(20 * NS, fake_imu(first=10 * NS, last=20 * NS - 1000),
                             roll_config(imu_preroll_s=10), stats)
        self.assertEqual(stats["startup_camera_timestamp_ns"], 20 * NS)
        self.assertEqual(stats["startup_imu_timestamp_ns"], 20 * NS - 1000)
        self.assertEqual(stats["startup_camera_imu_offset_ns"], 1000)
        for stamp in (14 * NS, 26 * NS):
            with self.assertRaisesRegex(RuntimeError, "different clock domains"):
                _check_startup_clock(20 * NS, fake_imu(last=stamp), roll_config(), {})

    def test_postroll_uses_latest_side_and_sensor_time_not_elapsed_sleep(self):
        clock = FakeClock()
        imu = fake_imu(last=12 * NS)
        target = 12 * NS + 20_000
        clock.on_advance = lambda: setattr(imu, "last_timestamp_ns", target)
        with mock.patch.object(capture.time, "monotonic", clock.monotonic), \
                mock.patch.object(capture.time, "sleep", clock.sleep):
            _wait_imu_postroll(imu, 11 * NS + 20_000, roll_config(),
                                lambda: None, deadline=1.3)
        self.assertEqual(imu.last_timestamp_ns, target)
        self.assertAlmostEqual(clock.now, 0.05)

    def test_postroll_already_covered_during_drain_does_not_sleep(self):
        with mock.patch.object(capture.time, "sleep") as sleep:
            _wait_imu_postroll(fake_imu(last=12 * NS), 11 * NS, roll_config(),
                                lambda: None, deadline=0)
        sleep.assert_not_called()

    def test_postroll_zero_does_not_wait_for_camera_coverage(self):
        with mock.patch.object(capture.time, "sleep") as sleep:
            _wait_imu_postroll(fake_imu(), 11 * NS, roll_config(imu_postroll_s=0),
                                lambda: None, deadline=0)
        sleep.assert_not_called()

    def test_postroll_stalled_imu_has_bounded_wait(self):
        clock = FakeClock()
        with mock.patch.object(capture.time, "monotonic", clock.monotonic), \
                mock.patch.object(capture.time, "sleep", clock.sleep):
            with self.assertRaisesRegex(RuntimeError, "post-roll timeout"):
                _wait_imu_postroll(fake_imu(), 11 * NS, roll_config(),
                                    lambda: None, deadline=0.3)
        self.assertAlmostEqual(clock.now, 0.3)

    def test_first_interrupt_preserves_postroll_second_cancels_it(self):
        interrupts = _CaptureInterrupts()
        interrupts._on_signal(signal.SIGINT, None)
        _wait_imu_postroll(fake_imu(last=12 * NS), 11 * NS, roll_config(),
                            interrupts.check, deadline=0)
        interrupts._on_signal(signal.SIGINT, None)
        with self.assertRaisesRegex(UserAbort, "second Ctrl"):
            _wait_imu_postroll(fake_imu(last=12 * NS), 11 * NS, roll_config(),
                                interrupts.check, deadline=0)

    def test_sigint_handler_is_scoped_and_does_not_raise_in_cleanup(self):
        previous = signal.getsignal(signal.SIGINT)
        with _CaptureInterrupts() as interrupts:
            handler = signal.getsignal(signal.SIGINT)
            handler(signal.SIGINT, None)
            handler(signal.SIGINT, None)
            self.assertEqual(interrupts.count, 2)
        self.assertIs(signal.getsignal(signal.SIGINT), previous)


class SubscriberTests(unittest.TestCase):
    def test_raw_frame_ids_are_preserved_and_ros_does_not_own_sigint(self):
        rclpy = mock.Mock()
        no_handlers = object()

        class Node:
            def __init__(self, name):
                pass

            def create_subscription(self, *args):
                return object()

        modules = {
            "rclpy": rclpy,
            "rclpy.node": SimpleNamespace(Node=Node),
            "rclpy.signals": SimpleNamespace(SignalHandlerOptions=SimpleNamespace(NO=no_handlers)),
            "rclpy.qos": SimpleNamespace(
                DurabilityPolicy=SimpleNamespace(VOLATILE=1),
                HistoryPolicy=SimpleNamespace(KEEP_LAST=1),
                ReliabilityPolicy=SimpleNamespace(BEST_EFFORT=1), QoSProfile=mock.Mock()),
            "sensor_msgs": mock.Mock(), "sensor_msgs.msg": SimpleNamespace(Imu=object),
        }
        subscriber = ImuSubscriber(build_capture_config(mode="stereo-imu"))
        with mock.patch.dict(sys.modules, modules), mock.patch.object(capture.threading, "Thread"):
            subscriber.start()
            for index, frame_id in enumerate(("imu_link", ""), 1):
                vector = SimpleNamespace(x=1.0, y=2.0, z=3.0, w=1.0)
                message = SimpleNamespace(
                    header=SimpleNamespace(stamp=SimpleNamespace(sec=index, nanosec=1), frame_id=frame_id),
                    orientation=vector, orientation_covariance=(0,) * 9,
                    angular_velocity=vector, angular_velocity_covariance=(0,) * 9,
                    linear_acceleration=vector, linear_acceleration_covariance=(0,) * 9)
                subscriber._node.on_imu(message)
        self.assertIsNone(subscriber.error)
        self.assertEqual([record.frame_id for record in subscriber.records], ["imu_link", ""])
        self.assertEqual([record.timestamp_ns for record in subscriber.records], [NS + 1, 2 * NS + 1])
        rclpy.init.assert_called_once_with(args=[], signal_handler_options=no_handlers)

    def test_wait_first_allows_cancellation_without_timeout_delay(self):
        subscriber = ImuSubscriber(build_capture_config(mode="stereo-imu"))
        with self.assertRaisesRegex(UserAbort, "startup stopped"):
            subscriber.wait_first(15, mock.Mock(side_effect=UserAbort("startup stopped")))

    def test_non_increasing_timestamp_error_contains_diagnostics(self):
        rclpy = mock.Mock()

        class Node:
            def __init__(self, name):
                pass

            def create_subscription(self, *args):
                return object()

        modules = {
            "rclpy": rclpy,
            "rclpy.node": SimpleNamespace(Node=Node),
            "rclpy.signals": SimpleNamespace(
                SignalHandlerOptions=SimpleNamespace(NO=object())
            ),
            "rclpy.qos": SimpleNamespace(
                DurabilityPolicy=SimpleNamespace(VOLATILE=1),
                HistoryPolicy=SimpleNamespace(KEEP_LAST=1),
                ReliabilityPolicy=SimpleNamespace(BEST_EFFORT=1),
                QoSProfile=mock.Mock(),
            ),
            "sensor_msgs": mock.Mock(),
            "sensor_msgs.msg": SimpleNamespace(Imu=object),
        }
        timestamp_error = mock.Mock()
        subscriber = ImuSubscriber(
            build_capture_config(mode="stereo-imu"),
            timestamp_error_callback=timestamp_error,
        )
        with mock.patch.dict(sys.modules, modules), mock.patch.object(capture.threading, "Thread"):
            subscriber.start()
            vector = SimpleNamespace(x=1.0, y=2.0, z=3.0, w=1.0)

            def message(nanosec):
                return SimpleNamespace(
                    header=SimpleNamespace(
                        stamp=SimpleNamespace(sec=4, nanosec=nanosec),
                        frame_id="imu_link",
                    ),
                    orientation=vector,
                    orientation_covariance=(0,) * 9,
                    angular_velocity=vector,
                    angular_velocity_covariance=(0,) * 9,
                    linear_acceleration=vector,
                    linear_acceleration_covariance=(0,) * 9,
                )

            subscriber._node.on_imu(message(20))
            subscriber._node.on_imu(message(10))

        with self.assertRaisesRegex(
            RuntimeError,
            r"previous=4000000020, current=4000000010, delta_ns=-10, "
            r"accepted_count=1.*angular_velocity=\(1\.0, 2\.0, 3\.0\)",
        ):
            subscriber.raise_if_failed()
        timestamp_error.assert_called_once()
        previous, current = timestamp_error.call_args.args
        self.assertEqual(previous.timestamp_ns, 4_000_000_020)
        self.assertEqual(current.timestamp_ns, 4_000_000_010)

    def test_unexpected_ros_shutdown_is_an_error(self):
        subscriber = ImuSubscriber(build_capture_config(mode="stereo-imu"))
        subscriber._node = object()
        subscriber._rclpy = SimpleNamespace(ok=lambda: False)
        subscriber._spin()
        with self.assertRaisesRegex(RuntimeError, "context stopped"):
            subscriber.raise_if_failed()


class FinishTests(unittest.TestCase):
    def test_interrupted_finish_with_full_queue_can_be_retried(self):
        entered = threading.Event()
        release = threading.Event()

        class Dataset:
            config = SimpleNamespace(image_workers=2, width=1, height=1)

            def __init__(self):
                self.committed = []

            def raise_if_failed(self):
                pass

            def start_pair(self, pair):
                return SimpleNamespace(pair=pair, done=lambda: True)

            def finish_pair(self, job):
                entered.set()
                if not release.wait(3):
                    raise RuntimeError("test release timeout")
                self.committed.append(job.pair)

        dataset = Dataset()
        writer = PairDiskWriter(dataset, 1)
        try:
            first = writer.reserve()
            self.assertTrue(writer.submit(1, first))
            self.assertTrue(entered.wait(1))
            second = writer.reserve()
            self.assertTrue(writer.submit(2, second))
            self.assertTrue(writer.full())
            with self.assertRaisesRegex(UserAbort, "cancel wait"):
                writer.finish(mock.Mock(side_effect=UserAbort("cancel wait")))
            self.assertEqual(writer.pool.summary()["in_use_pairs"], 2)
        finally:
            release.set()
            writer.finish()
        self.assertEqual(dataset.committed, [1, 2])
        self.assertEqual(writer.pool.summary()["in_use_pairs"], 0)
        writer.finish()


class CaptureFlowTests(unittest.TestCase):
    """Exercise the full lifecycle without ROS, image encoding or SDK calls."""

    def run_capture(self, *, interrupt=None, drain_s=0.2, writer_error=False):
        clock = FakeClock()
        interrupts = _CaptureInterrupts()
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        built = build_capture_config(mode="stereo-imu", overrides={
            "duration_s": 0.6, "imu_preroll_s": 1.0, "imu_postroll_s": 1.0,
            "startup_timeout_s": 0.3,
        })
        config = replace(built, width=5, height=3, output=str(Path(temporary.name) / "dataset"))
        state = SimpleNamespace(events=[], frames=[], copies=0, failed_session=None,
                                saved_session=None, finish_calls=0)

        class Dataset(DatasetWriter):
            def abandon(self, exc, session):
                state.failed_session = session
                state.events.append("abandon")
                return super().abandon(exc, session)

        dataset = Dataset(config.output, config, True)
        self.addCleanup(dataset._close_streams)

        class Imu:
            first_timestamp_ns = 10 * NS
            last_timestamp_ns = 10 * NS
            running = False

            def __init__(self, config, callback):
                self.callback = callback

            def start(self):
                self.running = True
                self.save()
                state.events.append("imu-start")
                clock.on_advance = self.tick

            def tick(self):
                if self.running:
                    self.last_timestamp_ns = 10 * NS + round(clock.now * NS)
                    self.save()
                if interrupt == "preroll" and not state.copies:
                    interrupts._on_signal(signal.SIGINT, None)

            def save(self):
                self.callback(ImuRecord(
                    self.last_timestamp_ns, "imu_link", (0., 0., 0., 1.), (0.,) * 9,
                    (0., 0., 0.), (0.,) * 9, (0., 0., 9.81), (0.,) * 9,
                    self.last_timestamp_ns,
                ))
                if interrupt == "postroll" and "drain" in state.events:
                    interrupts._on_signal(signal.SIGINT, None)

            def wait_first(self, timeout, check):
                check()
                return self.first_timestamp_ns

            def raise_if_failed(self):
                pass

            def stop(self):
                self.running = False
                state.events.append("imu-stop")

        imu = Imu(config, dataset.write_imu)

        class Grabber:
            stats = {}
            timings = SimpleNamespace(summary=lambda: {})

            def __init__(self):
                from calib_capture.axera import AcquisitionStats
                self.acquisition_stats = AcquisitionStats()
                self.sequence_window = None

            def start_sequence_window(self):
                from calib_capture.axera import AcquisitionStats
                self.sequence_window = AcquisitionStats()

            def grab_frames(self):
                clock.sleep(0.1)
                if interrupt == "capture_get_error" and state.copies:
                    interrupts._on_signal(signal.SIGINT, None)
                    raise VinGetFrameError("left", 2, 0, -2146369241)
                if interrupt in {"capture", "second"} and state.copies:
                    interrupts._on_signal(signal.SIGINT, None)
                pts_us = (10 * NS + round(clock.now * NS)) // 1000
                frame = SimpleNamespace(
                    left=SimpleNamespace(u64PTS=pts_us, u64SeqNum=len(state.frames) + 1),
                    right=SimpleNamespace(u64PTS=pts_us + 20, u64SeqNum=len(state.frames) + 1),
                    released=False)
                state.frames.append(frame)
                for side in ("left", "right"):
                    seq = getattr(frame, side).u64SeqNum
                    self.acquisition_stats.acquired(side, seq)
                    if self.sequence_window is not None:
                        self.sequence_window.acquired(side, seq)
                return frame

            def release_frames(self, frame):
                if frame.released:
                    raise AssertionError("double frame release")
                frame.released = True

            def copy_gray_pair(self, frame, **kwargs):
                state.copies += 1
                self.release_frames(frame)
                left, right = kwargs["buffers"]
                left[:], right[:] = bytes(range(15)), bytes(range(15))
                return GrayPair(
                    left_timestamp_ns=frame.left.u64PTS * 1000,
                    right_timestamp_ns=frame.right.u64PTS * 1000,
                    left_sequence=frame.left.u64SeqNum, right_sequence=frame.right.u64SeqNum,
                    width=5, height=3, left_pixels=left, right_pixels=right,
                    left_format=3, right_format=3, bottom_up=True,
                )

        class Writer(PairDiskWriter):
            def finish(self, check=None):
                state.finish_calls += 1
                if check is not None:
                    if not imu.running:
                        raise AssertionError("IMU must keep running during writer drain")
                    if interrupt == "second":
                        interrupts._on_signal(signal.SIGINT, None)
                    clock.sleep(drain_s)
                    check()
                state.events.append("drain")
                super().finish(check)
                if writer_error:
                    raise RuntimeError("writer failed during drain")

        api = SimpleNamespace(close=mock.Mock(return_value=0), library_dir="fake",
                              header_contract={}, ax_errors=[])

        original_finish = capture._finish_dataset
        original_validate = capture.validate_dataset

        def validate(*args, **kwargs):
            report = original_validate(*args, **kwargs)
            if interrupt == "validation":
                interrupts._on_signal(signal.SIGINT, None)
                interrupts._on_signal(signal.SIGINT, None)
            return report

        def finish(dataset_arg, session, **kwargs):
            state.events.append("finish")
            result = original_finish(dataset_arg, session, **kwargs)
            state.saved_session = session
            state.saved_report = result[1]
            return result

        patches = {
            "DatasetWriter": mock.Mock(return_value=dataset),
            "AxVinApi": mock.Mock(return_value=api),
            "ImuSubscriber": mock.Mock(return_value=imu),
            "_make_grabber": mock.Mock(return_value=Grabber()),
            "PairDiskWriter": Writer,
            "_warmup_pair": mock.Mock(return_value=({
                "left": {"timestamp_us": 10_000_000},
                "right": {"timestamp_us": 10_000_020}}, 10 * NS + 10_000)),
            "_finish_dataset": mock.Mock(side_effect=finish),
            "validate_dataset": mock.Mock(side_effect=validate),
        }
        with mock.patch.multiple(capture, **patches), \
                mock.patch.object(capture.time, "monotonic", clock.monotonic), \
                mock.patch.object(capture.time, "sleep", clock.sleep), \
                mock.patch("builtins.print"):
            try:
                state.result = capture._record_continuous(
                    config, include_imu=True, interrupts=interrupts)
            except CaptureFailed as exc:
                state.error = exc
        self.assertTrue(all(frame.released for frame in state.frames))
        self.assertFalse(imu.running)
        self.assertGreaterEqual(api.close.call_count, 1)
        state.dataset, state.imu, state.clock = dataset, imu, clock
        return state

    def test_camera_duration_excludes_padding_and_imu_runs_through_drain(self):
        state = self.run_capture()
        self.assertIsNotNone(state.saved_session)
        self.assertEqual(state.saved_report.errors, [])
        self.assertGreaterEqual(state.dataset.first_camera_timestamp_ns - state.imu.first_timestamp_ns, NS)
        self.assertGreaterEqual(state.imu.last_timestamp_ns - state.dataset.last_camera_timestamp_ns, NS)
        stats = state.saved_session["capture_stats"]
        self.assertLessEqual(stats["capture_wall_s"], 0.7)
        self.assertGreaterEqual(stats["capture_wall_s"], 0.6)
        self.assertAlmostEqual(stats["imu_preroll_wall_s"], 1.0)
        self.assertGreater(stats["imu_postroll_wait_wall_s"], 0)
        self.assertLess(state.events.index("drain"), state.events.index("imu-stop"))
        self.assertGreater(len(state.frames), state.copies)
        self.assertEqual(stats["source_sequence_accounting"], "successful_vin_get_v1")
        self.assertEqual(stats["source_left_acquired_frames"], stats["source_pairs"])
        self.assertGreater(state.saved_session["acquisition_lifetime_stats"]["source_left_acquired_frames"],
                           stats["source_left_acquired_frames"])

    def test_long_drain_already_covers_postroll_without_another_second_wait(self):
        state = self.run_capture(drain_s=2.0)
        self.assertIsNotNone(state.saved_session)
        self.assertEqual(state.saved_session["capture_stats"]["imu_postroll_wait_wall_s"], 0)

    def test_first_ctrl_c_in_camera_window_still_finishes_postroll(self):
        state = self.run_capture(interrupt="capture")
        self.assertIsNotNone(state.saved_session)
        self.assertEqual(state.saved_session["capture_status"], "operator_stopped")
        self.assertTrue(state.saved_session["capture_stats"]["operator_interrupted"])
        self.assertGreaterEqual(state.imu.last_timestamp_ns - state.dataset.last_camera_timestamp_ns, NS)

    def test_first_ctrl_c_during_blocking_vin_get_still_finishes_postroll(self):
        state = self.run_capture(interrupt="capture_get_error")
        self.assertIsNotNone(state.saved_session)
        self.assertEqual(state.saved_session["capture_status"], "operator_stopped")
        self.assertEqual(
            state.saved_session["capture_stats"]["operator_stop_get_error"],
            {
                "call": "AX_VIN_GetYuvFrame",
                "side": "left",
                "pipe": 2,
                "chn": 0,
                "result": -2146369241,
                "result_hex": "0x80110127",
            },
        )

    def test_vin_get_error_without_sigint_remains_fatal(self):
        class FailingGrabber:
            def grab_frames(self):
                raise VinGetFrameError("left", 2, 0, -2146369241)

        with self.assertRaises(VinGetFrameError):
            _grab_frames_or_stop(FailingGrabber(), _CaptureInterrupts(), {})

    def test_ctrl_c_before_any_camera_saves_failed_dataset(self):
        state = self.run_capture(interrupt="preroll")
        self.assertIsNotNone(state.failed_session)
        self.assertEqual(state.copies, 0)
        self.assertEqual(state.finish_calls, 0)

    def test_second_ctrl_c_cancels_padding_but_retries_safe_writer_cleanup(self):
        state = self.run_capture(interrupt="second")
        self.assertIsNotNone(state.failed_session)
        self.assertEqual(state.finish_calls, 2)
        self.assertIn("second Ctrl", str(state.error))

    def test_postroll_second_interrupt_fails_without_discarding_saved_pairs(self):
        state = self.run_capture(interrupt="postroll")
        self.assertIsNotNone(state.failed_session)
        self.assertGreater(state.dataset.pair_count, 0)
        self.assertEqual(state.finish_calls, 1)

    def test_writer_failure_while_imu_runs_is_a_failed_capture(self):
        state = self.run_capture(writer_error=True)
        self.assertIsNotNone(state.failed_session)
        self.assertIn("writer failed", str(state.error))
        self.assertEqual(state.finish_calls, 2)

    def test_second_ctrl_c_during_validation_prevents_successful_commit(self):
        state = self.run_capture(interrupt="validation")
        self.assertIsNone(state.saved_session)
        self.assertIsNotNone(state.failed_session)
        self.assertIn("second Ctrl", str(state.error))
        self.assertFalse(state.dataset.final_path.exists())
        self.assertTrue(Path(str(state.dataset.final_path) + ".failed").is_dir())


if __name__ == "__main__":
    unittest.main()
