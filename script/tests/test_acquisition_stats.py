"""VIN acquisition gaps must not include frames deliberately discarded later."""

import json
from pathlib import Path
import tempfile
import unittest

from calib_capture.capture import RateLimiter, _capture_session
from calib_capture.config import build_capture_config
from calib_capture.validator import validate_dataset
from test_calibration_data import FakeVinApi, create_dataset, make_frame, make_grabber


class AcquisitionStatsTests(unittest.TestCase):
    def test_resync_counts_discarded_frames_not_source_gaps(self):
        api = FakeVinApi({
            2: [make_frame(1000, 1), make_frame(21000, 2), make_frame(41000, 3)],
            0: [make_frame(1000, 1), make_frame(21201, 2), make_frame(41000, 3)],
        })
        grabber = make_grabber(api, 200)
        grabber.start_sequence_window()
        for _ in range(2):
            grabber.release_frames(grabber.grab_frames())
        stats = grabber.sequence_window.summary()
        self.assertEqual(stats["source_left_sequence_gaps"], 0)
        self.assertEqual(stats["source_right_sequence_gaps"], 0)
        self.assertEqual(stats["sync_left_discarded_frames"], 1)
        self.assertEqual(stats["sync_right_discarded_frames"], 1)
        self.assertEqual(stats["source_left_acquired_frames"], 3)
        self.assertEqual(stats["source_right_acquired_frames"], 3)
        self.assertCountEqual(api.releases, [(pipe, seq) for pipe in (2, 0) for seq in (1, 2, 3)])

    def test_window_excludes_warmup_resync_and_transition_gap(self):
        api = FakeVinApi({
            2: [make_frame(1000, 1), make_frame(2000, 2),
                make_frame(9000, 9), make_frame(11000, 11)],
            0: [make_frame(2000, 1), make_frame(9000, 8), make_frame(11000, 9)],
        })
        grabber = make_grabber(api)
        grabber.release_frames(grabber.grab_frames())  # Warmup discards left seq 1.
        grabber.start_sequence_window()
        for _ in range(2):
            grabber.release_frames(grabber.grab_frames())
        stats = grabber.sequence_window.summary()
        self.assertEqual(stats["sync_discarded_frames"], 0)
        self.assertEqual(stats["source_left_sequence_gaps"], 1)
        self.assertEqual(stats["source_right_sequence_gaps"], 0)
        self.assertEqual(grabber.stats["sync_retries"], 1)
        self.assertEqual(grabber.acquisition_stats.summary()["sync_discarded_frames"], 1)

    def test_get_error_does_not_count_as_a_frame_or_advance_baseline(self):
        api = FakeVinApi({
            2: [make_frame(1000, 1), make_frame(2000, 2), make_frame(3000, 3)],
            0: [make_frame(1000, 1), -1, make_frame(3000, 3)],
        })
        grabber = make_grabber(api)
        grabber.start_sequence_window()
        grabber.release_frames(grabber.grab_frames())
        with self.assertRaises(RuntimeError):
            grabber.grab_frames()
        grabber.release_frames(grabber.grab_frames())
        stats = grabber.sequence_window.summary()
        self.assertEqual(stats["source_left_sequence_gaps"], 0)
        self.assertEqual(stats["source_right_sequence_gaps"], 1)
        self.assertEqual(stats["source_right_acquired_frames"], 2)
        self.assertEqual(stats["sync_discarded_frames"], 0)
        self.assertEqual(len(api.releases), 5)
        self.assertEqual(len(set(api.releases)), 5)

    def test_failed_resync_keeps_successful_get_stats_in_failed_session(self):
        api = FakeVinApi({2: [make_frame(1000, 1), -1], 0: [make_frame(2000, 1)]})
        grabber = make_grabber(api)
        grabber.start_sequence_window()
        with self.assertRaises(RuntimeError):
            grabber.grab_frames()
        session = _capture_session(build_capture_config(mode="stereo-continuous"),
                                   None, grabber, {}, {}, None, failed=True)
        stats = session["capture_stats"]
        self.assertEqual(stats["sync_left_discarded_frames"], 1)
        self.assertEqual(stats["source_left_acquired_frames"], 1)
        self.assertEqual(stats["source_left_sequence_gaps"], 0)
        self.assertCountEqual(api.releases, [(2, 1), (0, 1)])

    def test_duplicates_and_regressions_are_not_positive_gaps(self):
        api = FakeVinApi({pipe: [make_frame(i * 1000, seq)
                                for i, seq in enumerate((5, 5, 3, 4), 1)]
                          for pipe in (2, 0)})
        grabber = make_grabber(api)
        grabber.start_sequence_window()
        for _ in range(4):
            grabber.release_frames(grabber.grab_frames())
        stats = grabber.sequence_window.summary()
        self.assertEqual(stats["source_sequence_regressions"], 4)
        self.assertEqual(stats["source_left_sequence_gaps"], 0)
        self.assertEqual(stats["source_right_sequence_gaps"], 0)

    def test_5_8_10_hz_decimation_does_not_change_acquisition_gaps(self):
        for hz in (5, 8, 10):
            with self.subTest(hz=hz):
                api = FakeVinApi({pipe: [make_frame(1000 + i * 20000, i + 1)
                                        for i in range(50)] for pipe in (2, 0)})
                grabber = make_grabber(api)
                grabber.start_sequence_window()
                limiter = RateLimiter(hz)
                saved = 0
                for _ in range(50):
                    frames = grabber.grab_frames()
                    saved += limiter.accept(int(frames.left.u64PTS) * 1000)
                    grabber.release_frames(frames)
                self.assertEqual(saved, hz)
                stats = grabber.sequence_window.summary()
                self.assertEqual(stats["source_left_sequence_gaps"], 0)
                self.assertEqual(stats["source_right_sequence_gaps"], 0)
                self.assertEqual(stats["sync_discarded_frames"], 0)
                self.assertEqual(stats["source_left_acquired_frames"], 50)

    def test_validator_reports_independent_counts_and_labels_legacy(self):
        with tempfile.TemporaryDirectory() as raw:
            dataset = create_dataset(Path(raw), mode="stereo-continuous")
            path = dataset / "meta/session.json"
            session = json.loads(path.read_text(encoding="utf-8"))
            session["grabber_stats"] = {"sync_retries": 999}  # Includes warmup.
            session["capture_stats"] = {
                "source_sequence_accounting": "successful_vin_get_v1",
                "source_left_sequence_gaps": 2, "source_right_sequence_gaps": 3,
                "sync_left_discarded_frames": 4, "sync_right_discarded_frames": 6,
                "rate_limiter_skipped_pairs": 40, "frame_divisor_skipped_pairs": 0,
                "writer_queue_drop_pairs": 7,
            }
            path.write_text(json.dumps(session), encoding="utf-8")
            report = validate_dataset(dataset)
            self.assertFalse(report.errors)
            self.assertEqual(report.stats["camera_source_sequence_gap_count"], 5)
            self.assertEqual(report.stats["sync_discarded_frames"], 10)
            self.assertEqual(report.stats["rate_limiter_skipped_pairs"], 40)
            self.assertEqual(report.stats["writer_queue_drop_pairs"], 7)
            del session["capture_stats"]["source_sequence_accounting"]
            path.write_text(json.dumps(session), encoding="utf-8")
            legacy = validate_dataset(dataset)
            self.assertFalse(legacy.errors)
            self.assertIsNone(legacy.stats["sync_discarded_frames"])
            self.assertTrue(any("may include synchronization discards" in w for w in legacy.warnings))


if __name__ == "__main__":
    unittest.main()
