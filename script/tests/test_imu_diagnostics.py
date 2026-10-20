from __future__ import annotations

import math
import tempfile
import unittest
from pathlib import Path

try:
    import matplotlib  # noqa: F401
    import numpy as np
    import scipy  # noqa: F401
except ImportError:
    np = None

from calib_capture.imu_diagnostics import (
    AnalysisOutput,
    ImuDiagnosticError,
    extrema_downsample,
    format_seconds,
    interval_histogram,
    load_window,
    noise_window,
    scan_imu_csv,
    timeseries_window,
)

if np is not None:
    from analyze_imu_noise import analyze as analyze_noise, analyze_axis
    from analyze_imu_timeseries import analyze as analyze_timeseries


@unittest.skipUnless(np is not None, "PC diagnostics require numpy, scipy and matplotlib")
class ImuDiagnosticsTests(unittest.TestCase):
    def make_session(self, root: Path, values, rate_hz=100.0) -> Path:
        session = root / "imu_test"
        (session / "imu").mkdir(parents=True)
        csv_path = session / "imu/imu0.csv"
        with csv_path.open("w", encoding="utf-8", newline="") as stream:
            stream.write("timestamp_ns,wx,wy,wz,ax,ay,az,temperature\n")
            for index, row in enumerate(values):
                stamp = 1_000_000_000 + round(index * 1e9 / rate_hz)
                stream.write(
                    str(stamp) + "," + ",".join(str(float(value)) for value in row)
                    + f",{39.0 + index / max(len(values), 1):.9f}\n"
                )
        return session

    def gaussian_rows(self, count=6000, seed=123):
        rng = np.random.default_rng(seed)
        return rng.normal(size=(count, 6))

    def test_gaussian_white_noise_passes_practical_and_statistical_checks(self):
        values = np.random.default_rng(5).normal(size=12000)
        result, _ = analyze_axis(
            values, rate_hz=100.0, max_lag_s=0.5, alpha=0.05, acf_limit=0.05
        )
        self.assertEqual(result["gaussian"]["verdict"], "pass")
        self.assertEqual(result["white_noise"]["verdict"], "pass")
        self.assertEqual(result["overall_verdict"], "pass")

    def test_gaussian_ar1_is_not_white(self):
        rng = np.random.default_rng(11)
        innovations = rng.normal(size=12000)
        values = np.empty_like(innovations)
        values[0] = innovations[0]
        for index in range(1, len(values)):
            values[index] = 0.4 * values[index - 1] + innovations[index]
        result, _ = analyze_axis(
            values, rate_hz=100.0, max_lag_s=0.5, alpha=0.05, acf_limit=0.05
        )
        self.assertNotEqual(result["gaussian"]["verdict"], "fail")
        self.assertEqual(result["white_noise"]["verdict"], "fail")
        self.assertEqual(result["overall_verdict"], "fail")

    def test_iid_laplace_is_not_gaussian_but_is_practically_white(self):
        values = np.random.default_rng(17).laplace(size=12000)
        result, _ = analyze_axis(
            values, rate_hz=100.0, max_lag_s=0.5, alpha=0.05, acf_limit=0.05
        )
        self.assertEqual(result["gaussian"]["verdict"], "fail")
        self.assertNotEqual(result["white_noise"]["verdict"], "fail")

    def test_center_explicit_and_boundary_windows(self):
        with tempfile.TemporaryDirectory() as directory:
            session = self.make_session(
                Path(directory), self.gaussian_rows(count=4001), rate_hz=100.0
            )
            scan = scan_imu_csv(session)
            centered = noise_window(scan, None, None, 10.0)
            self.assertEqual(centered.start_offset_ns, 15_000_000_000)
            self.assertEqual(centered.end_offset_ns, 25_000_000_000)
            selection = load_window(scan, centered, minimum_samples=20)
            self.assertEqual(len(selection.timestamps_ns), 1000)
            explicit = timeseries_window(scan, 5.0, 7.0)
            selected = load_window(scan, explicit)
            self.assertEqual(len(selected.timestamps_ns), 200)
            with self.assertRaisesRegex(ImuDiagnosticError, "cannot be combined"):
                noise_window(scan, 1.0, 2.0, 1.0)

    def test_bad_header_nonfinite_and_timestamp_regression(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            cases = (
                ("bad_header", "time,wx,wy,wz,ax,ay,az,temperature\n1,0,0,0,0,0,0,39\n"),
                ("nan", "timestamp_ns,wx,wy,wz,ax,ay,az,temperature\n1,nan,0,0,0,0,0,39\n2,0,0,0,0,0,0,39\n"),
                ("duplicate", "timestamp_ns,wx,wy,wz,ax,ay,az,temperature\n1,0,0,0,0,0,0,39\n1,0,0,0,0,0,0,39\n"),
                ("regress", "timestamp_ns,wx,wy,wz,ax,ay,az,temperature\n2,0,0,0,0,0,0,39\n1,0,0,0,0,0,0,39\n"),
            )
            for name, content in cases:
                session = root / name
                (session / "imu").mkdir(parents=True)
                (session / "imu/imu0.csv").write_text(content, encoding="utf-8")
                with self.subTest(name=name), self.assertRaises(ImuDiagnosticError):
                    scan_imu_csv(session)

    def test_constant_axis_fails_explicitly(self):
        with self.assertRaisesRegex(ImuDiagnosticError, "at least 20"):
            analyze_axis(
                np.ones(19), rate_hz=100.0, max_lag_s=1.0,
                alpha=0.05, acf_limit=0.05,
            )
        with self.assertRaisesRegex(ImuDiagnosticError, "constant axis"):
            analyze_axis(
                np.ones(1000), rate_hz=100.0, max_lag_s=1.0,
                alpha=0.05, acf_limit=0.05,
            )

    def test_extrema_downsample_keeps_spike_and_labels(self):
        x = np.arange(10000)
        y = np.zeros(10000)
        y[5432] = 99.0
        sampled_x, sampled_y = extrema_downsample(x, y, 200)
        self.assertEqual(sampled_x[0], 0)
        self.assertEqual(sampled_x[-1], 9999)
        self.assertIn(99.0, sampled_y)
        self.assertLessEqual(len(sampled_y), 202)
        self.assertEqual(format_seconds(1800.0), "1800")
        self.assertEqual(format_seconds(1.25), "1.25")

    def test_interval_histogram_is_right_closed_and_reports_ratios(self):
        histogram = interval_histogram(
            np.asarray([7.5, 7.5001, 8.5, 20.0, 20.0001]),
            minimum_ms=7.5,
            maximum_ms=20.0,
            width_ms=1.0,
        )
        self.assertEqual(histogram["bins"][0]["count"], 1)
        self.assertEqual(histogram["bins"][1]["count"], 2)
        self.assertEqual(histogram["bins"][-2]["count"], 1)
        self.assertEqual(histogram["bins"][-1]["count"], 1)
        self.assertAlmostEqual(histogram["above_maximum"]["ratio"], 0.2)
        self.assertAlmostEqual(histogram["at_or_below_minimum"]["ratio"], 0.2)

    def test_output_transaction_preserves_failure_and_refuses_overwrite(self):
        with tempfile.TemporaryDirectory() as directory:
            final = Path(directory) / "result"
            with self.assertRaisesRegex(RuntimeError, "synthetic"):
                with AnalysisOutput(final) as staging:
                    (staging / "partial.txt").write_text("partial", encoding="utf-8")
                    raise RuntimeError("synthetic")
            self.assertTrue(Path(str(final) + ".failed").is_dir())
            with self.assertRaises(FileExistsError):
                with AnalysisOutput(final):
                    pass

    def test_end_to_end_outputs_and_source_is_read_only(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            session = self.make_session(root, self.gaussian_rows(count=6000))
            csv_path = session / "imu/imu0.csv"
            before = csv_path.read_bytes()
            time_output = root / "time_output"
            noise_output = root / "noise_output"
            final_time, time_report = analyze_timeseries(
                session, output=time_output, start_s=0, end_s=20,
                max_plot_points=500, dpi=40,
            )
            final_noise, noise_report = analyze_noise(
                session, output=noise_output, start_s=0, end_s=20,
                max_lag_s=0.2, dpi=40,
            )
            self.assertEqual(final_time, time_output.resolve())
            self.assertEqual(final_noise, noise_output.resolve())
            for filename in (
                "timestamp_intervals.png", "imu_axes_temperature.png",
                "summary.json", "statistics.csv",
            ):
                self.assertGreater((time_output / filename).stat().st_size, 0)
            for filename in (
                "noise_distribution.png", "noise_qq.png", "noise_acf.png",
                "noise_psd.png", "noise_report.json", "axis_verdicts.csv",
            ):
                self.assertGreater((noise_output / filename).stat().st_size, 0)
            self.assertEqual(time_report["selection"]["selected_samples"], 2000)
            self.assertEqual(noise_report["selection"]["selected_samples"], 2000)
            self.assertEqual(csv_path.read_bytes(), before)
            with self.assertRaises(FileExistsError):
                analyze_timeseries(session, output=time_output)


if __name__ == "__main__":
    unittest.main()
