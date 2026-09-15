import csv
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock

from calib_capture.allan import analyze_allan, timing_summary, select_time_window, analysis_directory_name
from calib_capture.cli import build_parser
import test_imu_only as imu_fixture
from dataclasses import replace

AVAILABLE = all(importlib.util.find_spec(x) for x in ("numpy", "matplotlib"))
if AVAILABLE:
    # Load optional modules before patch.dict snapshots sys.modules.
    import numpy
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    from matplotlib.figure import Figure


def direct_oadev(values, rate, data_type, taus):
    """Small independent moving-average reference, only used as a test double."""
    import numpy as np
    assert data_type == "freq"
    averages = np.convolve(values, np.ones(2) / 2, mode="valid")
    differences = averages[2:] - averages[:-2]
    adev = np.sqrt(np.mean(differences**2) / 2)
    n = len(differences)
    return np.array([2 / rate]), np.array([adev]), np.array([adev / np.sqrt(n)]), np.array([n])


@unittest.skipUnless(AVAILABLE, "Allan tests require optional numpy and matplotlib")
class AllanTests(unittest.TestCase):
    def test_interval_directory_names(self):
        self.assertEqual(analysis_directory_name("imu"), "imu")
        self.assertEqual(analysis_directory_name("imu", 3600, 86400), "imu_1h-24h")
        self.assertEqual(analysis_directory_name("imu", 5400, 18540), "imu_1.5h-5.2h")
        self.assertEqual(analysis_directory_name("imu", None, 18000), "imu_0h-5h")
        self.assertEqual(analysis_directory_name("imu", 7200), "imu_2h-end")
        self.assertEqual(analysis_directory_name("imu", 0), "imu_0h-end")

    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.output_root = Path(directory.name) / "allantools_output"
        patcher = mock.patch("calib_capture.allan.DEFAULT_OUTPUT_ROOT", self.output_root)
        patcher.start()
        self.addCleanup(patcher.stop)

    def make_dataset(self, root):
        records = [replace(imu_fixture.sample(10**16 + i * 10_000_000),
                           angular_velocity=(0.001 * (i % 3), -0.002 * (i % 4), 0.0))
                   for i in range(100)]
        return imu_fixture.ImuOnlyTests().run_capture(root / "session", records=records)[0]

    def test_timing_keeps_integer_precision_and_rate(self):
        import numpy as np
        stamps = np.array([10**16 + i * 10_000_001 for i in range(100)], dtype=np.int64)
        result = timing_summary(stamps)
        self.assertAlmostEqual(result["measured_rate_hz"], 1e9 / 10_000_001)
        self.assertEqual(result["intervals_outside_10_percent"], 0)
        for rate in (0, -1, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                timing_summary(stamps, rate)
        with self.assertRaises(ValueError):
            timing_summary(np.array([3, 2, 4, 5]))

    def test_files_units_variance_zero_values_and_no_source_mutation(self):
        with tempfile.TemporaryDirectory() as raw:
            source = self.make_dataset(Path(raw))
            before = {str(p.relative_to(source)): p.read_bytes() for p in source.rglob("*") if p.is_file()}
            with mock.patch.dict(sys.modules, {"allantools": SimpleNamespace(oadev=direct_oadev)}):
                report = analyze_allan(source)
            output = Path(report["output"])
            self.assertEqual(output, self.output_root / source.name)
            self.assertEqual(report["timing"]["samples"], 100)
            self.assertEqual(report["temperature"]["mean"], 39.)
            self.assertEqual(report["axes"]["wz"]["zero_count"], 100)
            for axis in ("wx", "wy", "wz", "ax", "ay", "az"):
                with (output / f"allan_{axis}.csv").open() as stream:
                    row = next(csv.DictReader(stream))
                self.assertAlmostEqual(float(row["avar"]), float(row["adev"])**2)
                self.assertEqual(int(row["n"]), 97)
            for name in ("allan_deviation.png", "allan_variance.png"):
                self.assertEqual((output / name).read_bytes()[:8], b'\x89PNG\r\n\x1a\n')
            self.assertEqual(json.loads((output / "analysis.json").read_text())["kind"], "imu_allan_analysis")
            after = {str(p.relative_to(source)): p.read_bytes() for p in source.rglob("*") if p.is_file()}
            self.assertEqual(before, after)
            with mock.patch.dict(sys.modules, {"allantools": SimpleNamespace(oadev=direct_oadev)}):
                with self.assertRaises(FileExistsError):
                    analyze_allan(source)
                with self.assertRaises(ValueError):
                    analyze_allan(source, output=source / "analysis")

    def test_irregular_requires_explicit_acknowledgment(self):
        with tempfile.TemporaryDirectory() as raw:
            source = self.make_dataset(Path(raw))
            with mock.patch.dict(sys.modules, {"allantools": SimpleNamespace(oadev=direct_oadev)}):
                with self.assertRaisesRegex(ValueError, "sampling intervals"):
                    analyze_allan(source, rate_hz=200)
                report = analyze_allan(source, rate_hz=200, allow_irregular=True)
            self.assertTrue(any("approximate" in w for w in report["warnings"]))
            self.assertFalse(report["resampled"])

    def test_failure_preserves_partial_output(self):
        with tempfile.TemporaryDirectory() as raw:
            source = self.make_dataset(Path(raw))
            fake = SimpleNamespace(oadev=mock.Mock(side_effect=RuntimeError("calculation failed")))
            with mock.patch.dict(sys.modules, {"allantools": fake}), self.assertRaises(RuntimeError):
                analyze_allan(source)
            self.assertTrue((self.output_root / (source.name + ".failed")).is_dir())

    def test_explicit_output_overrides_default(self):
        with tempfile.TemporaryDirectory() as raw:
            source = self.make_dataset(Path(raw))
            destination = Path(raw) / "custom"
            with mock.patch.dict(sys.modules, {"allantools": SimpleNamespace(oadev=direct_oadev)}):
                report = analyze_allan(source, output=destination)
            self.assertEqual(Path(report["output"]), destination.resolve())
            self.assertFalse(self.output_root.exists())

    def test_missing_allantools_has_clear_error(self):
        with mock.patch.dict(sys.modules, {"allantools": None}), self.assertRaisesRegex(RuntimeError, "needs allantools"):
            analyze_allan("unused")

    @unittest.skipUnless(importlib.util.find_spec("allantools"), "optional allantools not installed")
    def test_real_allantools_against_moving_average_reference(self):
        import allantools
        import numpy as np
        values = np.random.default_rng(42).normal(size=1000)
        actual = allantools.oadev(values, rate=100, data_type="freq", taus=[0.02])
        expected = direct_oadev(values, 100, "freq", "decade")
        for a, b in zip(actual, expected):
            np.testing.assert_allclose(a, b, rtol=1e-9)

    def test_cli(self):
        args = build_parser().parse_args(["allan", "example", "--taus", "octave", "--start-s", "600", "--end-s", "3600"])
        self.assertEqual(args.taus, "octave")
        self.assertEqual((args.start_s, args.end_s), (600, 3600))
        self.assertIsNone(args.rate_hz)

    def test_time_window_boundaries(self):
        import numpy as np
        data = np.array([(10**16 + i * 10_000_000,) for i in range(100)], dtype=[("timestamp_ns", "i8")])
        selected, info = select_time_window(data, 0.1, 0.2)
        np.testing.assert_array_equal(selected, data[10:20])
        self.assertEqual(info["excluded_before"], 10)
        self.assertEqual(info["excluded_after"], 80)
        np.testing.assert_array_equal(select_time_window(data)[0], data)
        np.testing.assert_array_equal(select_time_window(data, 0.9, 100)[0], data[90:])
        np.testing.assert_array_equal(select_time_window(data, 0.100000001, 0.200000001)[0], data[11:21])
        for start, end in [(-1, None), (float("nan"), None), (0, float("inf")), (1, 1), (1, 0), (100, None), (0, 0.02)]:
            with self.assertRaises(ValueError):
                select_time_window(data, start, end)

    def test_analysis_uses_selected_interval(self):
        with tempfile.TemporaryDirectory() as raw:
            source = self.make_dataset(Path(raw))
            engine = mock.Mock(side_effect=direct_oadev)
            with mock.patch.dict(sys.modules, {"allantools": SimpleNamespace(oadev=engine)}):
                report = analyze_allan(source, start_s=0.1, end_s=0.5)
            self.assertEqual(report["timing"]["samples"], 40)
            self.assertEqual(Path(report["output"]).name, source.name + "_0.0h-0.0h")
            self.assertEqual(report["selection"]["source_samples"], 100)
            self.assertEqual(report["selection"]["first_timestamp_ns"], 10**16 + 100_000_000)
            self.assertTrue(all(len(call.args[0]) == 40 for call in engine.call_args_list))

    def test_incomplete_fit_preserves_analysis_without_yaml(self):
        with tempfile.TemporaryDirectory() as raw:
            source = self.make_dataset(Path(raw))
            with mock.patch.dict(sys.modules, {"allantools": SimpleNamespace(oadev=direct_oadev)}):
                report = analyze_allan(source, kalibr_noise=True)
            self.assertEqual(report["noise_fit"]["status"], "incomplete")
            output = Path(report["output"])
            self.assertFalse((output / "kalibr_imu.yaml").exists())
            self.assertTrue((output / "analysis.json").exists())

    def test_complete_fit_writes_yaml(self):
        import numpy as np
        def engine(values, **kwargs):
            tau = np.logspace(-5, -1.1, 80)
            adev = np.sqrt(0.001**2 / tau + tau / 3)
            return tau, adev, adev / 10, np.full(len(tau), 100)
        with tempfile.TemporaryDirectory() as raw:
            source = self.make_dataset(Path(raw))
            with mock.patch.dict(sys.modules, {"allantools": SimpleNamespace(oadev=engine)}):
                report = analyze_allan(source, kalibr_noise=True, kalibr_imu_topic="/custom/imu")
            self.assertEqual(report["noise_fit"]["status"], "candidate")
            output = Path(report["output"])
            self.assertIn('/custom/imu', (output / "kalibr_imu.yaml").read_text())
            self.assertEqual(report["noise_fit"]["yaml_file"], "kalibr_imu.yaml")

    def test_incomplete_cli_exit_code(self):
        from calib_capture.cli import main
        with mock.patch("calib_capture.allan.analyze_allan", return_value={"noise_fit": {"status": "incomplete"}}), mock.patch("builtins.print"):
            self.assertEqual(main(["allan", "example", "--kalibr-noise"]), 1)


if __name__ == "__main__":
    unittest.main()
