import csv
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest

from imu.allan.input_format import COLUMNS, resolve_dataset, time_bounds, time_suffix, validate_input


NUMPY_AVAILABLE = importlib.util.find_spec("numpy") is not None
SCIPY_AVAILABLE = importlib.util.find_spec("scipy") is not None


class InputFormatTests(unittest.TestCase):
    def write_dataset(self, root, rows):
        dataset = root / "imu_test"
        imu = dataset / "imu"
        imu.mkdir(parents=True, exist_ok=True)
        with (imu / "imu0.csv").open("w", encoding="utf-8", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(COLUMNS)
            writer.writerows(rows)
        return dataset

    def test_time_bounds_and_names(self):
        args = SimpleNamespace(start_s=None, start_h="1", end_s=None, end_h="23")
        self.assertEqual(time_bounds(args), (3_600_000_000_000, 82_800_000_000_000))
        self.assertEqual(time_suffix(*time_bounds(args)), "_1h-23h")
        args = SimpleNamespace(start_s="5400", start_h=None, end_s="18720", end_h=None)
        self.assertEqual(time_suffix(*time_bounds(args)), "_1.5h-5.2h")

    def test_complete_source_validation_and_half_open_selection(self):
        with tempfile.TemporaryDirectory() as raw:
            root = Path(raw)
            rows = [
                [1_000_000_000 + index * 10_000_000, 1, 2, 3, 4, 5, 6, 30]
                for index in range(6)
            ]
            dataset = self.write_dataset(root, rows)
            resolved, csv_path = resolve_dataset(dataset)
            result = validate_input(resolved, csv_path, 10_000_000, 40_000_000)
            self.assertEqual(result["messages"], 3)
            self.assertEqual(result["selection"]["excluded_before"], 1)
            self.assertEqual(result["selection"]["excluded_after"], 2)
            self.assertEqual(result["effective_rate_hz"], 100.0)

            rows[-1][0] = rows[-2][0]
            self.write_dataset(root, rows)
            with self.assertRaisesRegex(ValueError, "non-increasing"):
                validate_input(resolved, csv_path, 10_000_000, 40_000_000)


@unittest.skipUnless(NUMPY_AVAILABLE, "requires NumPy")
class CoreTests(unittest.TestCase):
    def test_upstream_divisible_boundary_omits_last_full_bin(self):
        import numpy as np
        from imu.allan.allan_core import _deviation

        values = np.arange(20, dtype=float)
        prefix = np.concatenate(([0.0], np.cumsum(values)))
        # N=20,m=5 gives floor((N-1)/m)=3 bins: [0:5], [5:10], [10:15].
        expected_means = np.array([2.0, 7.0, 12.0])
        expected = np.sqrt(np.mean(np.diff(expected_means) ** 2) / 2)
        self.assertAlmostEqual(_deviation(prefix, 20, 5), expected)


@unittest.skipUnless(NUMPY_AVAILABLE and SCIPY_AVAILABLE, "requires NumPy and SciPy")
class FittingTests(unittest.TestCase):
    def test_fixed_slope_fits_and_axis_maximum(self):
        import numpy as np
        from imu.allan.upstream_analysis import _fit_group

        tau = np.arange(1, 10000, dtype=float) * 0.1
        values = np.column_stack(
            [
                0.01 / np.sqrt(tau) + 0.0001 * np.sqrt(tau / 3),
                0.02 / np.sqrt(tau) + 0.0002 * np.sqrt(tau / 3),
                0.03 / np.sqrt(tau) + 0.0003 * np.sqrt(tau / 3),
            ]
        )
        result = _fit_group(tau, values)
        self.assertEqual(len(result["details"]), 3)
        self.assertGreater(result["white"][2], result["white"][0])
        self.assertGreater(result["walk"][2], result["walk"][0])
        for detail in result["details"]:
            self.assertGreaterEqual(detail["white_fit_slope"], -0.5)
            self.assertLessEqual(detail["white_fit_slope"], -0.499)
            self.assertGreaterEqual(detail["random_walk_fit_slope"], 0.5)
            self.assertLessEqual(detail["random_walk_fit_slope"], 0.501)


if __name__ == "__main__":
    unittest.main()
