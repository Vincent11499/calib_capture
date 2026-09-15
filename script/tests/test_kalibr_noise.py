import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

from calib_capture.kalibr_noise import AXES, fit_region, fit_kalibr_noise, write_kalibr_yaml


@unittest.skipUnless(importlib.util.find_spec("numpy"), "optional numpy not installed")
class KalibrNoiseTests(unittest.TestCase):
    def test_power_law_coefficients(self):
        import numpy as np
        tau = np.logspace(-2, 3, 40)
        for slope, coefficient in ((-0.5, 0.003), (0.5, 0.00004)):
            adev = coefficient * tau**slope / (np.sqrt(3) if slope > 0 else 1)
            fit = fit_region(tau, adev, 10000, slope)
            self.assertEqual(fit["status"], "fitted")
            self.assertAlmostEqual(fit["value"], coefficient, places=12)
            self.assertAlmostEqual(fit["measured_slope"], slope)

    def test_flat_zero_short_and_unsupported_tail_rejected(self):
        import numpy as np
        tau = np.logspace(-2, 3, 40)
        for values in (np.ones(40), np.zeros(40)):
            for slope in (-0.5, 0.5):
                self.assertEqual(fit_region(tau, values, 10000, slope)["status"], "unavailable")
        self.assertEqual(fit_region(tau[:3], tau[:3]**0.5, 10000, 0.5)["status"], "unavailable")
        self.assertEqual(fit_region(tau, tau**0.5, 0.01, 0.5)["status"], "unavailable")

    def test_complete_combined_model_yaml_and_axis_maximum(self):
        import numpy as np
        tau = np.logspace(-3, 4, 100)
        curves = {axis: (tau, np.sqrt((0.001 * (i + 1))**2 / tau +
                                     (0.0001 * (i + 1))**2 * tau / 3))
                  for i, axis in enumerate(AXES)}
        fit = fit_kalibr_noise(curves, 1e6, 100.5)
        self.assertEqual(fit["status"], "candidate")
        self.assertEqual(fit["missing"], [])
        self.assertAlmostEqual(fit["parameters"]["gyroscope_noise_density"], 0.003, delta=0.0003)
        self.assertAlmostEqual(fit["parameters"]["accelerometer_random_walk"], 0.0006, delta=0.00006)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "kalibr_imu.yaml"
            write_kalibr_yaml(path, fit, "/custom/imu")
            rows = dict(line.split(": ", 1) for line in path.read_text().splitlines() if not line.startswith("#"))
            self.assertEqual(len(rows), 6)
            self.assertEqual(json.loads(rows["rostopic"]), "/custom/imu")
            self.assertEqual(float(rows["update_rate"]), 100.5)
            for name, value in fit["parameters"].items():
                self.assertEqual(float(rows[name]), value)
            with self.assertRaises(FileExistsError):
                write_kalibr_yaml(path, fit, "/custom/imu")

    def test_incomplete_never_exports_yaml(self):
        import numpy as np
        tau = np.logspace(-2, 3, 40)
        curves = {axis: (tau, 0.001 / np.sqrt(tau)) for axis in AXES}
        fit = fit_kalibr_noise(curves, 10000, 100)
        self.assertEqual(len(fit["missing"]), 6)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "kalibr_imu.yaml"
            with self.assertRaises(ValueError):
                write_kalibr_yaml(path, fit, "/imu/data")
            self.assertFalse(path.exists())
