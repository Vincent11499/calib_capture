"""Reproduce the pinned upstream analysis.py fitting, plotting, and YAML output."""

from __future__ import annotations

import math
from pathlib import Path


def line_func(x, slope, intercept):
    return slope * x + intercept


def get_intercept(x, y, slope, evaluation_tau):
    import numpy as np
    from scipy.optimize import curve_fit

    log_x = np.log(x)
    log_y = np.log(y)
    coefficients, _ = curve_fit(
        line_func,
        log_x,
        log_y,
        bounds=([slope, -np.inf], [slope + 0.001, np.inf]),
    )
    polynomial = np.poly1d(coefficients)

    def fitted(values):
        return np.exp(polynomial(np.log(values)))

    return float(fitted(evaluation_tau)), fitted, [float(value) for value in coefficients]


def generate_prediction(
    tau,
    q_quantization=0.0,
    q_white=0.0,
    q_bias_instability=0.0,
    q_walk=0.0,
    q_ramp=0.0,
):
    import numpy as np

    matrix = np.empty((len(tau), 5))
    matrix[:, 0] = 3 / tau**2
    matrix[:, 1] = 1 / tau
    matrix[:, 2] = 2 * np.log(2) / np.pi
    matrix[:, 3] = tau / 3
    matrix[:, 4] = tau**2 / 2
    parameters = np.array(
        [q_quantization**2, q_white**2, q_bias_instability**2, q_walk**2, q_ramp**2]
    )
    return np.sqrt(matrix.dot(parameters))


def _fit_group(period, values):
    import numpy as np

    breakpoint = int(np.where(period == 10)[0][0])
    white = []
    walk = []
    minimums = []
    white_functions = []
    walk_functions = []
    details = []
    for axis in range(3):
        white_value, white_function, white_coefficients = get_intercept(
            period[:breakpoint], values[:breakpoint, axis], -0.5, 1.0
        )
        walk_value, walk_function, walk_coefficients = get_intercept(
            period, values[:, axis], 0.5, 3.0
        )
        minimum_index = int(np.argmin(values[:, axis]))
        white.append(white_value)
        walk.append(walk_value)
        minimums.append((float(values[minimum_index, axis]), minimum_index))
        white_functions.append(white_function)
        walk_functions.append(walk_function)
        details.append(
            {
                "white_noise_at_1s": white_value,
                "white_fit_slope": white_coefficients[0],
                "white_fit_log_intercept": white_coefficients[1],
                "random_walk_at_3s": walk_value,
                "random_walk_fit_slope": walk_coefficients[0],
                "random_walk_fit_log_intercept": walk_coefficients[1],
                "minimum_allan_deviation": minimums[-1][0],
                "minimum_tau_s": float(period[minimum_index]),
            }
        )
    return {
        "white": white,
        "walk": walk,
        "minimums": minimums,
        "white_functions": white_functions,
        "walk_functions": walk_functions,
        "details": details,
    }


def _plot(path: Path, period, values, fit, kind: str) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    figure = plt.figure(num=kind, dpi=90, figsize=(16, 9))
    measured_styles = ("r--", "g--", "b--") if kind == "Accelerometer" else ("r-", "g-", "b-")
    for axis, style in enumerate(measured_styles):
        plt.loglog(period, values[:, axis], style, label=("X", "Y", "Z")[axis])
    for axis in range(3):
        plt.loglog(
            period,
            fit["white_functions"][axis](period),
            "m-",
            label="White noise fit line" if axis == 2 else None,
        )
        plt.loglog(
            period,
            fit["walk_functions"][axis](period),
            "y-",
            label=("Random Rate fit line" if kind == "Accelerometer" else "Random rate fit line")
            if axis == 2
            else None,
        )
    colors = ("r", "g", "b")
    for axis, color in enumerate(colors):
        plt.loglog(1.0, fit["white"][axis], color + "o", markersize=20)
        plt.loglog(3.0, fit["walk"][axis], color + "*", markersize=20)
        minimum, minimum_index = fit["minimums"][axis]
        plt.loglog(period[minimum_index], minimum, color + "^", markersize=20)
    fitted_model = generate_prediction(
        period,
        q_white=sum(fit["white"]) / 3,
        q_bias_instability=sum(value for value, _ in fit["minimums"]) / 3,
        q_walk=sum(fit["walk"]) / 3,
    )
    plt.loglog(period, fitted_model, "-k", label="fitted model")
    plt.title(kind, fontsize=30)
    plt.ylabel(
        "Allan Deviation m/s^2" if kind == "Accelerometer" else "Allan Deviation deg/s",
        fontsize=30,
    )
    plt.legend(fontsize=25)
    plt.grid(True)
    plt.xlabel("Period (s)", fontsize=30)
    plt.tight_layout()
    figure.savefig(path, dpi=600, bbox_inches="tight")
    plt.close(figure)


def analyze(curve_path: Path, output: Path, topic: str, update_rate: float) -> tuple[dict, dict]:
    import numpy as np

    curve = np.loadtxt(curve_path)
    period = curve[:, 0]
    acceleration = curve[:, 1:4]
    rotation_rate = curve[:, 4:7]
    acceleration_fit = _fit_group(period, acceleration)
    gyroscope_fit = _fit_group(period, rotation_rate)
    parameters = {
        "accelerometer_noise_density": float(max(acceleration_fit["white"])),
        "accelerometer_random_walk": float(max(acceleration_fit["walk"])),
        "gyroscope_noise_density": float(max(gyroscope_fit["white"])) * math.pi / 180.0,
        "gyroscope_random_walk": float(max(gyroscope_fit["walk"])) * math.pi / 180.0,
        "rostopic": topic,
        "update_rate": float(update_rate),
    }
    yaml_text = (
        "#Accelerometer\n"
        f"accelerometer_noise_density: {parameters['accelerometer_noise_density']!r} \n"
        f"accelerometer_random_walk: {parameters['accelerometer_random_walk']!r} \n\n"
        "#Gyroscope\n"
        f"gyroscope_noise_density: {parameters['gyroscope_noise_density']!r} \n"
        f"gyroscope_random_walk: {parameters['gyroscope_random_walk']!r} \n\n"
        f"rostopic: {topic!r} \n"
        f"update_rate: {float(update_rate)!r} \n\n"
    )
    (output / "imu.yaml").write_text(yaml_text, encoding="utf-8", newline="\n")
    _plot(output / "acceleration.png", period, acceleration, acceleration_fit, "Accelerometer")
    _plot(output / "gyro.png", period, rotation_rate, gyroscope_fit, "Gyroscope")
    details = {
        "fit_contract": {
            "white_tau_s": "[0.1, 10.0)",
            "white_slope_bounds": [-0.5, -0.499],
            "white_evaluation_tau_s": 1.0,
            "random_walk_tau_s": "all 9999 points",
            "random_walk_slope_bounds": [0.5, 0.501],
            "random_walk_evaluation_tau_s": 3.0,
            "axis_aggregation": "maximum of x/y/z",
        },
        "accelerometer": dict(zip(("x", "y", "z"), acceleration_fit["details"])),
        "gyroscope_deg_per_s": dict(zip(("x", "y", "z"), gyroscope_fit["details"])),
    }
    return parameters, details
