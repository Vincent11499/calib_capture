#!/usr/bin/env python3
"""Generate PNG plots and a Chinese Markdown report from imu_utils outputs."""

import json
import argparse
import re
from datetime import datetime, timezone
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


ROOT = Path("/output")
RESULT = ROOT / "work"
NAME = ''
COLORS = {"x": "#d62728", "y": "#2ca02c", "z": "#1f77b4"}


def load_values(path):
    values = np.loadtxt(str(path), dtype=float)
    if values.ndim != 1 or values.size == 0 or not np.all(np.isfinite(values)):
        raise RuntimeError("invalid curve data: {}".format(path))
    return values


def draw_sensor(ax, sensor, ylabel):
    tau = load_values(RESULT / "data_{}_{}_t.txt".format(NAME, sensor))
    for axis in "xyz":
        measured = load_values(
            RESULT / "data_{}_{}_{}.txt".format(NAME, sensor, axis)
        )
        fitted = load_values(
            RESULT / "data_{}_sim_{}_{}.txt".format(NAME, sensor, axis)
        )
        if measured.size != tau.size or fitted.size != tau.size:
            raise RuntimeError("curve length mismatch for {}_{}".format(sensor, axis))
        color = COLORS[axis]
        ax.loglog(
            tau,
            measured,
            "o",
            color=color,
            markersize=3.6,
            markerfacecolor="none",
            label="{} measured".format(axis.upper()),
        )
        ax.loglog(
            tau,
            fitted,
            "-",
            color=color,
            linewidth=1.6,
            label="{} fitted".format(axis.upper()),
        )
    ax.set_xlabel("Averaging time tau (s)")
    ax.set_ylabel(ylabel)
    ax.grid(True, which="both", linestyle=":", alpha=0.55)
    ax.legend(fontsize=8, ncol=2)


def save_combined():
    fig, axes = plt.subplots(1, 2, figsize=(15, 6.2))
    draw_sensor(axes[0], "gyr", "Allan deviation (deg/h)")
    axes[0].set_title("Gyroscope")
    draw_sensor(axes[1], "acc", "Allan deviation (m/s^2)")
    axes[1].set_title("Accelerometer")
    fig.suptitle("imu_utils Allan deviation: " + NAME, fontsize=14)
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(str(ROOT / "imu_utils_allan_deviation.png"), dpi=180)
    plt.close(fig)


def save_single(sensor, title, ylabel, filename):
    fig, ax = plt.subplots(figsize=(8.6, 6.4))
    draw_sensor(ax, sensor, ylabel)
    ax.set_title(title)
    fig.tight_layout()
    fig.savefig(str(ROOT / filename), dpi=180)
    plt.close(fig)


def parse_parameter_yaml(path):
    result = {"Gyr": {}, "Acc": {}}
    sensor = None
    axis = None
    for raw in path.read_text(encoding="utf-8").splitlines():
        stripped = raw.strip()
        if stripped in ("Gyr:", "Acc:"):
            sensor = stripped[:-1]
            axis = None
        elif sensor and stripped in ("avg-axis:", "x-axis:", "y-axis:", "z-axis:"):
            axis = stripped[:-1]
            result[sensor][axis] = {}
        elif sensor and axis and ":" in stripped:
            key, value = stripped.split(":", 1)
            if key.endswith("_n") or key.endswith("_w"):
                result[sensor][axis][key] = float(value.strip())
    return result


def table_rows(parameters, sensor, prefix):
    labels = (("avg-axis", "平均轴"), ("x-axis", "X"), ("y-axis", "Y"), ("z-axis", "Z"))
    rows = []
    for key, label in labels:
        values = parameters[sensor][key]
        rows.append(
            "| {} | `{:.10e}` | `{:.10e}` |".format(
                label, values[prefix + "_n"], values[prefix + "_w"]
            )
        )
    return "\n".join(rows)


def write_report():
    yaml_path = ROOT / (NAME + "_imu_param.yaml")
    parameters = parse_parameter_yaml(yaml_path)
    bag_check = json.loads((RESULT / (NAME + ".validation.json")).read_text(encoding="utf-8"))
    conversion = json.loads((RESULT / (NAME + '.conversion.json')).read_text(encoding='utf-8'))
    selected = conversion['input_validation'].get('selection')
    selection_text = '全段'
    if selected is not None:
        start_h = selected['start_s'] / 3600
        end_h = '末尾' if selected['end_s'] is None else format(selected['end_s'] / 3600, 'g') + 'h'
        selection_text = format(start_h, 'g') + 'h 至 ' + end_h
    run_check = json.loads((RESULT / "validation.json").read_text(encoding="utf-8"))
    log = (RESULT / "imu_utils.log").read_text(encoding="utf-8")
    frequencies = [float(value) for value in re.findall(r"^gyr [xyz]  freq ([0-9.eE+-]+)$", log, re.MULTILINE)]
    used_rate = frequencies[0] if frequencies else float("nan")

    report = """# imu_utils Allan 方差分析报告

生成时间（UTC）：{generated}

## 数据与运行结论

- 数据集：`{name}`
- ROS1 topic：`{topic}`，消息类型：`{message_type}`
- 消息数：`{messages}`，六轴全部消费，无回放丢失
- 数据时长：{duration:.9f} 秒
- 选择区间：{selection}（相对原始首条 IMU，起点包含、终点不包含，保留原时间戳）
- `imu_utils` 使用平均采样率：{used_rate:.3f} Hz
- 每轴 Allan 曲线点数：{curve_points}
- bag 完整回读：`{bag_status}`
- `imu_utils` 输出验证：`{run_status}`
- 时间间隔范围：{min_dt}–{max_dt} ns；使用平均周期近似，无插值、重采样或删样。
- 源码版本：见 `work/upstream_revision.txt` 和 `work/local.patch`。一键模式参数见 `work/run.json`；手动模式以执行命令和日志为准。

消息完整性验证不等于静止性或噪声模型适用性验收；温漂与非均匀采样会影响结果。

![imu_utils Allan deviation](imu_utils_allan_deviation.png)

图中空心圆为 `imu_utils` 计算值，实线为其五项噪声模型的 Ceres 联合拟合值。
陀螺仪 TXT 曲线使用上游内部单位 `deg/h`；加速度计使用 `m/s²`。

## 陀螺仪输出

| 轴 | `gyr_n` | `gyr_w` |
|---|---:|---:|
{gyr_rows}

## 加速度计输出

| 轴 | `acc_n` | `acc_w` |
|---|---:|---:|
{acc_rows}

## 产物

- `imu_utils_allan_deviation.png`：陀螺仪与加速度计组合图
- `imu_utils_gyro_allan.png`：陀螺仪实测与拟合图
- `imu_utils_accel_allan.png`：加速度计实测与拟合图
- `{name}_imu_param.yaml`：上游参数 YAML
- `work/validation.json`：消息数量与曲线完整性验证
- `work/imu_utils.log`：完整拟合日志
- `work/data_{name}_*.txt`：实测与拟合曲线数据

## 参数解释

这里报告的是原版 `imu_utils` 输出。`gyr_w` 和 `acc_w` 是拟合曲线最低值所表示的
bias instability，不应直接作为 Kalibr 的 `gyroscope_random_walk` 或
`accelerometer_random_walk`。Kalibr 参数仍需依据 Allan 标准差曲线中约
`-1/2` 与 `+1/2` 的斜率区间及对应单位单独提取和核对。
""".format(
        generated=datetime.now(timezone.utc).isoformat(),
        name=NAME,
        duration=conversion['duration_s'],
        selection=selection_text,
        min_dt=conversion['input_validation']['minimum_interval_ns'],
        max_dt=conversion['input_validation']['maximum_interval_ns'],
        topic=bag_check["topic"],
        message_type=bag_check["message_type"],
        messages=bag_check["messages"],
        used_rate=used_rate,
        curve_points=run_check["curve_points"]["gyr_x"],
        bag_status=bag_check["status"],
        run_status=run_check["status"],
        gyr_rows=table_rows(parameters, "Gyr", "gyr"),
        acc_rows=table_rows(parameters, "Acc", "acc"),
    )
    (ROOT / "imu_utils_report.md").write_text(report, encoding="utf-8")


def main():
    global ROOT, RESULT, NAME
    parser = argparse.ArgumentParser()
    parser.add_argument('result_dir', type=Path)
    parser.add_argument('--name', required=True)
    args = parser.parse_args()
    RESULT, ROOT, NAME = args.result_dir, args.result_dir.parent, args.name
    for path in (RESULT / 'validation.json', RESULT / (NAME + '.validation.json')):
        if json.loads(path.read_text())['status'] != 'pass':
            raise RuntimeError('report requires validated results')
    save_combined()
    save_single("gyr", "Gyroscope Allan deviation", "Allan deviation (deg/h)", "imu_utils_gyro_allan.png")
    save_single("acc", "Accelerometer Allan deviation", "Allan deviation (m/s^2)", "imu_utils_accel_allan.png")
    write_report()
    print("generated plots and report in {}".format(ROOT))


if __name__ == "__main__":
    main()
