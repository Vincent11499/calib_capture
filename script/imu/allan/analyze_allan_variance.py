#!/usr/bin/env python3
"""Run the pinned allan_variance_ros algorithm directly on CSV, without ROS or C++."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import sys

if __package__:
    from .allan_core import UPSTREAM_REVISION, compute_curves, validate_curves
    from .input_format import (
        add_time_arguments,
        materialize_selected_samples,
        resolve_dataset,
        time_bounds,
        time_suffix,
        validate_input,
    )
    from .upstream_analysis import analyze
else:  # Direct execution: python script/imu/allan/analyze_allan_variance.py
    from allan_core import UPSTREAM_REVISION, compute_curves, validate_curves
    from input_format import (
        add_time_arguments,
        materialize_selected_samples,
        resolve_dataset,
        time_bounds,
        time_suffix,
        validate_input,
    )
    from upstream_analysis import analyze


SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parents[2]
DEFAULT_OUTPUT_ROOT = PROJECT_ROOT / "data" / "allan_variance" / "allan_variance_output"


def save_json(path: Path, value: dict) -> None:
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def _output_paths(output: Path) -> tuple[Path, Path]:
    return output.with_name(output.name + ".inprogress"), output.with_name(output.name + ".failed")


def _report(source: dict, parameters: dict, measure_rate: int, topic: str) -> str:
    selection = source["selection"]
    rows = "\n".join(f"| {key} | {value} |" for key, value in parameters.items())
    return f"""# Python Allan variance 分析报告

- 输入：`{source['source']}`，原文件只读。
- 选择区间：相对原始首条 IMU 的 [{selection['start_s']}, {selection['end_s'] if selection['end_s'] is not None else 'end'}) 秒。
- 样本数：{source['messages']}；时长：{source['duration_s']:.9f} s。
- 实际平均采样率：{source['effective_rate_hz']:.12f} Hz；计算 `measure_rate`：{measure_rate} Hz。
- 兼容目标：`ori-drs/allan_variance_ros@{UPSTREAM_REVISION}`。
- 实现：纯 Python/NumPy/SciPy；无 ROS、bag、C++、Docker、PyYAML。
- 曲线：六轴各 9,999 点，tau=0.1–999.9 s；五个尺度独立复算通过。
- 陀螺仪曲线单位 deg/s，加速度计为 m/s²；YAML 中陀螺仪参数已转回弧度制。

![Accelerometer](acceleration.png)

![Gyroscope](gyro.png)

## Kalibr 参数候选

| 参数 | 值 |
|---|---|
{rows}

本工具严格保留原版拟合策略：白噪声使用 tau<10 s，随机游走强制使用全部尺度并把斜率约束为
+0.5 到 +0.501。结果复现原版算法，但不表示数据中确实存在可信的 +1/2 区域，也不等同于
IMU 噪声模型或 Kalibr 标定质量验收。

## 文件

- `imu.yaml`：与原版字段和单位一致的 Kalibr 候选参数；topic 为 `{topic}`。
- `acceleration.png`、`gyro.png`：原版样式的曲线、拟合线与标记。
- `work/allan_variance.csv`：与原 C++ 相同的空格分隔 Allan deviation 曲线。
- `work/run.json`、`work/validation.json`、`work/fit.json`：输入、计算和拟合明细。
"""


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dataset", type=Path, help="session directory or <session>/imu/imu0.csv")
    parser.add_argument("--output", type=Path, help="complete result directory; must not exist")
    parser.add_argument("--topic", default="/imu/data", help="rostopic written to imu.yaml")
    add_time_arguments(parser)
    args = parser.parse_args(argv)
    try:
        start_ns, end_ns = time_bounds(args)
        if not args.topic or any(character.isspace() for character in args.topic):
            raise ValueError("topic must be a non-empty string without whitespace")
        dataset, csv_path = resolve_dataset(args.dataset)
        name = dataset.name + time_suffix(start_ns, end_ns) + "_allan_variance"
        output = (args.output or DEFAULT_OUTPUT_ROOT / name).resolve()
        if output == dataset or dataset in output.parents:
            raise ValueError("output must not be inside the immutable source dataset")
        staging, failed = _output_paths(output)
        for path in (output, staging, failed):
            if path.exists():
                raise FileExistsError(f"output already exists: {path}")

        print("Validating the complete source CSV...", flush=True)
        source = validate_input(dataset, csv_path, start_ns, end_ns)
        actual_rate = float(source["effective_rate_hz"])
        measure_rate = math.floor(actual_rate)
        maximum_bin_size = int((9999 * 0.1) * measure_rate)
        if measure_rate < 10 or source["messages"] <= 2 * maximum_bin_size:
            raise ValueError(
                "the fixed 0.1..999.9 s grid requires >=10 Hz and more than two "
                "complete bins at tau=999.9 s"
            )
        if int(actual_rate / measure_rate) != 1:
            raise ValueError("the original configuration would subsample input")

        staging.mkdir(parents=True)
        work = staging / "work"
        work.mkdir()
        started = datetime.now(timezone.utc).isoformat()
        run = {
            "schema_version": "1.0.0",
            "kind": "allan_variance_python_run",
            "status": "running",
            "started_utc": started,
            "source": source,
            "compatibility_target": {
                "repository": "ori-drs/allan_variance_ros",
                "revision": UPSTREAM_REVISION,
                "replaced_components": ["rosbag ingestion", "C++ Allan computation", "analysis.py"],
            },
            "options": {
                "dataset": str(dataset),
                "output": str(output),
                "topic": args.topic,
                "start_offset_ns": start_ns,
                "end_offset_ns": end_ns,
            },
            "measure_rate_hz": measure_rate,
            "imu_skip": 1,
            "tau_scale_note": (
                "Integer measure_rate reproduces the original nominal bin durations; "
                "nonuniform timestamps are not resampled."
            ),
        }
        save_json(work / "run.json", run)
        save_json(
            work / "config.json",
            {
                "imu_topic": args.topic,
                "imu_rate": actual_rate,
                "measure_rate": measure_rate,
                "sequence_time": math.ceil(source["messages"] / actual_rate) + 1,
            },
        )

        selected_path = work / "selected_samples.npy"
        try:
            print(f"Materializing {source['messages']} selected samples...", flush=True)
            materialize_selected_samples(
                csv_path, selected_path, source["messages"], start_ns, end_ns
            )
            curve_path = work / "allan_variance.csv"
            compute_curves(
                selected_path,
                curve_path,
                measure_rate,
                progress=lambda message: print(message, flush=True),
            )
            validation = validate_curves(selected_path, curve_path, measure_rate)
            print("Curves validated; fitting parameters and rendering plots...", flush=True)
            parameters, fit = analyze(curve_path, staging, args.topic, actual_rate)
            if any(
                not math.isfinite(parameters[key]) or parameters[key] <= 0
                for key in (
                    "accelerometer_noise_density",
                    "accelerometer_random_walk",
                    "gyroscope_noise_density",
                    "gyroscope_random_walk",
                )
            ):
                raise RuntimeError("invalid fitted parameter")
            validation["parameters"] = parameters
            save_json(work / "validation.json", validation)
            save_json(work / "fit.json", fit)
            (staging / "allan_variance_report.md").write_text(
                _report(source, parameters, measure_rate, args.topic), encoding="utf-8"
            )
            selected_path.unlink()
            run["status"] = "pass"
        except BaseException as exc:
            run["status"] = "fail"
            run["error"] = f"{type(exc).__name__}: {exc}"
            raise
        finally:
            run["finished_utc"] = datetime.now(timezone.utc).isoformat()
            save_json(work / "run.json", run)
        os.replace(staging, output)
        print(f"PASS: {output}", flush=True)
        return 0
    except BaseException as exc:
        if "staging" in locals() and staging.exists():
            os.replace(staging, failed)
            print(f"FAILED DATA: {failed}", file=sys.stderr)
        print(f"error: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
