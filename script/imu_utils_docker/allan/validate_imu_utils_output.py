#!/usr/bin/env python3
"""Validate that imu_utils consumed every bag message and wrote finite curves."""

import argparse
import json
import math
import re
import yaml
from pathlib import Path


AXES = ("gyr_x", "gyr_y", "gyr_z", "acc_x", "acc_y", "acc_z")


def read_numbers(path: Path):
    values = [float(line) for line in path.read_text(encoding="utf-8").splitlines()]
    if not values or not all(math.isfinite(value) for value in values):
        raise RuntimeError(f"missing or non-finite values in {path}")
    return values


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("result_dir", type=Path)
    parser.add_argument("--expected-messages", type=int, required=True)
    parser.add_argument("--name", required=True)
    args = parser.parse_args()

    log_path = args.result_dir / "imu_utils.log"
    log = log_path.read_text(encoding="utf-8")
    matches = re.findall(r"^(gyr|acc) ([xyz])  numData ([0-9]+)$", log, flags=re.MULTILINE)
    counts = {f"{kind}_{axis}": int(value) for kind, axis, value in matches}
    if len(matches) != 6 or set(counts) != set(AXES):
        raise RuntimeError(f"did not find six numData results: {counts}")
    if any(value != args.expected_messages for value in counts.values()):
        raise RuntimeError(f"imu_utils message loss: {counts}")

    curve_points = {}
    for sensor in ("gyr", "acc"):
        tau = read_numbers(args.result_dir / f"data_{args.name}_{sensor}_t.txt")
        sim_tau = read_numbers(args.result_dir / f"data_{args.name}_sim_{sensor}_t.txt")
        if tau[0] <= 0 or sim_tau != tau:
            raise RuntimeError('invalid measured/simulated tau grid')
        if any(right <= left for left, right in zip(tau, tau[1:])):
            raise RuntimeError(f"non-increasing tau values for {sensor}")
        for axis in "xyz":
            measured = read_numbers(args.result_dir / f"data_{args.name}_{sensor}_{axis}.txt")
            fitted = read_numbers(args.result_dir / f"data_{args.name}_sim_{sensor}_{axis}.txt")
            if len(measured) != len(tau) or len(fitted) != len(tau):
                raise RuntimeError(f"curve length mismatch for {sensor}_{axis}")
            if any(value < 0 for value in measured + fitted):
                raise RuntimeError(f"negative deviation for {sensor}_{axis}")
            curve_points[f"{sensor}_{axis}"] = len(tau)

    yaml_path = args.result_dir / f"{args.name}_imu_param.yaml"
    final_yaml = args.result_dir.parent / f'{args.name}_imu_param.yaml'
    if not yaml_path.is_file():
        yaml_path = final_yaml
    if not yaml_path.is_file() or yaml_path.stat().st_size == 0:
        raise RuntimeError("imu_utils parameter YAML is missing")
    parameters = yaml.safe_load('\n'.join(yaml_path.read_text().splitlines()[1:]))
    for group, prefix in [('Gyr', 'gyr'), ('Acc', 'acc')]:
        for axis in ['avg-axis', 'x-axis', 'y-axis', 'z-axis']:
            for suffix in ['n', 'w']:
                value = parameters[group][axis][prefix + '_' + suffix]
                if not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0:
                    raise RuntimeError('invalid YAML parameter')

    if yaml_path != final_yaml:
        if final_yaml.exists():
            raise FileExistsError(final_yaml)
        yaml_path.rename(final_yaml)
        yaml_path = final_yaml

    report = {
        "schema_version": "1.0.0",
        "kind": "imu_utils_output_validation",
        "status": "pass",
        "expected_messages": args.expected_messages,
        "imu_utils_num_data": counts,
        "curve_points": curve_points,
        "parameter_yaml": str(yaml_path),
        "checks": [
            "all six axes consumed every bag message",
            "tau values are finite and strictly increasing",
            "measured and fitted deviations are finite, non-negative, and length-matched",
            "all 16 YAML noise parameters are finite and non-negative",
        ],
    }
    report_path = args.result_dir / "validation.json"
    report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
