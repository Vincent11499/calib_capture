from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from .capture import (
    format_probe_result,
    probe_hardware,
    record_continuous,
    record_triggered,
)
from .config import (
    APP_VERSION,
    DEFAULTS,
    CaptureConfig,
    build_capture_config,
    detect_sdk_include_dir,
    session_output_path,
)
from .ros1_export import export_ros1_bag
from .selftest import run_self_test
from .validator import ValidationReport, validate_dataset


def _add_capture_options(
    parser: argparse.ArgumentParser, *, output: bool, continuous: bool = False
) -> None:
    if output:
        parser.add_argument(
            "--output",
            default="./data",
            metavar="DIR",
            help=(
                "session root directory (default: ./data); the dataset is created "
                "as DIR/YYYY-MM-DD_HH-MM-SS"
            ),
        )
    parser.add_argument("--ax-lib-dir", default=None)
    parser.add_argument("--sdk-include-dir", default=None)
    parser.add_argument("--left-pipe", type=int, default=None)
    parser.add_argument("--left-chn", type=int, default=None)
    parser.add_argument("--right-pipe", type=int, default=None)
    parser.add_argument("--right-chn", type=int, default=None)
    parser.add_argument("--resolution", default=None, metavar="WIDTHxHEIGHT")
    parser.add_argument("--image-format", choices=("bmp", "jpeg"), default=None)
    parser.add_argument("--image-workers", type=int, default=None, help="image worker threads, 1..8 (default: 2)")
    parser.add_argument("--jpeg-quality", type=int, default=None, help="JPEG quality 1..100 (default: 95)")
    parser.add_argument("--expected-source-rate-hz", type=float, default=None)
    if continuous:
        rate_control = parser.add_mutually_exclusive_group()
        rate_control.add_argument(
            "--save-rate-hz",
            type=float,
            default=None,
            help=(
                f"target stereo pair rate by PTS (default: {DEFAULTS['save_rate_hz']:g} Hz); "
                "IMU messages are not rate-limited"
            ),
        )
        rate_control.add_argument(
            "--frame-divisor",
            type=int,
            default=None,
            metavar="N",
            help=(
                "save after both AX source sequence numbers advance by N; "
                "0 disables frame division and uses --save-rate-hz"
            ),
        )
    parser.add_argument("--duration-s", type=float, default=None)
    parser.add_argument("--poses", type=int, default=None)
    parser.add_argument("--sync-tolerance-us", type=int, default=None)
    parser.add_argument("--max-sync-retries", type=int, default=None)
    parser.add_argument("--capture-timeout-ms", type=int, default=None)
    parser.add_argument("--writer-queue-pairs", type=int, default=None)
    parser.add_argument("--startup-timeout-s", type=float, default=None)
    parser.add_argument("--imu-topic", default=None)
    parser.add_argument("--imu-expected-rate-hz", type=float, default=None)
    parser.add_argument("--imu-qos-depth", type=int, default=None)
    parser.add_argument("--clock-domain-max-offset-s", type=float, default=None)
    parser.add_argument("--left-topic", default=None)
    parser.add_argument("--right-topic", default=None)
    parser.add_argument("--left-frame-id", default=None)
    parser.add_argument("--right-frame-id", default=None)
    parser.add_argument("--imu-frame-id", default=None)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Pure-Python AXERA stereo and stereo-IMU calibration data capture"
    )
    parser.add_argument("--version", action="version", version=APP_VERSION)
    parser.add_argument(
        "--config",
        help="optional INI configuration ([capture], no extra dependency); command-line options override it",
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    probe = subparsers.add_parser("probe", help="probe VIN and optional IMU")
    _add_capture_options(probe, output=False)
    probe.add_argument("--samples", type=int, default=50)
    probe.add_argument("--with-imu", action="store_true")

    triggered = subparsers.add_parser(
        "stereo-triggered", help="interactive pose-by-pose stereo capture"
    )
    _add_capture_options(triggered, output=True)

    continuous = subparsers.add_parser(
        "stereo-continuous", help="continuous stereo capture"
    )
    _add_capture_options(continuous, output=True, continuous=True)

    camera_imu = subparsers.add_parser("camera-imu", help="continuous stereo plus IMU capture")
    _add_capture_options(camera_imu, output=True, continuous=True)

    validate = subparsers.add_parser("validate", help="validate a directory dataset")
    validate.add_argument("dataset")

    export = subparsers.add_parser("export-ros1", help="export a dataset to ROS1 bag")
    export.add_argument("dataset")
    export.add_argument("bag")
    export.add_argument("--left-topic")
    export.add_argument("--right-topic")
    export.add_argument("--imu-topic")
    export.add_argument("--compression", choices=("none", "lz4", "bz2"), default="none")
    export.add_argument("--force-dataset", action="store_true")

    self_test = subparsers.add_parser("self-test", help="run tests without AX or ROS hardware")
    self_test.add_argument("--sdk-include-dir", default=None)
    return parser


def _capture_config(args: argparse.Namespace, mode: str) -> CaptureConfig:
    values = vars(args)
    overrides = {key: values.get(key) for key in DEFAULTS if values.get(key) is not None}
    output = ""
    if mode in {"stereo-triggered", "stereo-continuous", "camera-imu"}:
        output = session_output_path(str(values.get("output") or "./data"))
    return build_capture_config(
        mode=mode,
        output=output,
        config_path=args.config,
        overrides=overrides,
    )


def _print_capture_result(path: Path, report: ValidationReport) -> None:
    print(
        json.dumps(
            {
                "dataset": str(path),
                "validation": report.to_dict(),
            },
            indent=2,
            sort_keys=True,
            ensure_ascii=False,
        )
    )


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        if args.command == "probe":
            config = _capture_config(args, "probe")
            result = probe_hardware(config, samples=args.samples, with_imu=args.with_imu)
            print(format_probe_result(result))
            return 1 if result.get("status") == "fail" else 0
        if args.command == "stereo-triggered":
            path, report = record_triggered(_capture_config(args, "stereo-triggered"))
            _print_capture_result(path, report)
            return 0 if not report.errors else 1
        if args.command == "stereo-continuous":
            path, report = record_continuous(
                _capture_config(args, "stereo-continuous"), include_imu=False
            )
            _print_capture_result(path, report)
            return 0 if not report.errors else 1
        if args.command == "camera-imu":
            path, report = record_continuous(
                _capture_config(args, "camera-imu"), include_imu=True
            )
            _print_capture_result(path, report)
            return 0 if not report.errors else 1
        if args.command == "validate":
            report = validate_dataset(
                args.dataset,
                write_report=True,
            )
            print(json.dumps(report.to_dict(), indent=2, sort_keys=True, ensure_ascii=False))
            return 0 if not report.errors else 1
        if args.command == "export-ros1":
            result = export_ros1_bag(
                args.dataset,
                args.bag,
                left_topic=args.left_topic,
                right_topic=args.right_topic,
                imu_topic=args.imu_topic,
                compression=args.compression,
                force_dataset=args.force_dataset,
            )
            print(json.dumps(result, indent=2, sort_keys=True, ensure_ascii=False))
            return 0
        if args.command == "self-test":
            include_dir = detect_sdk_include_dir(args.sdk_include_dir or "")
            result = run_self_test(include_dir)
            print(json.dumps(result, indent=2, sort_keys=True, ensure_ascii=False))
            return 0
        parser.error(f"unknown command: {args.command}")
    except KeyboardInterrupt:
        print("interrupted", file=sys.stderr)
        return 130
    except Exception as exc:
        print(f"error: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 1
    return 1
