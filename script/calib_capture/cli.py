from __future__ import annotations

import argparse
import json
import sys
from dataclasses import replace
from pathlib import Path

from .capture import (
    format_probe_result,
    probe_hardware,
    record_continuous,
    record_triggered,
    record_base_link_triggered,
)
from .base_link import BASE_LINK_POSES
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
from .modes import (
    STEREO_BASELINK_MODE,
    STEREO_IMU_MODE,
    USB_BASELINK_MODE,
    USB_CONTINUOUS_MODE,
)


def _add_capture_options(
    parser: argparse.ArgumentParser, *, output: bool, continuous: bool = False,
    include_poses: bool = True,
) -> None:
    if output:
        parser.add_argument(
            "--output",
            default="./data",
            metavar="DIR",
            help=(
                "session root directory (default: ./data); the dataset is created "
                "as DIR/<dataset_name>_YYMMDD_hhmm; existing names are rejected"
            ),
        )
        parser.add_argument("--dataset-name", default=None, help="session prefix (default: stereo or stereo_imu)")
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
    if include_poses:
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


def _add_usb_options(parser: argparse.ArgumentParser, *, continuous: bool) -> None:
    parser.add_argument("--output", default="./data", metavar="DIR")
    parser.add_argument("--dataset-name", default=None)
    parser.add_argument("--ax-lib-dir", default=None)
    parser.add_argument("--sdk-include-dir", default=None)
    parser.add_argument("--image-format", choices=("bmp", "jpeg"), default=None)
    parser.add_argument("--image-workers", type=int, default=None)
    parser.add_argument("--jpeg-quality", type=int, default=None)
    parser.add_argument("--capture-timeout-ms", type=int, default=None)
    parser.add_argument("--writer-queue-pairs", type=int, default=None,
                        help="bounded USB image queue capacity (default: 2)")
    parser.add_argument("--expected-source-rate-hz", type=float, default=None)
    if continuous:
        parser.add_argument(
            "--save-rate-hz", "--rate-hz", dest="save_rate_hz", type=float, default=None,
            help=f"target USB image rate by PTS (default: {DEFAULTS['save_rate_hz']:g} Hz)",
        )
        parser.add_argument("--duration-s", type=float, default=None)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Pure-Python AXERA stereo, stereo-IMU and USB IVPS data capture"
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

    base_link_triggered = subparsers.add_parser(
        STEREO_BASELINK_MODE,
        help="fixed 14-pose stereo capture for base_link calibration",
    )
    _add_capture_options(base_link_triggered, output=True, include_poses=False)

    raw = subparsers.add_parser("stereo-raw-triggered", help="experimental pose-by-pose IFE/L RAW dump (not Kalibr images)")
    raw.add_argument("--output", default="./data")
    raw.add_argument("--dataset-name", default=None)
    for name in ("ax-lib-dir", "sdk-include-dir", "resolution"):
        raw.add_argument("--" + name, default=None)
    for name in ("left-pipe", "right-pipe", "poses", "sync-tolerance-us",
                 "max-sync-retries", "capture-timeout-ms"):
        raw.add_argument("--" + name, type=int, default=None)

    continuous = subparsers.add_parser(
        "stereo-continuous", help="continuous stereo capture"
    )
    _add_capture_options(continuous, output=True, continuous=True)

    stereo_imu = subparsers.add_parser(STEREO_IMU_MODE, help="continuous stereo plus IMU capture")
    _add_capture_options(stereo_imu, output=True, continuous=True)
    stereo_imu.add_argument("--imu-preroll-s", type=float, default=None,
                            help="IMU coverage before images in seconds (default 1; 0 disables)")
    stereo_imu.add_argument("--imu-postroll-s", type=float, default=None,
                            help="IMU coverage after images in seconds (default 1; 0 disables)")

    usb_continuous = subparsers.add_parser(
        USB_CONTINUOUS_MODE,
        help="single USB camera capture from IVPS grp 3/chn 0 at fixed 1920x1080",
    )
    _add_usb_options(usb_continuous, continuous=True)

    usb_baselink = subparsers.add_parser(
        USB_BASELINK_MODE,
        help="fixed 14-pose USB capture from IVPS grp 3/chn 0",
    )
    _add_usb_options(usb_baselink, continuous=False)

    imu_only = subparsers.add_parser("imu-continuous", help="IMU-only CSV capture for Allan analysis; no camera")
    imu_only.add_argument("--output", default="./data", metavar="DIR")
    imu_only.add_argument("--dataset-name", default=None, help="session prefix (default: imu)")
    imu_only.add_argument("--duration-s", type=float, default=None, help="recording seconds after first sample (default 60)")
    imu_only.add_argument("--imu-topic", default=None)
    imu_only.add_argument("--imu-temperature-topic", default=None, help="sensor_msgs/Temperature topic (default /imu/temperature)")
    imu_only.add_argument(
        "--ignore-temperature", dest="use_temperature", action="store_false", default=None,
        help="do not subscribe to temperature; write a seven-column IMU CSV",
    )
    imu_only.add_argument("--imu-frame-id", default=None)
    imu_only.add_argument("--imu-expected-rate-hz", type=float, default=None, help="diagnostics only; never limits samples")
    imu_only.add_argument("--imu-qos-depth", type=int, default=None)
    imu_only.add_argument("--startup-timeout-s", type=float, default=None, help="first-message and stream-stall timeout")

    validate = subparsers.add_parser("validate", help="validate a directory dataset")
    validate.add_argument("dataset")

    from .allan import add_options
    allan = subparsers.add_parser("allan", help="offline six-axis Allan deviation and variance")
    add_options(allan)

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
    config = build_capture_config(
        mode=mode,
        config_path=args.config,
        overrides=overrides,
    )
    if mode in {STEREO_BASELINK_MODE, USB_BASELINK_MODE}:
        config = replace(config, poses=len(BASE_LINK_POSES))
    if mode in {
        "stereo-triggered", STEREO_BASELINK_MODE, "stereo-raw-triggered",
        "stereo-continuous", STEREO_IMU_MODE, "imu-continuous",
        USB_CONTINUOUS_MODE, USB_BASELINK_MODE,
    }:
        config = replace(config, output=session_output_path(
            str(values.get("output") or "./data"), dataset_name=config.dataset_name))
    return config


def _print_capture_result(path: Path, report: ValidationReport) -> None:
    print(
        json.dumps(
            {
                "schema_version": APP_VERSION,
                "kind": "capture_result",
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
        if args.command == "stereo-raw-triggered":
            from .raw_capture import record_raw_triggered
            path = record_raw_triggered(_capture_config(args, args.command))
            print(json.dumps(dict(schema_version=APP_VERSION, kind="raw_capture_result",
                                  dataset=str(path)), ensure_ascii=False, indent=2))
            return 0
        if args.command == "allan":
            from .allan import analyze_allan
            result = analyze_allan(args.dataset, output=args.output, rate_hz=args.rate_hz,
                                   taus=args.taus, allow_irregular=args.allow_irregular,
                                   start_s=args.start_s, end_s=args.end_s,
                                   kalibr_noise=args.kalibr_noise, kalibr_imu_topic=args.kalibr_imu_topic)
            print(json.dumps(result, indent=2, ensure_ascii=False))
            if args.kalibr_noise and result["noise_fit"]["status"] == "incomplete":
                return 1
            return 0
        if args.command == "probe":
            config = _capture_config(args, "probe")
            result = probe_hardware(config, samples=args.samples, with_imu=args.with_imu)
            print(format_probe_result(result))
            return 1 if result.get("status") == "fail" else 0
        if args.command == "stereo-triggered":
            path, report = record_triggered(_capture_config(args, "stereo-triggered"))
            _print_capture_result(path, report)
            return 0 if not report.errors else 1
        if args.command == STEREO_BASELINK_MODE:
            path, report = record_base_link_triggered(
                _capture_config(args, STEREO_BASELINK_MODE)
            )
            _print_capture_result(path, report)
            return 0 if not report.errors else 1
        if args.command == "stereo-continuous":
            path, report = record_continuous(
                _capture_config(args, "stereo-continuous"), include_imu=False
            )
            _print_capture_result(path, report)
            return 0 if not report.errors else 1
        if args.command == STEREO_IMU_MODE:
            path, report = record_continuous(
                _capture_config(args, STEREO_IMU_MODE), include_imu=True
            )
            _print_capture_result(path, report)
            return 0 if not report.errors else 1
        if args.command in {USB_CONTINUOUS_MODE, USB_BASELINK_MODE}:
            from .usb_capture import record_usb_baselink_triggered, record_usb_continuous
            config = _capture_config(args, args.command)
            if args.command == USB_CONTINUOUS_MODE:
                path, report = record_usb_continuous(config)
            else:
                path, report = record_usb_baselink_triggered(config)
            _print_capture_result(path, report)
            return 0 if not report.errors else 1
        if args.command == "validate":
            from .imu_only import is_imu_only, validate_imu_only
            validator = validate_imu_only if is_imu_only(args.dataset) else validate_dataset
            report = validator(
                args.dataset,
                write_report=True,
            )
            print(json.dumps(report.to_dict(), indent=2, sort_keys=True, ensure_ascii=False))
            return 0 if not report.errors else 1
        if args.command == "export-ros1":
            from .imu_only import is_imu_only
            if is_imu_only(args.dataset):
                raise ValueError("IMU-only Allan CSV does not support ROS1 bag export; use imu/imu0.csv")
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
        if args.command == "imu-continuous":
            from .imu_only import record_imu_only
            path, report = record_imu_only(_capture_config(args, "imu-continuous"))
            _print_capture_result(path, report)
            return 0 if not report.errors else 1
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
