#!/usr/bin/env python3
"""Stream an imu-continuous CSV into a ROS 1 sensor_msgs/Imu bag."""

import argparse
import csv
import json
from pathlib import Path

import rosbag
import rospy
from sensor_msgs.msg import Imu
from input_format import validate_input, selected_rows, add_time_arguments, time_bounds


EXPECTED_COLUMNS = [
    "timestamp_ns",
    "wx",
    "wy",
    "wz",
    "ax",
    "ay",
    "az",
    "temperature",
]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("dataset", type=Path)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--topic", default="/imu/data")
    parser.add_argument("--compression", choices=("none", "bz2", "lz4"), default="lz4")
    parser.add_argument('--frame-id', default='imu_link')
    add_time_arguments(parser)
    args = parser.parse_args()
    start_ns, end_ns = time_bounds(args)
    work = args.bag.parent / 'work'
    work.mkdir(parents=True, exist_ok=True)
    source_check = validate_input(args.dataset, args.frame_id, start_ns, end_ns)

    csv_path = args.dataset if args.dataset.is_file() else args.dataset / 'imu/imu0.csv'
    frame_id = args.frame_id
    args.bag.parent.mkdir(parents=True, exist_ok=True)
    if args.bag.exists():
        raise FileExistsError(args.bag)
    partial = work / (args.bag.name + '.inprogress')
    if partial.exists():
        raise FileExistsError(partial)

    compression = {
        "none": rosbag.Compression.NONE,
        "bz2": rosbag.Compression.BZ2,
        "lz4": rosbag.Compression.LZ4,
    }[args.compression]

    count = 0
    first_ns = None
    last_ns = None
    with rosbag.Bag(str(partial), 'w', compression=compression) as bag:
        for timestamp_ns, values in selected_rows(args.dataset, start_ns, end_ns):
            stamp = rospy.Time(timestamp_ns // 1_000_000_000, timestamp_ns % 1_000_000_000)
            msg = Imu()
            msg.header.seq = count
            msg.header.stamp = stamp
            msg.header.frame_id = frame_id
            msg.orientation_covariance[0] = -1.0
            msg.angular_velocity.x = values[0]
            msg.angular_velocity.y = values[1]
            msg.angular_velocity.z = values[2]
            msg.linear_acceleration.x = values[3]
            msg.linear_acceleration.y = values[4]
            msg.linear_acceleration.z = values[5]
            bag.write(args.topic, msg, stamp)

            if first_ns is None:
                first_ns = timestamp_ns
            last_ns = timestamp_ns
            count += 1
            if count % 200_000 == 0:
                print(f"written={count}", flush=True)

    expected = source_check["messages"]
    if count != expected:
        raise RuntimeError(f"message count mismatch: wrote {count}, expected {expected}")
    partial.rename(args.bag)

    report = {
        "schema_version": "1.0.0",
        "kind": "imu_utils_bag_conversion",
        "input_validation": source_check,
        "status": "pass",
        "source": str(args.dataset),
        "bag": str(args.bag),
        "topic": args.topic,
        "message_type": "sensor_msgs/Imu",
        "frame_id": frame_id,
        "compression": args.compression,
        "messages": count,
        "first_timestamp_ns": first_ns,
        "last_timestamp_ns": last_ns,
        "duration_s": (last_ns - first_ns) / 1_000_000_000,
    }
    report_path = work / args.bag.with_suffix(".conversion.json").name
    report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
