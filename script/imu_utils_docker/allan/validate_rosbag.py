#!/usr/bin/env python3
"""Fully compare the derived ROS 1 bag against the source IMU CSV."""

import argparse
import csv
import json
from pathlib import Path

import rosbag
from input_format import validate_input, selected_rows, add_time_arguments, time_bounds


FIELDS = ("wx", "wy", "wz", "ax", "ay", "az")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("dataset", type=Path)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--topic", default="/imu/data")
    parser.add_argument('--frame-id', default='imu_link')
    add_time_arguments(parser)
    args = parser.parse_args()
    start_ns, end_ns = time_bounds(args)
    work = args.bag.parent / 'work'
    work.mkdir(parents=True, exist_ok=True)
    source_check = validate_input(args.dataset, args.frame_id, start_ns, end_ns)

    csv_path = args.dataset if args.dataset.is_file() else args.dataset / 'imu/imu0.csv'
    expected_frame = args.frame_id
    count = 0
    first_ns = None
    last_ns = None

    with rosbag.Bag(str(args.bag), "r") as bag:
        messages = bag.read_messages(topics=[args.topic])
        for timestamp_ns, values in selected_rows(args.dataset, start_ns, end_ns):
            try:
                topic, msg, record_time = next(messages)
            except StopIteration as exc:
                raise RuntimeError(f"bag ended after {count} messages") from exc

            message_ns = msg.header.stamp.to_nsec()
            record_ns = record_time.to_nsec()
            if topic != args.topic or msg._type != "sensor_msgs/Imu":
                raise RuntimeError(f"unexpected topic/type at message {count}")
            if message_ns != timestamp_ns or record_ns != timestamp_ns:
                raise RuntimeError(f"timestamp mismatch at message {count}")
            if msg.header.frame_id != expected_frame:
                raise RuntimeError(f"frame mismatch at message {count}")
            if msg.header.seq != count or msg.orientation_covariance[0] != -1:
                raise RuntimeError(f"sequence/orientation mismatch at message {count}")
            actual = (
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z,
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z,
            )
            expected = values[:6]
            if actual != expected:
                raise RuntimeError(f"six-axis value mismatch at message {count}")

            if first_ns is None:
                first_ns = timestamp_ns
            last_ns = timestamp_ns
            count += 1

        try:
            next(messages)
        except StopIteration:
            pass
        else:
            raise RuntimeError("bag contains extra IMU messages")

        info = bag.get_type_and_topic_info()
        if set(info.topics) != {args.topic}:
            raise RuntimeError(f"unexpected topics: {sorted(info.topics)}")
        if info.topics[args.topic].message_count != count:
            raise RuntimeError("bag index count does not match full readback")

    expected_count = source_check["messages"]
    if count != expected_count:
        raise RuntimeError(f"read {count}, expected {expected_count}")

    report = {
        "schema_version": "1.0.0",
        "kind": "imu_utils_bag_validation",
        "status": "pass",
        "selection": source_check["selection"],
        "bag": str(args.bag),
        "topic": args.topic,
        "message_type": "sensor_msgs/Imu",
        "messages": count,
        "frame_id": expected_frame,
        "first_timestamp_ns": first_ns,
        "last_timestamp_ns": last_ns,
        "header_equals_record_time": True,
        "timestamps_match_csv": True,
        "six_axis_values_match_csv": True,
    }
    report_path = work / args.bag.with_suffix(".validation.json").name
    report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
