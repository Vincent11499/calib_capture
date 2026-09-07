from __future__ import annotations

import csv
import heapq
import json
from dataclasses import dataclass
from importlib.metadata import PackageNotFoundError, version as package_version
from pathlib import Path
from typing import Any, Iterable, Iterator

from .image_io import read_mono8_image
from .validator import validate_dataset


IMAGE_MSGTYPE = "sensor_msgs/msg/Image"
IMU_MSGTYPE = "sensor_msgs/msg/Imu"


@dataclass(frozen=True)
class ExportEvent:
    timestamp_ns: int
    order: int
    kind: str
    sequence: int
    payload: Any


def _read_camera_events(dataset: Path, camera: str, order: int) -> Iterator[ExportEvent]:
    path = dataset / "cameras" / camera / "timestamps.csv"
    with path.open("r", encoding="utf-8", newline="") as stream:
        for sequence, row in enumerate(csv.DictReader(stream), 1):
            timestamp_ns = int(row["timestamp_ns"])
            image_path = dataset / "cameras" / camera / row["filename"]
            resolved = image_path.resolve()
            resolved.relative_to(dataset)
            yield ExportEvent(timestamp_ns, order, camera, sequence, image_path)


def _read_imu_events(dataset: Path, order: int) -> Iterator[ExportEvent]:
    path = dataset / "meta/imu_full.jsonl"
    with path.open("r", encoding="utf-8") as stream:
        sequence = 0
        for raw in stream:
            if not raw.strip():
                continue
            sequence += 1
            payload = json.loads(raw)
            yield ExportEvent(int(payload["timestamp_ns"]), order, "imu", sequence, payload)


def _type(typestore: Any, name: str) -> Any:
    try:
        return typestore.types[name]
    except KeyError as exc:
        raise RuntimeError(f"rosbags typestore lacks {name}") from exc


def _construct(message_type: Any, **values: Any) -> Any:
    annotations = getattr(message_type, "__annotations__", {})
    if annotations:
        values = {key: value for key, value in values.items() if key in annotations}
    return message_type(**values)


def _make_time(typestore: Any, timestamp_ns: int) -> Any:
    time_type = _type(typestore, "builtin_interfaces/msg/Time")
    sec, nanosec = divmod(timestamp_ns, 1_000_000_000)
    return _construct(time_type, sec=int(sec), nanosec=int(nanosec))


def _make_header(
    typestore: Any,
    timestamp_ns: int,
    sequence: int,
    frame_id: str,
) -> Any:
    header_type = _type(typestore, "std_msgs/msg/Header")
    return _construct(
        header_type,
        seq=sequence,
        stamp=_make_time(typestore, timestamp_ns),
        frame_id=frame_id,
    )


def _make_image(
    typestore: Any,
    np: Any,
    event: ExportEvent,
    frame_id: str,
) -> Any:
    info, pixels = read_mono8_image(event.payload)
    image_type = _type(typestore, IMAGE_MSGTYPE)
    return _construct(
        image_type,
        header=_make_header(typestore, event.timestamp_ns, event.sequence, frame_id),
        height=info.height,
        width=info.width,
        encoding="mono8",
        is_bigendian=0,
        step=info.width,
        data=np.frombuffer(pixels, dtype=np.uint8).copy(),
    )


def _vector3(typestore: Any, value: dict[str, Any]) -> Any:
    vector_type = _type(typestore, "geometry_msgs/msg/Vector3")
    return _construct(
        vector_type,
        x=float(value.get("x", 0.0)),
        y=float(value.get("y", 0.0)),
        z=float(value.get("z", 0.0)),
    )


def _make_imu(typestore: Any, np: Any, event: ExportEvent, default_frame_id: str) -> Any:
    payload = event.payload
    quaternion_type = _type(typestore, "geometry_msgs/msg/Quaternion")
    orientation = payload.get("orientation", {})
    quaternion = _construct(
        quaternion_type,
        x=float(orientation.get("x", 0.0)),
        y=float(orientation.get("y", 0.0)),
        z=float(orientation.get("z", 0.0)),
        w=float(orientation.get("w", 0.0)),
    )

    def covariance(name: str, unknown: bool = False) -> Any:
        default = [-1.0] + [0.0] * 8 if unknown else [0.0] * 9
        values = payload.get(name, default)
        if len(values) != 9:
            raise ValueError(f"{name} must contain 9 values")
        return np.asarray(values, dtype=np.float64)

    imu_type = _type(typestore, IMU_MSGTYPE)
    return _construct(
        imu_type,
        header=_make_header(
            typestore,
            event.timestamp_ns,
            event.sequence,
            str(payload.get("frame_id") or default_frame_id),
        ),
        orientation=quaternion,
        orientation_covariance=covariance("orientation_covariance", unknown=True),
        angular_velocity=_vector3(typestore, payload.get("angular_velocity", {})),
        angular_velocity_covariance=covariance("angular_velocity_covariance"),
        linear_acceleration=_vector3(typestore, payload.get("linear_acceleration", {})),
        linear_acceleration_covariance=covariance("linear_acceleration_covariance"),
    )


def _load_session(dataset: Path) -> dict[str, Any]:
    return json.loads((dataset / "meta/session.json").read_text(encoding="utf-8"))


def _merged_events(dataset: Path, include_imu: bool) -> Iterable[ExportEvent]:
    streams: list[Iterator[ExportEvent]] = [
        _read_camera_events(dataset, "cam0", 0),
        _read_camera_events(dataset, "cam1", 1),
    ]
    if include_imu:
        streams.append(_read_imu_events(dataset, 2))
    return heapq.merge(streams[0], *streams[1:], key=lambda event: (event.timestamp_ns, event.order))


def _set_compression(writer: Any, compression: str) -> None:
    if compression == "none":
        return
    enum = getattr(type(writer), "CompressionFormat", None)
    if enum is None:
        try:
            from rosbags.rosbag1.writer import CompressionFormat
        except ImportError as exc:
            raise RuntimeError("installed rosbags does not expose ROS1 compression") from exc
        enum = CompressionFormat
    selected = getattr(enum, compression.upper())
    writer.set_compression(selected)


def _verify_bag(
    path: Path,
    typestore: Any,
    expected_counts: dict[str, int],
    expected_types: dict[str, str],
    expected_ranges: dict[str, dict[str, int]],
) -> dict[str, Any]:
    from rosbags.highlevel import AnyReader

    counts = {topic: 0 for topic in expected_counts}
    first: dict[str, int] = {}
    last: dict[str, int] = {}
    with AnyReader([path], default_typestore=typestore) as reader:
        connections = [connection for connection in reader.connections if connection.topic in counts]
        actual_types = {connection.topic: connection.msgtype for connection in connections}
        if actual_types != expected_types:
            raise RuntimeError(f"bag topic type mismatch: {actual_types} != {expected_types}")
        for connection, timestamp, raw in reader.messages(connections=connections):
            if connection.topic not in counts:
                continue
            if connection.topic in last and timestamp < last[connection.topic]:
                raise RuntimeError(f"bag timestamps are not monotonic on {connection.topic}")
            first.setdefault(connection.topic, timestamp)
            last[connection.topic] = timestamp
            counts[connection.topic] += 1
            message = reader.deserialize(raw, connection.msgtype)
            header = getattr(message, "header", None)
            stamp = getattr(header, "stamp", None)
            if stamp is None:
                raise RuntimeError(f"message on {connection.topic} has no Header stamp")
            header_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
            if header_ns != timestamp:
                raise RuntimeError(
                    f"Header stamp differs from bag record time on {connection.topic}: "
                    f"{header_ns} != {timestamp}"
                )
    if counts != expected_counts:
        raise RuntimeError(f"bag message count mismatch: {counts} != {expected_counts}")
    actual_ranges = {
        topic: {"first": first[topic], "last": last[topic]}
        for topic, count in counts.items()
        if count
    }
    if actual_ranges != expected_ranges:
        raise RuntimeError(
            f"bag first/last timestamp mismatch: {actual_ranges} != {expected_ranges}"
        )
    return {"counts": counts, "first_timestamp_ns": first, "last_timestamp_ns": last}


def export_ros1_bag(
    dataset_root: str | Path,
    destination: str | Path,
    *,
    left_topic: str | None = None,
    right_topic: str | None = None,
    imu_topic: str | None = None,
    compression: str = "none",
    force_dataset: bool = False,
) -> dict[str, Any]:
    if compression not in {"none", "lz4", "bz2"}:
        raise ValueError("compression must be none, lz4, or bz2")
    try:
        import numpy as np
        from rosbags.rosbag1 import Writer
        from rosbags.typesys import Stores, get_typestore
    except ImportError as exc:
        raise RuntimeError(
            "ROS1 export requires numpy and rosbags==0.11.0; install "
            "script/requirements-export.txt from the project root"
        ) from exc
    try:
        rosbags_version = package_version("rosbags")
    except PackageNotFoundError as exc:
        raise RuntimeError("cannot determine the installed rosbags version") from exc
    if rosbags_version != "0.11.0":
        raise RuntimeError(
            f"ROS1 export requires rosbags==0.11.0; installed version is {rosbags_version}"
        )

    dataset = Path(dataset_root).expanduser().resolve()
    validation = validate_dataset(dataset, write_report=True)
    if validation.errors and not force_dataset:
        raise RuntimeError(
            "dataset validation failed; fix it or pass --force-dataset: "
            + "; ".join(validation.errors)
        )
    session = _load_session(dataset)
    config = session.get("config", {})
    include_imu = str(config.get("mode", session.get("mode", ""))) == "camera-imu"
    topics = {
        "cam0": left_topic or str(config.get("left_topic", "/stereo/left/image_raw")),
        "cam1": right_topic or str(config.get("right_topic", "/stereo/right/image_raw")),
        "imu": imu_topic or str(config.get("imu_topic", "/imu/data")),
    }
    active_topics = [topics["cam0"], topics["cam1"]] + ([topics["imu"]] if include_imu else [])
    if len(set(active_topics)) != len(active_topics):
        raise ValueError("ROS1 output topics must be distinct")
    frame_ids = {
        "cam0": str(config.get("left_frame_id", "cam0")),
        "cam1": str(config.get("right_frame_id", "cam1")),
        "imu": str(config.get("imu_frame_id", "imu0")),
    }

    target = Path(destination).expanduser().resolve()
    if target.suffix.lower() != ".bag":
        raise ValueError("ROS1 destination must use the .bag extension")
    if target.exists():
        raise FileExistsError(f"bag destination already exists: {target}")
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = target.with_name(target.stem + ".inprogress.bag")
    if temporary.exists():
        raise FileExistsError(f"temporary bag already exists: {temporary}")

    typestore = get_typestore(Stores.ROS1_NOETIC)
    writer = Writer(temporary)
    _set_compression(writer, compression)
    counts = {topics["cam0"]: 0, topics["cam1"]: 0}
    expected_ranges: dict[str, dict[str, int]] = {}
    if include_imu:
        counts[topics["imu"]] = 0
    try:
        writer.open()
        connections = {
            "cam0": writer.add_connection(topics["cam0"], IMAGE_MSGTYPE, typestore=typestore),
            "cam1": writer.add_connection(topics["cam1"], IMAGE_MSGTYPE, typestore=typestore),
        }
        if include_imu:
            connections["imu"] = writer.add_connection(
                topics["imu"], IMU_MSGTYPE, typestore=typestore
            )
        for event in _merged_events(dataset, include_imu):
            if event.kind in {"cam0", "cam1"}:
                message = _make_image(typestore, np, event, frame_ids[event.kind])
                msgtype = IMAGE_MSGTYPE
            else:
                message = _make_imu(typestore, np, event, frame_ids["imu"])
                msgtype = IMU_MSGTYPE
            raw = typestore.serialize_ros1(message, msgtype)
            writer.write(connections[event.kind], event.timestamp_ns, raw)
            topic = topics[event.kind]
            counts[topic] += 1
            expected_ranges.setdefault(
                topic, {"first": event.timestamp_ns, "last": event.timestamp_ns}
            )["last"] = event.timestamp_ns
        writer.close()
        expected_types = {
            topics["cam0"]: IMAGE_MSGTYPE,
            topics["cam1"]: IMAGE_MSGTYPE,
        }
        if include_imu:
            expected_types[topics["imu"]] = IMU_MSGTYPE
        verification = _verify_bag(
            temporary, typestore, counts, expected_types, expected_ranges
        )
        temporary.replace(target)
        return {
            "status": "pass",
            "dataset": str(dataset),
            "bag": str(target),
            "compression": compression,
            "rosbags_version": rosbags_version,
            "topics": topics,
            **verification,
        }
    except BaseException:
        try:
            writer.close()
        except BaseException:
            pass
        if temporary.exists():
            failed = target.with_name(target.stem + ".failed.bag")
            suffix = 1
            while failed.exists():
                failed = target.with_name(f"{target.stem}.failed.{suffix}.bag")
                suffix += 1
            temporary.replace(failed)
        raise
