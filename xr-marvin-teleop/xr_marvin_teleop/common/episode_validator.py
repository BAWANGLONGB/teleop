"""Offline integrity checks for state/vision rosbag2 episode data."""

import hashlib
import json
import time
from pathlib import Path


def _sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def inspect_bag(path, storage_id="mcap"):
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except (ImportError, OSError) as error:
        raise RuntimeError("validation requires a sourced ROS2 environment") from error

    path = Path(path)
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id=storage_id),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {
        item.name: item.type for item in reader.get_all_topics_and_types()
    }
    message_types = {}
    statistics = {
        topic: {
            "type": type_name,
            "count": 0,
            "first_bag_time_ns": None,
            "last_bag_time_ns": None,
            "bag_time_regressions": 0,
            "source_time_regressions": 0,
            "sequence_gaps": 0,
        }
        for topic, type_name in topic_types.items()
    }
    last_bag_time = {}
    last_source_time = {}
    last_sequence = {}
    while reader.has_next():
        topic, serialized, bag_time_ns = reader.read_next()
        item = statistics[topic]
        item["count"] += 1
        if item["first_bag_time_ns"] is None:
            item["first_bag_time_ns"] = int(bag_time_ns)
        item["last_bag_time_ns"] = int(bag_time_ns)
        if bag_time_ns < last_bag_time.get(topic, bag_time_ns):
            item["bag_time_regressions"] += 1
        last_bag_time[topic] = bag_time_ns
        try:
            if topic not in message_types:
                message_types[topic] = get_message(topic_types[topic])
            message_type = message_types[topic]
            message = deserialize_message(serialized, message_type)
        except Exception:
            continue
        source_time = int(getattr(message, "source_timestamp_ns", 0))
        if source_time:
            if source_time < last_source_time.get(topic, source_time):
                item["source_time_regressions"] += 1
            last_source_time[topic] = source_time
        sequence = int(getattr(message, "sequence_id", 0))
        if sequence:
            previous = last_sequence.get(topic)
            if previous is not None and sequence > previous + 1:
                item["sequence_gaps"] += sequence - previous - 1
            elif previous is not None and sequence <= previous:
                item["sequence_gaps"] += 1
            last_sequence[topic] = sequence
    for item in statistics.values():
        duration_ns = (
            0
            if item["count"] < 2
            else item["last_bag_time_ns"] - item["first_bag_time_ns"]
        )
        item["duration_ns"] = duration_ns
        item["mean_rate_hz"] = (
            0.0 if duration_ns <= 0 else (item["count"] - 1) * 1e9 / duration_ns
        )
    return statistics


def validate_episode(
    episode_directory,
    required_topics=(
        "/raw/pico/frame",
        "/raw/marvin/joint_state",
        "/command/marvin/joint_target",
    ),
):
    episode_directory = Path(episode_directory).resolve()
    bags = {}
    errors = []
    degraded = []
    for bag_name in ("state", "vision"):
        bag_path = episode_directory / bag_name
        if not bag_path.exists():
            if bag_name == "state":
                errors.append("state bag is missing")
            continue
        try:
            bags[bag_name] = inspect_bag(bag_path)
        except Exception as error:
            errors.append(f"{bag_name} bag unreadable: {error}")
    state_topics = bags.get("state", {})
    for topic in required_topics:
        if state_topics.get(topic, {}).get("count", 0) == 0:
            errors.append(f"required topic has no messages: {topic}")
    vision_topics = bags.get("vision")
    if vision_topics is not None:
        for topic in ("/raw/das/left/image", "/raw/das/right/image"):
            if vision_topics.get(topic, {}).get("count", 0) == 0:
                errors.append(f"required vision topic has no messages: {topic}")
    for bag_name, topics in bags.items():
        for topic, item in topics.items():
            if (
                item["bag_time_regressions"]
                or item["source_time_regressions"]
                or item["sequence_gaps"]
            ):
                degraded.append(f"{bag_name}:{topic}")
    files = {
        str(path.relative_to(episode_directory)): {
            "size_bytes": path.stat().st_size,
            "sha256": _sha256(path),
        }
        for path in sorted(episode_directory.rglob("*"))
        if path.is_file() and path.name != "manifest.json"
    }
    status = "rejected" if errors else "degraded" if degraded else "validated"
    manifest = {
        "schema_version": 1,
        "validated_at_ns": time.time_ns(),
        "status": status,
        "errors": errors,
        "degraded_topics": degraded,
        "bags": bags,
        "files": files,
    }
    temporary_path = episode_directory / "manifest.json.tmp"
    temporary_path.write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    temporary_path.replace(episode_directory / "manifest.json")
    return manifest
