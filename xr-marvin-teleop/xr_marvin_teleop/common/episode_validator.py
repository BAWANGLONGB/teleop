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


def inspect_bag(
    path,
    storage_id="mcap",
    expected_steady_offset_ns=None,
    expected_topic_offsets_ns=None,
):
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
            "steady_alignment_errors": 0,
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
        steady_time = int(
            getattr(
                message,
                "receive_steady_ns",
                getattr(message, "issue_steady_ns", 0),
            )
        )
        if (
            steady_time
            and expected_steady_offset_ns is not None
            and bag_time_ns
            != steady_time
            + expected_steady_offset_ns
            - (expected_topic_offsets_ns or {}).get(topic, 0)
        ):
            item["steady_alignment_errors"] += 1
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
    metadata_path = episode_directory / "metadata.json"
    metadata = (
        json.loads(metadata_path.read_text(encoding="utf-8"))
        if metadata_path.is_file()
        else {}
    )
    processed_bag = metadata.get("processed_bag")
    steady_offset_ns = (
        metadata.get("postprocessing", {})
        .get("alignment", {})
        .get("steady_to_wall_offset_ns")
    )
    topic_offsets_ns = (
        metadata.get("postprocessing", {})
        .get("alignment", {})
        .get("topic_time_offsets_ns", {})
    )
    bags = {}
    errors = []
    degraded = []
    declared_bags = metadata.get("bags")
    bag_names = list(declared_bags or ("state", "vision"))
    if processed_bag and processed_bag not in bag_names:
        bag_names.append(processed_bag)
    elif not processed_bag and (episode_directory / "data").exists():
        bag_names.append("data")
    for bag_name in bag_names:
        bag_path = episode_directory / bag_name
        if not bag_path.exists():
            if bag_name == "state" or declared_bags or bag_name == processed_bag:
                errors.append(f"{bag_name} bag is missing")
            continue
        try:
            bags[bag_name] = inspect_bag(
                bag_path,
                expected_steady_offset_ns=(
                    steady_offset_ns if bag_name == processed_bag else None
                ),
                expected_topic_offsets_ns=(
                    topic_offsets_ns if bag_name == processed_bag else None
                ),
            )
        except Exception as error:
            errors.append(f"{bag_name} bag unreadable: {error}")
    state_topics = bags.get("state", {})
    for topic in required_topics:
        if state_topics.get(topic, {}).get("count", 0) == 0:
            errors.append(f"required topic has no messages: {topic}")
    vision_topics = bags.get("vision")
    processed_vision_topics = []
    if vision_topics is not None:
        for topic in ("/raw/das/left/image", "/raw/das/right/image"):
            if vision_topics.get(topic, {}).get("count", 0) == 0:
                errors.append(f"required vision topic has no messages: {topic}")
            processed_vision_topics.append(topic)
    for side in ("left", "right"):
        bag_name = f"vision_{side}"
        if bag_name not in bags:
            continue
        topic = f"/raw/das/{side}/image/compressed"
        if bags[bag_name].get(topic, {}).get("count", 0) == 0:
            errors.append(f"required vision topic has no messages: {topic}")
        processed_vision_topics.append(topic)
    if processed_bag:
        data_topics = bags.get(processed_bag, {})
        processed_required_topics = [
            *required_topics,
            "/raw/marvin/left/tcp_pose",
            "/raw/marvin/right/tcp_pose",
            "/command/marvin/left/tcp_target",
            "/command/marvin/right/tcp_target",
        ]
        processed_required_topics.extend(processed_vision_topics)
        for topic in processed_required_topics:
            if data_topics.get(topic, {}).get("count", 0) == 0:
                errors.append(f"processed topic has no messages: {topic}")
    for bag_name, topics in bags.items():
        for topic, item in topics.items():
            if (
                item["bag_time_regressions"]
                or item["source_time_regressions"]
                or item["sequence_gaps"]
                or item.get("steady_alignment_errors", 0)
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
