"""Merge one episode into a complete MCAP and derive world-frame TCP poses."""

import heapq
import json
import math
import shutil
import time
import uuid
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

from .config import write_json
from .episode_validator import sha256_file
from ..ros.protocol import (
    DIAGNOSTICS_TOPIC,
    JOINT_NAMES,
    MARVIN_JOINT_COMMAND_TOPIC,
    MARVIN_JOINT_STATE_TOPIC,
    MARVIN_TCP_COMMAND_TOPICS,
    MARVIN_TCP_STATE_TOPICS,
    joint_positions,
)


PROJECT_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_URDF = PROJECT_ROOT / "assets" / "marvin" / "marvin_dual.urdf"
DEFAULT_STORAGE_CONFIG = PROJECT_ROOT / "config" / "ros" / "mcap_vision.yaml"
DERIVED_TOPICS = {
    MARVIN_JOINT_STATE_TOPIC: MARVIN_TCP_STATE_TOPICS,
    MARVIN_JOINT_COMMAND_TOPIC: MARVIN_TCP_COMMAND_TOPICS,
}


def rpy_matrix(rpy):
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


def _transform(xyz, rpy):
    result = np.eye(4)
    result[:3, :3] = rpy_matrix(rpy)
    result[:3, 3] = xyz
    return result


def _axis_angle(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    cosine, sine = math.cos(angle), math.sin(angle)
    complement = 1.0 - cosine
    result = np.eye(4)
    result[:3, :3] = (
        (
            cosine + x * x * complement,
            x * y * complement - z * sine,
            x * z * complement + y * sine,
        ),
        (
            y * x * complement + z * sine,
            cosine + y * y * complement,
            y * z * complement - x * sine,
        ),
        (
            z * x * complement - y * sine,
            z * y * complement + x * sine,
            cosine + z * z * complement,
        ),
    )
    return result


def matrix_rpy(rotation):
    horizontal = math.hypot(rotation[0, 0], rotation[1, 0])
    pitch = math.atan2(-rotation[2, 0], horizontal)
    if horizontal > 1e-9:
        roll = math.atan2(rotation[2, 1], rotation[2, 2])
        yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    else:
        roll = math.atan2(-rotation[1, 2], rotation[1, 1])
        yaw = 0.0
    return (roll, pitch, yaw)


def _numbers(element, attribute, default):
    return tuple(float(value) for value in element.get(attribute, default).split())


class UrdfForwardKinematics:
    """Small URDF chain evaluator for Marvin's two seven-axis TCP links."""

    def __init__(self, urdf_path=DEFAULT_URDF):
        joints_by_child = {}
        for joint in ET.parse(urdf_path).getroot().findall("joint"):
            origin = joint.find("origin")
            origin = ET.Element("origin") if origin is None else origin
            axis = joint.find("axis")
            joints_by_child[joint.find("child").get("link")] = (
                joint.get("type"),
                joint.find("parent").get("link"),
                _transform(
                    _numbers(origin, "xyz", "0 0 0"),
                    _numbers(origin, "rpy", "0 0 0"),
                ),
                (1.0, 0.0, 0.0)
                if axis is None
                else _numbers(axis, "xyz", "1 0 0"),
            )
        self._chains = tuple(
            self._chain(joints_by_child, target)
            for target in ("left_tcp", "right_tcp")
        )

    @staticmethod
    def _chain(joints_by_child, target):
        chain = []
        while target in joints_by_child:
            joint = joints_by_child[target]
            chain.append(joint)
            target = joint[1]
        chain.reverse()
        if sum(item[0] in ("revolute", "continuous") for item in chain) != 7:
            raise ValueError("each Marvin TCP chain must contain seven joints")
        return tuple(chain)

    def forward(self, arm, q_rad):
        q_rad = np.asarray(q_rad, dtype=float).reshape(-1)
        if (
            arm not in (0, 1)
            or q_rad.shape != (7,)
            or not np.all(np.isfinite(q_rad))
        ):
            raise ValueError("arm must be 0/1 and q_rad must contain seven finite values")
        result = np.eye(4)
        joint_index = 0
        for joint_type, _parent, origin, axis in self._chains[arm]:
            result = result @ origin
            if joint_type in ("revolute", "continuous"):
                result = result @ _axis_angle(axis, q_rad[joint_index])
                joint_index += 1
        return result


def _header_time_ns(message):
    if hasattr(message, "timestamp"):
        return int(message.timestamp.sec) * 1_000_000_000 + int(message.timestamp.nanosec)
    header = getattr(message, "header", None)
    if header is None and hasattr(message, "image"):
        header = message.image.header
    if header is not None:
        return int(header.stamp.sec) * 1_000_000_000 + int(
            header.stamp.nanosec
        )
    if hasattr(message, "data"):
        try:
            return int(json.loads(message.data).get("wall_time_ns", 0))
        except (AttributeError, TypeError, ValueError):
            pass
    return 0


def _topic_time_ns(topic, message, bag_time_ns):
    """Use acquisition wall time, except diagnostics whose publishers may interleave."""
    if topic == DIAGNOSTICS_TOPIC:
        return int(bag_time_ns), "bag_time_ns"
    # Keep acquisition wall time, never reconstruct it from a monotonic clock.
    header_time_ns = _header_time_ns(message)
    if header_time_ns:
        return header_time_ns, (
            "timestamp" if hasattr(message, "timestamp") else "header.stamp"
            if getattr(message, "header", None) is not None
            or hasattr(message, "image")
            else "payload.wall_time_ns"
        )
    return int(bag_time_ns), "bag_time_ns"


def _matrix_quaternion(rotation):
    """Unit xyzw quaternion, including rotations near pi (no trace division)."""
    r = np.asarray(rotation)
    k = np.array([
        [r[0, 0]-r[1, 1]-r[2, 2], r[0, 1]+r[1, 0], r[0, 2]+r[2, 0], r[2, 1]-r[1, 2]],
        [r[0, 1]+r[1, 0], r[1, 1]-r[0, 0]-r[2, 2], r[1, 2]+r[2, 1], r[0, 2]-r[2, 0]],
        [r[0, 2]+r[2, 0], r[1, 2]+r[2, 1], r[2, 2]-r[0, 0]-r[1, 1], r[1, 0]-r[0, 1]],
        [r[2, 1]-r[1, 2], r[0, 2]-r[2, 0], r[1, 0]-r[0, 1], np.trace(r)],
    ])
    quaternion = np.linalg.eigh(k)[1][:, -1]
    return quaternion if quaternion[3] >= 0 else -quaternion


def _pose_message(message_type, source_message, transform):
    result = message_type()
    result.header.stamp = source_message.header.stamp
    result.header.frame_id = "world"
    result.pose.position.x, result.pose.position.y, result.pose.position.z = transform[:3, 3].tolist()
    (result.pose.orientation.x, result.pose.orientation.y,
     result.pose.orientation.z, result.pose.orientation.w) = _matrix_quaternion(transform[:3, :3]).tolist()
    return result


def _input_bag_paths(episode_directory, metadata):
    state_path = episode_directory / "state"
    if not (state_path / "metadata.yaml").is_file():
        raise FileNotFoundError(f"state bag not found: {state_path}")
    declared_bags = metadata.get("bags")
    input_paths = []
    for bag_name in declared_bags or ["state", "vision"]:
        input_path = episode_directory / bag_name
        if (input_path / "metadata.yaml").is_file():
            input_paths.append(input_path)
        elif declared_bags or bag_name == "state":
            raise FileNotFoundError(f"declared input bag not found: {input_path}")
    return input_paths


def _topic_cursors(input_paths, rosbag2_py, get_message):
    topic_metadata = {}
    topic_input_paths = {}
    for input_path in input_paths:
        reader = rosbag2_py.SequentialReader()
        reader.open(
            rosbag2_py.StorageOptions(uri=str(input_path), storage_id="mcap"),
            rosbag2_py.ConverterOptions("", ""),
        )
        for item in reader.get_all_topics_and_types():
            if item.type.startswith("teleop_msgs/"):
                raise ValueError(
                    "legacy teleop_msgs bag: run "
                    "uv run teleop-migrate-messages first"
                )
            previous = topic_metadata.get(item.name)
            if previous is not None and previous.type != item.type:
                raise ValueError(f"topic type mismatch for {item.name}")
            if previous is not None:
                raise ValueError(f"topic occurs in multiple input bags: {item.name}")
            topic_metadata[item.name] = item
            topic_input_paths[item.name] = input_path

    message_types = {
        name: get_message(item.type) for name, item in topic_metadata.items()
    }
    cursors = []
    for topic, input_path in topic_input_paths.items():
        reader = rosbag2_py.SequentialReader()
        reader.open(
            rosbag2_py.StorageOptions(uri=str(input_path), storage_id="mcap"),
            rosbag2_py.ConverterOptions("", ""),
        )
        reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
        cursors.append(
            {
                "reader": reader,
                "topic": topic,
                "message_type": message_types[topic],
            }
        )
    return topic_metadata, cursors


def postprocess_episode(
    episode_directory,
    output_path=None,
    urdf_path=DEFAULT_URDF,
    storage_config_path=DEFAULT_STORAGE_CONFIG,
):
    """Copy state/vision streams into one MCAP and append derived TCP streams."""
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message, serialize_message
        from rosidl_runtime_py.utilities import get_message
        from geometry_msgs.msg import PoseStamped
    except (ImportError, OSError) as error:
        raise RuntimeError(
            "post-processing requires sourced ROS2 and foxglove_msgs"
        ) from error

    episode_directory = Path(episode_directory).expanduser().resolve()
    metadata_path = episode_directory / "metadata.json"
    metadata = (
        json.loads(metadata_path.read_text(encoding="utf-8"))
        if metadata_path.is_file()
        else {}
    )
    input_paths = _input_bag_paths(episode_directory, metadata)
    output_path = (
        episode_directory / "data"
        if output_path is None
        else Path(output_path).expanduser().resolve()
    )
    if output_path.exists():
        raise FileExistsError(f"output bag already exists: {output_path}")
    if any(
        output_path == path or output_path.is_relative_to(path)
        for path in input_paths
    ):
        raise ValueError("output bag must not be inside an input bag")
    urdf_path = Path(urdf_path).expanduser().resolve()
    storage_config_path = Path(storage_config_path).expanduser().resolve()
    if not urdf_path.is_file() or not storage_config_path.is_file():
        raise FileNotFoundError("URDF or MCAP storage configuration is missing")

    topic_metadata, cursors = _topic_cursors(
        input_paths, rosbag2_py, get_message
    )

    kinematics = UrdfForwardKinematics(urdf_path)
    temporary_root = output_path.parent / f".postprocess-{uuid.uuid4().hex}"
    temporary_path = temporary_root / output_path.name
    temporary_root.mkdir(parents=True)
    writer = rosbag2_py.SequentialWriter()
    counts = {name: 0 for name in topic_metadata}
    alignment_source_counts = {name: {} for name in topic_metadata}
    bag_delay_stats = {
        name: {"count": 0, "minimum_ns": None, "maximum_ns": None, "sum_ns": 0}
        for name in topic_metadata
    }
    for names in DERIVED_TOPICS.values():
        counts.update((name, 0) for name in names)
    opened = False
    try:
        writer.open(
            rosbag2_py.StorageOptions(
                uri=str(temporary_path),
                storage_id="mcap",
                storage_config_uri=str(storage_config_path),
            ),
            rosbag2_py.ConverterOptions("", ""),
        )
        opened = True
        for item in topic_metadata.values():
            writer.create_topic(item)
        for name in (name for names in DERIVED_TOPICS.values() for name in names):
            writer.create_topic(
                rosbag2_py.TopicMetadata(
                    name=name,
                    type="geometry_msgs/msg/PoseStamped",
                    serialization_format="cdr",
                )
            )

        pending = []

        def push_next(index):
            cursor = cursors[index]
            reader = cursor["reader"]
            if not reader.has_next():
                return
            topic, serialized, bag_time_ns = reader.read_next()
            if topic != cursor["topic"]:
                raise RuntimeError(f"unexpected filtered topic: {topic}")
            message = deserialize_message(serialized, cursor["message_type"])
            aligned_time_ns, alignment_source = _topic_time_ns(topic, message, bag_time_ns)
            # System time can step backwards. Preserve it; validation reports it.
            sources = alignment_source_counts[topic]
            sources[alignment_source] = sources.get(alignment_source, 0) + 1
            delay_ns = int(bag_time_ns) - aligned_time_ns
            delay = bag_delay_stats[topic]
            delay["count"] += 1
            delay["sum_ns"] += delay_ns
            delay["minimum_ns"] = (
                delay_ns
                if delay["minimum_ns"] is None
                else min(delay["minimum_ns"], delay_ns)
            )
            delay["maximum_ns"] = (
                delay_ns
                if delay["maximum_ns"] is None
                else max(delay["maximum_ns"], delay_ns)
            )
            heapq.heappush(
                pending,
                (
                    aligned_time_ns,
                    index,
                    serialized,
                    message,
                ),
            )

        for index in range(len(cursors)):
            push_next(index)
        while pending:
            aligned_time_ns, index, serialized, message = (
                heapq.heappop(pending)
            )
            topic = cursors[index]["topic"]
            writer.write(topic, serialized, aligned_time_ns)
            counts[topic] += 1
            if topic in DERIVED_TOPICS:
                positions = joint_positions(message)
                for arm, target_topic in enumerate(DERIVED_TOPICS[topic]):
                    transform = kinematics.forward(
                        arm, positions[arm * 7 : (arm + 1) * 7]
                    )
                    pose = _pose_message(
                        PoseStamped, message, transform
                    )
                    writer.write(
                        target_topic, serialize_message(pose), aligned_time_ns
                    )
                    counts[target_topic] += 1
            push_next(index)
        writer.close()
        opened = False
        temporary_path.replace(output_path)
        temporary_root.rmdir()
    except Exception:
        if opened:
            writer.close()
        shutil.rmtree(temporary_root, ignore_errors=True)
        raise

    for item in bag_delay_stats.values():
        total_ns = item.pop("sum_ns")
        item["mean_ns"] = 0 if not item["count"] else total_ns // item["count"]
    summary = {
        "schema_version": 3,
        "message_protocol_version": 2,
        "derived_sources": {target: source for source, targets in DERIVED_TOPICS.items() for target in targets},
        "processed_at_ns": time.time_ns(),
        "input_bags": [
            str(path.relative_to(episode_directory)) for path in input_paths
        ],
        "output_bag": str(output_path),
        "urdf": str(urdf_path),
        "urdf_sha256": sha256_file(urdf_path),
        "alignment": {
            "clock": "CLOCK_REALTIME",
            # Retain schema fields so historical readers can distinguish old alignment.
            "steady_to_wall_offset_ns": None,
            "calibration_samples": 0,
            "calibration_span_ns": 0,
            "topic_time_offsets_ns": {},
            "source_counts": alignment_source_counts,
            "original_bag_delay": bag_delay_stats,
        },
        "topic_counts": counts,
    }
    if metadata_path.is_file() and output_path == episode_directory / "data":
        metadata["processed_bag"] = output_path.name
        metadata["postprocessing"] = summary
        write_json(metadata_path, metadata)
    return summary
