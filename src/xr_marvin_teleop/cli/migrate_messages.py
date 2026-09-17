#!/usr/bin/env python3
"""Copy a legacy raw Episode to v2; requires the archived teleop_msgs only here."""

import argparse
import json
from pathlib import Path
import shutil
import tempfile
import uuid
from dataclasses import asdict

from xr_marvin_teleop.ros.protocol import (
    JOINT_NAMES, GRIPPER_NAMES, PICO_TOPICS, joint_state, trajectory, pico_messages,
    tactile_grid, sample_status, status_topic, stamp_ns,
)
from xr_marvin_teleop.adapters.das_finger import load_das_finger_configurations
from xr_marvin_teleop.adapters.xr import XrSnapshot


def convert(topic, message, session, configurations):
    name = type(message).__name__
    frame = message.image if name in ("ImageFrame", "CompressedImageFrame") else message
    timestamp = stamp_ns(frame)
    extra = {}
    if name == "PicoFrame":
        snapshot = (XrSnapshot(message.source_timestamp_ns, message.left_controller_pose,
                              message.right_controller_pose, tuple(message.grip_values),
                              message.button_a, message.button_b, tuple(message.trigger_values),
                              tuple(message.thumbstick_y_values)) if message.valid else None)
        payloads = list(zip(PICO_TOPICS, pico_messages(snapshot, timestamp)))
        extra = dict(source_clock="openxr", unavailable_buttons=["x", "y"])
    elif name == "MarvinState":
        payloads = [(topic, joint_state(message.q_rad, JOINT_NAMES, timestamp, message.dq_rad_s))]
        extra = {key: list(getattr(message, key)) for key in ("frame_serial", "arm_state", "error_code", "low_speed")}
    elif name == "JointCommand":
        payloads = [(topic, trajectory(message.q_rad, JOINT_NAMES, timestamp))]
    elif name == "GripperCommand":
        if configurations is None:
            raise ValueError("legacy gripper commands require --das-config matching the recording calibration")
        payloads = [(topic, trajectory([1.0 - value for value in message.closedness], GRIPPER_NAMES, timestamp))]
    elif name == "DasState":
        side = ("left", "right").index(message.side)
        payloads = [(topic, joint_state([message.distance_m], [GRIPPER_NAMES[side]], timestamp))]
        extra = dict(target_distance_m=message.target_distance_m, status_flags=message.status_flags)
    elif name == "TactileFrame":
        payloads = [(topic, tactile_grid(bytes(message.data), message.side, timestamp))]
    elif name in ("ImageFrame", "CompressedImageFrame"):
        side = topic.split("/")[3]
        frame.header.frame_id = f"das_{side}_camera_optical_frame"
        payloads = [(topic, frame)]
        extra = dict(source_clock="legacy_camera_unspecified")
    else:
        raise ValueError(f"unsupported legacy raw schema: {name}")
    targets = [target for target, _ in payloads]
    status = sample_status(targets, timestamp, session, message.sequence_id,
                           getattr(message, "receive_steady_ns", getattr(message, "issue_steady_ns", 0)),
                           valid=getattr(message, "valid", True),
                           source_timestamp_ns=getattr(message, "source_timestamp_ns", 0),
                           command=hasattr(message, "issue_steady_ns"), **extra)
    return [*payloads, (status_topic(targets[0]), status)]


def migrate(source, output, configurations=None):
    import rosbag2_py
    from rclpy.serialization import deserialize_message, serialize_message
    from rosidl_runtime_py.utilities import get_message

    source, output = Path(source).resolve(), Path(output).absolute()
    if output.exists() or output.is_symlink() or output.resolve().is_relative_to(source):
        raise ValueError("output must be a new directory outside the source Episode")
    metadata = json.loads((source / "metadata.json").read_text())
    if metadata.get("message_protocol_version") == 2:
        raise ValueError("source already uses message protocol v2")
    bags = metadata.get("bags") or [name for name in ("state", "vision", "vision_left", "vision_right")
                                    if (source / name / "metadata.yaml").is_file()]
    if "state" not in bags or any(name not in ("state", "vision", "vision_left", "vision_right") for name in bags):
        raise ValueError("invalid source raw bag list")
    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix=".messages-v2-", dir=output.parent) as directory:
        staging = Path(directory) / "episode"
        staging.mkdir()
        session = uuid.uuid4().hex
        for bag in bags:
            reader = rosbag2_py.SequentialReader()
            reader.open(rosbag2_py.StorageOptions(uri=str(source / bag), storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
            types = {item.name: item.type for item in reader.get_all_topics_and_types()}
            classes = {topic: get_message(name) for topic, name in types.items()}
            writer = rosbag2_py.SequentialWriter()
            writer.open(rosbag2_py.StorageOptions(uri=str(staging / bag), storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
            registered, epochs, previous = {}, {}, {}
            try:
                while reader.has_next():
                    topic, data, timestamp = reader.read_next()
                    message = deserialize_message(data, classes[topic])
                    if types[topic].startswith("teleop_msgs/"):
                        sequence = int(message.sequence_id)
                        if sequence == 1 and previous.get(topic, 0) > 1:
                            epochs[topic] = epochs.get(topic, 0) + 1
                        previous[topic] = sequence
                        records = convert(topic, message, f"{session}:{topic}:{epochs.get(topic, 0)}", configurations)
                    else:
                        records = [(topic, message)]
                    for target, converted in records:
                        name = type(converted).__module__.split(".")[0] + "/msg/" + type(converted).__name__
                        if target not in registered:
                            writer.create_topic(rosbag2_py.TopicMetadata(name=target, type=name, serialization_format="cdr"))
                            registered[target] = name
                        elif registered[target] != name:
                            raise ValueError(f"conflicting migrated schemas on {target}")
                        writer.write(target, serialize_message(converted), timestamp)
            finally:
                writer.close()
        for entry in [*metadata.get("calibrations", []), *metadata.get("config_files", [])]:
            relative = Path(entry["snapshot"])
            original = (source / relative).resolve()
            if relative.is_absolute() or ".." in relative.parts or not original.is_relative_to(source):
                raise ValueError("unsafe snapshot path")
            target = staging / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(original, target)
        for key in ("processed_bag", "postprocessing", "final_outputs", "export_options", "validation"):
            metadata.pop(key, None)
        metadata.update(message_protocol_version=2, bags=bags, export_status="pending",
                        migration={"source": str(source), "source_protocol_version": 1,
                                   "gripper_configuration": None if configurations is None else [asdict(c) for c in configurations],
                                   "unavailable_pico_buttons": ["x", "y"]})
        (staging / "metadata.json").write_text(json.dumps(metadata, ensure_ascii=False, indent=2))
        from xr_marvin_teleop.collection.episode_validator import validate_episode
        manifest = validate_episode(staging)
        if manifest["status"] == "rejected":
            raise ValueError(f"migrated Episode failed validation: {manifest['errors']}")
        staging.rename(output)
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--das-config", type=Path, help="original recording's gripper calibration")
    args = parser.parse_args()
    print(migrate(args.source, args.output, load_das_finger_configurations(args.das_config) if args.das_config else None))


if __name__ == "__main__":
    main()
