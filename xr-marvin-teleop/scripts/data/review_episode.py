#!/usr/bin/env python3
"""Render an episode's aligned cameras and robot/gripper state to MP4."""

import argparse
import bisect
import itertools
import math
import os
from pathlib import Path


LEFT_IMAGE = "/raw/das/left/image/compressed"
RIGHT_IMAGE = "/raw/das/right/image/compressed"
ROBOT_STATE = "/raw/marvin/joint_state"
JOINT_COMMAND = "/command/marvin/joint_target"
GRIPPER_COMMAND = "/command/das/target"
DAS_STATE = {
    "left": "/raw/das/left/state",
    "right": "/raw/das/right/state",
}
TCP_STATE = {
    "left": "/raw/marvin/left/tcp_pose",
    "right": "/raw/marvin/right/tcp_pose",
}
TCP_COMMAND = {
    "left": "/command/marvin/left/tcp_target",
    "right": "/command/marvin/right/tcp_target",
}
STATE_TOPICS = (
    ROBOT_STATE,
    JOINT_COMMAND,
    GRIPPER_COMMAND,
    *DAS_STATE.values(),
    *TCP_STATE.values(),
    *TCP_COMMAND.values(),
)


def _bag_path(path):
    path = Path(path).expanduser().resolve()
    if (path / "data" / "metadata.yaml").is_file():
        return path / "data", path
    if (path / "metadata.yaml").is_file():
        return path, path.parent
    raise FileNotFoundError(
        f"processed data bag not found under {path}; finish post-processing first"
    )


def _messages(path, topics):
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except (ImportError, OSError) as error:
        raise RuntimeError(
            "review requires sourced ROS2, rosbag2_py, and built teleop_msgs"
        ) from error

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    type_names = {
        item.name: item.type for item in reader.get_all_topics_and_types()
    }
    selected = [topic for topic in topics if topic in type_names]
    if not selected:
        return
    message_types = {
        topic: get_message(type_names[topic]) for topic in selected
    }
    reader.set_filter(rosbag2_py.StorageFilter(topics=selected))
    while reader.has_next():
        topic, serialized, timestamp_ns = reader.read_next()
        yield topic, deserialize_message(serialized, message_types[topic]), int(
            timestamp_ns
        )


def _state_value(topic, message):
    if topic == ROBOT_STATE:
        return tuple(message.q_rad), tuple(message.dq_rad_s), bool(message.valid)
    if topic == JOINT_COMMAND:
        return tuple(message.q_rad)
    if topic == GRIPPER_COMMAND:
        return tuple(message.closedness)
    if topic in DAS_STATE.values():
        return (
            float(message.distance_m),
            float(message.target_distance_m),
            bool(message.valid),
        )
    return (
        tuple(message.xyz_m),
        tuple(message.rpy_rad),
        bool(message.valid),
    )


def _load_state(path):
    # ponytail: state-only index is small for normal episodes; stream/index it
    # on disk if multi-hour recordings ever outgrow RAM.
    series = {topic: ([], []) for topic in STATE_TOPICS}
    for topic, message, timestamp_ns in _messages(path, STATE_TOPICS):
        times, values = series[topic]
        times.append(timestamp_ns)
        values.append(_state_value(topic, message))
    return series


def _nearest(series, timestamp_ns):
    times, values = series
    if not times:
        return None, None
    index = bisect.bisect_left(times, timestamp_ns)
    candidates = [item for item in (index - 1, index) if 0 <= item < len(times)]
    index = min(candidates, key=lambda item: abs(times[item] - timestamp_ns))
    return values[index], (times[index] - timestamp_ns) / 1e6


def _previous(series, timestamp_ns):
    times, values = series
    index = bisect.bisect_right(times, timestamp_ns) - 1
    if index < 0:
        return None, None
    return values[index], (timestamp_ns - times[index]) / 1e6


def _camera_pairs(path):
    left_messages = _messages(path, (LEFT_IMAGE,))
    right_messages = _messages(path, (RIGHT_IMAGE,))
    try:
        first_left = next(left_messages)
        right_after = next(right_messages)
    except StopIteration as error:
        raise ValueError(
            "processed bag must contain both compressed camera topics"
        ) from error
    right_before = None
    for left in itertools.chain((first_left,), left_messages):
        left_time = left[2]
        while right_after is not None and right_after[2] <= left_time:
            right_before = right_after
            try:
                right_after = next(right_messages)
            except StopIteration:
                right_after = None
        choices = [item for item in (right_before, right_after) if item is not None]
        right = min(choices, key=lambda item: abs(item[2] - left_time))
        yield left, right, first_left[2]


def _degrees(values):
    return tuple(math.degrees(value) for value in values)


def _vector(values, digits=1):
    return " ".join(f"{value:.{digits}f}" for value in values)


def _state_lines(state, timestamp_ns, zero_ns, right_delta_ms):
    robot, robot_delta = _nearest(state[ROBOT_STATE], timestamp_ns)
    joint_target, joint_age = _previous(state[JOINT_COMMAND], timestamp_ns)
    gripper_target, gripper_age = _previous(
        state[GRIPPER_COMMAND], timestamp_ns
    )
    lines = [
        f"t={(timestamp_ns - zero_ns) / 1e9:.3f}s  right-left={right_delta_ms:+.2f}ms"
    ]
    if robot is None:
        lines.extend(("robot feedback: missing",) * 5)
    else:
        q_rad, dq_rad_s, valid = robot
        lines.extend(
            (
                f"robot feedback dt={robot_delta:+.2f}ms valid={valid}",
                "q L deg:  " + _vector(_degrees(q_rad[:7])),
                "q R deg:  " + _vector(_degrees(q_rad[7:])),
                "dq L d/s: " + _vector(_degrees(dq_rad_s[:7])),
                "dq R d/s: " + _vector(_degrees(dq_rad_s[7:])),
            )
        )
    if joint_target is None:
        lines.extend(("q target L: missing", "q target R: missing"))
    else:
        lines.extend(
            (
                f"q target L deg age={joint_age:.2f}ms: "
                + _vector(_degrees(joint_target[:7])),
                "q target R deg: " + _vector(_degrees(joint_target[7:])),
            )
        )
    if gripper_target is None:
        lines.append("gripper command: missing")
    else:
        lines.append(
            f"gripper closedness age={gripper_age:.2f}ms: "
            f"L={gripper_target[0]:.3f} R={gripper_target[1]:.3f}"
        )
    for side in ("left", "right"):
        value, delta = _nearest(state[DAS_STATE[side]], timestamp_ns)
        if value is None:
            lines.append(f"gripper {side}: missing")
        else:
            distance, target, valid = value
            lines.append(
                f"gripper {side} dt={delta:+.2f}ms valid={valid}: "
                f"actual={distance:.4f}m target={target:.4f}m"
            )
    for side in ("left", "right"):
        value, delta = _nearest(state[TCP_STATE[side]], timestamp_ns)
        if value is None:
            lines.append(f"TCP {side}: missing")
        else:
            xyz, rpy, valid = value
            lines.append(
                f"TCP {side} dt={delta:+.2f}ms valid={valid}: xyz[m] "
                f"{_vector(xyz, 3)} rpy[deg] {_vector(_degrees(rpy))}"
            )
    for side in ("left", "right"):
        value, age = _previous(state[TCP_COMMAND[side]], timestamp_ns)
        if value is None:
            lines.append(f"TCP target {side}: missing")
        else:
            xyz, rpy, _valid = value
            lines.append(
                f"TCP target {side} age={age:.2f}ms: xyz[m] "
                f"{_vector(xyz, 3)} rpy[deg] {_vector(_degrees(rpy))}"
            )
    return lines


def _decode(message):
    import cv2
    import numpy as np

    frame = cv2.imdecode(
        np.frombuffer(bytes(message.image.data), dtype=np.uint8),
        cv2.IMREAD_COLOR,
    )
    if frame is None:
        raise ValueError("MCAP contains an invalid MJPEG frame")
    return frame


def _review_frame(left_message, right_message, lines):
    import cv2
    import numpy as np

    left = _decode(left_message)
    right = cv2.resize(
        _decode(right_message),
        (left.shape[1], left.shape[0]),
        interpolation=cv2.INTER_AREA,
    )
    cv2.putText(left, "LEFT", (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    cv2.putText(
        right, "RIGHT", (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2
    )
    images = np.hstack((left, right))
    line_height = 20
    panel = np.zeros((line_height * len(lines) + 16, images.shape[1], 3), np.uint8)
    for index, line in enumerate(lines):
        cv2.putText(
            panel,
            line,
            (8, 18 + index * line_height),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.43,
            (235, 235, 235),
            1,
            cv2.LINE_AA,
        )
    return np.vstack((images, panel))


def render_review(bag_path, output_path, fps=60.0, start_seconds=0.0, duration=None):
    import cv2

    fps = float(fps)
    start_seconds = float(start_seconds)
    if not math.isfinite(fps) or fps <= 0.0:
        raise ValueError("fps must be positive and finite")
    if not math.isfinite(start_seconds) or start_seconds < 0.0:
        raise ValueError("start must be non-negative and finite")
    if duration is not None and (
        not math.isfinite(duration) or duration <= 0.0
    ):
        raise ValueError("duration must be positive and finite")

    output_path = Path(output_path).expanduser().resolve()
    if output_path.suffix.lower() != ".mp4":
        raise ValueError("review output must use the .mp4 extension")
    if output_path.exists():
        raise FileExistsError(f"review output already exists: {output_path}")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    temporary_path = output_path.with_name(
        f".{output_path.stem}.{os.getpid()}.tmp.mp4"
    )
    state = _load_state(bag_path)
    writer = None
    previous_frame = None
    source_frames = 0
    written_frames = 0
    first_selected_time = None
    try:
        for left, right, zero_ns in _camera_pairs(bag_path):
            timestamp_ns = left[2]
            elapsed_seconds = (timestamp_ns - zero_ns) / 1e9
            if elapsed_seconds < start_seconds:
                continue
            if duration is not None and elapsed_seconds >= start_seconds + duration:
                break
            if first_selected_time is None:
                first_selected_time = timestamp_ns
            lines = _state_lines(
                state,
                timestamp_ns,
                zero_ns,
                (right[2] - timestamp_ns) / 1e6,
            )
            frame = _review_frame(left[1], right[1], lines)
            if writer is None:
                writer = cv2.VideoWriter(
                    str(temporary_path),
                    cv2.VideoWriter_fourcc(*"mp4v"),
                    fps,
                    (frame.shape[1], frame.shape[0]),
                )
                if not writer.isOpened():
                    raise RuntimeError("OpenCV could not open the MP4 video writer")
            target_index = round((timestamp_ns - first_selected_time) / 1e9 * fps)
            while previous_frame is not None and written_frames < target_index:
                writer.write(previous_frame)
                written_frames += 1
            if written_frames <= target_index:
                writer.write(frame)
                written_frames += 1
            previous_frame = frame
            source_frames += 1
            if source_frames % 300 == 0:
                print(f"Rendered {source_frames} camera frames...", flush=True)
        if writer is None:
            raise ValueError("no camera frames fall within the selected interval")
        writer.release()
        writer = None
        temporary_path.replace(output_path)
    finally:
        if writer is not None:
            writer.release()
        temporary_path.unlink(missing_ok=True)
    return source_frames, written_frames


def main(arguments=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("episode", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--fps", type=float, default=60.0)
    parser.add_argument("--start", type=float, default=0.0, metavar="SECONDS")
    parser.add_argument("--duration", type=float, metavar="SECONDS")
    parsed = parser.parse_args(arguments)
    bag, episode = _bag_path(parsed.episode)
    output = episode / "review.mp4" if parsed.output is None else parsed.output
    source_frames, written_frames = render_review(
        bag, output, parsed.fps, parsed.start, parsed.duration
    )
    print(
        f"Review video written: {Path(output).expanduser().resolve()} "
        f"({source_frames} source frames, {written_frames} video frames)",
        flush=True,
    )


if __name__ == "__main__":
    main()
