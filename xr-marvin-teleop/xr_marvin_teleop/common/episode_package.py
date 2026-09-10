"""Build and read one self-contained LeRobot episode MCAP."""

import bisect
import json
import math
import shutil
import struct
import tempfile
import time
import zlib
from pathlib import Path, PurePosixPath


MAGIC = b"\x89MCAP0\r\n"
ATTACHMENT = 0x09
DATA_END = 0x0F
FOOTER = 0x02
ROBOT_STATE = "/raw/marvin/joint_state"
JOINT_COMMAND = "/command/marvin/joint_target"
GRIPPER_COMMAND = "/command/das/target"
PICO = "/raw/pico/frame"
DAS_STATE = {side: f"/raw/das/{side}/state" for side in ("left", "right")}
TACTILE = {side: f"/raw/das/{side}/tactile" for side in ("left", "right")}
TCP_STATE = {
    side: f"/raw/marvin/{side}/tcp_pose" for side in ("left", "right")
}
TCP_COMMAND = {
    side: f"/command/marvin/{side}/tcp_target" for side in ("left", "right")
}
CAMERA = {
    side: f"/raw/das/{side}/image/compressed" for side in ("left", "right")
}
STATE_TOPICS = (
    ROBOT_STATE,
    JOINT_COMMAND,
    GRIPPER_COMMAND,
    PICO,
    *DAS_STATE.values(),
    *TACTILE.values(),
    *TCP_STATE.values(),
    *TCP_COMMAND.values(),
)
LEROBOT_CHUNK_SIZE = 1000


def _lerobot_mcap_path(episode_directory, metadata, data_directory=None):
    data_directory = (
        episode_directory.parent / "data"
        if data_directory is None
        else Path(data_directory).expanduser().resolve()
    )
    episode_index = metadata.get("lerobot", {}).get("episode_index")
    if episode_index is None:
        existing = [
            int(path.stem.removeprefix("episode_"))
            for path in data_directory.glob("chunk-*/episode_*.mcap")
            if path.stem.removeprefix("episode_").isdigit()
        ]
        # ponytail: collection permits one recorder; add a session lock if that changes.
        episode_index = max(existing, default=-1) + 1
    episode_index = int(episode_index)
    if episode_index < 0:
        raise ValueError("LeRobot episode index must be non-negative")
    return (
        data_directory
        / f"chunk-{episode_index // LEROBOT_CHUNK_SIZE:03d}"
        / f"episode_{episode_index:06d}.mcap",
        episode_index,
    )


def _string(value):
    encoded = value.encode("utf-8")
    return struct.pack("<I", len(encoded)) + encoded


def _record(output, opcode, content):
    output.write(bytes((opcode,)))
    output.write(struct.pack("<Q", len(content)))
    output.write(content)


def _safe_attachment_name(name):
    path = PurePosixPath(name)
    if path.is_absolute() or not path.parts or ".." in path.parts or "\\" in name:
        raise ValueError(f"unsafe MCAP attachment name: {name!r}")
    return path


def write_episode_mcap(output_path, attachments, timestamp_ns=None):
    """Write files as standards-compliant MCAP Attachment records."""
    output_path = Path(output_path)
    attachments = [(str(_safe_attachment_name(n)), m, Path(p)) for n, m, p in attachments]
    if not attachments or len({item[0] for item in attachments}) != len(attachments):
        raise ValueError("MCAP attachments must be non-empty and uniquely named")
    if any(not path.is_file() for _name, _media_type, path in attachments):
        raise FileNotFoundError("an MCAP attachment source file is missing")
    timestamp_ns = time.time_ns() if timestamp_ns is None else int(timestamp_ns)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("xb") as output:
        output.write(MAGIC)
        _record(
            output,
            0x01,
            _string("org.huggingface.lerobot")
            + _string("xr-marvin-teleop"),
        )
        for name, media_type, path in attachments:
            prefix = (
                struct.pack("<QQ", timestamp_ns, int(path.stat().st_mtime_ns))
                + _string(name)
                + _string(media_type)
                + struct.pack("<Q", path.stat().st_size)
            )
            content_length = len(prefix) + path.stat().st_size + 4
            output.write(bytes((ATTACHMENT,)))
            output.write(struct.pack("<Q", content_length))
            output.write(prefix)
            checksum = zlib.crc32(prefix)
            with path.open("rb") as source:
                for chunk in iter(lambda: source.read(1024 * 1024), b""):
                    output.write(chunk)
                    checksum = zlib.crc32(chunk, checksum)
            output.write(struct.pack("<I", checksum & 0xFFFFFFFF))
        _record(output, DATA_END, struct.pack("<I", 0))
        _record(output, FOOTER, struct.pack("<QQI", 0, 0, 0))
        output.write(MAGIC)


def _read_exact(source, size):
    value = source.read(size)
    if len(value) != size:
        raise ValueError("truncated MCAP")
    return value


def _read_string(source):
    size = struct.unpack("<I", _read_exact(source, 4))[0]
    return _read_exact(source, size).decode("utf-8")


def attachment_entries(path):
    """Return Attachment locations without loading their data into memory."""
    path = Path(path)
    entries = []
    with path.open("rb") as source:
        if _read_exact(source, len(MAGIC)) != MAGIC:
            raise ValueError("not an MCAP file")
        file_size = path.stat().st_size
        saw_header = saw_data_end = saw_footer = False
        while not saw_footer:
            opcode = _read_exact(source, 1)[0]
            length = struct.unpack("<Q", _read_exact(source, 8))[0]
            content_start = source.tell()
            content_end = content_start + length
            if content_end > file_size - len(MAGIC):
                raise ValueError("MCAP record exceeds file size")
            if not saw_header:
                if opcode != 0x01:
                    raise ValueError("MCAP Header is missing")
                saw_header = True
            elif opcode == ATTACHMENT:
                log_time, create_time = struct.unpack("<QQ", _read_exact(source, 16))
                name = _read_string(source)
                media_type = _read_string(source)
                data_size = struct.unpack("<Q", _read_exact(source, 8))[0]
                data_offset = source.tell()
                if data_offset + data_size + 4 != content_end:
                    raise ValueError("invalid MCAP Attachment length")
                source.seek(data_size, 1)
                checksum = struct.unpack("<I", _read_exact(source, 4))[0]
                entries.append(
                    {
                        "name": name,
                        "media_type": media_type,
                        "log_time": log_time,
                        "create_time": create_time,
                        "data_offset": data_offset,
                        "data_size": data_size,
                        "content_start": content_start,
                        "checksum": checksum,
                    }
                )
            elif opcode == DATA_END:
                saw_data_end = True
            elif opcode == FOOTER:
                if not saw_data_end or length != 20:
                    raise ValueError("invalid MCAP Footer")
                saw_footer = True
            source.seek(content_end)
        if _read_exact(source, len(MAGIC)) != MAGIC or source.read(1):
            raise ValueError("invalid MCAP trailing magic")
    if len({item["name"] for item in entries}) != len(entries):
        raise ValueError("duplicate MCAP attachment name")
    return entries


def read_attachment(path, name):
    for entry in attachment_entries(path):
        if entry["name"] == name:
            with Path(path).open("rb") as source:
                source.seek(entry["data_offset"])
                return _read_exact(source, entry["data_size"])
    raise KeyError(name)


def extract_episode_mcap(path, output_directory):
    validate_episode_mcap(path)
    output_directory = Path(output_directory).resolve()
    output_directory.mkdir(parents=True, exist_ok=True)
    for entry in attachment_entries(path):
        relative = _safe_attachment_name(entry["name"])
        destination = output_directory.joinpath(*relative.parts)
        if output_directory not in destination.resolve().parents:
            raise ValueError(f"unsafe MCAP attachment name: {entry['name']!r}")
        destination.parent.mkdir(parents=True, exist_ok=True)
        with Path(path).open("rb") as source, destination.open("xb") as target:
            source.seek(entry["data_offset"])
            remaining = entry["data_size"]
            while remaining:
                chunk = source.read(min(1024 * 1024, remaining))
                if not chunk:
                    raise ValueError("truncated MCAP Attachment")
                target.write(chunk)
                remaining -= len(chunk)
    return output_directory


def validate_episode_mcap(path):
    entries = attachment_entries(path)
    by_name = {item["name"]: item for item in entries}
    for required in ("meta/meta.json", "data/data.parquet"):
        if required not in by_name:
            raise ValueError(f"required MCAP attachment is missing: {required}")
    for entry in entries:
        with Path(path).open("rb") as source:
            source.seek(entry["content_start"])
            remaining = entry["data_offset"] + entry["data_size"] - entry["content_start"]
            checksum = 0
            while remaining:
                chunk = _read_exact(source, min(1024 * 1024, remaining))
                checksum = zlib.crc32(chunk, checksum)
                remaining -= len(chunk)
        if entry["checksum"] and checksum & 0xFFFFFFFF != entry["checksum"]:
            raise ValueError(f"MCAP attachment CRC mismatch: {entry['name']}")
    metadata = json.loads(read_attachment(path, "meta/meta.json"))
    if metadata.get("dataset_format") != "lerobot" or not metadata.get("episode_id"):
        raise ValueError("invalid LeRobot episode metadata")
    expected = {
        "data/data.parquet",
        *metadata.get("video_paths", {}).values(),
    }
    if not expected.issubset(by_name):
        raise ValueError("metadata references a missing MCAP attachment")
    return metadata


def _messages(path, topics):
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except (ImportError, OSError) as error:
        raise RuntimeError("episode packaging requires sourced ROS2") from error
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    type_names = {item.name: item.type for item in reader.get_all_topics_and_types()}
    selected = [topic for topic in topics if topic in type_names]
    if not selected:
        return
    types = {topic: get_message(type_names[topic]) for topic in selected}
    reader.set_filter(rosbag2_py.StorageFilter(topics=selected))
    while reader.has_next():
        topic, serialized, timestamp_ns = reader.read_next()
        yield topic, deserialize_message(serialized, types[topic]), int(timestamp_ns)


def _state_value(topic, message):
    if topic == ROBOT_STATE:
        return tuple(message.q_rad), tuple(message.dq_rad_s), bool(message.valid)
    if topic == JOINT_COMMAND:
        return tuple(message.q_rad)
    if topic == GRIPPER_COMMAND:
        return tuple(message.closedness)
    if topic == PICO:
        return (
            *message.left_controller_pose,
            *message.right_controller_pose,
            *message.grip_values,
            *message.trigger_values,
            *message.thumbstick_y_values,
            float(message.button_a),
            float(message.button_b),
        ), bool(message.valid)
    if topic in DAS_STATE.values():
        return float(message.distance_m), float(message.target_distance_m), bool(message.valid)
    if topic in TACTILE.values():
        return bytes(message.data), bool(message.valid)
    return (*message.xyz_m, *message.rpy_rad), bool(message.valid)


def _load_state(path):
    # ponytail: episode state fits RAM; use an on-disk index only for multi-hour episodes.
    series = {topic: ([], []) for topic in STATE_TOPICS}
    for topic, message, timestamp_ns in _messages(path, STATE_TOPICS):
        series[topic][0].append(timestamp_ns)
        series[topic][1].append(_state_value(topic, message))
    return series


def _nearest(series, timestamp_ns):
    times, values = series
    if not times:
        return None
    index = bisect.bisect_left(times, timestamp_ns)
    candidates = [item for item in (index - 1, index) if 0 <= item < len(times)]
    return values[min(candidates, key=lambda item: abs(times[item] - timestamp_ns))]


def _previous(series, timestamp_ns):
    times, values = series
    index = bisect.bisect_right(times, timestamp_ns) - 1
    return None if index < 0 else values[index]


def _vector(value, size):
    if value is None:
        return [0.0] * size
    return [float(item) if math.isfinite(float(item)) else 0.0 for item in value[:size]]


def _write_parquet(bag_path, output_path, episode_index, index_offset, task_index):
    try:
        import pyarrow as pa
        import pyarrow.parquet as pq
    except ImportError as error:
        raise RuntimeError("LeRobot packaging requires pyarrow") from error
    state = _load_state(bag_path)
    action_times, actions = state[JOINT_COMMAND]
    if not actions:
        raise ValueError("joint command stream is empty")
    rows = {name: [] for name in (
        "observation.state", "observation.velocity", "observation.tcp_pose",
        "observation.pico", "observation.tactile.left", "observation.tactile.right",
        "observation.valid", "action", "action.tcp_pose", "timestamp",
        "frame_index", "episode_index", "index", "task_index", "next.done",
    )}
    first_time = action_times[0]
    for frame_index, (timestamp_ns, joint_action) in enumerate(zip(action_times, actions)):
        robot = _nearest(state[ROBOT_STATE], timestamp_ns)
        gripper = _previous(state[GRIPPER_COMMAND], timestamp_ns)
        das = [_nearest(state[DAS_STATE[side]], timestamp_ns) for side in ("left", "right")]
        tcp = [_nearest(state[TCP_STATE[side]], timestamp_ns) for side in ("left", "right")]
        tcp_target = [_previous(state[TCP_COMMAND[side]], timestamp_ns) for side in ("left", "right")]
        pico = _nearest(state[PICO], timestamp_ns)
        tactile = [_nearest(state[TACTILE[side]], timestamp_ns) for side in ("left", "right")]
        rows["observation.state"].append(
            _vector(None if robot is None else robot[0], 14)
            + [0.0 if item is None or not math.isfinite(item[0]) else item[0] for item in das]
        )
        rows["observation.velocity"].append(_vector(None if robot is None else robot[1], 14))
        rows["observation.tcp_pose"].append(
            sum((_vector(None if item is None else item[0], 6) for item in tcp), [])
        )
        rows["observation.pico"].append(_vector(None if pico is None else pico[0], 22))
        for side, item in zip(("left", "right"), tactile):
            rows[f"observation.tactile.{side}"].append(b"" if item is None else item[0])
        rows["observation.valid"].append(
            bool(robot and robot[2] and all(item and item[2] for item in das))
        )
        rows["action"].append(_vector(joint_action, 14) + _vector(gripper, 2))
        rows["action.tcp_pose"].append(
            sum((_vector(None if item is None else item[0], 6) for item in tcp_target), [])
        )
        rows["timestamp"].append((timestamp_ns - first_time) / 1e9)
        rows["frame_index"].append(frame_index)
        rows["episode_index"].append(episode_index)
        rows["index"].append(index_offset + frame_index)
        rows["task_index"].append(task_index)
        rows["next.done"].append(frame_index == len(actions) - 1)
    types = {
        "observation.state": pa.list_(pa.float32(), 16),
        "observation.velocity": pa.list_(pa.float32(), 14),
        "observation.tcp_pose": pa.list_(pa.float32(), 12),
        "observation.pico": pa.list_(pa.float32(), 22),
        "observation.tactile.left": pa.binary(),
        "observation.tactile.right": pa.binary(),
        "observation.valid": pa.bool_(),
        "action": pa.list_(pa.float32(), 16),
        "action.tcp_pose": pa.list_(pa.float32(), 12),
        "timestamp": pa.float32(),
        "frame_index": pa.int64(),
        "episode_index": pa.int64(),
        "index": pa.int64(),
        "task_index": pa.int64(),
        "next.done": pa.bool_(),
    }
    table = pa.table({name: pa.array(values, type=types[name]) for name, values in rows.items()})
    pq.write_table(table, output_path, compression="zstd")
    return rows, len(actions), first_time, (action_times[-1] - first_time) / 1e9


def _write_video(bag_path, topic, output_path, episode_start_ns):
    timestamps = [timestamp_ns for _topic, _message, timestamp_ns in _messages(bag_path, (topic,))]
    if not timestamps:
        return None
    import cv2
    import numpy as np

    fps = 30.0 if len(timestamps) == 1 else (len(timestamps) - 1) * 1e9 / (timestamps[-1] - timestamps[0])
    writer = None
    size = None
    try:
        for _topic, message, _timestamp_ns in _messages(bag_path, (topic,)):
            frame = cv2.imdecode(np.frombuffer(bytes(message.image.data), np.uint8), cv2.IMREAD_COLOR)
            if frame is None:
                raise ValueError(f"invalid JPEG frame on {topic}")
            if writer is None:
                size = (frame.shape[1], frame.shape[0])
                writer = cv2.VideoWriter(str(output_path), cv2.VideoWriter_fourcc(*"mp4v"), fps, size)
                if not writer.isOpened():
                    raise RuntimeError("OpenCV could not open the MP4 writer")
            if (frame.shape[1], frame.shape[0]) != size:
                frame = cv2.resize(frame, size)
            writer.write(frame)
    finally:
        if writer is not None:
            writer.release()
    if writer is None or not output_path.is_file():
        raise ValueError(f"no video written for {topic}")
    return {
        "frames": len(timestamps),
        "fps": fps,
        "codec": "mp4v",
        "width": size[0],
        "height": size[1],
        "timestamps": [(item - episode_start_ns) / 1e9 for item in timestamps],
    }


def _stats(rows):
    import numpy as np

    result = {}
    for name in ("observation.state", "observation.velocity", "observation.tcp_pose", "observation.pico", "action", "action.tcp_pose"):
        values = np.asarray(rows[name], dtype=np.float64)
        result[name] = {
            "min": values.min(axis=0).tolist(),
            "max": values.max(axis=0).tolist(),
            "mean": values.mean(axis=0).tolist(),
            "std": values.std(axis=0).tolist(),
        }
    return result


def package_episode(episode_directory, remove_source=True, output_directory=None):
    """Convert a validated work directory and atomically publish one MCAP."""
    episode_directory = Path(episode_directory).expanduser().resolve()
    metadata = json.loads((episode_directory / "metadata.json").read_text(encoding="utf-8"))
    manifest = json.loads((episode_directory / "manifest.json").read_text(encoding="utf-8"))
    bag_path = episode_directory / metadata.get("processed_bag", "data")
    if not bag_path.exists():
        raise FileNotFoundError(f"processed bag is missing: {bag_path}")
    final_path, episode_index = _lerobot_mcap_path(
        episode_directory, metadata, output_directory
    )
    if final_path.exists():
        raise FileExistsError(f"episode package already exists: {final_path}")
    final_path.parent.mkdir(parents=True, exist_ok=True)
    previous = [
        json.loads(read_attachment(path, "meta/meta.json"))
        for path in sorted(final_path.parents[1].glob("chunk-*/episode_*.mcap"))
    ]
    index_offset = sum(int(item["length"]) for item in previous)
    tasks = list(dict.fromkeys(item.get("task", "") for item in previous))
    task = metadata.get("task", "")
    task_index = tasks.index(task) if task in tasks else len(tasks)
    work = Path(tempfile.mkdtemp(prefix=".episode-package-", dir=final_path.parent))
    try:
        data_directory = work / "data"
        video_directory = work / "videos"
        meta_directory = work / "meta"
        data_directory.mkdir()
        video_directory.mkdir()
        meta_directory.mkdir()
        parquet_path = data_directory / "data.parquet"
        rows, length, episode_start_ns, duration = _write_parquet(
            bag_path, parquet_path, episode_index, index_offset, task_index
        )
        videos = {}
        for side, topic in CAMERA.items():
            path = video_directory / f"observation.images.{side}.mp4"
            info = _write_video(bag_path, topic, path, episode_start_ns)
            if info is not None:
                videos[f"observation.images.{side}"] = {
                    **info,
                    "path": f"videos/{path.name}",
                }
        source_files = {}
        for path in sorted((*episode_directory.glob("*_recorder.log"), *(episode_directory / "calibration").glob("*"))):
            if path.is_file():
                source_files[str(path.relative_to(episode_directory))] = path.read_text(encoding="utf-8", errors="replace")
        joint_names = [f"{side}_joint_{joint}" for side in ("left", "right") for joint in range(1, 8)]
        tcp_names = [f"{side}_{axis}" for side in ("left", "right") for axis in ("x", "y", "z", "roll", "pitch", "yaw")]
        pico_names = [
            *[f"left_pose_{item}" for item in ("x", "y", "z", "qx", "qy", "qz", "qw")],
            *[f"right_pose_{item}" for item in ("x", "y", "z", "qx", "qy", "qz", "qw")],
            "left_grip", "right_grip", "left_trigger", "right_trigger",
            "left_thumbstick_y", "right_thumbstick_y", "button_a", "button_b",
        ]
        package_metadata = {
            "schema_version": 1,
            "dataset_format": "lerobot",
            "codebase_version": "v2.1",
            "episode_id": metadata.get("episode_id", episode_directory.name),
            "episode_index": episode_index,
            "session": episode_directory.parent.name,
            "task": task,
            "task_index": task_index,
            "operator": metadata.get("operator", ""),
            "robot_model": metadata.get("robot_model", ""),
            "status": manifest.get("status", metadata.get("status", "unknown")),
            "fps": 50,
            "length": length,
            "duration_seconds": duration,
            "data_path": "data/data.parquet",
            "video_paths": {name: item["path"] for name, item in videos.items()},
            "videos": videos,
            "features": {
                "observation.state": {"dtype": "float32", "shape": [16], "names": [*joint_names, "gripper_left_m", "gripper_right_m"]},
                "observation.velocity": {"dtype": "float32", "shape": [14], "names": joint_names},
                "observation.tcp_pose": {"dtype": "float32", "shape": [12], "names": tcp_names},
                "observation.pico": {"dtype": "float32", "shape": [22], "names": pico_names},
                "observation.tactile.left": {"dtype": "binary", "shape": []},
                "observation.tactile.right": {"dtype": "binary", "shape": []},
                "observation.valid": {"dtype": "bool", "shape": []},
                "action": {"dtype": "float32", "shape": [16], "names": [*joint_names, "gripper_left_closedness", "gripper_right_closedness"]},
                "action.tcp_pose": {"dtype": "float32", "shape": [12], "names": tcp_names},
                "timestamp": {"dtype": "float32", "shape": []},
                "frame_index": {"dtype": "int64", "shape": []},
                "episode_index": {"dtype": "int64", "shape": []},
                "index": {"dtype": "int64", "shape": []},
                "task_index": {"dtype": "int64", "shape": []},
                "next.done": {"dtype": "bool", "shape": []},
                **{name: {"dtype": "video", "shape": [item["height"], item["width"], 3]} for name, item in videos.items()},
            },
            "stats": _stats(rows),
            "source_metadata": metadata,
            "validation": manifest,
            "source_files": source_files,
        }
        meta_path = meta_directory / "meta.json"
        meta_path.write_text(json.dumps(package_metadata, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        attachments = [
            ("meta/meta.json", "application/json", meta_path),
            ("data/data.parquet", "application/vnd.apache.parquet", parquet_path),
            *[(item["path"], "video/mp4", video_directory / Path(item["path"]).name) for item in videos.values()],
        ]
        temporary_package = work / final_path.name
        write_episode_mcap(temporary_package, attachments, metadata.get("ended_at_ns"))
        validate_episode_mcap(temporary_package)
        temporary_package.replace(final_path)
    finally:
        shutil.rmtree(work, ignore_errors=True)
    if remove_source:
        shutil.rmtree(episode_directory)
    return final_path
