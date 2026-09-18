#!/usr/bin/env python3
"""Record one synchronized-by-timestamp state/vision collection episode."""

import argparse
import json
import os
import platform
import shutil
import signal
import subprocess
import threading
import time
from pathlib import Path

from xr_marvin_teleop.collection.episode_video import (
    activity_lock, add_video_arguments, video_options,
)
from xr_marvin_teleop.collection.config import (
    DEFAULT_CONFIG, read_json, write_json, configure_parser, apply_config,
    freeze_arguments, check_active_devices,
)
from xr_marvin_teleop.collection.episode_validator import sha256_file
from xr_marvin_teleop.collection.episode_review import EPISODE_ID, new_episode_id, session_path, session_record
from xr_marvin_teleop.adapters.das_finger import (
    ARM_NAMES,
    load_das_finger_configurations,
)


STATE_TOPICS = tuple(read_json(DEFAULT_CONFIG)["recording"]["state_topics"])


def _git_metadata(project_root):
    def run(*arguments):
        result = subprocess.run(
            arguments,
            cwd=project_root,
            text=True,
            capture_output=True,
            check=False,
        )
        return result.stdout.strip() if result.returncode == 0 else None

    commit = run("git", "rev-parse", "HEAD")
    status = run("git", "status", "--porcelain")
    return {"commit": commit, "dirty": None if status is None else bool(status)}


def _parse_metadata(values):
    result = {}
    for value in values:
        if "=" not in value:
            raise ValueError(f"metadata must use KEY=VALUE: {value!r}")
        key, item = value.split("=", 1)
        if not key.strip():
            raise ValueError("metadata key must not be empty")
        result[key.strip()] = item
    return result


class EpisodePublisher:
    def __init__(self):
        try:
            import rclpy
            from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
            from std_msgs.msg import String
        except (ImportError, OSError) as error:
            raise RuntimeError("episode recording requires sourced ROS2") from error
        self._rclpy = rclpy
        self._string_type = String
        self._owns_context = not rclpy.ok()
        if self._owns_context:
            rclpy.init()
        self._node = rclpy.create_node("teleop_episode_recorder")
        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        event_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._state = self._node.create_publisher(
            String, "/episode/state", state_qos
        )
        self._event = self._node.create_publisher(
            String, "/episode/event", event_qos
        )

    def publish(self, publisher, payload):
        payload = {"message_protocol_version": 2, **payload}
        message = self._string_type()
        message.data = json.dumps(payload, ensure_ascii=False)
        publisher.publish(message)
        self._rclpy.spin_once(self._node, timeout_sec=0.05)

    def publish_state(self, status, episode_id):
        self.publish(
            self._state,
            {
                "status": status,
                "episode_id": episode_id,
                "wall_time_ns": time.time_ns(),
            },
        )

    def publish_event(self, event, episode_id):
        self.publish(
            self._event,
            {"event": event, "episode_id": episode_id, "wall_time_ns": time.time_ns()},
        )

    def spin_once(self):
        self._rclpy.spin_once(self._node, timeout_sec=0.05)

    def close(self):
        self._node.destroy_node()
        if self._owns_context and self._rclpy.ok():
            self._rclpy.shutdown()


def _recorder_command(output, topics, config, qos, cache_bytes=None):
    return [
        "ros2",
        "bag",
        "record",
        "--storage",
        "mcap",
        "--output",
        str(output),
        "--storage-config-file",
        str(config),
        "--qos-profile-overrides-path",
        str(qos),
        "--max-cache-size",
        str(read_json(DEFAULT_CONFIG)["recording"]["state_cache_bytes"] if cache_bytes is None else cache_bytes),
        *topics,
    ]


def _camera_command(
    project_root,
    side,
    configuration,
    output,
    storage_config,
    ready_file,
    preview_file=None,
    preview_fps=None,
):
    command = [
        str(project_root / ".venv" / "bin" / "python"),
        "-m",
        "xr_marvin_teleop.cli.capture",
        "--side",
        side,
        "--device",
        configuration.camera_device,
        "--resolution",
        configuration.camera_resolution,
        "--fps",
        str(configuration.camera_fps),
        "--output",
        str(output),
        "--storage-config",
        str(storage_config),
        "--ready-file",
        str(ready_file),
    ]
    if preview_file is not None:
        command.extend(("--preview-file", str(preview_file)))
    if preview_fps is not None:
        command.extend(("--preview-fps", str(preview_fps)))
    return command


def _start_recorder(command, log_path):
    log = log_path.open("w", encoding="utf-8")
    try:
        process = subprocess.Popen(
            command,
            stdout=log,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
        )
    except Exception:
        log.close()
        raise
    return process, log


def _stop_recorder(process, stop_timeout=10.0, term_timeout=3.0):
    if process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGINT)
    except ProcessLookupError:
        return
    try:
        process.wait(timeout=stop_timeout)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=term_timeout)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=1.0)


def _require_mcap():
    result = subprocess.run(
        ("ros2", "pkg", "prefix", "rosbag2_storage_mcap"),
        text=True,
        capture_output=True,
        check=False,
    )
    if result.returncode != 0:
        raise RuntimeError(
            "MCAP storage plugin is missing; install it with: "
            "sudo apt-get install ros-humble-rosbag2-storage-mcap"
        )
    try:
        import rosbag2_py  # noqa: F401
        from sensor_msgs.msg import CompressedImage  # noqa: F401
        from geometry_msgs.msg import PoseStamped  # noqa: F401
        from foxglove_msgs.msg import Grid  # noqa: F401
    except (ImportError, OSError) as error:
        raise RuntimeError(
            "recording requires sourced ROS2 and ros-humble-foxglove-msgs"
        ) from error


def _snapshot_calibrations(paths, episode_directory):
    calibrations = []
    calibration_directory = episode_directory / "calibration"
    for index, path in enumerate(paths):
        calibration_directory.mkdir(exist_ok=True)
        snapshot_path = calibration_directory / f"{index:02d}_{path.name}"
        shutil.copy2(path, snapshot_path)
        calibrations.append(
            {
                "source_path": str(path),
                "snapshot": str(snapshot_path.relative_to(episode_directory)),
                "size_bytes": snapshot_path.stat().st_size,
                "sha256": sha256_file(snapshot_path),
            }
        )
    return calibrations


def _recorder_process_specs(
    project_root, episode_directory, config, configurations, preview_root
):
    process_specs = [
        (
            "state",
            _recorder_command(
                episode_directory / "state",
                config["recording"]["state_topics"],
                config["recording"]["state_storage"],
                config["recording"]["qos"],
                config["recording"]["state_cache_bytes"],
            ),
        )
    ]
    camera_ready_files = {}
    if configurations is not None:
        camera_ready_files = {
            f"vision_{side}": episode_directory / f".vision_{side}.ready"
            for side in ARM_NAMES
        }
        process_specs.extend(
            (
                f"vision_{side}",
                _camera_command(
                    project_root,
                    side,
                    configuration,
                    episode_directory / f"vision_{side}",
                    config["recording"]["camera_storage"],
                    camera_ready_files[f"vision_{side}"],
                    None if preview_root is None else preview_root / f"{side}.jpg",
                    config["preview"]["fps"],
                ),
            )
            for side, configuration in zip(ARM_NAMES, configurations)
        )
    return process_specs, camera_ready_files


def _wait_for_recorders_ready(
    recorders, camera_ready_files, stop_requested, timeout_seconds
):
    time.sleep(1.0)
    for name, process, _log in recorders:
        if process.poll() is not None:
            raise RuntimeError(f"{name} recorder exited during startup")
    pending = dict(camera_ready_files)
    ready_deadline = time.monotonic() + timeout_seconds
    while (
        pending
        and not stop_requested.is_set()
        and time.monotonic() < ready_deadline
    ):
        for name, process, _log in recorders:
            if process.poll() is not None:
                raise RuntimeError(f"{name} recorder exited during startup")
        pending = {
            name: path for name, path in pending.items() if not path.is_file()
        }
        if pending:
            time.sleep(0.05)
    if stop_requested.is_set():
        raise RuntimeError("recording stopped during camera startup")
    if pending:
        raise TimeoutError(
            "camera writers produced no MJPEG frame: "
            + ", ".join(sorted(pending))
        )


def main():
    project_root = Path(__file__).resolve().parents[3]
    parser = argparse.ArgumentParser(description="Record one teleoperation episode")
    parser.add_argument("--task", required=True)
    parser.add_argument("--session", help="existing stable Session ID; default groups by date")
    parser.add_argument("--episode-id", help="UI-provided identity for reliable result annotation")
    parser.add_argument("--operator", default=os.environ.get("USER", "unknown"))
    parser.add_argument("--robot-model")
    parser.add_argument("--notes", default="")
    parser.add_argument("--metadata", action="append", default=[], metavar="KEY=VALUE")
    parser.add_argument("--calibration", action="append", type=Path)
    parser.add_argument("--das-config", type=Path)
    parser.add_argument("--output-root", type=Path)
    parser.add_argument("--no-vision", action="store_true")
    parser.add_argument("--max-duration", type=float)
    parser.add_argument("--unlimited-duration", dest="max_duration", action="store_const", const=None)
    parser.add_argument("--ready-file", type=Path)
    parser.add_argument("--camera-startup-timeout", type=float)
    parser.add_argument("--preview-root", type=Path)
    add_video_arguments(parser)
    parser.add_argument("--vision", dest="no_vision", action="store_false")
    parser.add_argument("--preview", action=argparse.BooleanOptionalAction)
    parser.add_argument("--preview-fps", type=int)
    configure_parser(parser)
    arguments = parser.parse_args()
    try:
        config = apply_config(arguments)
        outputs = video_options(arguments)
    except (ValueError, OSError) as error:
        parser.error(str(error))
    if arguments.print_effective_config:
        print(json.dumps(config, ensure_ascii=False, indent=2))
        return
    if not arguments.task or not arguments.task.strip():
        parser.error("--task must not be empty")
    session_id = arguments.session or f"session_{time.strftime('%Y-%m-%d')}"
    try:
        session_directory = session_path(arguments.output_root, session_id)
        if arguments.session and not session_directory.is_dir():
            raise ValueError("指定的 Session 不存在")
        episode_id = arguments.episode_id or new_episode_id()
        if not EPISODE_ID.fullmatch(episode_id):
            raise ValueError("Episode ID 格式无效")
    except ValueError as error:
        parser.error(str(error))
    if arguments.preview_root is not None:
        arguments.preview_root = arguments.preview_root.expanduser().resolve()
        arguments.preview_root.mkdir(mode=0o700, parents=True, exist_ok=True)
        for side in ARM_NAMES:
            (arguments.preview_root / f"{side}.jpg").unlink(missing_ok=True)
    try:
        extra_metadata = _parse_metadata(arguments.metadata)
    except ValueError as error:
        parser.error(str(error))

    if not arguments.no_vision:
        if not arguments.das_config.is_file():
            parser.error(f"DAS config not found: {arguments.das_config}")
        load_das_finger_configurations(arguments.das_config)

    for path in arguments.calibration:
        if not path.is_file():
            parser.error(f"calibration file not found: {path}")
    _require_mcap()
    episode_directory = session_directory / episode_id
    episode_directory.mkdir(parents=True, exist_ok=False)
    config_path = freeze_arguments(arguments, episode_directory / "config", require_devices=False)
    config = arguments.effective_config
    check_active_devices(config)
    # Only use snapshotted files after this point, including per-camera settings.
    configurations = None if arguments.no_vision else load_das_finger_configurations(arguments.das_config)
    calibrations = _snapshot_calibrations(
        arguments.calibration, episode_directory
    )
    metadata = {
        "message_protocol_version": 2,
        "message_contract": {
            "pico_pose_order": ["left", "right"],
            "pico_axes": ["left_grip", "right_grip", "left_trigger", "right_trigger", "left_thumbstick_y", "right_thumbstick_y"],
            "pico_buttons": ["a", "b", "x", "y"],
            "tactile_layout": "das-448-v1: two 224-byte pad rows, UINT8 raw bits, index coordinates; not calibrated pressure",
            "gripper_position": "logical opening distance in meters; not individual URDF finger displacement",
            "gripper_command": "normalized openness: 0 closed, 1 open",
        },
        "schema_version": 1,
        "episode_id": episode_id,
        "session": session_id,
        "session_name": session_record(session_directory)["name"],
        "status": "starting",
        "task": arguments.task,
        "operator": arguments.operator,
        "robot_model": arguments.robot_model,
        "notes": arguments.notes,
        "extra": extra_metadata,
        "started_at_ns": time.time_ns(),
        "host": platform.node(),
        "ros_distro": os.environ.get("ROS_DISTRO"),
        "git": _git_metadata(project_root),
        "calibrations": calibrations,
        "bags": (
            ["state"]
            if arguments.no_vision
            else ["state", "vision_left", "vision_right"]
        ),
        "camera_profiles": {
            side: {
                "resolution": configuration.camera_resolution,
                "fps": configuration.camera_fps,
                "latency_correction_ns": 0,
            }
            for side, configuration in zip(ARM_NAMES, configurations or ())
        },
    }
    metadata["video_outputs"] = outputs
    metadata["capture_config"] = read_json(config_path)
    metadata["config_snapshot"] = "config/collection.json"
    metadata["config_files"] = [
        {"snapshot": str(path.relative_to(episode_directory)), "sha256": sha256_file(path)}
        for path in sorted(config_path.parent.iterdir()) if path.is_file()
    ]
    metadata["timestamp_clock"] = "CLOCK_REALTIME"
    metadata_path = episode_directory / "metadata.json"
    write_json(metadata_path, metadata)

    process_specs, camera_ready_files = _recorder_process_specs(
        project_root,
        episode_directory,
        config,
        configurations,
        arguments.preview_root,
    )
    publisher = EpisodePublisher()
    recorders = []
    stop_requested = threading.Event()
    previous_handlers = {}
    recording_lock = None

    def request_stop(_signal_number, _frame):
        stop_requested.set()

    try:
        recording_lock = activity_lock(arguments.output_root)
        for signal_number in (signal.SIGINT, signal.SIGTERM):
            previous_handlers[signal_number] = signal.signal(
                signal_number, request_stop
            )
        for name, command in process_specs:
            process, log = _start_recorder(
                command, episode_directory / f"{name}_recorder.log"
            )
            recorders.append((name, process, log))
        _wait_for_recorders_ready(
            recorders,
            camera_ready_files,
            stop_requested,
            arguments.camera_startup_timeout,
        )
        for path in episode_directory.glob(".vision_*.ready"):
            path.unlink()
        metadata["status"] = "recording"
        write_json(metadata_path, metadata)
        publisher.publish_state("recording", episode_id)
        publisher.publish_event("start", episode_id)
        if arguments.ready_file is not None:
            arguments.ready_file.expanduser().resolve().write_text(
                f"{episode_directory}\n", encoding="utf-8"
            )
        print(f"Recording {episode_id} to {episode_directory}; Ctrl-C to stop")
        deadline = (
            None
            if arguments.max_duration is None
            else time.monotonic() + arguments.max_duration
        )
        while not stop_requested.is_set() and (
            deadline is None or time.monotonic() < deadline
        ):
            for name, process, _log in recorders:
                if process.poll() is not None:
                    raise RuntimeError(f"{name} recorder stopped unexpectedly")
            publisher.spin_once()
        publisher.publish_event("stop", episode_id)
        publisher.publish_state("finalizing", episode_id)
        time.sleep(0.25)
        for name, process, _log in reversed(recorders):
            _stop_recorder(process, config["runtime"]["recorder_stop_timeout_s"], config["runtime"]["recorder_term_timeout_s"])
            if process.returncode != 0:
                raise RuntimeError(f"{name} recorder did not close cleanly: {process.returncode}")
        metadata["status"] = "completed"
        metadata["ended_at_ns"] = time.time_ns()
        metadata["export_status"] = "pending"
        write_json(metadata_path, metadata)
        print(f"Recorded {episode_directory}; offline export pending. "
              "When teleoperation/collection are stopped, run `uv run teleop-postprocess` "
              f"{episode_directory}", flush=True)
    except Exception:
        metadata["status"] = "aborted"
        metadata["ended_at_ns"] = time.time_ns()
        write_json(metadata_path, metadata)
        raise
    finally:
        for _name, process, log in reversed(recorders):
            try:
                _stop_recorder(process, config["runtime"]["recorder_stop_timeout_s"], config["runtime"]["recorder_term_timeout_s"])
            finally:
                log.close()
        publisher.close()
        if recording_lock is not None:
            recording_lock.close()
        for signal_number, handler in previous_handlers.items():
            signal.signal(signal_number, handler)


if __name__ == "__main__":
    main()
