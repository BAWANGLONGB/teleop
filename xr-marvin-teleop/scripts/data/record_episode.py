#!/usr/bin/env python3
"""Record one synchronized-by-timestamp state/vision collection episode."""

import argparse
import hashlib
import json
import os
import platform
import shutil
import signal
import subprocess
import threading
import time
import uuid
from pathlib import Path

from xr_marvin_teleop.common.episode_validator import validate_episode


STATE_TOPICS = (
    "/raw/pico/frame",
    "/raw/marvin/joint_state",
    "/command/marvin/joint_target",
    "/command/das/target",
    "/raw/das/left/state",
    "/raw/das/right/state",
    "/raw/das/left/tactile",
    "/raw/das/right/tactile",
    "/diagnostics",
    "/episode/state",
    "/episode/event",
)
VISION_TOPICS = ("/raw/das/left/image", "/raw/das/right/image")


def _write_json(path, value):
    temporary_path = path.with_suffix(path.suffix + ".tmp")
    temporary_path.write_text(
        json.dumps(value, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    temporary_path.replace(path)


def _sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


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


def _recorder_command(output, topics, config, qos):
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
        str(256 * 1024 * 1024),
        *topics,
    ]


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


def _stop_recorder(process):
    if process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGINT)
    except ProcessLookupError:
        return
    try:
        process.wait(timeout=10.0)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=3.0)
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


def main():
    project_root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description="Record one teleoperation episode")
    parser.add_argument("--task", required=True)
    parser.add_argument("--operator", default=os.environ.get("USER", "unknown"))
    parser.add_argument("--robot-model", default="Marvin")
    parser.add_argument("--notes", default="")
    parser.add_argument("--metadata", action="append", default=[], metavar="KEY=VALUE")
    parser.add_argument("--calibration", action="append", default=[], type=Path)
    parser.add_argument("--output-root", type=Path, default=project_root / "dataset")
    parser.add_argument("--no-vision", action="store_true")
    parser.add_argument("--max-duration", type=float)
    arguments = parser.parse_args()
    if not arguments.task.strip():
        parser.error("--task must not be empty")
    if arguments.max_duration is not None and arguments.max_duration <= 0.0:
        parser.error("--max-duration must be positive")
    try:
        extra_metadata = _parse_metadata(arguments.metadata)
    except ValueError as error:
        parser.error(str(error))

    calibration_sources = []
    for path in arguments.calibration:
        path = path.expanduser().resolve()
        if not path.is_file():
            parser.error(f"calibration file not found: {path}")
        calibration_sources.append(path)
    _require_mcap()
    now = time.localtime()
    episode_id = f"episode_{time.strftime('%H%M%S', now)}_{uuid.uuid4().hex[:8]}"
    episode_directory = (
        arguments.output_root.expanduser().resolve()
        / f"session_{time.strftime('%Y-%m-%d', now)}"
        / episode_id
    )
    episode_directory.mkdir(parents=True, exist_ok=False)
    calibrations = []
    calibration_directory = episode_directory / "calibration"
    for index, path in enumerate(calibration_sources):
        calibration_directory.mkdir(exist_ok=True)
        snapshot_path = calibration_directory / f"{index:02d}_{path.name}"
        shutil.copy2(path, snapshot_path)
        calibrations.append(
            {
                "source_path": str(path),
                "snapshot": str(snapshot_path.relative_to(episode_directory)),
                "size_bytes": snapshot_path.stat().st_size,
                "sha256": _sha256(snapshot_path),
            }
        )
    metadata = {
        "schema_version": 1,
        "episode_id": episode_id,
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
        "bags": ["state"] if arguments.no_vision else ["state", "vision"],
    }
    metadata_path = episode_directory / "metadata.json"
    _write_json(metadata_path, metadata)

    config_root = project_root / "config" / "data_collection"
    recorder_specs = [
        (
            "state",
            STATE_TOPICS,
            config_root / "mcap_state.yaml",
        )
    ]
    if not arguments.no_vision:
        recorder_specs.append(
            ("vision", VISION_TOPICS, config_root / "mcap_vision.yaml")
        )
    publisher = EpisodePublisher()
    recorders = []
    stop_requested = threading.Event()
    previous_handlers = {}

    def request_stop(_signal_number, _frame):
        stop_requested.set()

    try:
        for signal_number in (signal.SIGINT, signal.SIGTERM):
            previous_handlers[signal_number] = signal.signal(
                signal_number, request_stop
            )
        for name, topics, config in recorder_specs:
            command = _recorder_command(
                episode_directory / name,
                topics,
                config,
                config_root / "qos_overrides.yaml",
            )
            process, log = _start_recorder(
                command, episode_directory / f"{name}_recorder.log"
            )
            recorders.append((name, process, log))
        time.sleep(1.0)
        for name, process, _log in recorders:
            if process.poll() is not None:
                raise RuntimeError(f"{name} recorder exited during startup")
        metadata["status"] = "recording"
        _write_json(metadata_path, metadata)
        publisher.publish_state("recording", episode_id)
        publisher.publish_event("start", episode_id)
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
        for _name, process, _log in recorders:
            _stop_recorder(process)
        metadata["status"] = "completed"
        metadata["ended_at_ns"] = time.time_ns()
        _write_json(metadata_path, metadata)
        manifest = validate_episode(episode_directory)
        print(f"Episode {manifest['status']}: {episode_directory}")
        raise SystemExit(0 if manifest["status"] != "rejected" else 1)
    except Exception:
        metadata["status"] = "aborted"
        metadata["ended_at_ns"] = time.time_ns()
        _write_json(metadata_path, metadata)
        raise
    finally:
        for _name, process, log in recorders:
            try:
                _stop_recorder(process)
            finally:
                log.close()
        publisher.close()
        for signal_number, handler in previous_handlers.items():
            signal.signal(signal_number, handler)


if __name__ == "__main__":
    main()
