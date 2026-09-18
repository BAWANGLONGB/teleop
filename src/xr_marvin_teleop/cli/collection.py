#!/usr/bin/env python3
"""Run device control and episode recording together or independently."""

import argparse
import json
import os
import shlex
import shutil
import signal
import subprocess
import sys
import threading
import tempfile
import time
from contextlib import closing, ExitStack
from pathlib import Path

from xr_marvin_teleop.adapters.das_finger import (
    load_das_finger_configurations,
)
from xr_marvin_teleop.ros.das_client import RosDasClient
from xr_marvin_teleop.ros.pico_client import RosPicoClient
from xr_marvin_teleop.collection.episode_video import (
    VIDEO_VARIANTS, activity_lock, add_video_arguments, video_options,
)
from xr_marvin_teleop.control.calibration import resolve_scale_factor
from xr_marvin_teleop.collection.episode_review import EPISODE_ID, session_path
from xr_marvin_teleop.collection.config import (
    DEFAULT_CONFIG, read_json, configure_parser, apply_config,
    freeze_arguments, active_devices, check_active_devices,
)


PROJECT_ROOT = Path(__file__).resolve().parents[3]
PROJECT_PYTHON = PROJECT_ROOT / ".venv" / "bin" / "python"
PROCESS_CPUS = {name: tuple(cpus) for name, cpus in read_json(DEFAULT_CONFIG)["runtime"]["cpus"].items()}


def parse_command_line_arguments(arguments=None):
    parser = argparse.ArgumentParser(
        description="Run one ROS2 PICO-Marvin-DAS collection job"
    )
    parser.add_argument(
        "--part",
        choices=("all", "devices", "recording"),
        default="all",
        help="run the full job, device lifecycle, or recording lifecycle",
    )
    parser.add_argument("--ready-file", type=Path)
    parser.add_argument("--task", required=True)
    parser.add_argument("--session")
    parser.add_argument("--episode-id")
    parser.add_argument("--operator", default=os.environ.get("USER", "unknown"))
    parser.add_argument("--robot-model")
    parser.add_argument("--enable-hardware", action="store_true")
    parser.add_argument("--confirmed-estop", action="store_true")
    parser.add_argument("--confirmed-joint-mapping", action="store_true")
    parser.add_argument("--das-config", type=Path)
    parser.add_argument("--das-sdk-root", type=Path)
    parser.add_argument(
        "--scale-calibration-path",
        type=Path,
    )
    parser.add_argument("--calibration", action="append", type=Path)
    parser.add_argument("--output-root", type=Path)
    parser.add_argument("--metadata", action="append", default=[], metavar="KEY=VALUE")
    parser.add_argument("--notes", default="")
    parser.add_argument("--max-duration", type=float)
    parser.add_argument("--unlimited-duration", dest="max_duration", action="store_const", const=None)
    parser.add_argument("--no-vision", action="store_true")
    parser.add_argument("--preview-root", type=Path)
    parser.add_argument("--pico-poll-hz", type=float)
    parser.add_argument("--startup-timeout", type=float)
    parser.add_argument("--camera-startup-timeout", type=float)
    parser.add_argument("--robot-ip")
    parser.add_argument("--thumbstick-y-sign", type=int, choices=(-1, 1))
    parser.add_argument("--gripper-mode", choices=("binary", "continuous"))
    parser.add_argument("--scale-factor", type=float)
    parser.add_argument("--joint-command-max-speed-deg-s", type=float)
    parser.add_argument("--nsp-lateral", action="store_true")
    parser.add_argument("--nsp-max-angle", type=float)
    parser.add_argument("--nsp-angle-rate", type=float)
    parser.add_argument("--nsp-lateral-deadzone", type=float)
    parser.add_argument("--nsp-lateral-range", type=float)
    parser.add_argument(
        "--nsp-lateral-sign-left", type=int, choices=(-1, 1)
    )
    parser.add_argument(
        "--nsp-lateral-sign-right", type=int, choices=(-1, 1)
    )
    add_video_arguments(parser)
    parser.add_argument("--vision", dest="no_vision", action="store_false")
    parser.add_argument("--no-nsp-lateral", dest="nsp_lateral", action="store_false")
    parser.add_argument("--preview", action=argparse.BooleanOptionalAction)
    parser.add_argument("--preview-fps", type=int)
    configure_parser(parser)
    parsed = parser.parse_args(arguments)
    try:
        apply_config(parsed)
    except (ValueError, OSError) as error:
        parser.error(str(error))
    if parsed.print_effective_config:
        return parsed
    try:
        if parsed.session and not session_path(parsed.output_root, parsed.session).is_dir():
            raise ValueError("指定的 Session 不存在")
        if parsed.episode_id and not EPISODE_ID.fullmatch(parsed.episode_id):
            raise ValueError("Episode ID 格式无效")
    except ValueError as error:
        parser.error(str(error))
    missing = [
        flag
        for flag, enabled in (
            ("--enable-hardware", parsed.enable_hardware),
            ("--confirmed-estop", parsed.confirmed_estop),
            ("--confirmed-joint-mapping", parsed.confirmed_joint_mapping),
        )
        if not enabled
    ]
    if missing:
        parser.error("required hardware confirmations: " + ", ".join(missing))
    if not parsed.task or not parsed.task.strip() or not parsed.robot_model.strip():
        parser.error("--task and --robot-model must not be empty")
    if any(
        "=" not in item or not item.split("=", 1)[0].strip()
        for item in parsed.metadata
    ):
        parser.error("--metadata must use a non-empty KEY=VALUE")
    return parsed


def _preflight(arguments):
    if not PROJECT_PYTHON.is_file():
        raise FileNotFoundError(f"project Python is missing: {PROJECT_PYTHON}")
    if "/usr/lib/x86_64-linux-gnu/libstdc++.so.6" in os.environ.get(
        "LD_PRELOAD", ""
    ).split(":"):
        raise RuntimeError(
            "system libstdc++ is forced through LD_PRELOAD; run: unset LD_PRELOAD"
        )
    if shutil.which("ros2") is None:
        raise RuntimeError("ROS2 is not sourced: ros2 command not found")
    plugin = subprocess.run(
        ("ros2", "pkg", "prefix", "rosbag2_storage_mcap"),
        text=True,
        capture_output=True,
        check=False,
    )
    if plugin.returncode != 0:
        raise RuntimeError(
            "MCAP storage plugin is missing; install it with: "
            "sudo apt-get install ros-humble-rosbag2-storage-mcap"
        )
    try:
        from foxglove_msgs.msg import Grid
        from rclpy.serialization import serialize_message
        serialize_message(Grid())
    except (ImportError, OSError) as error:
        raise RuntimeError("v2 messages unavailable; source ROS2 and install ros-humble-foxglove-msgs") from error

    arguments.das_config = arguments.das_config.expanduser().resolve()
    arguments.das_sdk_root = arguments.das_sdk_root.expanduser().resolve()
    arguments.scale_calibration_path = (
        arguments.scale_calibration_path.expanduser().resolve()
    )
    arguments.output_root = arguments.output_root.expanduser().resolve()
    if not arguments.das_config.is_file():
        raise FileNotFoundError(f"DAS config not found: {arguments.das_config}")
    if not (arguments.das_sdk_root / "scripts" / "__init__.py").is_file():
        raise FileNotFoundError(
            f"DAS SDK scripts package not found: {arguments.das_sdk_root}"
        )
    resolve_scale_factor(arguments.scale_factor, arguments.scale_calibration_path)
    configurations = load_das_finger_configurations(arguments.das_config)
    missing_devices = [
        path
        for config in configurations
        for path in (
            (config.serial_port,)
            if arguments.no_vision
            else (config.serial_port, config.camera_device)
        )
        if not Path(path).exists()
    ]
    if missing_devices:
        raise FileNotFoundError(
            "DAS devices not found: " + ", ".join(missing_devices)
        )
    arguments.calibration = [
        path.expanduser().resolve() for path in arguments.calibration
    ]
    missing_calibrations = [
        str(path) for path in arguments.calibration if not path.is_file()
    ]
    if missing_calibrations:
        raise FileNotFoundError(
            "calibration files not found: " + ", ".join(missing_calibrations)
        )

    disk_path = arguments.output_root
    while not disk_path.exists() and disk_path != disk_path.parent:
        disk_path = disk_path.parent
    free_gib = shutil.disk_usage(disk_path).free / (1024**3)
    print(f"Preflight OK; output disk free: {free_gib:.1f} GiB", flush=True)


def _build_recorder_command(arguments, python):
    recorder_ready_file = _recorder_ready_file(arguments)
    calibrations = [arguments.das_config, arguments.scale_calibration_path]
    for path in arguments.calibration:
        if path not in calibrations:
            calibrations.append(path)
    recorder = [
        python,
        "-m", "xr_marvin_teleop.cli.record",
        "--task",
        arguments.task,
        "--operator",
        arguments.operator,
        "--robot-model",
        arguments.robot_model,
        "--output-root",
        str(arguments.output_root),
        "--das-config",
        str(arguments.das_config),
        "--ready-file",
        str(recorder_ready_file),
        "--camera-startup-timeout",
        str(arguments.camera_startup_timeout),
    ]
    if arguments.config is not None:
        recorder.extend(("--config", str(arguments.config)))
    for key in ("session", "episode_id"):
        value = getattr(arguments, key, None)
        if value:
            recorder.extend(("--" + key.replace("_", "-"), value))
    for key, value in video_options(arguments).items():
        if key in VIDEO_VARIANTS:
            recorder.append(f"--{key}" if value else f"--no-{key}")
        else:
            recorder.extend(("--" + key.replace("_", "-"), str(value)))
    for path in calibrations:
        recorder.extend(("--calibration", str(path)))
    for item in arguments.metadata:
        recorder.extend(("--metadata", item))
    if arguments.notes:
        recorder.extend(("--notes", arguments.notes))
    if arguments.max_duration is not None:
        recorder.extend(("--max-duration", str(arguments.max_duration)))
    if arguments.no_vision:
        recorder.append("--no-vision")
    elif arguments.preview_root is not None:
        recorder.extend(("--preview-root", str(arguments.preview_root)))
    return recorder


def _build_hardware_command(arguments, python):
    command = [
        python,
        "-m", "xr_marvin_teleop.cli.hardware",
        "--enable-hardware",
        "--confirmed-estop",
        "--confirmed-joint-mapping",
        "--confirmed-robot-model",
        arguments.robot_model,
        "--robot-ip",
        arguments.robot_ip,
        "--das-gripper-config",
        str(arguments.das_config),
        "--das-from-ros2",
        "--scale-calibration-path",
        str(arguments.scale_calibration_path),
        "--thumbstick-y-sign",
        str(arguments.thumbstick_y_sign),
        "--gripper-mode",
        arguments.gripper_mode,
        "--ros2",
        "--pico-from-ros2",
        "--joint-command-max-speed-deg-s",
        str(arguments.joint_command_max_speed_deg_s),
    ]
    if arguments.scale_factor is not None:
        command.extend(("--scale-factor", str(arguments.scale_factor)))
    if arguments.nsp_lateral:
        command.extend(
            (
                "--nsp-lateral",
                "--nsp-max-angle",
                str(arguments.nsp_max_angle),
                "--nsp-angle-rate",
                str(arguments.nsp_angle_rate),
                "--nsp-lateral-deadzone",
                str(arguments.nsp_lateral_deadzone),
                "--nsp-lateral-range",
                str(arguments.nsp_lateral_range),
                "--nsp-lateral-sign-left",
                str(arguments.nsp_lateral_sign_left),
                "--nsp-lateral-sign-right",
                str(arguments.nsp_lateral_sign_right),
            )
        )
    return command


def _build_das_commands(arguments, python):
    return {
        f"das_{side}": [
            python,
            "-m", "xr_marvin_teleop.cli.das",
            "--side",
            side,
            "--config",
            str(arguments.das_config),
            "--sdk-root",
            str(arguments.das_sdk_root),
            "--ready-timeout",
            str(arguments.startup_timeout),
            "--encoder-stale-timeout",
            str(arguments.effective_config["runtime"]["encoder_stale_timeout_s"]),
        ]
        for side in ("left", "right")
    }


def _build_commands(arguments):
    python = str(PROJECT_PYTHON)
    pico = [
        python,
        "-m", "xr_marvin_teleop.cli.pico",
        "--poll-hz",
        str(arguments.pico_poll_hz),
    ]
    return {
        "pico": pico,
        **_build_das_commands(arguments, python),
        "recorder": _build_recorder_command(arguments, python),
        "hardware": _build_hardware_command(arguments, python),
    }


def _recorder_ready_file(arguments):
    return Path(arguments.output_root) / f".recorder-ready-{os.getpid()}"


def _start_process(name, command, cpus, nice=0):
    print(f"Starting {name}: {shlex.join(command)}", flush=True)
    environment = os.environ.copy()
    environment["PYTHONUNBUFFERED"] = "1"
    process = subprocess.Popen(
        command,
        cwd=PROJECT_ROOT,
        env=environment,
        start_new_session=True,
    )
    try:
        os.sched_setaffinity(process.pid, cpus)
        if nice:
            os.setpriority(os.PRIO_PROCESS, process.pid, nice)
    except Exception:
        _signal_process(process, signal.SIGTERM)
        process.wait(timeout=5.0)
        raise
    print(
        f"{name} scheduling: CPUs {','.join(map(str, cpus))}, nice {nice:+d}",
        flush=True,
    )
    return process


def _validated_cpu_sets(cpu_sets=None):
    cpu_sets = PROCESS_CPUS if cpu_sets is None else cpu_sets
    available = set(os.sched_getaffinity(0))
    missing = {
        name: sorted(set(cpus) - available)
        for name, cpus in cpu_sets.items()
        if not set(cpus).issubset(available)
    }
    if missing:
        details = "; ".join(
            f"{name}: {','.join(map(str, cpus))}"
            for name, cpus in missing.items()
        )
        raise RuntimeError(f"planned CPU affinity is unavailable ({details})")
    return cpu_sets


def _wait_for_pico(process, timeout_seconds):
    deadline = time.monotonic() + timeout_seconds
    with closing(RosPicoClient()) as client:
        while time.monotonic() < deadline:
            return_code = process.poll()
            if return_code is not None:
                raise RuntimeError(
                    f"PICO publisher exited during startup with code {return_code}"
                )
            try:
                snapshot = client.wait_for_fresh_snapshot(
                    timeout_seconds=min(0.5, deadline - time.monotonic())
                )
            except TimeoutError:
                continue
            print(
                f"PICO ready; source timestamp: {snapshot.timestamp_ns}", flush=True
            )
            return
    raise TimeoutError("PICO produced no valid ROS2 frame before startup timeout")


def _wait_for_das(processes, configuration_path, timeout_seconds):
    deadline = time.monotonic() + timeout_seconds
    configurations = load_das_finger_configurations(configuration_path)
    with closing(
        RosDasClient(configurations, ready_timeout_seconds=timeout_seconds)
    ) as client:
        while time.monotonic() < deadline:
            for name, process in processes.items():
                return_code = process.poll()
                if return_code is not None:
                    raise RuntimeError(
                        f"{name} source exited during startup with code {return_code}"
                    )
            try:
                client.connect(
                    timeout_seconds=min(0.5, deadline - time.monotonic())
                )
            except TimeoutError:
                continue
            print(
                f"DAS ready; encoder distances: "
                f"{client.get_encoder_distances()}",
                flush=True,
            )
            return
    raise TimeoutError("DAS produced no valid ROS2 state before startup timeout")


def _wait_for_recorder(process, ready_file, timeout_seconds):
    deadline = time.monotonic() + timeout_seconds
    while time.monotonic() < deadline:
        return_code = process.poll()
        if return_code is not None:
            raise RuntimeError(
                f"episode recorder exited during startup with code {return_code}"
            )
        if ready_file.is_file():
            ready_file.unlink()
            return
        time.sleep(0.05)
    raise TimeoutError("episode recorder did not become ready")


def _start_devices(arguments, commands, cpu_sets, processes):
    processes["pico"] = _start_process(
        "PICO publisher", commands["pico"], cpu_sets["pico"],
        nice=arguments.effective_config["runtime"]["nice"]["pico"],
    )
    _wait_for_pico(processes["pico"], arguments.startup_timeout)
    for side in ("left", "right"):
        name = f"das_{side}"
        processes[name] = _start_process(
            f"DAS {side} source", commands[name], cpu_sets[name],
            nice=arguments.effective_config["runtime"]["nice"][name],
        )
    _wait_for_das(
        {name: processes[name] for name in ("das_left", "das_right")},
        arguments.das_config,
        arguments.startup_timeout,
    )


def _finish_starting_devices(arguments, commands, cpu_sets, processes):
    if processes["pico"].poll() is not None:
        raise RuntimeError("PICO publisher stopped before hardware startup")
    for name in ("das_left", "das_right"):
        if processes[name].poll() is not None:
            raise RuntimeError(f"{name} source stopped before hardware startup")
    processes["hardware"] = _start_process(
        "Marvin hardware", commands["hardware"], cpu_sets["hardware"],
        nice=arguments.effective_config["runtime"]["nice"]["hardware"],
    )


def _start_recording(arguments, commands, cpu_sets, processes):
    ready_file = _recorder_ready_file(arguments)
    ready_file.unlink(missing_ok=True)
    processes["recorder"] = _start_process(
        "episode recorder",
        commands["recorder"],
        cpu_sets["recorder"],
        nice=arguments.effective_config["runtime"]["nice"]["recorder"],
    )
    _wait_for_recorder(
        processes["recorder"], ready_file, arguments.camera_startup_timeout + 2.0
    )


def _signal_process(process, signal_number):
    try:
        os.killpg(process.pid, signal_number)
    except ProcessLookupError:
        pass


def _stop_process(process, name, timeout_seconds):
    return_code = process.poll()
    if return_code is not None:
        return return_code
    print(f"Stopping {name}...", flush=True)
    _signal_process(process, signal.SIGINT)
    if timeout_seconds is None:
        return process.wait()
    try:
        return process.wait(timeout=timeout_seconds)
    except subprocess.TimeoutExpired:
        print(f"{name} did not stop cleanly; sending SIGTERM", flush=True)
        _signal_process(process, signal.SIGTERM)
        try:
            return process.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            print(f"{name} still running; sending SIGKILL", flush=True)
            _signal_process(process, signal.SIGKILL)
            return process.wait()


def _shutdown_processes(processes, stop_process=_stop_process, timeouts=None):
    timeouts = read_json(DEFAULT_CONFIG)["runtime"]["shutdown_timeout_s"] if timeouts is None else timeouts
    results = {}
    # Stop motion first; keep recording alive until the hardware and DAS have exited.
    for name in ("hardware", "das_left", "das_right", "recorder", "pico"):
        process = processes.get(name)
        if process is not None:
            results[name] = stop_process(process, name, timeouts[name])
    return results


def _monitor(processes, stop_requested):
    while not stop_requested.is_set():
        for name in (
            "hardware",
            "das_left",
            "das_right",
            "recorder",
            "pico",
        ):
            process = processes.get(name)
            if process is None:
                continue
            return_code = process.poll()
            if return_code is not None:
                print(f"{name} exited with code {return_code}", flush=True)
                return name, return_code
        time.sleep(0.2)
    return "signal", 0


def main(arguments=None):
    parsed = parse_command_line_arguments(arguments)
    if parsed.print_effective_config:
        print(json.dumps(parsed.effective_config, ensure_ascii=False, indent=2))
        return 0
    resources = ExitStack()
    try:
        parsed.output_root.mkdir(parents=True, exist_ok=True)
        # Persistent devices may run during export; recording remains mutually exclusive.
        if parsed.part != "devices":
            resources.enter_context(activity_lock(parsed.output_root))
        temporary = resources.enter_context(tempfile.TemporaryDirectory(prefix=".collection-config-", dir=parsed.output_root))
        freeze_arguments(parsed, Path(temporary) / "config")
        if parsed.part != "recording":
            resources.enter_context(active_devices(parsed.effective_config))
        else:
            check_active_devices(parsed.effective_config)
        _preflight(parsed)
        commands = _build_commands(parsed)
        cpu_sets = _validated_cpu_sets(parsed.effective_config["runtime"]["cpus"])
    except Exception as error:
        resources.close()
        print(f"Collection preflight failed: {error}", file=sys.stderr, flush=True)
        return 1
    processes = {}
    stop_requested = threading.Event()
    previous_handlers = {}

    def request_stop(_signal_number, _frame):
        if not stop_requested.is_set():
            print("Stop requested; shutting down in safe order...", flush=True)
        stop_requested.set()

    exit_code = 0
    try:
        for signal_number in (signal.SIGINT, signal.SIGTERM):
            previous_handlers[signal_number] = signal.signal(
                signal_number, request_stop
            )
        if parsed.part != "recording":
            _start_devices(parsed, commands, cpu_sets, processes)
        if parsed.part != "devices":
            _start_recording(parsed, commands, cpu_sets, processes)
        if parsed.part != "recording":
            _finish_starting_devices(parsed, commands, cpu_sets, processes)
        active_name = "Collection" if parsed.part == "all" else parsed.part.capitalize()
        print(f"{active_name} active; press Ctrl-C once to stop safely", flush=True)
        if parsed.ready_file is not None:
            parsed.ready_file.touch()
        exited_name, return_code = _monitor(processes, stop_requested)
        if exited_name in ("pico", "das_left", "das_right"):
            exit_code = return_code or 1
        else:
            exit_code = 0 if return_code == 0 else 1
    except Exception as error:
        print(f"Collection supervisor error: {error}", file=sys.stderr, flush=True)
        exit_code = 1
    finally:
        try:
            shutdown_results = _shutdown_processes(processes, timeouts=parsed.effective_config["runtime"]["shutdown_timeout_s"])
        finally:
            resources.close()
        _recorder_ready_file(parsed).unlink(missing_ok=True)
        if parsed.ready_file is not None:
            parsed.ready_file.unlink(missing_ok=True)
        for signal_number, handler in previous_handlers.items():
            signal.signal(signal_number, handler)
        if any(return_code != 0 for return_code in shutdown_results.values()):
            exit_code = 1
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
