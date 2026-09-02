#!/usr/bin/env python3
"""Run PICO publishing, episode recording, and hardware control as one job."""

import argparse
import math
import os
import shlex
import shutil
import signal
import subprocess
import sys
import threading
import time
from contextlib import closing
from pathlib import Path

from xr_marvin_teleop.hardware.interface.das_finger import (
    load_das_finger_configurations,
)
from xr_marvin_teleop.ros.das_client import RosDasClient
from xr_marvin_teleop.ros.pico_client import RosPicoClient


PROJECT_ROOT = Path(__file__).resolve().parents[2]
PROCESS_CPUS = {
    "hardware": (2, 3, 18, 19),
    "pico": (4, 20),
    "das": (5, 6, 21, 22),
    "recorder": (*range(7, 16), *range(23, 32)),
}


def parse_command_line_arguments(arguments=None):
    parser = argparse.ArgumentParser(
        description="Run one ROS2 PICO-Marvin-DAS collection job"
    )
    parser.add_argument("--task", required=True)
    parser.add_argument("--operator", default=os.environ.get("USER", "unknown"))
    parser.add_argument("--robot-model", required=True)
    parser.add_argument("--enable-hardware", action="store_true")
    parser.add_argument("--confirmed-estop", action="store_true")
    parser.add_argument("--confirmed-joint-mapping", action="store_true")
    parser.add_argument("--das-config", required=True, type=Path)
    parser.add_argument("--das-sdk-root", required=True, type=Path)
    parser.add_argument(
        "--scale-calibration-path",
        type=Path,
        default=PROJECT_ROOT / "logs" / "marvin_scale_calibration.json",
    )
    parser.add_argument("--calibration", action="append", default=[], type=Path)
    parser.add_argument("--output-root", type=Path, default=PROJECT_ROOT / "dataset")
    parser.add_argument("--metadata", action="append", default=[], metavar="KEY=VALUE")
    parser.add_argument("--notes", default="")
    parser.add_argument("--max-duration", type=float)
    parser.add_argument("--no-vision", action="store_true")
    parser.add_argument("--pico-poll-hz", type=float, default=120.0)
    parser.add_argument("--startup-timeout", type=float, default=15.0)
    parser.add_argument("--robot-ip", default="192.168.1.190")
    parser.add_argument("--thumbstick-y-sign", type=int, choices=(-1, 1), default=1)
    parser.add_argument("--scale-factor", type=float)
    parser.add_argument("--nsp-lateral", action="store_true")
    parser.add_argument("--nsp-max-angle", type=float, default=5.0)
    parser.add_argument("--nsp-angle-rate", type=float, default=20.0)
    parser.add_argument("--nsp-lateral-deadzone", type=float, default=0.03)
    parser.add_argument("--nsp-lateral-range", type=float, default=0.12)
    parser.add_argument(
        "--nsp-lateral-sign-left", type=int, choices=(-1, 1), default=1
    )
    parser.add_argument(
        "--nsp-lateral-sign-right", type=int, choices=(-1, 1), default=1
    )
    parsed = parser.parse_args(arguments)
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
    if not parsed.task.strip() or not parsed.robot_model.strip():
        parser.error("--task and --robot-model must not be empty")
    if any(
        "=" not in item or not item.split("=", 1)[0].strip()
        for item in parsed.metadata
    ):
        parser.error("--metadata must use a non-empty KEY=VALUE")
    if parsed.max_duration is not None and (
        not math.isfinite(parsed.max_duration) or parsed.max_duration <= 0.0
    ):
        parser.error("--max-duration must be positive")
    if not 30.0 <= parsed.pico_poll_hz <= 240.0:
        parser.error("--pico-poll-hz must be within [30, 240]")
    if not math.isfinite(parsed.startup_timeout) or parsed.startup_timeout <= 0.0:
        parser.error("--startup-timeout must be positive")
    if parsed.scale_factor is not None and (
        not math.isfinite(parsed.scale_factor) or parsed.scale_factor <= 0.0
    ):
        parser.error("--scale-factor must be positive and finite")
    if (
        not math.isfinite(parsed.nsp_max_angle)
        or not 0.0 < parsed.nsp_max_angle <= 30.0
        or not math.isfinite(parsed.nsp_angle_rate)
        or parsed.nsp_angle_rate <= 0.0
        or not math.isfinite(parsed.nsp_lateral_deadzone)
        or parsed.nsp_lateral_deadzone < 0.0
        or not math.isfinite(parsed.nsp_lateral_range)
        or parsed.nsp_lateral_range <= parsed.nsp_lateral_deadzone
    ):
        parser.error("invalid NSP angle, rate, deadzone, or range")
    return parsed


def _preflight(arguments):
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
    if not arguments.scale_calibration_path.is_file():
        raise FileNotFoundError(
            "scale calibration not found; complete A/A calibration first: "
            f"{arguments.scale_calibration_path}"
        )
    configurations = load_das_finger_configurations(arguments.das_config)
    missing_devices = [
        path
        for config in configurations
        for path in (config.serial_port, config.camera_device)
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


def _build_commands(arguments):
    python = sys.executable
    calibrations = [arguments.das_config, arguments.scale_calibration_path]
    for path in arguments.calibration:
        if path not in calibrations:
            calibrations.append(path)
    recorder = [
        python,
        str(PROJECT_ROOT / "scripts" / "data" / "record_episode.py"),
        "--task",
        arguments.task,
        "--operator",
        arguments.operator,
        "--robot-model",
        arguments.robot_model,
        "--output-root",
        str(arguments.output_root),
    ]
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

    hardware = [
        python,
        str(PROJECT_ROOT / "scripts" / "hardware" / "teleop_marvin_hardware.py"),
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
        "--ros2",
        "--pico-from-ros2",
    ]
    if arguments.scale_factor is not None:
        hardware.extend(("--scale-factor", str(arguments.scale_factor)))
    if arguments.nsp_lateral:
        hardware.extend(
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
    pico = [
        python,
        str(PROJECT_ROOT / "scripts" / "data" / "publish_pico.py"),
        "--poll-hz",
        str(arguments.pico_poll_hz),
    ]
    das = [
        python,
        str(PROJECT_ROOT / "scripts" / "data" / "publish_das.py"),
        "--config",
        str(arguments.das_config),
        "--sdk-root",
        str(arguments.das_sdk_root),
        "--ready-timeout",
        str(arguments.startup_timeout),
    ]
    return {
        "pico": pico,
        "das": das,
        "recorder": recorder,
        "hardware": hardware,
    }


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


def _validated_cpu_sets():
    available = set(os.sched_getaffinity(0))
    missing = {
        name: sorted(set(cpus) - available)
        for name, cpus in PROCESS_CPUS.items()
        if not set(cpus).issubset(available)
    }
    if missing:
        details = "; ".join(
            f"{name}: {','.join(map(str, cpus))}"
            for name, cpus in missing.items()
        )
        raise RuntimeError(f"planned CPU affinity is unavailable ({details})")
    return PROCESS_CPUS


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


def _wait_for_das(process, configuration_path, timeout_seconds):
    deadline = time.monotonic() + timeout_seconds
    configurations = load_das_finger_configurations(configuration_path)
    with closing(
        RosDasClient(configurations, ready_timeout_seconds=timeout_seconds)
    ) as client:
        while time.monotonic() < deadline:
            return_code = process.poll()
            if return_code is not None:
                raise RuntimeError(
                    f"DAS source exited during startup with code {return_code}"
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


def _require_running(process, name, seconds=2.0):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        return_code = process.poll()
        if return_code is not None:
            raise RuntimeError(
                f"{name} exited during startup with code {return_code}"
            )
        time.sleep(0.1)


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


def _shutdown_processes(processes, stop_process=_stop_process):
    results = {}
    for name, timeout_seconds in (
        ("hardware", 15.0),
        ("das", 10.0),
        ("recorder", None),
        ("pico", 10.0),
    ):
        process = processes.get(name)
        if process is not None:
            results[name] = stop_process(process, name, timeout_seconds)
    return results


def _monitor(processes, stop_requested):
    while not stop_requested.is_set():
        for name in ("hardware", "das", "recorder", "pico"):
            return_code = processes[name].poll()
            if return_code is not None:
                print(f"{name} exited with code {return_code}", flush=True)
                return name, return_code
        time.sleep(0.2)
    return "signal", 0


def main(arguments=None):
    parsed = parse_command_line_arguments(arguments)
    try:
        _preflight(parsed)
        commands = _build_commands(parsed)
        cpu_sets = _validated_cpu_sets()
    except Exception as error:
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
        processes["pico"] = _start_process(
            "PICO publisher", commands["pico"], cpu_sets["pico"]
        )
        _wait_for_pico(processes["pico"], parsed.startup_timeout)
        processes["das"] = _start_process(
            "DAS source", commands["das"], cpu_sets["das"]
        )
        _wait_for_das(
            processes["das"], parsed.das_config, parsed.startup_timeout
        )
        processes["recorder"] = _start_process(
            "episode recorder",
            commands["recorder"],
            cpu_sets["recorder"],
            nice=10,
        )
        _require_running(processes["recorder"], "episode recorder")
        if processes["pico"].poll() is not None:
            raise RuntimeError("PICO publisher stopped before hardware startup")
        if processes["das"].poll() is not None:
            raise RuntimeError("DAS source stopped before hardware startup")
        processes["hardware"] = _start_process(
            "Marvin hardware", commands["hardware"], cpu_sets["hardware"]
        )
        print("Collection active; press Ctrl-C once to stop safely", flush=True)
        exited_name, return_code = _monitor(processes, stop_requested)
        if exited_name in ("pico", "das"):
            exit_code = return_code or 1
        else:
            exit_code = 0 if return_code == 0 else 1
    except Exception as error:
        print(f"Collection supervisor error: {error}", file=sys.stderr, flush=True)
        exit_code = 1
    finally:
        shutdown_results = _shutdown_processes(processes)
        for signal_number, handler in previous_handlers.items():
            signal.signal(signal_number, handler)
        if any(return_code != 0 for return_code in shutdown_results.values()):
            exit_code = 1
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
