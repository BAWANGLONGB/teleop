#!/usr/bin/env python3
"""Reset Marvin to its initial pose without starting data collection."""

import argparse
import math
import os
import sys
import time
from pathlib import Path

import numpy as np

from xr_marvin_teleop.common.marvin_postures import MARVIN_INITIAL_POSE_Q_RAD
from xr_marvin_teleop.hardware.interface.marvin import (
    MarvinSdkAdapter,
    load_active_tool_configs,
)
from xr_marvin_teleop.hardware.marvin_teleop_controller import (
    DEFAULT_CONTROL_HZ,
    DEFAULT_JOINT_ACCELERATION_RATIO,
    DEFAULT_JOINT_D,
    DEFAULT_JOINT_K,
    DEFAULT_JOINT_VELOCITY_RATIO,
    MarvinHardwareTeleopController,
)


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_SDK_ROOT = PROJECT_ROOT.parent / "TJArm" / "tj_fx_robot-master"
DEFAULT_TOOLS_CONFIG = PROJECT_ROOT.parent / "TJArm" / "tools_cfg.json"
CONFLICTING_PROGRAMS = (
    "run_collection.py",
    "record_episode.py",
    "teleop_marvin_hardware.py",
)


def running_processes(program_names=CONFLICTING_PROGRAMS):
    found = set()
    for process in Path("/proc").iterdir():
        if not process.name.isdigit() or int(process.name) == os.getpid():
            continue
        try:
            arguments = (process / "cmdline").read_bytes().split(b"\0")
        except OSError:
            continue
        found.update(
            Path(argument.decode(errors="ignore")).name
            for argument in arguments
            if argument
            and Path(argument.decode(errors="ignore")).name in program_names
        )
    return sorted(found)


def parse_command_line_arguments(arguments=None):
    parser = argparse.ArgumentParser(
        description="Reset Marvin without PICO, DAS, ROS2, or data collection"
    )
    parser.add_argument("--enable-hardware", action="store_true")
    parser.add_argument("--confirmed-estop", action="store_true")
    parser.add_argument("--confirmed-workspace-clear", action="store_true")
    parser.add_argument("--confirmed-robot-model", default="")
    parser.add_argument("--robot-ip", default="192.168.1.190")
    parser.add_argument("--sdk-root", type=Path, default=DEFAULT_SDK_ROOT)
    parser.add_argument("--tools-config", type=Path, default=DEFAULT_TOOLS_CONFIG)
    parser.add_argument("--duration", type=float, default=3.0)
    parser.add_argument("--control-hz", type=float, default=DEFAULT_CONTROL_HZ)
    parser.add_argument("--position-tolerance-deg", type=float, default=1.0)
    parser.add_argument("--settle-timeout", type=float, default=3.0)
    parser.add_argument("--expected-sdk-version", type=int, default=100343014)
    parsed = parser.parse_args(arguments)
    missing = [
        flag
        for flag, enabled in (
            ("--enable-hardware", parsed.enable_hardware),
            ("--confirmed-estop", parsed.confirmed_estop),
            ("--confirmed-workspace-clear", parsed.confirmed_workspace_clear),
            (
                "--confirmed-robot-model <exact model>",
                parsed.confirmed_robot_model.strip(),
            ),
        )
        if not enabled
    ]
    if missing:
        parser.error("required hardware confirmations: " + ", ".join(missing))
    if (
        not math.isfinite(parsed.duration)
        or parsed.duration <= 0.0
        or not math.isfinite(parsed.control_hz)
        or not 50.0 <= parsed.control_hz <= 200.0
        or not math.isclose(
            1000.0 / parsed.control_hz, round(1000.0 / parsed.control_hz)
        )
        or not math.isfinite(parsed.position_tolerance_deg)
        or parsed.position_tolerance_deg <= 0.0
        or not math.isfinite(parsed.settle_timeout)
        or parsed.settle_timeout <= 0.0
    ):
        parser.error("invalid duration, control rate, tolerance, or settle timeout")
    return parsed


def reset_robot(
    adapter,
    tool_configurations,
    duration=3.0,
    control_hz=DEFAULT_CONTROL_HZ,
    position_tolerance_deg=1.0,
    settle_timeout=3.0,
    expected_sdk_version=100343014,
    monotonic=time.monotonic,
    sleep=time.sleep,
):
    period = 1.0 / control_hz
    active_error = None
    try:
        adapter.connect()
        actual_sdk_version = adapter.sdk_version()
        if (
            expected_sdk_version is not None
            and actual_sdk_version != expected_sdk_version
        ):
            raise RuntimeError(
                "Marvin control SDK version mismatch: "
                f"expected {expected_sdk_version}, "
                f"got {actual_sdk_version}"
            )
        feedback = adapter.wait_for_fresh_feedback()
        MarvinHardwareTeleopController._require_healthy_feedback(feedback, False)
        if not all(feedback.low_speed):
            raise RuntimeError("Marvin must be stationary before reset")
        adapter.configure_control_parameters(
            DEFAULT_JOINT_K,
            DEFAULT_JOINT_D,
            DEFAULT_JOINT_K,
            DEFAULT_JOINT_D,
            tool_configurations,
            joint_velocity_ratio=DEFAULT_JOINT_VELOCITY_RATIO,
            joint_acceleration_ratio=DEFAULT_JOINT_ACCELERATION_RATIO,
        )
        sleep(0.2)
        adapter.enter_joint_impedance()
        sleep(1.0)
        feedback = adapter.wait_for_fresh_feedback(required_updates=1)
        MarvinHardwareTeleopController._require_healthy_feedback(feedback, True)
        adapter.enable_pd_feedforward(round(1000.0 / control_hz))
        sleep(1.0)
        feedback = adapter.wait_for_fresh_feedback(required_updates=1)
        MarvinHardwareTeleopController._require_healthy_feedback(feedback, True)

        start_q_rad = feedback.q_rad.copy()
        started_at = next_cycle = monotonic()
        while True:
            progress = min(1.0, (monotonic() - started_at) / duration)
            blend = 0.5 - 0.5 * np.cos(np.pi * progress)
            target_q_rad = start_q_rad + blend * (
                MARVIN_INITIAL_POSE_Q_RAD - start_q_rad
            )
            feedback = adapter.read_state()
            MarvinHardwareTeleopController._require_healthy_feedback(
                feedback, True
            )
            adapter.send_joint_command(target_q_rad)
            if progress >= 1.0:
                break
            next_cycle += period
            sleep(max(0.0, next_cycle - monotonic()))

        settle_deadline = monotonic() + settle_timeout
        tolerance_rad = np.deg2rad(position_tolerance_deg)
        while True:
            feedback = adapter.read_state()
            MarvinHardwareTeleopController._require_healthy_feedback(
                feedback, True
            )
            if all(feedback.low_speed) and np.max(
                np.abs(feedback.q_rad - MARVIN_INITIAL_POSE_Q_RAD)
            ) <= tolerance_rad:
                return
            if monotonic() >= settle_deadline:
                raise TimeoutError("Marvin did not reach the initial pose")
            adapter.send_joint_command(MARVIN_INITIAL_POSE_Q_RAD)
            sleep(period)
    except BaseException as error:
        active_error = error
        raise
    finally:
        try:
            adapter.set_idle()
        except Exception as cleanup_error:
            if active_error is None:
                raise
            print(f"Marvin reset cleanup warning: {cleanup_error}", file=sys.stderr)
        finally:
            adapter.release()


def main(arguments=None):
    parsed = parse_command_line_arguments(arguments)
    conflicts = running_processes()
    if conflicts:
        print(
            "Marvin reset refused while these processes are running: "
            + ", ".join(conflicts),
            file=sys.stderr,
        )
        return 2
    adapter = MarvinSdkAdapter(
        robot_ip_address=parsed.robot_ip,
        sdk_root_path=parsed.sdk_root,
    )
    try:
        reset_robot(
            adapter,
            load_active_tool_configs(parsed.tools_config),
            duration=parsed.duration,
            control_hz=parsed.control_hz,
            position_tolerance_deg=parsed.position_tolerance_deg,
            settle_timeout=parsed.settle_timeout,
            expected_sdk_version=parsed.expected_sdk_version,
        )
    except Exception as error:
        print(f"Marvin reset failed: {error}", file=sys.stderr, flush=True)
        return 1
    print("Marvin reset completed.", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
