"""Read-only Marvin connectivity, identity, state, and frame freshness check."""

import hashlib
import json
import os
import time

import numpy as np
import tyro

from xrobotoolkit_teleop.common.marvin_calibration_recorder import (
    MarvinCalibrationRecorder,
)
from xrobotoolkit_teleop.hardware.interface.marvin import MarvinSdkAdapter


PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
DEFAULT_SDK_ROOT = os.path.abspath(
    os.path.join(PROJECT_ROOT, "..", "TJArm", "tj_fx_robot-master")
)
DEFAULT_LOG_DIR = os.path.join(PROJECT_ROOT, "logs")
DEFAULT_ROBOT_URDF = os.path.join(PROJECT_ROOT, "assets", "marvin", "marvin_dual.urdf")
DEFAULT_EXPECTED_SDK_VERSION = 100343014
MARVIN_JOINT_NAMES = [
    *(f"Joint{index}_L" for index in range(1, 8)),
    *(f"Joint{index}_R" for index in range(1, 8)),
]


def _sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as source_file:
        for chunk in iter(lambda: source_file.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _require_sdk_version(adapter, expected_sdk_version):
    actual_sdk_version = adapter.sdk_version()
    if actual_sdk_version != expected_sdk_version:
        raise RuntimeError(
            f"Marvin SDK version mismatch: expected {expected_sdk_version}, "
            f"got {actual_sdk_version}"
        )
    return actual_sdk_version


def main(
    robot_ip: str = "192.168.1.190",
    sdk_root: str = DEFAULT_SDK_ROOT,
    expected_sdk_version: int = DEFAULT_EXPECTED_SDK_VERSION,
    duration_s: float = 10.0,
    sample_hz: float = 200.0,
    save_calibration_data: bool = True,
    log_dir: str = DEFAULT_LOG_DIR,
    robot_urdf_path: str = DEFAULT_ROBOT_URDF,
):
    """Inspect Marvin without changing state, limits, Tool, K/D, or joint targets."""
    if duration_s <= 0.0 or sample_hz <= 0.0:
        raise ValueError("duration_s and sample_hz must be positive")
    adapter = MarvinSdkAdapter(robot_ip=robot_ip, sdk_root=sdk_root)
    recorder = None
    terminal_state = {"state_before_shutdown": "initializing"}
    try:
        adapter.connect()
        sdk_version = _require_sdk_version(adapter, expected_sdk_version)
        first = adapter.wait_for_fresh_feedback()
        if save_calibration_data:
            raw_inputs = adapter.last_raw_state["inputs"]
            recorder = MarvinCalibrationRecorder(
                log_dir,
                {
                    "mode": "strictly_read_only",
                    "robot_ip": robot_ip,
                    "sdk_version": sdk_version,
                    "expected_sdk_version": expected_sdk_version,
                    "sample_hz": sample_hz,
                    "joint_names": MARVIN_JOINT_NAMES,
                    "robot_urdf_path": os.path.abspath(robot_urdf_path),
                    "robot_urdf_sha256": _sha256(robot_urdf_path),
                    "controller_tool_left": {
                        "kinematics_mm_deg": raw_inputs[0].get("tool_kine"),
                        "dynamics_vendor_units": raw_inputs[0].get("tool_dyn"),
                    },
                    "controller_tool_right": {
                        "kinematics_mm_deg": raw_inputs[1].get("tool_kine"),
                        "dynamics_vendor_units": raw_inputs[1].get("tool_dyn"),
                    },
                },
                MARVIN_JOINT_NAMES,
            )
            recorder.record(
                first,
                command=None,
                control=None,
                safety_state="read_only",
                safety_reason="initial fresh feedback",
                sdk_read_duration_ms=0.0,
            )
        print(
            json.dumps(
                {
                    "sdk_version": sdk_version,
                    "frame_serial": first.frame_serial,
                    "arm_state": first.arm_state,
                    "error_code": first.error_code,
                    "q_deg": [round(value, 4) for value in np.rad2deg(first.q_rad)],
                },
                ensure_ascii=False,
                indent=2,
            )
        )
        previous = first.frame_serial
        advances = [0, 0]
        observed_errors = set(first.error_code)
        observed_states = set(first.arm_state)
        deadline = time.monotonic() + duration_s
        period = 1.0 / sample_hz
        while time.monotonic() < deadline:
            read_start_ns = time.monotonic_ns()
            state = adapter.read_state()
            read_duration_ms = (time.monotonic_ns() - read_start_ns) / 1e6
            for index in range(2):
                if state.frame_serial[index] != previous[index]:
                    advances[index] += 1
            observed_errors.update(state.error_code)
            observed_states.update(state.arm_state)
            previous = state.frame_serial
            if recorder is not None:
                recorder.record(
                    state,
                    command=None,
                    control=None,
                    safety_state="read_only",
                    safety_reason="read-only feedback collection",
                    sdk_read_duration_ms=read_duration_ms,
                )
            time.sleep(period)
        passed = all(advances) and observed_errors == {0} and 100 not in observed_states
        print(
            json.dumps(
                {
                    "duration_s": duration_s,
                    "observed_frame_advances": advances,
                    "last_frame_serial": previous,
                    "observed_arm_states": sorted(observed_states),
                    "observed_error_codes": sorted(observed_errors),
                    "result": "read-only check passed" if passed else "read-only check failed",
                },
                ensure_ascii=False,
                indent=2,
            )
        )
        if not passed:
            terminal_state = {
                "state_before_shutdown": "failed",
                "reason_before_shutdown": "read-only validation failed",
            }
            raise RuntimeError(
                "Marvin read-only validation failed; do not enable hardware motion"
            )
        terminal_state = {
            "state_before_shutdown": "read_only",
            "reason_before_shutdown": "read-only validation passed",
        }
    finally:
        if recorder is not None:
            recorder.close(terminal_state=terminal_state)
            print(f"Calibration CSV: {recorder.csv_path.resolve()}")
            print(f"Calibration metadata: {recorder.metadata_path.resolve()}")
        adapter.release()


if __name__ == "__main__":
    tyro.cli(main)
