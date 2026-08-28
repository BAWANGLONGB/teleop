"""Narrow, unit-safe adapter around the vendor Marvin Python SDK."""

from __future__ import annotations

import configparser
import importlib.util
import json
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from xrobotoolkit_teleop.common.marvin_types import MarvinRobotState


def validate_vendor_driver_dependency(sdk_root):
    """Validate the exact local SDK payload and its motion-safety configuration."""
    root = Path(sdk_root).expanduser().resolve()
    sdk_file = root / "SDK_PYTHON" / "fx_robot.py"
    sdk_library = root / "SDK_PYTHON" / "libMarvinSDK.so"
    robot_config = root / "robot.ini"
    for description, path in (
        ("Marvin Python SDK", sdk_file),
        ("Marvin native SDK", sdk_library),
        ("Marvin robot configuration", robot_config),
    ):
        if not path.is_file():
            raise FileNotFoundError(f"{description} not found: {path}")

    parser = configparser.ConfigParser(interpolation=None)
    parser.optionxform = str
    try:
        with robot_config.open(encoding="utf-8") as source_file:
            parser.read_file(source_file)
        use_emg = parser.getint("R.BASIC", "UseEMG")
        joint_pid_types = (
            parser.getint("R.A0.BASIC", "JointPIDCtlType"),
            parser.getint("R.A1.BASIC", "JointPIDCtlType"),
        )
    except (configparser.Error, KeyError, ValueError) as error:
        raise ValueError(
            f"cannot validate required Marvin settings in {robot_config}: {error}"
        ) from error

    if joint_pid_types != (1, 1):
        raise RuntimeError(
            "Marvin PD teleoperation requires JointPIDCtlType=1 for both arms; "
            f"found A/B={joint_pid_types} in {robot_config}"
        )
    if use_emg != 1:
        raise PermissionError(
            "hardware motion is blocked because the vendor robot.ini has UseEMG="
            f"{use_emg}; restore emergency-stop monitoring through the official "
            "Marvin configuration procedure, verify the physical E-stop, and export "
            f"the accepted configuration back to {robot_config}"
        )
    return {
        "sdk_root": str(root),
        "sdk_python_path": str(sdk_file),
        "sdk_library_path": str(sdk_library),
        "robot_config_path": str(robot_config),
        "use_emg": use_emg,
        "joint_pid_ctl_type": joint_pid_types,
    }


@dataclass(frozen=True)
class MarvinToolConfig:
    kinematics_mm_deg: tuple[float, ...]
    dynamics_vendor_units: tuple[float, ...]

    def __post_init__(self):
        if len(self.kinematics_mm_deg) != 6:
            raise ValueError("tool kinematics must contain XYZABC")
        if len(self.dynamics_vendor_units) != 10:
            raise ValueError("tool dynamics must contain mass, COM, and six inertia values")
        if not np.all(np.isfinite(self.kinematics_mm_deg)) or not np.all(
            np.isfinite(self.dynamics_vendor_units)
        ):
            raise ValueError("tool parameters must be finite")


def load_vendor_sdk(sdk_root):
    sdk_directory = Path(sdk_root).expanduser().resolve() / "SDK_PYTHON"
    sdk_file = sdk_directory / "fx_robot.py"
    sdk_library = sdk_directory / "libMarvinSDK.so"
    if not sdk_file.is_file():
        raise FileNotFoundError(f"Marvin SDK module not found: {sdk_file}")
    if not sdk_library.is_file():
        raise FileNotFoundError(f"Marvin native SDK library not found: {sdk_library}")
    module_name = "xrobotoolkit_vendor_marvin_fx_robot"
    spec = importlib.util.spec_from_file_location(module_name, sdk_file)
    if spec is None or spec.loader is None:
        raise ImportError(f"cannot load Marvin SDK module from {sdk_file}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module.Marvin_Robot(), module.DCSS()


def load_active_tool_configs(path):
    """Load the selected arm0/arm1 vendor tool records without changing their units."""
    with Path(path).expanduser().open(encoding="utf-8") as tool_file:
        data = json.load(tool_file)
    result = []
    for arm_key in ("arm0", "arm1"):
        selected = data.get("current_tool", {}).get(arm_key)
        if not isinstance(selected, str):
            raise ValueError(f"tools config must select an active tool for {arm_key}")
        record = data[arm_key][selected]
        result.append(
            MarvinToolConfig(
                tuple(float(value) for value in record["kine"]),
                tuple(float(value) for value in record["dyn"]),
            )
        )
    return tuple(result)


class MarvinSdkAdapter:
    """Own SDK connection, validation, SI conversion, and dual-arm transactions."""

    _CLEAR_SET_RETRY_TIMEOUT_S = 0.02
    _CLEAR_SET_RETRY_INTERVAL_S = 0.001

    def __init__(self, robot_ip="192.168.1.190", sdk_root=None, robot=None, dcss=None):
        if robot is None or dcss is None:
            if sdk_root is None:
                raise ValueError("sdk_root is required unless robot and dcss are injected")
            robot, dcss = load_vendor_sdk(sdk_root)
        self.robot_ip = robot_ip
        self.sdk_root = None if sdk_root is None else str(Path(sdk_root).expanduser().resolve())
        self.robot = robot
        self.dcss = dcss
        self.connected = False
        self.released = False
        self.last_raw_state = None
        self._last_frame_serial = [None, None]
        self._frame_change_monotonic_ns = [None, None]

    def connect(self):
        if self.connected:
            return
        if self.released:
            raise RuntimeError("released Marvin SDK adapter cannot be reconnected")
        compatibility = getattr(self.robot, "check_sdk_type_compat", None)
        if compatibility is not None:
            result = compatibility()
            status = result[0] if isinstance(result, tuple) else result
            if status is not None and status < 0:
                raise RuntimeError(f"Marvin SDK ABI compatibility check failed: {result!r}")
        if not self.robot.connect(self.robot_ip):
            raise ConnectionError(
                f"failed to connect to Marvin at {self.robot_ip}; the controller may be occupied"
            )
        self.connected = True

    def sdk_version(self):
        getter = getattr(self.robot, "SDK_version", None)
        return None if getter is None else getter()

    def robot_name(self):
        getter = getattr(self.robot, "get_robot_name", None)
        return None if getter is None else getter()

    @staticmethod
    def _pair(raw, group, key, default=None):
        values = raw.get(group)
        if not isinstance(values, list) or len(values) < 2:
            raise ValueError(f"SDK feedback group '{group}' must contain both arms")
        result = []
        for arm in range(2):
            value = values[arm].get(key, default)
            if value is None:
                raise ValueError(f"SDK feedback missing {group}[{arm}].{key}")
            result.append(value)
        return result

    @staticmethod
    def _joint_pair(raw, group, key):
        pair = MarvinSdkAdapter._pair(raw, group, key)
        arrays = [np.asarray(values, dtype=float).reshape(-1) for values in pair]
        if any(values.size != 7 or not np.all(np.isfinite(values)) for values in arrays):
            raise ValueError(f"SDK field {group}.{key} must contain two finite 7-joint vectors")
        return np.concatenate(arrays)

    @staticmethod
    def _flag(value):
        if isinstance(value, (bytes, bytearray)):
            return any(byte != 0 for byte in value)
        return bool(value)

    def read_state(self):
        if not self.connected:
            raise RuntimeError("Marvin SDK is not connected")
        raw = self.robot.subscribe(self.dcss)
        if not isinstance(raw, dict):
            raise RuntimeError("Marvin subscribe() did not return a state dictionary")
        self.last_raw_state = raw
        frame_serial = tuple(int(value) for value in self._pair(raw, "outputs", "frame_serial"))
        now_ns = time.monotonic_ns()
        for index, serial in enumerate(frame_serial):
            if serial != self._last_frame_serial[index]:
                self._last_frame_serial[index] = serial
                self._frame_change_monotonic_ns[index] = now_ns
        # Use the older of the two last-advance times. Repeated reads of a
        # frozen SDK buffer therefore age naturally and either stalled arm
        # trips the feedback watchdog.
        last_advance_ns = min(
            value for value in self._frame_change_monotonic_ns if value is not None
        )
        q_deg = self._joint_pair(raw, "outputs", "fb_joint_pos")
        dq_deg_s = self._joint_pair(raw, "outputs", "fb_joint_vel")
        torque = self._joint_pair(raw, "outputs", "fb_joint_sToq")
        commanded_deg = self._joint_pair(raw, "inputs", "joint_cmd_pos")
        return MarvinRobotState(
            receipt_monotonic_ns=last_advance_ns,
            frame_serial=frame_serial,
            q_rad=np.deg2rad(q_deg),
            dq_rad_s=np.deg2rad(dq_deg_s),
            torque_nm=torque,
            arm_state=tuple(int(value) for value in self._pair(raw, "states", "cur_state")),
            command_state=tuple(int(value) for value in self._pair(raw, "states", "cmd_state")),
            error_code=tuple(int(value) for value in self._pair(raw, "states", "err_code")),
            low_speed=tuple(self._flag(value) for value in self._pair(raw, "outputs", "low_speed_flag")),
            input_frame_serial=tuple(
                int(value) for value in self._pair(raw, "inputs", "in_frame_serial", 0)
            ),
            frame_miss_count=tuple(
                int(value) for value in self._pair(raw, "inputs", "frame_miss_cnt", 0)
            ),
            system_cycle_miss_count=tuple(
                int(value) for value in self._pair(raw, "inputs", "sys_cyc_miss_cnt", 0)
            ),
            commanded_q_rad=np.deg2rad(commanded_deg),
        )

    def wait_for_fresh_feedback(self, timeout=2.0, required_updates=5):
        deadline = time.monotonic() + timeout
        previous = [None, None]
        updates = [0, 0]
        latest = None
        while time.monotonic() < deadline:
            latest = self.read_state()
            for arm, serial in enumerate(latest.frame_serial):
                if serial != 0 and serial != previous[arm]:
                    updates[arm] += 1
                    previous[arm] = serial
            if min(updates) >= required_updates:
                return latest
            time.sleep(0.01)
        raise TimeoutError(
            f"Marvin A/B feedback frames did not each advance {required_updates} times "
            f"within {timeout}s (A={updates[0]}, B={updates[1]})"
        )

    def _clear_set_when_ready(self):
        """Wait briefly for the vendor SDK's previous send buffer to clear."""
        deadline = time.monotonic() + self._CLEAR_SET_RETRY_TIMEOUT_S
        attempts = 0
        while True:
            attempts += 1
            if self.robot.clear_set():
                return
            if time.monotonic() >= deadline:
                raise TimeoutError(
                    "Marvin clear_set() remained busy for "
                    f"{self._CLEAR_SET_RETRY_TIMEOUT_S * 1000.0:.0f} ms "
                    f"({attempts} attempts)"
                )
            time.sleep(self._CLEAR_SET_RETRY_INTERVAL_S)

    def _send_transaction(
        self,
        setters,
        wait_response=False,
        timeout_ms=100,
        transaction_name="command",
    ):
        if not self.connected:
            raise RuntimeError("Marvin SDK is not connected")
        self._clear_set_when_ready()
        for description, setter in setters:
            if not setter():
                raise RuntimeError(f"Marvin SDK setter failed: {description}")
        if wait_response:
            result = self.robot.send_cmd_wait_response(timeout_ms)
            if result is not None and result < 0:
                raise RuntimeError(
                    f"Marvin {transaction_name} response returned an SDK error"
                )
            if result is None or result == 0:
                raise TimeoutError(
                    f"Marvin {transaction_name} response timed out after "
                    f"{timeout_ms} ms"
                )
        elif not self.robot.send_cmd():
            raise RuntimeError("Marvin send_cmd() failed")

    def configure_parameters(
        self,
        velocity_ratio,
        acceleration_ratio,
        left_k,
        left_d,
        right_k,
        right_d,
        left_tool=None,
        right_tool=None,
    ):
        setters = [
            (
                "left velocity/acceleration",
                lambda: self.robot.set_vel_acc("A", velocity_ratio, acceleration_ratio),
            ),
            (
                "right velocity/acceleration",
                lambda: self.robot.set_vel_acc("B", velocity_ratio, acceleration_ratio),
            ),
            ("left joint impedance", lambda: self.robot.set_joint_kd_params("A", left_k, left_d)),
            ("right joint impedance", lambda: self.robot.set_joint_kd_params("B", right_k, right_d)),
        ]
        if left_tool is not None:
            setters.append(
                (
                    "left tool",
                    lambda: self.robot.set_tool(
                        "A",
                        list(left_tool.kinematics_mm_deg),
                        list(left_tool.dynamics_vendor_units),
                    ),
                )
            )
        if right_tool is not None:
            setters.append(
                (
                    "right tool",
                    lambda: self.robot.set_tool(
                        "B",
                        list(right_tool.kinematics_mm_deg),
                        list(right_tool.dynamics_vendor_units),
                    ),
                )
            )
        self._send_transaction(
            setters,
            wait_response=True,
            transaction_name="parameter configuration",
        )

    def enter_joint_impedance(self):
        self._send_transaction(
            [
                ("left torque state", lambda: self.robot.set_state("A", 3)),
                ("right torque state", lambda: self.robot.set_state("B", 3)),
                (
                    "left joint impedance mode",
                    lambda: self.robot.set_impedance_type("A", 1),
                ),
                (
                    "right joint impedance mode",
                    lambda: self.robot.set_impedance_type("B", 1),
                ),
            ],
            wait_response=True,
            transaction_name="joint impedance mode switch",
        )

    def enable_pd_feedforward(self, pd_period_ms):
        if not 1 <= int(pd_period_ms) <= 20:
            raise ValueError("Marvin PD feedforward period must be in [1, 20] ms")
        self._send_transaction(
            [
                (
                    "left PD velocity-estimation period",
                    lambda: self.robot.set_PD_vel_est_step("A", int(pd_period_ms)),
                ),
                (
                    "right PD velocity-estimation period",
                    lambda: self.robot.set_PD_vel_est_step("B", int(pd_period_ms)),
                ),
            ],
            wait_response=True,
            transaction_name="PD velocity-estimation period",
        )

    def send_joint_command(self, q_rad, wait_response=False):
        q_rad = np.asarray(q_rad, dtype=float).reshape(-1)
        if q_rad.size != 14 or not np.all(np.isfinite(q_rad)):
            raise ValueError("q_rad must be a finite 14-joint vector")
        q_deg = np.rad2deg(q_rad)
        self._send_transaction(
            [
                ("left joint command", lambda: self.robot.set_joint_cmd_pose("A", q_deg[:7].tolist())),
                ("right joint command", lambda: self.robot.set_joint_cmd_pose("B", q_deg[7:].tolist())),
            ],
            wait_response=wait_response,
            transaction_name="joint position target",
        )

    def set_idle(self, wait_response=True):
        if not self.connected:
            return
        self._send_transaction(
            [
                ("left idle state", lambda: self.robot.set_state("A", 0)),
                ("right idle state", lambda: self.robot.set_state("B", 0)),
            ],
            wait_response=wait_response,
            transaction_name="idle mode switch",
        )

    def release(self):
        if self.released:
            return
        self.released = True
        self.robot.release_robot()
        self.connected = False
