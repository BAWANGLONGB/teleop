"""SI-unit boundary for the Marvin robot control SDK."""

import configparser
import importlib
import json
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np


def validate_vendor_driver_dependency(sdk_root_path):
    sdk_root_path = Path(sdk_root_path).expanduser().resolve()
    sdk_python_path = sdk_root_path / "SDK_PYTHON" / "fx_robot.py"
    sdk_library_path = sdk_root_path / "SDK_PYTHON" / "libMarvinSDK.so"
    robot_configuration_path = sdk_root_path / "robot.ini"
    for description, required_path in (
        ("Marvin control SDK", sdk_python_path),
        ("Marvin control library", sdk_library_path),
        ("Marvin robot configuration", robot_configuration_path),
    ):
        if not required_path.is_file():
            raise FileNotFoundError(f"{description} not found: {required_path}")

    configuration = configparser.ConfigParser(interpolation=None)
    configuration.optionxform = str
    with robot_configuration_path.open(encoding="utf-8") as configuration_file:
        configuration.read_file(configuration_file)
    emergency_stop_enabled = configuration.getint("R.BASIC", "UseEMG")
    joint_control_types = tuple(
        configuration.getint(f"R.A{arm_index}.BASIC", "JointPIDCtlType")
        for arm_index in range(2)
    )
    if emergency_stop_enabled != 1:
        raise PermissionError(
            "robot.ini must enable physical emergency-stop monitoring"
        )
    if joint_control_types != (1, 1):
        raise RuntimeError("robot.ini must use joint PD control for both arms")
    return sdk_root_path


def load_vendor_sdk(sdk_root_path):
    sdk_root_path = validate_vendor_driver_dependency(sdk_root_path)
    sdk_root = str(sdk_root_path)
    if sdk_root not in sys.path:
        sys.path.insert(0, sdk_root)
    sdk_module = importlib.import_module("SDK_PYTHON.fx_robot")
    return sdk_module.Marvin_Robot(), sdk_module.DCSS()


def _validated_joint_vector(values, field_name):
    joint_vector = np.asarray(values, dtype=float).reshape(-1).copy()
    if joint_vector.shape != (14,) or not np.all(np.isfinite(joint_vector)):
        raise ValueError(f"{field_name} must be a finite 14-joint vector")
    return joint_vector


def _convert_sdk_flag_to_boolean(value):
    if isinstance(value, (bytes, bytearray)):
        return any(byte != 0 for byte in value)
    return bool(value)


@dataclass(frozen=True)
class MarvinToolConfiguration:
    kinematics_mm_deg: tuple
    dynamics_vendor_units: tuple

    def __post_init__(self):
        kinematics = np.asarray(self.kinematics_mm_deg, dtype=float).reshape(-1)
        dynamics = np.asarray(self.dynamics_vendor_units, dtype=float).reshape(-1)
        if kinematics.shape != (6,) or not np.all(np.isfinite(kinematics)):
            raise ValueError("tool kinematics must contain six finite XYZABC values")
        if dynamics.shape != (10,) or not np.all(np.isfinite(dynamics)):
            raise ValueError("tool dynamics must contain ten finite values")
        object.__setattr__(self, "kinematics_mm_deg", tuple(kinematics))
        object.__setattr__(self, "dynamics_vendor_units", tuple(dynamics))


@dataclass(frozen=True)
class MarvinModbusGripperConfiguration:
    slave_id: int
    position_register: int
    open_position: int
    closed_position: int
    initial_closedness: float
    channel: int = 2

    def __post_init__(self):
        for field_name, lower_bound, upper_bound in (
            ("slave_id", 1, 247),
            ("position_register", 0, 0xFFFF),
            ("open_position", 0, 0xFFFF),
            ("closed_position", 0, 0xFFFF),
        ):
            value = getattr(self, field_name)
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, np.integer))
                or not lower_bound <= value <= upper_bound
            ):
                raise ValueError(
                    f"{field_name} must be an integer within "
                    f"[{lower_bound}, {upper_bound}]"
                )
            object.__setattr__(self, field_name, int(value))
        if self.open_position == self.closed_position:
            raise ValueError("gripper open and closed positions must differ")
        initial_closedness = float(self.initial_closedness)
        if (
            not np.isfinite(initial_closedness)
            or not 0.0 <= initial_closedness <= 1.0
        ):
            raise ValueError("initial_closedness must be within [0, 1]")
        object.__setattr__(self, "initial_closedness", initial_closedness)
        if isinstance(self.channel, bool) or self.channel not in (2, 3):
            raise ValueError("gripper channel must be COM1 (2) or COM2 (3)")
        object.__setattr__(self, "channel", int(self.channel))


def _modbus_write_single_register_frame(slave_id, register, value):
    frame = bytes((slave_id, 0x06)) + int(register).to_bytes(
        2, "big"
    ) + int(value).to_bytes(2, "big")
    crc = 0xFFFF
    for byte in frame:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return frame + crc.to_bytes(2, "little")


@dataclass(frozen=True)
class MarvinRobotState:
    frame_serial: tuple[int, int]
    q_rad: np.ndarray
    dq_rad_s: np.ndarray
    arm_state: tuple[int, int]
    error_code: tuple[int, int]
    low_speed: tuple[bool, bool]

    def __post_init__(self):
        for field_name in (
            "frame_serial",
            "arm_state",
            "error_code",
            "low_speed",
        ):
            if len(getattr(self, field_name)) != 2:
                raise ValueError(f"{field_name} must contain left and right values")
        object.__setattr__(
            self,
            "q_rad",
            _validated_joint_vector(self.q_rad, "q_rad"),
        )
        object.__setattr__(
            self,
            "dq_rad_s",
            _validated_joint_vector(self.dq_rad_s, "dq_rad_s"),
        )


class MarvinSdkAdapter:
    """Connect, read feedback, and send dual-arm joint targets."""

    def __init__(
        self,
        robot_ip_address="192.168.1.190",
        sdk_root_path=None,
        marvin_robot=None,
        dcss_structure=None,
        gripper_configurations=None,
    ):
        if marvin_robot is None or dcss_structure is None:
            if sdk_root_path is None:
                raise ValueError(
                    "sdk_root_path is required unless SDK objects are injected"
                )
            marvin_robot, dcss_structure = load_vendor_sdk(sdk_root_path)
        self.robot_ip_address = robot_ip_address
        self._marvin_robot = marvin_robot
        self._dcss_structure = dcss_structure
        self._is_connected = False
        self._is_released = False
        if gripper_configurations is not None:
            if (
                len(gripper_configurations) != 2
                or not all(
                    isinstance(config, MarvinModbusGripperConfiguration)
                    for config in gripper_configurations
                )
            ):
                raise TypeError(
                    "gripper_configurations must contain two "
                    "MarvinModbusGripperConfiguration values"
                )
            gripper_configurations = tuple(gripper_configurations)
        self.gripper_configurations = gripper_configurations

    def connect(self):
        if self._is_connected:
            return
        if self._is_released:
            raise RuntimeError("a released Marvin control SDK cannot reconnect")
        compatibility_check = getattr(
            self._marvin_robot, "check_sdk_type_compat", None
        )
        if compatibility_check is not None:
            compatibility_result = compatibility_check()
            status_code = (
                compatibility_result[0]
                if isinstance(compatibility_result, tuple)
                else compatibility_result
            )
            if status_code is not None and status_code < 0:
                raise RuntimeError("Marvin SDK ABI compatibility check failed")
        if not self._marvin_robot.connect(self.robot_ip_address):
            raise ConnectionError(
                f"failed to connect to Marvin at {self.robot_ip_address}"
            )
        self._is_connected = True
        if self.gripper_configurations is not None:
            for arm_name in ("A", "B"):
                if not self._marvin_robot.clear_ch_data(arm_name):
                    raise RuntimeError(
                        f"failed to clear arm {arm_name} gripper channel"
                    )

    def sdk_version(self):
        version_getter = getattr(self._marvin_robot, "SDK_version", None)
        return None if version_getter is None else version_getter()

    @staticmethod
    def _read_arm_values(raw_feedback, group_name, field_name):
        feedback_group = raw_feedback.get(group_name)
        if not isinstance(feedback_group, list) or len(feedback_group) < 2:
            raise ValueError(f"feedback group {group_name!r} must contain both arms")
        return tuple(feedback_group[index][field_name] for index in range(2))

    @classmethod
    def _read_joint_values(cls, raw_feedback, group_name, field_name):
        arm_values = cls._read_arm_values(raw_feedback, group_name, field_name)
        joint_values = [
            np.asarray(values, dtype=float).reshape(-1) for values in arm_values
        ]
        if any(
            values.shape != (7,) or not np.all(np.isfinite(values))
            for values in joint_values
        ):
            raise ValueError(f"{group_name}.{field_name} must contain two 7-vectors")
        return np.concatenate(joint_values)

    def read_state(self):
        if not self._is_connected:
            raise RuntimeError("Marvin control SDK is not connected")
        raw_feedback = self._marvin_robot.subscribe(self._dcss_structure)
        if not isinstance(raw_feedback, dict):
            raise RuntimeError("Marvin subscribe() returned invalid feedback")
        q_deg = self._read_joint_values(
            raw_feedback, "outputs", "fb_joint_pos"
        )
        dq_deg_s = self._read_joint_values(
            raw_feedback, "outputs", "fb_joint_vel"
        )
        return MarvinRobotState(
            frame_serial=tuple(
                int(value)
                for value in self._read_arm_values(
                    raw_feedback, "outputs", "frame_serial"
                )
            ),
            q_rad=np.deg2rad(q_deg),
            dq_rad_s=np.deg2rad(dq_deg_s),
            arm_state=tuple(
                int(value)
                for value in self._read_arm_values(
                    raw_feedback, "states", "cur_state"
                )
            ),
            error_code=tuple(
                int(value)
                for value in self._read_arm_values(
                    raw_feedback, "states", "err_code"
                )
            ),
            low_speed=tuple(
                _convert_sdk_flag_to_boolean(value)
                for value in self._read_arm_values(
                    raw_feedback, "outputs", "low_speed_flag"
                )
            ),
        )

    def wait_for_fresh_feedback(self, timeout_seconds=2.0, required_updates=3):
        deadline = time.monotonic() + timeout_seconds
        previous_serials = [None, None]
        update_counts = [0, 0]
        last_error = None
        while time.monotonic() < deadline:
            try:
                robot_feedback = self.read_state()
            except Exception as error:
                last_error = error
                time.sleep(0.01)
                continue
            for arm_index, frame_serial in enumerate(robot_feedback.frame_serial):
                if frame_serial != 0 and frame_serial != previous_serials[arm_index]:
                    update_counts[arm_index] += 1
                    previous_serials[arm_index] = frame_serial
            if min(update_counts) >= required_updates:
                return robot_feedback
            time.sleep(0.01)
        raise TimeoutError(
            "Marvin feedback did not advance on both arms within "
            f"{timeout_seconds:g} seconds"
        ) from last_error

    def _wait_for_command_buffer(self):
        deadline = time.monotonic() + 0.02
        while not self._marvin_robot.clear_set():
            if time.monotonic() >= deadline:
                raise TimeoutError("Marvin command buffer remained busy for 20 ms")
            time.sleep(0.001)

    def _send_transaction(
        self, setters, wait_for_response=False, transaction_name="command"
    ):
        if not self._is_connected:
            raise RuntimeError("Marvin control SDK is not connected")
        self._wait_for_command_buffer()
        for setter_description, setter in setters:
            if not setter():
                raise RuntimeError(f"Marvin setter failed: {setter_description}")
        if wait_for_response:
            response = self._marvin_robot.send_cmd_wait_response(100)
            if response is None or response == 0:
                raise TimeoutError(f"Marvin {transaction_name} response timed out")
            if response < 0:
                raise RuntimeError(f"Marvin {transaction_name} returned an error")
        elif not self._marvin_robot.send_cmd():
            raise RuntimeError(f"Marvin {transaction_name} send failed")

    def send_joint_command(self, q_rad, wait_response=False):
        q_rad = _validated_joint_vector(q_rad, "q_rad")
        q_deg = np.rad2deg(q_rad)
        self._send_transaction(
            (
                (
                    "left joint target",
                    lambda: self._marvin_robot.set_joint_cmd_pose(
                        arm="A", joints=q_deg[:7].tolist()
                    ),
                ),
                (
                    "right joint target",
                    lambda: self._marvin_robot.set_joint_cmd_pose(
                        arm="B", joints=q_deg[7:].tolist()
                    ),
                ),
            ),
            wait_response,
            "joint position command",
        )

    def send_gripper_command(self, closedness):
        if not self._is_connected:
            raise RuntimeError("Marvin control SDK is not connected")
        if self.gripper_configurations is None:
            raise RuntimeError("Marvin gripper protocol is not configured")
        closedness = np.asarray(closedness, dtype=float).reshape(-1)
        if (
            closedness.shape != (2,)
            or not np.all(np.isfinite(closedness))
            or np.any(closedness < 0.0)
            or np.any(closedness > 1.0)
        ):
            raise ValueError(
                "gripper closedness must contain two values within [0, 1]"
            )
        targets = []
        for arm_name, value, config in zip(
            ("A", "B"), closedness, self.gripper_configurations
        ):
            target = round(
                config.open_position
                + value * (config.closed_position - config.open_position)
            )
            frame = _modbus_write_single_register_frame(
                config.slave_id, config.position_register, target
            )
            sent_length = self._marvin_robot.set_ch_data(
                arm_name, frame, len(frame), config.channel
            )
            if sent_length != len(frame):
                raise RuntimeError(
                    f"arm {arm_name} gripper command sent "
                    f"{sent_length} of {len(frame)} bytes"
                )
            targets.append(target)
        # ponytail: validate the Modbus echo once the vendor's reply timing is known.
        return tuple(targets)

    @staticmethod
    def _validate_joint_impedance(values, field_name, upper_bound):
        values = np.asarray(values, dtype=float).reshape(-1)
        if (
            values.shape != (7,)
            or not np.all(np.isfinite(values))
            or np.any(values < 0.0)
            or np.any(values > upper_bound)
        ):
            raise ValueError(
                f"{field_name} must contain seven finite values within "
                f"[0, {upper_bound:g}]"
            )
        return values.tolist()

    def configure_control_parameters(
        self,
        left_k,
        left_d,
        right_k,
        right_d,
        tool_configurations=None,
        joint_velocity_ratio=10,
        joint_acceleration_ratio=10,
    ):
        left_k = self._validate_joint_impedance(
            left_k, "left_k", 22.0
        )
        right_k = self._validate_joint_impedance(
            right_k, "right_k", 22.0
        )
        left_d = self._validate_joint_impedance(
            left_d, "left_d", 1.0
        )
        right_d = self._validate_joint_impedance(
            right_d, "right_d", 1.0
        )
        joint_velocity_ratio = self._validate_percentage(
            joint_velocity_ratio, "joint_velocity_ratio"
        )
        joint_acceleration_ratio = self._validate_percentage(
            joint_acceleration_ratio, "joint_acceleration_ratio"
        )
        setters = [
            (
                "left joint impedance",
                lambda: self._marvin_robot.set_joint_kd_params(
                    arm="A", K=left_k, D=left_d
                ),
            ),
            (
                "right joint impedance",
                lambda: self._marvin_robot.set_joint_kd_params(
                    arm="B", K=right_k, D=right_d
                ),
            ),
            (
                "left joint velocity and acceleration ratios",
                lambda: self._marvin_robot.set_vel_acc(
                    arm="A",
                    velRatio=joint_velocity_ratio,
                    AccRatio=joint_acceleration_ratio,
                ),
            ),
            (
                "right joint velocity and acceleration ratios",
                lambda: self._marvin_robot.set_vel_acc(
                    arm="B",
                    velRatio=joint_velocity_ratio,
                    AccRatio=joint_acceleration_ratio,
                ),
            ),
        ]
        if tool_configurations is not None:
            if len(tool_configurations) != 2:
                raise ValueError("tool_configurations must contain both arms")
            for arm_name, tool_configuration in zip(
                ("A", "B"), tool_configurations
            ):
                if not isinstance(tool_configuration, MarvinToolConfiguration):
                    raise TypeError(
                        "tool_configurations must contain MarvinToolConfiguration"
                    )
                setters.append(
                    (
                        f"arm {arm_name} tool",
                        lambda arm_name=arm_name, tool=tool_configuration: (
                            self._marvin_robot.set_tool(
                                arm_name,
                                list(tool.kinematics_mm_deg),
                                list(tool.dynamics_vendor_units),
                            )
                        ),
                    )
                )
        self._send_transaction(
            setters,
            wait_for_response=True,
            transaction_name="control parameter configuration",
        )

    @staticmethod
    def _validate_percentage(value, field_name):
        numeric_value = float(value)
        if (
            isinstance(value, (bool, np.bool_))
            or not np.isfinite(numeric_value)
            or not numeric_value.is_integer()
            or not 0.0 <= numeric_value <= 100.0
        ):
            raise ValueError(f"{field_name} must be an integer within [0, 100]")
        return int(numeric_value)

    def enter_joint_impedance(self):
        self._send_transaction(
            (
                (
                    "left arm state",
                    lambda: self._marvin_robot.set_state(arm="A", state=3),
                ),
                (
                    "right arm state",
                    lambda: self._marvin_robot.set_state(arm="B", state=3),
                ),
                (
                    "left impedance type",
                    lambda: self._marvin_robot.set_impedance_type(arm="A", type=1),
                ),
                (
                    "right impedance type",
                    lambda: self._marvin_robot.set_impedance_type(arm="B", type=1),
                ),
            ),
            True,
            "joint impedance mode switch",
        )

    def enable_pd_feedforward(self, period_milliseconds):
        period_milliseconds = int(period_milliseconds)
        if not 1 <= period_milliseconds <= 20:
            raise ValueError("PD feedforward period must be within [1, 20] ms")
        self._send_transaction(
            tuple(
                (
                    f"arm {sdk_arm_name} PD period",
                    lambda sdk_arm_name=sdk_arm_name: (
                        self._marvin_robot.set_PD_vel_est_step(
                            arm=sdk_arm_name, step=period_milliseconds
                        )
                    ),
                )
                for sdk_arm_name in ("A", "B")
            ),
            True,
            "PD feedforward configuration",
        )

    def set_idle(self):
        if not self._is_connected:
            return False
        self._send_transaction(
            tuple(
                (
                    f"arm {sdk_arm_name} idle state",
                    lambda sdk_arm_name=sdk_arm_name: self._marvin_robot.set_state(
                        arm=sdk_arm_name, state=0
                    ),
                )
                for sdk_arm_name in ("A", "B")
            ),
            True,
            "idle mode switch",
        )
        return True

    def release(self):
        if not self._is_released:
            self._is_released = True
            if self._is_connected:
                self._marvin_robot.release_robot()
            self._is_connected = False


def load_active_tool_configs(tools_configuration_path):
    with Path(tools_configuration_path).expanduser().open(
        encoding="utf-8"
    ) as tools_configuration_file:
        tools_configuration = json.load(tools_configuration_file)
    tool_configurations = []
    for arm_index in range(2):
        arm_key = f"arm{arm_index}"
        selected_tool_name = tools_configuration["current_tool"][arm_key]
        selected_tool = tools_configuration[arm_key][selected_tool_name]
        tool_configurations.append(
            MarvinToolConfiguration(
                selected_tool["kine"],
                selected_tool["dyn"],
            )
        )
    return tuple(tool_configurations)


def load_modbus_gripper_configurations(configuration_path):
    with Path(configuration_path).expanduser().open(
        encoding="utf-8"
    ) as configuration_file:
        configuration = json.load(configuration_file)
    try:
        return tuple(
            MarvinModbusGripperConfiguration(**configuration[arm_name])
            for arm_name in ("left", "right")
        )
    except (KeyError, TypeError) as error:
        raise ValueError(
            "gripper config must contain left/right Modbus settings"
        ) from error
