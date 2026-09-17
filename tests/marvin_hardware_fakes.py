"""Shared fakes for Marvin hardware, ROS, and controller tests."""

import json
import queue
import runpy
import struct
import threading
import tempfile
import time
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from xr_marvin_teleop.collection import episode_validator
from xr_marvin_teleop.control.calibration import (
    ArmLengthScaleCalibrator,
    resolve_scale_factor,
    save_scale_calibration,
)
from xr_marvin_teleop.control.postures import MARVIN_INITIAL_POSE_Q_RAD
from xr_marvin_teleop.collection.session_logger import (
    MarvinSessionLogger,
    read_marvin_session,
)
from xr_marvin_teleop.adapters.xr import XrClient, XrSnapshot
from xr_marvin_teleop.control.mapping import (
    XrTargetMapper,
    transform_controller_poses_to_marvin_frame,
)
from xr_marvin_teleop.adapters.marvin import (
    MarvinModbusGripperConfiguration,
    MarvinRobotState,
    MarvinSdkAdapter,
    MarvinToolConfiguration,
    _modbus_write_single_register_frame,
)
from xr_marvin_teleop.adapters.das_finger import (
    DASFingerAdapter,
    DASFingerConfiguration,
    _decode_encoder_value,
    closedness_to_das_distances,
    load_das_finger_configurations,
)
from xr_marvin_teleop.adapters.marvin_kinematics import (
    MarvinVendorKinematics,
    VendorIkResult,
)
from xr_marvin_teleop.control.controller import (
    MarvinHardwareTeleopController,
)
from xr_marvin_teleop.ros.das_client import RosDasClient
from xr_marvin_teleop.ros.pico_client import RosPicoClient
from xr_marvin_teleop.ros.telemetry_bridge import Ros2DataBridge

def make_openxr_pose(x_meters=0.0, y_meters=0.0, z_meters=0.0):
    return np.array(
        [x_meters, y_meters, z_meters, 0.0, 0.0, 0.0, 1.0]
    )


class FakeXrSdk:
    def __init__(self, timestamps):
        self.timestamps = list(timestamps)
        self.initialized = False
        self.closed = False

    def init(self):
        self.initialized = True

    def get_snapshot(self):
        if len(self.timestamps) > 1:
            timestamp_ns = self.timestamps.pop(0)
        else:
            timestamp_ns = self.timestamps[0]
        return {
            "timestamp_ns": timestamp_ns,
            "left_controller_pose": make_openxr_pose(-0.1),
            "right_controller_pose": make_openxr_pose(0.1),
            "grip_values": (0.0, 0.0),
            "trigger_values": (0.0, 0.0),
            "thumbstick_y_values": (0.0, 0.0),
            "button_a": False,
            "button_b": False,
        }

    def close(self):
        self.closed = True


class FakeMarvinRobot:
    def __init__(self):
        self.frame_serial = 0
        self.invalid_feedback_reads = 0
        self.q_commands_deg = {}
        self.joint_impedance = {}
        self.joint_motion_limits = {}
        self.tools = {}
        self.wait_response_calls = 0
        self.channel_frames = []
        self.cleared_channels = []
        self.released = False
        self.connect_calls = 0

    def connect(self, _robot_ip_address):
        self.connect_calls += 1
        return True

    def subscribe(self, _dcss_structure):
        if self.invalid_feedback_reads > 0:
            self.invalid_feedback_reads -= 1
            return None
        self.frame_serial += 1
        return {
            "outputs": [
                {
                    "fb_joint_pos": [0.0] * 7,
                    "fb_joint_vel": [0.0] * 7,
                    "frame_serial": self.frame_serial,
                    "low_speed_flag": b"\x01",
                },
                {
                    "fb_joint_pos": [0.0] * 7,
                    "fb_joint_vel": [0.0] * 7,
                    "frame_serial": self.frame_serial,
                    "low_speed_flag": b"\x01",
                },
            ],
            "states": [
                {"cur_state": 3, "err_code": 0},
                {"cur_state": 3, "err_code": 0},
            ],
        }

    def clear_set(self):
        return True

    def set_joint_cmd_pose(self, arm, joints):
        self.q_commands_deg[arm] = joints
        return True

    def set_joint_kd_params(self, arm, K, D):
        self.joint_impedance[arm] = (K, D)
        return True

    def set_vel_acc(self, arm, velRatio, AccRatio):
        self.joint_motion_limits[arm] = (velRatio, AccRatio)
        return True

    def set_tool(self, arm, kineParams, dynamicParams):
        self.tools[arm] = (kineParams, dynamicParams)
        return True

    def send_cmd(self):
        return True

    def send_cmd_wait_response(self, _timeout_milliseconds):
        self.wait_response_calls += 1
        return 1

    def clear_ch_data(self, arm):
        self.cleared_channels.append(arm)
        return True

    def set_ch_data(self, arm, data, size_int, channel):
        self.channel_frames.append((arm, bytes(data), channel))
        return size_int

    def set_state(self, arm, state):
        return True

    def release_robot(self):
        self.released = True


class FakeXRClient:
    def __init__(self, snapshots):
        self._snapshots = iter(snapshots)
        self.closed = False

    def read_snapshot(self):
        return next(self._snapshots)

    def close(self):
        self.closed = True


class RecordingTelemetry:
    def __init__(self):
        self.events = []
        self.closed = False

    def publish_pico(self, snapshot, **_timestamps):
        self.events.append(("pico", snapshot.timestamp_ns))

    def publish_marvin_state(self, state, **_timestamps):
        self.events.append(("marvin", state.frame_serial))

    def publish_joint_command(self, q_rad, **_timestamps):
        self.events.append(("joint_command", tuple(q_rad)))

    def publish_gripper_command(self, closedness, **_timestamps):
        self.events.append(("gripper_command", tuple(closedness)))

    def close(self):
        self.closed = True


class FakeMarvinSdkAdapter:
    def __init__(self):
        self.q_rad = np.zeros(14)
        self.arm_state = (0, 0)
        self.frame_serial = 0
        self.sent_commands_rad = []
        self.events = []
        self.configured_parameters = None
        self.configured_named_parameters = None
        self.pd_period_milliseconds = None
        self.gripper_commands = []
        self.released = False
        self.idle = False

    def connect(self):
        pass

    def sdk_version(self):
        return 1

    def _feedback(self):
        self.frame_serial += 1
        return MarvinRobotState(
            frame_serial=(self.frame_serial, self.frame_serial),
            q_rad=self.q_rad,
            dq_rad_s=np.zeros(14),
            arm_state=self.arm_state,
            error_code=(0, 0),
            low_speed=(True, True),
        )

    def wait_for_fresh_feedback(self, **_kwargs):
        return self._feedback()

    def read_state(self):
        return self._feedback()

    def send_joint_command(self, q_rad, wait_response=False):
        del wait_response
        self.events.append("send_joint_command")
        self.q_rad = np.asarray(q_rad).copy()
        self.sent_commands_rad.append(self.q_rad.copy())

    def send_gripper_command(self, closedness):
        self.gripper_commands.append(tuple(closedness))

    def configure_control_parameters(self, *parameters, **named_parameters):
        self.events.append("configure_control_parameters")
        self.configured_parameters = parameters
        self.configured_named_parameters = named_parameters

    def enter_joint_impedance(self):
        self.events.append("enter_joint_impedance")
        self.arm_state = (3, 3)

    def enable_pd_feedforward(self, _period_milliseconds):
        self.events.append("enable_pd_feedforward")
        self.pd_period_milliseconds = _period_milliseconds

    def set_idle(self):
        self.idle = True
        self.arm_state = (0, 0)
        return True

    def release(self):
        self.released = True


class FakeDasFingerSystem:
    def __init__(self, encoder_callback, encoder_distance):
        self.databus = None
        self._encoder_callback = encoder_callback
        self._encoder_distance = encoder_distance
        self._stopped = threading.Event()
        self.targets = []

    def start(self):
        self.databus = self
        self._encoder_callback(struct.pack(">f", self._encoder_distance))
        self._stopped.wait()

    def set_finger_distance(self, distance):
        self.targets.append(float(distance))

    def stop(self):
        self._stopped.set()


class FakeMarvinVendorKinematics:
    def __init__(self):
        self.fail_inverse_kinematics = False
        self.nsp_reference_calls = []
        self.nsp_angles_deg = []

    def set_nsp_reference(self, arm, q_rad):
        self.nsp_reference_calls.append((arm, np.asarray(q_rad).copy()))

    def fk_world(self, _arm, q_rad):
        tcp_transform = np.eye(4)
        tcp_transform[0, 3] = q_rad[0]
        return tcp_transform

    def ik_world(
        self,
        _arm,
        T_world_tcp_m,
        q_ref_rad,
        nsp_angle_deg=None,
    ):
        self.nsp_angles_deg.append(nsp_angle_deg)
        if self.fail_inverse_kinematics:
            return VendorIkResult(False, None, "singular or out of range")
        q_rad = np.asarray(q_ref_rad).copy()
        q_rad[0] = T_world_tcp_m[0, 3]
        return VendorIkResult(True, q_rad, None)
