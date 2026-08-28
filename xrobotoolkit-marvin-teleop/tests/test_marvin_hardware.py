import csv
import json
import time
from pathlib import Path

import numpy as np
import pytest

import xrobotoolkit_teleop.common.base_teleop_controller as base_controller_module
from xrobotoolkit_teleop.common.joint_command_limiter import FeedbackAwareJointLimiter
from xrobotoolkit_teleop.common.cartesian_target_guard import CartesianTargetGuard
from xrobotoolkit_teleop.common.marvin_calibration_recorder import (
    MarvinCalibrationRecorder,
)
from xrobotoolkit_teleop.common.marvin_motion_limits import (
    HUMAN_PEAK_TCP_ANGULAR_SPEED_RAD_S,
    HUMAN_PEAK_TCP_LINEAR_SPEED_M_S,
    MARVIN_PEAK_JOINT_ACCELERATION_RAD_S2,
    MARVIN_PEAK_JOINT_JERK_RAD_S3,
    MARVIN_PEAK_JOINT_VELOCITY_RAD_S,
    MARVIN_PEAK_TARGET_NATURAL_FREQUENCY_RAD_S,
)
from xrobotoolkit_teleop.common.marvin_observation import MarvinControlObservation
from xrobotoolkit_teleop.common.marvin_postures import MARVIN_HUMAN_REST_Q_RAD
from xrobotoolkit_teleop.common.marvin_safety import (
    MarvinControlState,
    MarvinSafetyConfig,
    MarvinSafetySupervisor,
)
from xrobotoolkit_teleop.common.marvin_session_logger import MarvinSessionLogger
from xrobotoolkit_teleop.common.marvin_scale_calibration import (
    DEFAULT_SCALE_FACTOR,
    controller_positions_in_marvin_head_yaw_frame,
    load_scale_calibration,
    resolve_scale_factor,
    save_scale_calibration,
)
from xrobotoolkit_teleop.common.marvin_types import MarvinJointCommand, MarvinRobotState
from xrobotoolkit_teleop.hardware.interface.marvin import (
    MarvinSdkAdapter,
    MarvinToolConfig,
    load_active_tool_configs,
    validate_vendor_driver_dependency,
)
from xrobotoolkit_teleop.hardware.marvin_teleop_controller import (
    MarvinHardwareTeleopController,
)
from scripts.misc.prepare_marvin_mujoco_calibration import convert
from scripts.hardware.inspect_marvin_state import _require_sdk_version
from xrobotoolkit_teleop.common.arm_length_calibration import ArmLengthScaleCalibrator


REPO_ROOT = Path(__file__).resolve().parents[1]
MARVIN_URDF = REPO_ROOT / "assets" / "marvin" / "marvin_dual.urdf"
MARVIN_JOINT_NAMES = [
    *(f"Joint{index}_L" for index in range(1, 8)),
    *(f"Joint{index}_R" for index in range(1, 8)),
]


def test_marvin_human_peak_motion_profile_is_robot_bounded():
    assert HUMAN_PEAK_TCP_LINEAR_SPEED_M_S == pytest.approx(0.62)
    assert HUMAN_PEAK_TCP_ANGULAR_SPEED_RAD_S == pytest.approx(np.deg2rad(122.0))
    assert MARVIN_PEAK_JOINT_VELOCITY_RAD_S == tuple(
        [1.0, 1.0, 1.0, 1.2, 1.2, 0.35, 1.0] * 2
    )
    assert MARVIN_PEAK_JOINT_ACCELERATION_RAD_S2 == tuple(
        [3.0, 3.0, 3.0, 4.0, 4.0, 3.0, 3.0] * 2
    )
    assert MARVIN_PEAK_JOINT_JERK_RAD_S3 == tuple(
        [20.0, 20.0, 20.0, 25.0, 25.0, 20.0, 20.0] * 2
    )
    assert MARVIN_PEAK_TARGET_NATURAL_FREQUENCY_RAD_S == pytest.approx(15.0)

    joint6_limiter = FeedbackAwareJointLimiter(
        lower_limits=[-np.deg2rad(60.0)],
        upper_limits=[np.deg2rad(60.0)],
        max_velocity=[MARVIN_PEAK_JOINT_VELOCITY_RAD_S[5]],
        max_acceleration=[MARVIN_PEAK_JOINT_ACCELERATION_RAD_S2[5]],
        max_jerk=[MARVIN_PEAK_JOINT_JERK_RAD_S3[5]],
        target_natural_frequency=MARVIN_PEAK_TARGET_NATURAL_FREQUENCY_RAD_S,
        limit_margin=np.deg2rad(5.0),
        dt=0.005,
    )
    assert np.rad2deg(joint6_limiter.target_upper[0]) > 52.0


class FakeHardwareXrClient:
    def __init__(self):
        self.closed = False

    def get_pose_by_name(self, _name):
        return np.array([0.0, 1.5, 0.0, 0.0, 0.0, 0.0, 1.0])

    def get_key_value_by_name(self, _name):
        return 0.0

    def get_button_state_by_name(self, _name):
        return False

    def get_motion_tracker_data(self):
        return {}

    def get_timestamp_ns(self):
        return 1

    def close(self):
        self.closed = True


def make_state(now_ns=None, q=None, error=(0, 0)):
    return MarvinRobotState(
        receipt_monotonic_ns=time.monotonic_ns() if now_ns is None else now_ns,
        frame_serial=(10, 20),
        q_rad=np.zeros(14) if q is None else q,
        dq_rad_s=np.zeros(14),
        torque_nm=np.zeros(14),
        arm_state=(3, 3),
        command_state=(3, 3),
        error_code=error,
        low_speed=(True, True),
        commanded_q_rad=np.zeros(14),
    )


class FakeMarvinRobot:
    def __init__(self):
        self.frames = [0, 0]
        self.connected = False
        self.released = False
        self.pending = False
        self.clear_set_calls = 0
        self.send_cmd_calls = 0
        self.send_cmd_wait_response_calls = 0
        self.send_cmd_wait_response_results = []
        self.clear_set_failures_remaining = 0
        self.raw = {
            "states": [
                {"cur_state": 0, "cmd_state": 0, "err_code": 0},
                {"cur_state": 0, "cmd_state": 0, "err_code": 0},
            ],
            "outputs": [],
            "inputs": [],
        }
        for _ in range(2):
            self.raw["outputs"].append(
                {
                    "frame_serial": 0,
                    "fb_joint_pos": [0.0] * 7,
                    "fb_joint_vel": [0.0] * 7,
                    "fb_joint_sToq": [0.0] * 7,
                    "low_speed_flag": b"\x00",
                }
            )
            self.raw["inputs"].append(
                {
                    "in_frame_serial": 0,
                    "frame_miss_cnt": 0,
                    "sys_cyc_miss_cnt": 0,
                    "joint_cmd_pos": [0.0] * 7,
                    "joint_vel_ratio": 0,
                    "joint_acc_ratio": 0,
                    "joint_k": [0.0] * 7,
                    "joint_d": [0.0] * 7,
                    "imp_type": 0,
                    "tool_kine": [0.0] * 6,
                    "tool_dyn": [0.0] * 10,
                }
            )

    def check_sdk_type_compat(self):
        return 0, 0

    def connect(self, _ip):
        self.connected = True
        return True

    def subscribe(self, _dcss):
        for arm in range(2):
            self.frames[arm] += 1
            self.raw["outputs"][arm]["frame_serial"] = self.frames[arm]
        return self.raw

    def clear_set(self):
        self.clear_set_calls += 1
        if self.clear_set_failures_remaining > 0:
            self.clear_set_failures_remaining -= 1
            return False
        self.pending = True
        return True

    def send_cmd(self):
        self.send_cmd_calls += 1
        self.pending = False
        return True

    def send_cmd_wait_response(self, _timeout):
        self.send_cmd_wait_response_calls += 1
        self.pending = False
        if self.send_cmd_wait_response_results:
            return self.send_cmd_wait_response_results.pop(0)
        return 2

    def set_vel_acc(self, arm, velocity, acceleration):
        index = 0 if arm == "A" else 1
        self.raw["inputs"][index]["joint_vel_ratio"] = velocity
        self.raw["inputs"][index]["joint_acc_ratio"] = acceleration
        return True

    def set_joint_kd_params(self, arm, k, d):
        index = 0 if arm == "A" else 1
        self.raw["inputs"][index]["joint_k"] = list(k)
        self.raw["inputs"][index]["joint_d"] = list(d)
        return True

    def set_tool(self, arm, kine, dyn):
        index = 0 if arm == "A" else 1
        self.raw["inputs"][index]["tool_kine"] = list(kine)
        self.raw["inputs"][index]["tool_dyn"] = list(dyn)
        return True

    def set_state(self, arm, state):
        index = 0 if arm == "A" else 1
        self.raw["states"][index]["cur_state"] = state
        self.raw["states"][index]["cmd_state"] = state
        return True

    def set_impedance_type(self, arm, mode):
        index = 0 if arm == "A" else 1
        self.raw["inputs"][index]["imp_type"] = mode
        return True

    def set_PD_vel_est_step(self, arm, step):
        self.raw["inputs"][0 if arm == "A" else 1]["pd_step"] = step
        return True

    def set_joint_cmd_pose(self, arm, joints):
        index = 0 if arm == "A" else 1
        self.raw["inputs"][index]["joint_cmd_pos"] = list(joints)
        return True

    def SDK_version(self):
        return 1003

    def get_robot_name(self):
        return "fake-marvin"

    def release_robot(self):
        self.released = True
        return True


def make_hardware_controller(monkeypatch, **kwargs):
    monkeypatch.setattr(base_controller_module, "XrClient", FakeHardwareXrClient)
    kwargs.setdefault("parameter_settle_s", 0.0)
    kwargs.setdefault("mode_settle_s", 0.0)
    kwargs.setdefault("pd_settle_s", 0.0)
    robot = FakeMarvinRobot()
    adapter = MarvinSdkAdapter(robot=robot, dcss=object())
    controller = MarvinHardwareTeleopController(
        adapter=adapter,
        robot_urdf_path=MARVIN_URDF,
        manipulator_config={
            "left_hand": {
                "link_name": "TCP_Link_L",
                "pose_source": "left_controller",
                "control_trigger": "left_grip",
                "manipulability_weight": 0.0,
            },
            "right_hand": {
                "link_name": "TCP_Link_R",
                "pose_source": "right_controller",
                "control_trigger": "right_grip",
                "manipulability_weight": 0.0,
            },
        },
        joint_names=MARVIN_JOINT_NAMES,
        R_headset_world=np.eye(3),
        **kwargs,
    )
    return controller, robot


def test_read_only_inspector_rejects_sdk_version_mismatch():
    robot = FakeMarvinRobot()
    adapter = MarvinSdkAdapter(robot=robot, dcss=object())
    adapter.connect()
    assert _require_sdk_version(adapter, 1003) == 1003
    with pytest.raises(RuntimeError, match="SDK version mismatch"):
        _require_sdk_version(adapter, 9999)


def test_pico_scale_calibration_is_persisted_and_loaded_after_restart(tmp_path):
    calibrator = ArmLengthScaleCalibrator(robot_motion_range=0.8)
    calibrator.capture(
        {
            "left_hand": np.array([0.0, 0.2, -0.7]),
            "right_hand": np.array([0.0, -0.2, -0.7]),
        }
    )
    result = calibrator.capture(
        {
            "left_hand": np.array([-0.7, 0.2, 0.0]),
            "right_hand": np.array([-0.7, -0.2, 0.0]),
        }
    )
    assert result.status == "completed"

    current_path, history_path = save_scale_calibration(
        tmp_path / "marvin_scale_calibration.json",
        result,
        workspace_margin=0.95,
    )
    assert current_path.is_file()
    assert history_path.is_file()
    assert current_path != history_path
    loaded = load_scale_calibration(current_path)
    scale_factor, metadata = resolve_scale_factor(None, current_path)
    assert scale_factor == pytest.approx(result.scale_factor)
    assert loaded["scale_factor"] == pytest.approx(result.scale_factor)
    assert metadata["source"] == "pico_calibration"
    assert metadata["sha256"] == loaded["sha256"]


def test_pico_scale_calibration_uses_same_head_yaw_and_marvin_axes_as_teleop():
    identity_pose = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
    left_pose = np.array([1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0])
    right_pose = np.array([-1.0, -2.0, -3.0, 0.0, 0.0, 0.0, 1.0])
    positions, _ = controller_positions_in_marvin_head_yaw_frame(
        identity_pose,
        left_pose,
        right_pose,
    )
    np.testing.assert_allclose(positions["left_hand"], [3.0, 1.0, 2.0])
    np.testing.assert_allclose(positions["right_hand"], [-3.0, -1.0, -2.0])


def test_explicit_scale_overrides_saved_calibration_and_missing_file_uses_default(tmp_path):
    missing_path = tmp_path / "missing.json"
    scale_factor, metadata = resolve_scale_factor(None, missing_path)
    assert scale_factor == DEFAULT_SCALE_FACTOR
    assert metadata["source"] == "code_default"

    missing_path.write_text("not valid json", encoding="utf-8")
    scale_factor, metadata = resolve_scale_factor(0.4, missing_path)
    assert scale_factor == pytest.approx(0.4)
    assert metadata["source"] == "cli"


def test_invalid_saved_scale_calibration_fails_closed(tmp_path):
    calibration_path = tmp_path / "invalid.json"
    calibration_path.write_text(
        json.dumps({"schema_version": 1, "scale_factor": 2.0}),
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="saved scale_factor"):
        resolve_scale_factor(None, calibration_path)


def test_grip_release_returns_only_that_arm_to_measured_startup_pose(monkeypatch):
    controller, robot = make_hardware_controller(
        monkeypatch,
        enable_release_return=True,
        return_duration=2.0,
    )
    moved_q = np.zeros(14)
    moved_q[0] = 0.1
    state = make_state(q=moved_q)
    controller._robot_state_slot.set(state)
    placo_q = controller.placo_robot.state.q.copy()
    placo_q[controller._joint_offsets] = moved_q
    controller.placo_robot.state.q = placo_q
    controller.placo_robot.update_kinematics()
    controller._previous_active_arms = (True, False)
    controller.active = {"left_hand": False, "right_hand": False}

    controller._send_command()
    released_command = controller._command_slot.get()
    assert released_command.returning_arms == (True, False)
    assert "automatic reset" in controller._scale_calibration_sample_rejection_reason()
    np.testing.assert_allclose(released_command.q_rad[:7], moved_q[:7], atol=1e-12)
    np.testing.assert_allclose(released_command.q_rad[7:], 0.0, atol=1e-12)

    controller.safety.arm()
    decision = controller.safety.evaluate(
        state,
        released_command,
        xr_source_age_ms=0.0,
    )
    assert decision.state == MarvinControlState.RETURNING

    controller._return_trajectories[0]["start_monotonic"] -= 1.0
    controller._send_command()
    halfway_command = controller._command_slot.get()
    assert 0.0 < halfway_command.q_rad[0] < moved_q[0]
    np.testing.assert_allclose(halfway_command.q_rad[7:], 0.0, atol=1e-12)

    controller.active["left_hand"] = True
    controller._send_command()
    assert controller._command_slot.get().returning_arms == (False, False)
    assert controller._return_trajectories[0] is None
    controller.xr_client.close()
    controller.adapter.release()
    assert robot.released


def test_feedback_joint_state_is_written_to_placo_and_tcp_query_restores_solver_state(
    monkeypatch,
):
    controller, robot = make_hardware_controller(monkeypatch)

    feedback_q = np.zeros(14)
    feedback_q[0] = 0.2
    controller._robot_state_slot.set(make_state(q=feedback_q))
    controller._update_robot_state()
    np.testing.assert_allclose(
        controller.placo_robot.state.q[controller._joint_offsets],
        feedback_q,
    )

    expected_feedback_tcp = {
        name: controller.placo_robot.get_T_world_frame(config["link_name"]).copy()
        for name, config in controller.manipulator_config.items()
    }

    solver_q = controller.placo_robot.state.q.copy()
    solver_q[controller._joint_offsets[0]] = -0.15
    controller.placo_robot.state.q = solver_q
    controller.placo_robot.update_kinematics()

    actual_tcp = controller._actual_tcp_transforms(make_state(q=feedback_q))

    np.testing.assert_allclose(controller.placo_robot.state.q, solver_q)
    for name in controller.manipulator_config:
        np.testing.assert_allclose(actual_tcp[name], expected_feedback_tcp[name])

    controller.xr_client.close()
    controller.adapter.release()
    assert robot.released


def test_hardware_ab_scale_calibration_saves_and_applies_on_next_grip(monkeypatch, tmp_path):
    calibration_path = tmp_path / "marvin_scale_calibration.json"
    controller, robot = make_hardware_controller(
        monkeypatch,
        enable_arm_length_calibration=True,
        scale_calibration_path=calibration_path,
    )
    assert set(controller.xr_client._button_names) == {"A", "B"}
    buffered_client = controller.xr_client
    original_scale = controller.scale_factor

    class ScriptedCalibrationXr:
        def __init__(self):
            self.a_pressed = True
            self.forward = False

        def get_button_state_by_name(self, name):
            return self.a_pressed if name == "A" else False

        def get_key_value_by_name(self, _name):
            return 0.0

        def get_pose_by_name(self, name):
            if name == "headset":
                return np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
            side = 0.2 if name == "left_controller" else -0.2
            xyz = [-0.7, side, 0.0] if self.forward else [0.0, side, -0.7]
            return np.array([*xyz, 0.0, 0.0, 0.0, 1.0])

    scripted_xr = ScriptedCalibrationXr()
    controller.xr_client = scripted_xr
    controller._robot_state_slot.set(make_state())
    head_yaw_reference = (np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0]))
    controller._update_scale_calibration(head_yaw_reference)
    scripted_xr.a_pressed = False
    controller._update_scale_calibration(head_yaw_reference)
    scripted_xr.forward = True
    scripted_xr.a_pressed = True
    controller._update_scale_calibration(head_yaw_reference)

    assert calibration_path.is_file()
    assert not controller._stop_event.is_set()
    assert controller.scale_factor != original_scale
    assert load_scale_calibration(calibration_path)["scale_factor"] == pytest.approx(
        controller.scale_factor
    )
    assert all(value is None for value in controller.ref_ee_xyz.values())
    assert all(value is None for value in controller.ref_controller_xyz.values())

    # The first pose after Grip establishes a new zero; subsequent motion uses
    # the just-calibrated scale without restarting the process.
    scripted_xr.forward = False
    down_pose = scripted_xr.get_pose_by_name("left_controller")
    zero_delta, _ = controller._process_xr_pose(
        down_pose, "left_hand", head_yaw_reference
    )
    moved_pose = down_pose.copy()
    moved_pose[0] += 0.1
    scaled_delta, _ = controller._process_xr_pose(
        moved_pose, "left_hand", head_yaw_reference
    )
    np.testing.assert_allclose(zero_delta, 0.0, atol=1e-12)
    assert np.linalg.norm(scaled_delta) == pytest.approx(0.1 * controller.scale_factor)
    buffered_client.close()
    controller.adapter.release()
    assert robot.released


def test_sdk_adapter_converts_units_and_uses_dual_arm_transactions():
    robot = FakeMarvinRobot()
    robot.raw["outputs"][0]["fb_joint_pos"] = [180.0] + [0.0] * 6
    robot.raw["outputs"][1]["fb_joint_vel"] = [90.0] + [0.0] * 6
    adapter = MarvinSdkAdapter(robot=robot, dcss=object(), robot_ip="127.0.0.1")
    adapter.connect()
    state = adapter.wait_for_fresh_feedback(timeout=0.2, required_updates=2)
    assert state.q_rad[0] == pytest.approx(np.pi)
    assert state.dq_rad_s[7] == pytest.approx(np.pi / 2)
    assert state.low_speed == (False, False)

    tool = MarvinToolConfig((0.0,) * 6, (1.0,) + (0.0,) * 9)
    k = [5, 5, 5, 4, 3, 3, 2]
    d = [0.3] * 7
    adapter.configure_parameters(10, 10, k, d, k, d, tool, tool)
    adapter.send_joint_command(np.arange(14) * np.pi / 180.0)
    assert robot.raw["inputs"][0]["joint_cmd_pos"] == pytest.approx(range(7))
    assert robot.raw["inputs"][1]["joint_cmd_pos"] == pytest.approx(range(7, 14))
    adapter.enter_joint_impedance()
    adapter.enable_pd_feedforward(20)
    assert tuple(item["cur_state"] for item in robot.raw["states"]) == (3, 3)
    assert tuple(item["imp_type"] for item in robot.raw["inputs"]) == (1, 1)
    adapter.set_idle()
    adapter.release()
    adapter.release()
    assert robot.released


def test_vendor_dependency_blocks_emergency_stop_bypass(tmp_path):
    sdk_directory = tmp_path / "SDK_PYTHON"
    sdk_directory.mkdir()
    (sdk_directory / "fx_robot.py").write_text("", encoding="utf-8")
    (sdk_directory / "libMarvinSDK.so").write_bytes(b"test")
    (tmp_path / "robot.ini").write_text(
        "[R.A0.BASIC]\nJointPIDCtlType=1\n"
        "[R.A1.BASIC]\nJointPIDCtlType=1\n"
        "[R.BASIC]\nUseEMG=0\n",
        encoding="utf-8",
    )

    with pytest.raises(PermissionError, match="UseEMG=0"):
        validate_vendor_driver_dependency(tmp_path)


def test_vendor_dependency_accepts_required_pd_and_estop_settings(tmp_path):
    sdk_directory = tmp_path / "SDK_PYTHON"
    sdk_directory.mkdir()
    (sdk_directory / "fx_robot.py").write_text("", encoding="utf-8")
    (sdk_directory / "libMarvinSDK.so").write_bytes(b"test")
    (tmp_path / "robot.ini").write_text(
        "[R.A0.BASIC]\nJointPIDCtlType=1\n"
        "[R.A1.BASIC]\nJointPIDCtlType=1\n"
        "[R.BASIC]\nUseEMG=1\n",
        encoding="utf-8",
    )

    result = validate_vendor_driver_dependency(tmp_path)

    assert result["use_emg"] == 1
    assert result["joint_pid_ctl_type"] == (1, 1)


def test_sdk_adapter_retries_transient_clear_set_busy():
    robot = FakeMarvinRobot()
    robot.clear_set_failures_remaining = 2
    adapter = MarvinSdkAdapter(robot=robot, dcss=object())
    adapter.connect()

    adapter.send_joint_command(np.zeros(14))

    assert robot.clear_set_calls == 3
    assert robot.send_cmd_calls == 1


def test_sdk_adapter_fails_closed_when_clear_set_stays_busy():
    robot = FakeMarvinRobot()
    robot.clear_set_failures_remaining = 2
    adapter = MarvinSdkAdapter(robot=robot, dcss=object())
    adapter._CLEAR_SET_RETRY_TIMEOUT_S = 0.0
    adapter.connect()

    with pytest.raises(TimeoutError, match="clear_set.*remained busy"):
        adapter.send_joint_command(np.zeros(14))

    assert robot.send_cmd_calls == 0


def test_hardware_startup_waits_for_hold_target_before_impedance_switch(monkeypatch):
    controller, robot = make_hardware_controller(monkeypatch)
    assert controller.control_hz == 200.0
    assert controller.command_hz == 200.0
    assert controller.pd_period_ms == 5
    assert controller.velocity_ratio == 100
    assert controller.acceleration_ratio == 100
    assert controller.left_d == pytest.approx([0.3] * 7)
    assert controller.right_d == pytest.approx([0.3] * 7)
    for output in robot.raw["outputs"]:
        output["low_speed_flag"] = b"\x01"

    controller._configure_hardware()

    # configure parameters, feedback-equal startup hold, impedance mode, and
    # PD period are four acknowledged transactions. No fire-and-forget send is
    # allowed before the mode switch.
    assert robot.send_cmd_wait_response_calls == 4
    assert robot.send_cmd_calls == 0
    assert controller.safety.state == MarvinControlState.ARMED
    controller.xr_client.close()
    controller.adapter.release()


def test_hardware_allows_bounded_impedance_transition_then_requires_stationary(
    monkeypatch,
):
    controller, robot = make_hardware_controller(monkeypatch)
    for output in robot.raw["outputs"]:
        output["low_speed_flag"] = b"\x01"

    original_enter_joint_impedance = controller.adapter.enter_joint_impedance
    original_subscribe = robot.subscribe
    transition_reads_remaining = 0

    def enter_joint_impedance():
        nonlocal transition_reads_remaining
        transition_reads_remaining = 1
        return original_enter_joint_impedance()

    def subscribe(dcss):
        nonlocal transition_reads_remaining
        if transition_reads_remaining:
            transition_reads_remaining -= 1
            robot.raw["outputs"][1]["low_speed_flag"] = b"\x00"
            robot.raw["outputs"][1]["fb_joint_vel"][0] = 0.7
        else:
            robot.raw["outputs"][1]["low_speed_flag"] = b"\x01"
            robot.raw["outputs"][1]["fb_joint_vel"][0] = 0.0
        return original_subscribe(dcss)

    controller.adapter.enter_joint_impedance = enter_joint_impedance
    robot.subscribe = subscribe

    controller._configure_hardware()

    assert controller.safety.state == MarvinControlState.ARMED
    controller.xr_client.close()
    controller.adapter.release()


def test_hardware_natural_rest_move_uses_detected_nonzero_start_pose(monkeypatch):
    controller, robot = make_hardware_controller(
        monkeypatch,
        initial_pose_q_rad=MARVIN_HUMAN_REST_Q_RAD,
        startup_move_duration_s=0.5,
        parameter_settle_s=0.0,
        mode_settle_s=0.0,
        pd_settle_s=0.0,
    )
    for output in robot.raw["outputs"]:
        output["low_speed_flag"] = b"\x01"
    detected_start = MARVIN_HUMAN_REST_Q_RAD + np.tile(
        [0.01, -0.01, 0.01, -0.01, 0.01, -0.01, 0.01], 2
    )
    robot.raw["outputs"][0]["fb_joint_pos"] = list(
        np.rad2deg(detected_start[:7])
    )
    robot.raw["outputs"][1]["fb_joint_pos"] = list(
        np.rad2deg(detected_start[7:])
    )

    original_set_joint_cmd_pose = robot.set_joint_cmd_pose

    def follow_joint_command(arm, joints):
        result = original_set_joint_cmd_pose(arm, joints)
        index = 0 if arm == "A" else 1
        robot.raw["outputs"][index]["fb_joint_pos"] = list(joints)
        robot.raw["outputs"][index]["fb_joint_vel"] = [0.0] * 7
        return result

    robot.set_joint_cmd_pose = follow_joint_command
    controller.xr_client.start()
    assert controller.xr_client.wait_until_ready(timeout=1.0)

    controller._configure_hardware()

    state = controller._robot_state_slot.get()
    np.testing.assert_allclose(
        state.q_rad, MARVIN_HUMAN_REST_Q_RAD, atol=np.deg2rad(0.05)
    )
    assert controller.safety.state == MarvinControlState.ARMED
    controller.xr_client.close()
    controller.adapter.release()


def test_hardware_rejects_detected_pose_that_cannot_reach_rest_in_three_seconds(
    monkeypatch,
):
    controller, _ = make_hardware_controller(
        monkeypatch,
        initial_pose_q_rad=MARVIN_HUMAN_REST_Q_RAD,
        startup_move_duration_s=3.0,
    )
    detected_start = MARVIN_HUMAN_REST_Q_RAD.copy()
    detected_start[0] = np.deg2rad(-90.0)
    controller.xr_client.start()
    assert controller.xr_client.wait_until_ready(timeout=1.0)

    with pytest.raises(RuntimeError, match="cannot reach natural rest safely within 3"):
        controller._move_to_configured_initial_pose(make_state(q=detected_start))

    controller.xr_client.close()
    controller.adapter.release()


def test_hardware_rejects_impedance_transition_above_startup_speed_bound(
    monkeypatch,
):
    controller, robot = make_hardware_controller(monkeypatch)
    for output in robot.raw["outputs"]:
        output["low_speed_flag"] = b"\x01"

    original_enter_joint_impedance = controller.adapter.enter_joint_impedance
    original_subscribe = robot.subscribe
    impedance_entered = False

    def enter_joint_impedance():
        nonlocal impedance_entered
        impedance_entered = True
        return original_enter_joint_impedance()

    def subscribe(dcss):
        if impedance_entered:
            robot.raw["outputs"][1]["low_speed_flag"] = b"\x00"
            robot.raw["outputs"][1]["fb_joint_vel"][0] = 2.0
        return original_subscribe(dcss)

    controller.adapter.enter_joint_impedance = enter_joint_impedance
    robot.subscribe = subscribe

    with pytest.raises(RuntimeError, match="exceeds the startup motion bound"):
        controller._configure_hardware()

    assert controller.safety.state == MarvinControlState.READ_ONLY
    controller.xr_client.close()
    controller.adapter.release()


def test_hardware_moves_to_configured_initial_pose_before_arming(monkeypatch):
    target = np.full(14, 0.02)
    controller, robot = make_hardware_controller(
        monkeypatch,
        initial_pose_q_rad=target,
        startup_move_duration_s=0.5,
        startup_pose_tolerance_deg=0.05,
        max_joint_velocity=1.0,
        max_joint_acceleration=10.0,
        max_joint_jerk=100.0,
        joint_target_natural_frequency=20.0,
    )
    for output in robot.raw["outputs"]:
        output["low_speed_flag"] = b"\x01"

    original_set_joint_cmd_pose = robot.set_joint_cmd_pose

    def follow_joint_command(arm, joints):
        result = original_set_joint_cmd_pose(arm, joints)
        index = 0 if arm == "A" else 1
        robot.raw["outputs"][index]["fb_joint_pos"] = list(joints)
        robot.raw["outputs"][index]["fb_joint_vel"] = [0.0] * 7
        return result

    robot.set_joint_cmd_pose = follow_joint_command

    controller.xr_client.start()
    assert controller.xr_client.wait_until_ready(timeout=1.0)
    controller._configure_hardware()

    state = controller._robot_state_slot.get()
    np.testing.assert_allclose(state.q_rad, target, atol=np.deg2rad(0.05))
    np.testing.assert_allclose(controller._return_joint_targets[0], target[:7])
    np.testing.assert_allclose(controller._return_joint_targets[1], target[7:])
    assert controller.safety.state == MarvinControlState.ARMED
    controller.xr_client.close()
    controller.adapter.release()


def test_hardware_and_mujoco_share_natural_rest_pose():
    from scripts.simulation.teleop_marvin_mujoco import MARVIN_HUMAN_REST_QPOS

    np.testing.assert_allclose(MARVIN_HUMAN_REST_Q_RAD, MARVIN_HUMAN_REST_QPOS)
    np.testing.assert_allclose(
        np.rad2deg(MARVIN_HUMAN_REST_Q_RAD),
        [90, -90, 90, 20, -90, 0, 0, -90, -90, -90, 20, 90, 0, 0],
    )


def test_hardware_startup_accepts_lost_parameter_ack_when_readback_matches(
    monkeypatch, capsys
):
    controller, robot = make_hardware_controller(monkeypatch)
    for output in robot.raw["outputs"]:
        output["low_speed_flag"] = b"\x01"
    robot.send_cmd_wait_response_results = [0]

    controller._configure_hardware()

    assert controller.safety.state == MarvinControlState.ARMED
    assert "readback matches all requested settings" in capsys.readouterr().out
    # The timed-out configuration is reconciled, not resent. The remaining
    # calls are startup hold, impedance mode, and PD period.
    assert robot.send_cmd_wait_response_calls == 4
    controller.xr_client.close()
    controller.adapter.release()


def test_hardware_startup_rejects_lost_parameter_ack_when_readback_mismatches(
    monkeypatch,
):
    controller, robot = make_hardware_controller(monkeypatch)
    for output in robot.raw["outputs"]:
        output["low_speed_flag"] = b"\x01"
    robot.send_cmd_wait_response_results = [0]
    robot.set_joint_kd_params = lambda _arm, _k, _d: True

    with pytest.raises(
        TimeoutError,
        match="response timed out and fresh controller readback does not match",
    ):
        controller._configure_hardware()

    assert controller.safety.state == MarvinControlState.READ_ONLY
    controller.xr_client.close()
    controller.adapter.release()


def test_active_tool_config_preserves_vendor_units(tmp_path):
    config = {
        "arm0": {"tool-1": {"kine": [0] * 6, "dyn": [0.481, 4.691, -34.036, 84.135, 0.001, 0, 0, 0.016, 0, 0.002]}},
        "arm1": {
            "tool-2": {
                "kine": [1, 2, 3, 4, 5, 6],
                "dyn": [0.459, -0.776, 29.685, 101.05, 0.007, 0, 0, 0.006, 0, 0.001],
            }
        },
        "current_tool": {"arm0": "tool-1", "arm1": "tool-2"},
    }
    path = tmp_path / "tools.json"
    path.write_text(json.dumps(config), encoding="utf-8")
    left, right = load_active_tool_configs(path)
    assert left.dynamics_vendor_units[1:4] == (4.691, -34.036, 84.135)
    assert right.kinematics_mm_deg == (1, 2, 3, 4, 5, 6)


def test_safety_supervisor_latches_fault_and_requires_grip_release_after_hold():
    safety = MarvinSafetySupervisor()
    safety.connected_read_only()
    safety.arm()
    now = time.monotonic_ns()
    state = make_state(now)
    active = MarvinJointCommand(1, now, np.zeros(14), (True, False))
    assert safety.evaluate(state, active, 1.0, now).state == MarvinControlState.TELEOP
    assert safety.evaluate(state, active, 150.0, now).state == MarvinControlState.HOLD
    decision = safety.evaluate(state, active, 1.0, now)
    assert decision.state == MarvinControlState.HOLD
    assert "release all Grip" in decision.reason
    inactive = MarvinJointCommand(2, now, np.zeros(14), (False, False))
    assert safety.evaluate(state, inactive, 1.0, now).state == MarvinControlState.ARMED
    assert safety.evaluate(state, active, 1.0, now).state == MarvinControlState.TELEOP

    error_state = make_state(now, error=(0, 42))
    assert safety.evaluate(error_state, active, 1.0, now).state == MarvinControlState.FAULT
    assert safety.evaluate(state, inactive, 1.0, now).request_idle


def test_safety_rejects_stale_feedback_and_command():
    config = MarvinSafetyConfig(feedback_hold_ms=20, feedback_fault_ms=50)
    safety = MarvinSafetySupervisor(config)
    safety.connected_read_only()
    safety.arm()
    now = time.monotonic_ns()
    stale_feedback = make_state(now - int(60e6))
    command = MarvinJointCommand(1, now, np.zeros(14), (False, False))
    assert safety.evaluate(stale_feedback, command, 0.0, now).state == MarvinControlState.FAULT

    safety = MarvinSafetySupervisor(config)
    safety.connected_read_only()
    safety.arm()
    fresh = make_state(now)
    stale_command = MarvinJointCommand(1, now - int(50e6), np.zeros(14), (False, False))
    assert safety.evaluate(fresh, stale_command, 0.0, now).state == MarvinControlState.HOLD


def test_safety_faults_if_runtime_leaves_joint_impedance_mode():
    safety = MarvinSafetySupervisor()
    safety.connected_read_only()
    safety.arm()
    now = time.monotonic_ns()
    state = make_state(now)
    object.__setattr__(state, "arm_state", (3, 1))
    command = MarvinJointCommand(1, now, np.zeros(14), (False, False))
    decision = safety.evaluate(state, command, 0.0, now)
    assert decision.state == MarvinControlState.FAULT
    assert decision.request_idle


def test_feedback_age_tracks_stalled_frame_serial_not_subscribe_calls():
    robot = FakeMarvinRobot()
    adapter = MarvinSdkAdapter(robot=robot, dcss=object())
    adapter.connect()
    first = adapter.read_state()
    original_subscribe = robot.subscribe

    def frozen_subscribe(dcss):
        frames = robot.frames.copy()
        raw = original_subscribe(dcss)
        robot.frames[:] = frames
        for index, frame in enumerate(frames):
            raw["outputs"][index]["frame_serial"] = frame
        return raw

    robot.subscribe = frozen_subscribe
    time.sleep(0.003)
    second = adapter.read_state()
    assert second.receipt_monotonic_ns == first.receipt_monotonic_ns
    assert second.age_ms() >= 2.0


def test_fresh_feedback_requires_both_arm_frames_to_advance():
    robot = FakeMarvinRobot()
    original_subscribe = robot.subscribe

    def right_arm_frozen(dcss):
        right_frame = robot.frames[1]
        raw = original_subscribe(dcss)
        robot.frames[1] = right_frame
        raw["outputs"][1]["frame_serial"] = max(1, right_frame)
        return raw

    robot.subscribe = right_arm_frozen
    adapter = MarvinSdkAdapter(robot=robot, dcss=object())
    adapter.connect()
    with pytest.raises(TimeoutError, match="A=.*B="):
        adapter.wait_for_fresh_feedback(timeout=0.04, required_updates=2)


def test_feedback_limiter_enforces_velocity_acceleration_jerk_and_soft_limits():
    limiter = FeedbackAwareJointLimiter(
        lower_limits=[-1.0, -1.0],
        upper_limits=[1.0, 1.0],
        max_velocity=[0.5, 0.5],
        max_acceleration=[1.0, 1.0],
        max_jerk=[10.0, 10.0],
        limit_margin=0.1,
        dt=0.01,
    )
    limiter.reset([0.0, 0.0])
    command = limiter.limit([2.0, -2.0], [0.0, 0.0], [0.0, 0.0])
    velocity = command / 0.01
    assert np.all(np.abs(velocity) <= 0.001 + 1e-12)  # jerk: da=0.1, dv=0.001
    assert np.all(command <= limiter.soft_upper)
    assert np.all(command >= limiter.soft_lower)

    velocities = [velocity.copy()]
    accelerations = [velocity / 0.01]
    for _ in range(20):
        command = limiter.limit([0.5, -0.5], [0.0, 0.0], [0.0, 0.0])
        velocities.append(limiter.last_velocity.copy())
        accelerations.append(limiter.last_acceleration.copy())
    velocities = np.asarray(velocities)
    accelerations = np.asarray(accelerations)
    assert np.max(np.abs(velocities)) <= 0.5 + 1e-12
    assert np.max(np.abs(accelerations)) <= 1.0 + 1e-12
    assert np.max(np.abs(np.diff(accelerations, axis=0) / 0.01)) <= 10.0 + 1e-10

    limiter.reset([0.895, 0.0], [0.4, 0.0])
    with pytest.raises(RuntimeError, match="enter FAULT"):
        limiter.limit([1.0, 0.0], [0.895, 0.0], [0.4, 0.0])

    with pytest.raises(RuntimeError, match="outside joint soft limits"):
        limiter.reset([0.95, 0.0])


def test_feedback_limiter_reaches_cruise_without_jerk_velocity_dead_end():
    limiter = FeedbackAwareJointLimiter(
        lower_limits=[-2.0],
        upper_limits=[2.0],
        max_velocity=[0.1],
        max_acceleration=[0.3],
        max_jerk=[2.0],
        dt=0.01,
    )
    limiter.reset([0.0])
    velocities = []
    accelerations = []
    command = np.array([0.0])
    for _ in range(800):
        previous_velocity = limiter.last_velocity.copy()
        previous_acceleration = limiter.last_acceleration.copy()
        command = limiter.limit([0.5], command, limiter.last_velocity)
        velocities.append(limiter.last_velocity.copy())
        accelerations.append(limiter.last_acceleration.copy())
        assert np.max(np.abs(limiter.last_velocity)) <= 0.1 + 1e-12
        assert np.max(np.abs(limiter.last_acceleration)) <= 0.3 + 1e-12
        assert np.max(
            np.abs((limiter.last_acceleration - previous_acceleration) / 0.01)
        ) <= 2.0 + 1e-9
        assert np.max(
            np.abs((limiter.last_velocity - previous_velocity) / 0.01)
        ) <= 0.3 + 1e-12
    assert command[0] == pytest.approx(0.5, abs=2e-4)
    assert abs(velocities[-1][0]) < 5e-4
    assert abs(accelerations[-1][0]) < 5e-3


def test_feedback_limiter_reserves_jerk_braking_distance_before_soft_limit():
    limiter = FeedbackAwareJointLimiter(
        lower_limits=[-1.0472],
        upper_limits=[1.0472],
        max_velocity=[0.1],
        max_acceleration=[0.3],
        max_jerk=[2.0],
        limit_margin=np.deg2rad(5.0),
        dt=0.005,
    )
    assert limiter.target_braking_guard[0] == pytest.approx(
        0.024666666666666667
    )
    assert limiter.jerk_braking_extra_distance_at_max_velocity[0] == pytest.approx(
        0.0075
    )

    command = np.array([-0.8])
    limiter.reset(command)
    minimum_command = command[0]
    for _ in range(2000):
        command = limiter.limit(
            [limiter.soft_lower[0]],
            command,
            limiter.last_velocity,
        )
        minimum_command = min(minimum_command, command[0])

    expected_boundary = limiter.target_lower[0]
    assert minimum_command >= expected_boundary - 2e-4
    assert abs(limiter.last_velocity[0]) < 5e-4


def test_feedback_limiter_exact_hold_clears_command_motion_state():
    limiter = FeedbackAwareJointLimiter(
        lower_limits=[-1.0, -1.0],
        upper_limits=[1.0, 1.0],
        max_velocity=0.1,
        max_acceleration=0.3,
        max_jerk=2.0,
        dt=0.01,
    )
    limiter.reset([0.1, -0.2], [0.05, -0.05])
    limiter.last_acceleration[:] = [0.2, -0.2]
    limiter.hold([0.1, -0.2], [False, True])
    command = limiter.limit([0.5, -0.2], [0.1, -0.2], [0.0, 0.0])
    assert command[1] == pytest.approx(-0.2)
    assert limiter.last_velocity[1] == pytest.approx(0.0)
    assert limiter.last_acceleration[1] == pytest.approx(0.0)


def test_cartesian_guard_limits_workspace_speed_and_rejects_jumps():
    initial = np.eye(4)
    guard = CartesianTargetGuard(
        initial,
        dt=0.01,
        max_displacement_m=0.2,
        max_linear_speed_m_s=0.1,
        max_frame_translation_m=0.15,
    )
    target = np.eye(4)
    target[0, 3] = 0.1
    limited = guard.filter(target)
    assert limited[0, 3] == pytest.approx(0.001)

    jump = target.copy()
    jump[0, 3] = 0.3
    with pytest.raises(RuntimeError, match="single-frame TCP translation jump"):
        guard.filter(jump)


def test_hardware_session_logger_writes_events_and_summary(tmp_path):
    logger = MarvinSessionLogger(tmp_path, {"command_hz": 50, "limits": np.array([0.1])})
    logger.record("robot_state", q_rad=np.zeros(14))
    logger.close(final_state="shutdown")
    events = logger.events_path.read_text(encoding="utf-8").splitlines()
    assert len(events) == 1
    assert json.loads(events[0])["event"] == "robot_state"
    summary = json.loads(logger.summary_path.read_text(encoding="utf-8"))
    assert summary["event_count"] == 1
    assert summary["configuration"]["command_hz"] == 50


def test_calibration_recorder_writes_synchronized_feedback_and_control(tmp_path):
    now_ns = time.monotonic_ns()
    state = make_state(now_ns)
    command = MarvinJointCommand(3, now_ns, np.linspace(0.0, 0.13, 14), (True, False))
    transforms = np.repeat(np.eye(4)[None, :, :], 2, axis=0)
    transforms[0, 0, 3] = 0.1
    observation = MarvinControlObservation(
        sequence=3,
        monotonic_ns=now_ns,
        duration_ms=1.2,
        deadline_lateness_ms=0.1,
        deadline_miss=False,
        xr_sequence=9,
        xr_source_timestamp_ns=1234,
        xr_poll_age_ms=2.0,
        xr_source_age_ms=3.0,
        q_ik_rad=np.zeros(14),
        q_command_rad=command.q_rad,
        active_arms=command.active_arms,
        raw_tcp_transforms=transforms,
        limited_tcp_transforms=transforms,
        actual_tcp_transforms=transforms,
        translational_sigma_min=[0.01, np.nan],
    )
    recorder = MarvinCalibrationRecorder(
        tmp_path,
        {"robot_model": "test", "tool_mass": np.array([0.481, 0.459])},
        MARVIN_JOINT_NAMES,
    )
    recorder.record(
        state,
        command,
        observation,
        safety_state="teleop",
        safety_reason="test",
        sdk_read_duration_ms=0.4,
        scale_factor=0.6,
    )
    time.sleep(0.001)
    recorder.record(
        state,
        command,
        observation,
        safety_state="teleop",
        safety_reason="test",
        sdk_read_duration_ms=0.5,
        scale_factor=0.6,
    )
    recorder.close(terminal_state={"state_before_shutdown": "teleop"})

    with recorder.csv_path.open(newline="", encoding="utf-8") as csv_file:
        rows = list(csv.DictReader(csv_file))
    assert len(rows) == 2
    assert rows[0]["software_command_sequence"] == "3"
    assert float(rows[0]["scale_factor"]) == pytest.approx(0.6)
    assert float(rows[0]["software_q_command_rad_Joint7_R"]) == pytest.approx(0.13)
    assert float(rows[0]["raw_target_left_T_03"]) == pytest.approx(0.1)
    metadata = json.loads(recorder.metadata_path.read_text(encoding="utf-8"))
    assert metadata["sample_count"] == 2
    assert metadata["configuration"]["tool_mass"] == [0.481, 0.459]
    npz_path, valid_count, total_count = convert(recorder.csv_path)
    assert (valid_count, total_count) == (2, 2)
    with np.load(npz_path, allow_pickle=False) as dataset:
        assert dataset["q_rad"].shape == (2, 14)
        assert dataset["actual_tcp_transform"].shape == (2, 2, 4, 4)


def test_hardware_controller_defaults_to_read_only_and_requires_explicit_enable(monkeypatch):
    class FakeXrClient:
        def __init__(self):
            self.closed = False

        def get_pose_by_name(self, _name):
            return np.array([0.0, 1.5, 0.0, 0.0, 0.0, 0.0, 1.0])

        def get_key_value_by_name(self, _name):
            return 0.0

        def get_button_state_by_name(self, _name):
            return False

        def get_motion_tracker_data(self):
            return {}

        def get_timestamp_ns(self):
            return 1

        def close(self):
            self.closed = True

    monkeypatch.setattr(base_controller_module, "XrClient", FakeXrClient)
    robot = FakeMarvinRobot()
    adapter = MarvinSdkAdapter(robot=robot, dcss=object())
    controller = MarvinHardwareTeleopController(
        adapter=adapter,
        robot_urdf_path=MARVIN_URDF,
        manipulator_config={
            "left_hand": {
                "link_name": "TCP_Link_L",
                "pose_source": "left_controller",
                "control_trigger": "left_grip",
                "manipulability_weight": 0.0,
            },
            "right_hand": {
                "link_name": "TCP_Link_R",
                "pose_source": "right_controller",
                "control_trigger": "right_grip",
                "manipulability_weight": 0.0,
            },
        },
        joint_names=MARVIN_JOINT_NAMES,
        R_headset_world=np.eye(3),
        enable_hardware=False,
    )
    assert controller.safety.state == MarvinControlState.READ_ONLY
    np.testing.assert_allclose(controller.placo_robot.state.q[controller._joint_offsets], 0.0)
    with pytest.raises(PermissionError, match="explicit"):
        controller.run()
    assert robot.released
    assert controller.xr_client._client.closed


def test_hardware_controller_releases_sdk_on_version_mismatch():
    robot = FakeMarvinRobot()
    adapter = MarvinSdkAdapter(robot=robot, dcss=object())
    with pytest.raises(RuntimeError, match="SDK version mismatch"):
        MarvinHardwareTeleopController(
            adapter=adapter,
            robot_urdf_path=MARVIN_URDF,
            manipulator_config={},
            joint_names=MARVIN_JOINT_NAMES,
            R_headset_world=np.eye(3),
            expected_sdk_version=9999,
        )
    assert robot.released
