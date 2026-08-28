"""Safety-oriented PICO teleoperation controller for Marvin dual-arm hardware."""

from __future__ import annotations

import threading
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import meshcat.transformations as tf
import numpy as np

from xrobotoolkit_teleop.common.marvin_motion_limits import (
    HUMAN_PEAK_TCP_ANGULAR_SPEED_RAD_S,
    HUMAN_PEAK_TCP_LINEAR_SPEED_M_S,
    MARVIN_PEAK_JOINT_ACCELERATION_RAD_S2,
    MARVIN_PEAK_JOINT_JERK_RAD_S3,
    MARVIN_PEAK_JOINT_VELOCITY_RAD_S,
    MARVIN_PEAK_TARGET_NATURAL_FREQUENCY_RAD_S,
    MARVIN_STARTUP_JOINT_ACCELERATION_RAD_S2,
    MARVIN_STARTUP_JOINT_JERK_RAD_S3,
    MARVIN_STARTUP_JOINT_VELOCITY_RAD_S,
)

from xrobotoolkit_teleop.common.base_teleop_controller import BaseTeleopController
from xrobotoolkit_teleop.common.buffered_xr_client import BufferedXrClient
from xrobotoolkit_teleop.common.cartesian_target_guard import CartesianTargetGuard
from xrobotoolkit_teleop.common.joint_command_limiter import FeedbackAwareJointLimiter
from xrobotoolkit_teleop.common.marvin_calibration_recorder import (
    MarvinCalibrationRecorder,
)
from xrobotoolkit_teleop.common.marvin_observation import MarvinControlObservation
from xrobotoolkit_teleop.common.marvin_safety import (
    MarvinControlState,
    MarvinSafetyConfig,
    MarvinSafetySupervisor,
)
from xrobotoolkit_teleop.common.marvin_session_logger import MarvinSessionLogger
from xrobotoolkit_teleop.common.marvin_scale_calibration import (
    make_marvin_scale_calibration_config,
    save_scale_calibration,
)
from xrobotoolkit_teleop.common.marvin_types import LatestValue, MarvinJointCommand
from xrobotoolkit_teleop.hardware.marvin_ros2_observer import MarvinRos2Observer


class MarvinHardwareTeleopController(BaseTeleopController):
    def __init__(
        self,
        adapter,
        robot_urdf_path,
        manipulator_config,
        joint_names,
        R_headset_world,
        scale_factor=0.5,
        reference_mode="head_yaw",
        enable_release_return=True,
        return_duration=3.0,
        startup_move_duration_s=10.0,
        initial_pose_q_rad=None,
        startup_pose_tolerance_deg=0.5,
        enable_arm_length_calibration=False,
        scale_calibration_path=None,
        calibration_workspace_margin=0.95,
        control_hz=200.0,
        feedback_hz=200.0,
        command_hz=200.0,
        xr_poll_hz=200.0,
        max_joint_velocity=MARVIN_PEAK_JOINT_VELOCITY_RAD_S,
        max_joint_acceleration=MARVIN_PEAK_JOINT_ACCELERATION_RAD_S2,
        max_joint_jerk=MARVIN_PEAK_JOINT_JERK_RAD_S3,
        joint_target_natural_frequency=MARVIN_PEAK_TARGET_NATURAL_FREQUENCY_RAD_S,
        max_tcp_displacement_m=0.25,
        max_tcp_linear_speed_m_s=HUMAN_PEAK_TCP_LINEAR_SPEED_M_S,
        max_tcp_angular_speed_rad_s=HUMAN_PEAK_TCP_ANGULAR_SPEED_RAD_S,
        max_tcp_frame_jump_m=0.15,
        max_tcp_frame_jump_deg=45.0,
        singularity_fault_sigma=0.003,
        singularity_full_speed_sigma=0.015,
        startup_max_joint_speed_rad_s=0.02,
        joint_limit_margin=np.deg2rad(5.0),
        velocity_ratio=100,
        acceleration_ratio=100,
        left_k=(2, 2, 2, 1.5, 0.8, 0.8, 0.8),
        left_d=(0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3),
        right_k=(2, 2, 2, 1.5, 0.8, 0.8, 0.8),
        right_d=(0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3),
        parameter_settle_s=0.2,
        mode_settle_s=1.0,
        pd_settle_s=1.0,
        left_tool=None,
        right_tool=None,
        configure_tools=False,
        enable_hardware=False,
        safety_config=MarvinSafetyConfig(),
        log_dir="logs",
        visualize_placo=False,
        session_metadata=None,
        expected_sdk_version=None,
        enable_ros2_observation=False,
        ros2_namespace="/marvin_teleop",
        ros2_publish_hz=100.0,
    ):
        if len(joint_names) != 14 or len(set(joint_names)) != 14:
            raise ValueError("joint_names must contain 14 unique joints in left-then-right order")
        for rate_name, rate in (
            ("control_hz", control_hz),
            ("feedback_hz", feedback_hz),
            ("command_hz", command_hz),
            ("xr_poll_hz", xr_poll_hz),
        ):
            if rate <= 0.0:
                raise ValueError(f"{rate_name} must be positive")
        if feedback_hz < command_hz:
            raise ValueError("feedback_hz must be at least command_hz")
        ratio = feedback_hz / command_hz
        if not np.isclose(ratio, round(ratio)):
            raise ValueError("feedback_hz must be an integer multiple of command_hz")
        control_ratio = control_hz / command_hz
        if control_ratio < 1.0 or not np.isclose(control_ratio, round(control_ratio)):
            raise ValueError("control_hz must be an integer multiple of command_hz")
        if command_hz > 200.0:
            raise ValueError("vendor real-time point commands must not exceed 200 Hz")
        pd_period = 1000.0 / command_hz
        if not np.isclose(pd_period, round(pd_period)):
            raise ValueError("command_hz must map to an integer-millisecond PD period")
        pd_period_ms = int(round(pd_period))
        if pd_period_ms > 20:
            raise ValueError("PD feedforward period exceeds the vendor-supported 20 ms maximum")
        if singularity_fault_sigma <= 0.0 or singularity_full_speed_sigma <= singularity_fault_sigma:
            raise ValueError("singularity thresholds must be positive and ordered")
        if startup_max_joint_speed_rad_s <= 0.0:
            raise ValueError("startup_max_joint_speed_rad_s must be positive")
        maximum_velocity = np.asarray(max_joint_velocity, dtype=float)
        if np.any(~np.isfinite(maximum_velocity)) or np.any(maximum_velocity <= 0.0):
            raise ValueError("max_joint_velocity must contain finite positive values")
        if np.any(maximum_velocity > np.pi):
            raise ValueError(
                "max_joint_velocity exceeds the vendor PD limit of 180 deg/s"
            )
        if return_duration <= 0.0:
            raise ValueError("return_duration must be positive")
        if startup_move_duration_s <= 0.0:
            raise ValueError("startup_move_duration_s must be positive")
        if startup_pose_tolerance_deg <= 0.0:
            raise ValueError("startup_pose_tolerance_deg must be positive")
        self.initial_pose_q_rad = (
            None
            if initial_pose_q_rad is None
            else np.asarray(initial_pose_q_rad, dtype=float).reshape(-1).copy()
        )
        if self.initial_pose_q_rad is not None and (
            self.initial_pose_q_rad.size != 14
            or not np.all(np.isfinite(self.initial_pose_q_rad))
        ):
            raise ValueError("initial_pose_q_rad must be a finite 14-joint vector")
        self.startup_pose_tolerance_rad = np.deg2rad(startup_pose_tolerance_deg)
        self.startup_move_duration_s = float(startup_move_duration_s)
        self.startup_motion_max_velocity = np.asarray(
            MARVIN_STARTUP_JOINT_VELOCITY_RAD_S, dtype=float
        )
        self.startup_motion_max_acceleration = np.asarray(
            MARVIN_STARTUP_JOINT_ACCELERATION_RAD_S2, dtype=float
        )
        self.startup_motion_max_jerk = np.asarray(
            MARVIN_STARTUP_JOINT_JERK_RAD_S3, dtype=float
        )
        if enable_arm_length_calibration and not str(scale_calibration_path or "").strip():
            raise ValueError(
                "scale_calibration_path is required when arm-length calibration is enabled"
            )

        self.adapter = adapter
        self.joint_names = tuple(joint_names)
        self.control_hz = float(control_hz)
        self.feedback_hz = float(feedback_hz)
        self.command_hz = float(command_hz)
        self.xr_poll_hz = float(xr_poll_hz)
        self.pd_period_ms = pd_period_ms
        self.velocity_ratio = int(velocity_ratio)
        self.acceleration_ratio = int(acceleration_ratio)
        if not 1 <= self.velocity_ratio <= 100 or not 1 <= self.acceleration_ratio <= 100:
            raise ValueError("velocity_ratio and acceleration_ratio must be in [1, 100]")
        self.left_k = tuple(float(value) for value in left_k)
        self.left_d = tuple(float(value) for value in left_d)
        self.right_k = tuple(float(value) for value in right_k)
        self.right_d = tuple(float(value) for value in right_d)
        if any(
            len(values) != 7
            for values in (self.left_k, self.left_d, self.right_k, self.right_d)
        ):
            raise ValueError("each Marvin K/D vector must contain seven values")
        for name, values, upper in (
            ("left_k", self.left_k, 22.0),
            ("right_k", self.right_k, 22.0),
            ("left_d", self.left_d, 1.0),
            ("right_d", self.right_d, 1.0),
        ):
            array = np.asarray(values)
            if (
                not np.all(np.isfinite(array))
                or np.any(array < 0.0)
                or np.any(array > upper)
            ):
                raise ValueError(f"{name} must contain finite values in [0, {upper:g}]")
        for name, duration in (
            ("parameter_settle_s", parameter_settle_s),
            ("mode_settle_s", mode_settle_s),
            ("pd_settle_s", pd_settle_s),
        ):
            if not np.isfinite(duration) or duration < 0.0:
                raise ValueError(f"{name} must be finite and non-negative")
        self.parameter_settle_s = float(parameter_settle_s)
        self.mode_settle_s = float(mode_settle_s)
        self.pd_settle_s = float(pd_settle_s)
        self.left_tool = left_tool
        self.right_tool = right_tool
        self.configure_tools = bool(configure_tools)
        self.enable_hardware = bool(enable_hardware)
        self.visualize_placo = bool(visualize_placo)
        self.log_dir = log_dir
        self.session_metadata = dict(session_metadata or {})
        self.expected_sdk_version = expected_sdk_version
        self.enable_ros2_observation = bool(enable_ros2_observation)
        self.ros2_namespace = str(ros2_namespace)
        self.ros2_publish_hz = float(ros2_publish_hz)
        if self.ros2_publish_hz <= 0.0:
            raise ValueError("ros2_publish_hz must be positive")
        self.startup_max_joint_speed_rad_s = float(startup_max_joint_speed_rad_s)
        self.enable_release_return = bool(enable_release_return)
        self.return_duration = float(return_duration)
        self.enable_arm_length_calibration = bool(enable_arm_length_calibration)
        self.scale_calibration_path = scale_calibration_path
        self.calibration_workspace_margin = float(calibration_workspace_margin)
        self.safety = MarvinSafetySupervisor(safety_config)
        self._robot_state_slot = LatestValue()
        self._command_slot = LatestValue()
        self._control_fault_slot = LatestValue()
        self._thread_errors = LatestValue()
        self._control_observation_slot = LatestValue()
        self._command_sequence = 0
        self._ik_failure_count = 0
        self._logger = None
        self._calibration_recorder = None
        self._ros2_observer = None
        self._previous_active_arms = (False, False)
        self._arm_hold_targets = initial_hold_targets = [None, None]
        self._return_joint_targets = [None, None]
        self._return_trajectories = [None, None]
        self._return_completion_tolerance_rad = np.deg2rad(0.5)
        self._io_hold_target = None
        self._tcp_guard_config = {
            "max_displacement_m": float(max_tcp_displacement_m),
            "max_linear_speed_m_s": float(max_tcp_linear_speed_m_s),
            "max_angular_speed_rad_s": float(max_tcp_angular_speed_rad_s),
            "max_frame_jump_m": float(max_tcp_frame_jump_m),
            "max_frame_jump_deg": float(max_tcp_frame_jump_deg),
            "singularity_fault_sigma": float(singularity_fault_sigma),
            "singularity_full_speed_sigma": float(singularity_full_speed_sigma),
        }

        try:
            adapter.connect()
            actual_sdk_version = adapter.sdk_version()
            if expected_sdk_version is not None and actual_sdk_version != expected_sdk_version:
                raise RuntimeError(
                    f"Marvin SDK version mismatch: expected {expected_sdk_version}, "
                    f"got {actual_sdk_version}"
                )
            initial_state = adapter.wait_for_fresh_feedback()
            if any(initial_state.error_code) or any(
                state == 100 for state in initial_state.arm_state
            ):
                raise RuntimeError(
                    f"Marvin starts in an error state: states={initial_state.arm_state}, "
                    f"errors={initial_state.error_code}; clear it manually before teleoperation"
                )
        except Exception:
            adapter.release()
            raise
        self._robot_state_slot.set(initial_state)
        initial_hold_targets[0] = initial_state.q_rad[:7].copy()
        initial_hold_targets[1] = initial_state.q_rad[7:].copy()
        self._return_joint_targets[0] = initial_state.q_rad[:7].copy()
        self._return_joint_targets[1] = initial_state.q_rad[7:].copy()
        self.safety.connected_read_only()

        try:
            joint_limits = self._read_urdf_joint_limits(robot_urdf_path, self.joint_names)
            super().__init__(
                robot_urdf_path=str(robot_urdf_path),
                manipulator_config=manipulator_config,
                floating_base=False,
                R_headset_world=R_headset_world,
                scale_factor=scale_factor,
                q_init=initial_state.q_rad,
                dt=1.0 / self.control_hz,
                reference_mode=reference_mode,
                scale_calibration_config=(
                    {
                        "button": "A",
                        "cancel_button": "B",
                        **make_marvin_scale_calibration_config(
                            self.calibration_workspace_margin
                        ),
                    }
                    if self.enable_arm_length_calibration
                    else None
                ),
            )
            self._joint_offsets = np.asarray(
                [self.placo_robot.get_joint_offset(name) for name in self.joint_names],
                dtype=int,
            )
            self._update_robot_state()
            self.sync_end_effector_poses_to_placo_tasks()
            self.limiter = FeedbackAwareJointLimiter(
                lower_limits=joint_limits[:, 0],
                upper_limits=joint_limits[:, 1],
                max_velocity=max_joint_velocity,
                max_acceleration=max_joint_acceleration,
                max_jerk=max_joint_jerk,
                target_natural_frequency=joint_target_natural_frequency,
                limit_margin=joint_limit_margin,
                dt=1.0 / self.control_hz,
            )
            # This state describes the acknowledged command trajectory. Measured
            # velocity is used by predictive braking, not as command velocity.
            self.limiter.reset(initial_state.q_rad)
            if self.initial_pose_q_rad is not None:
                outside = (self.initial_pose_q_rad < self.limiter.soft_lower) | (
                    self.initial_pose_q_rad > self.limiter.soft_upper
                )
                if np.any(outside):
                    joints = np.flatnonzero(outside).tolist()
                    raise ValueError(
                        "initial pose lies outside joint soft limits at indices "
                        f"{joints}"
                    )
                self._return_joint_targets[0] = self.initial_pose_q_rad[:7].copy()
                self._return_joint_targets[1] = self.initial_pose_q_rad[7:].copy()
            self._joint_limit_margin = float(joint_limit_margin)
            self._target_guards = {}
            self._guard_previous_active = {name: False for name in manipulator_config}
            self._raw_tcp_targets = {}
            self._limited_tcp_targets = {}
            self._raw_tcp_transforms = {}
            self._limited_tcp_transforms = {}
            self._singular_values = {}
            for name, config in manipulator_config.items():
                transform = self.placo_robot.get_T_world_frame(config["link_name"])
                self._target_guards[name] = CartesianTargetGuard(
                    transform,
                    dt=1.0 / self.control_hz,
                    max_displacement_m=max_tcp_displacement_m,
                    max_linear_speed_m_s=max_tcp_linear_speed_m_s,
                    max_angular_speed_rad_s=max_tcp_angular_speed_rad_s,
                    max_frame_translation_m=max_tcp_frame_jump_m,
                    max_frame_rotation_rad=np.deg2rad(max_tcp_frame_jump_deg),
                )

            pose_names = ["headset"]
            key_names = []
            button_names = (
                [self._calibration_button, self._calibration_cancel_button]
                if self.scale_calibrator is not None
                else []
            )
            for config in manipulator_config.values():
                pose_names.append(config["pose_source"])
                key_names.append(config["control_trigger"])
            self.xr_client = BufferedXrClient(
                self.xr_client,
                pose_names=pose_names,
                key_names=key_names,
                button_names=button_names,
                poll_hz=self.xr_poll_hz,
            )
            if self.visualize_placo:
                self._init_placo_viz()
        except Exception:
            xr_client = getattr(self, "xr_client", None)
            if xr_client is not None:
                try:
                    xr_client.close()
                except Exception:
                    pass
            adapter.release()
            raise

    @staticmethod
    def _read_urdf_joint_limits(urdf_path, joint_names):
        root = ET.parse(urdf_path).getroot()
        joints = {joint.attrib["name"]: joint for joint in root.findall("joint")}
        limits = []
        for name in joint_names:
            joint = joints.get(name)
            limit = None if joint is None else joint.find("limit")
            if limit is None or "lower" not in limit.attrib or "upper" not in limit.attrib:
                raise ValueError(f"URDF joint '{name}' has no finite position limits")
            limits.append([float(limit.attrib["lower"]), float(limit.attrib["upper"])])
        result = np.asarray(limits, dtype=float)
        if not np.all(np.isfinite(result)):
            raise ValueError("URDF joint limits must be finite")
        return result

    def _robot_setup(self):
        # Connection and first feedback are deliberately completed before the
        # BaseTeleopController creates its Placo tasks.
        pass

    def _get_link_pose(self, link_name):
        transform = self.placo_robot.get_T_world_frame(link_name)
        return transform[:3, 3].copy(), tf.quaternion_from_matrix(transform)

    def _update_robot_state(self):
        state = self._robot_state_slot.get()
        if state is None or not hasattr(self, "_joint_offsets"):
            return
        configuration = self.placo_robot.state.q.copy()
        configuration[self._joint_offsets] = state.q_rad
        self.placo_robot.state.q = configuration
        self.placo_robot.update_kinematics()

    def _actual_tcp_transforms(self, state):
        """Evaluate feedback TCPs without changing the solver's commanded state."""
        q_solver = self.placo_robot.state.q.copy()
        q_feedback = q_solver.copy()
        q_feedback[self._joint_offsets] = state.q_rad
        try:
            self.placo_robot.state.q = q_feedback
            self.placo_robot.update_kinematics()
            return {
                name: self.placo_robot.get_T_world_frame(config["link_name"]).copy()
                for name, config in self.manipulator_config.items()
            }
        finally:
            self.placo_robot.state.q = q_solver
            self.placo_robot.update_kinematics()

    def _send_command(self):
        # Hardware I/O is performed only by _io_loop. This method publishes a
        # latest-value command and is kept for the BaseTeleopController contract.
        state = self._robot_state_slot.get()
        if state is None:
            return
        target = self.placo_robot.state.q[self._joint_offsets].copy()
        active_arms = (
            bool(self.active.get("left_hand", False)),
            bool(self.active.get("right_hand", False)),
        )
        now = time.monotonic()
        hold_mask = np.zeros(len(self.joint_names), dtype=bool)
        for arm_index, active in enumerate(active_arms):
            joint_slice = slice(arm_index * 7, (arm_index + 1) * 7)
            if active:
                self._return_trajectories[arm_index] = None
            elif self._previous_active_arms[arm_index]:
                self._arm_hold_targets[arm_index] = state.q_rad[joint_slice].copy()
                self.limiter.last_command[joint_slice] = state.q_rad[joint_slice]
                self.limiter.last_velocity[joint_slice] = 0.0
                self.limiter.last_acceleration[joint_slice] = 0.0
                if self.enable_release_return:
                    self._return_trajectories[arm_index] = {
                        "start_monotonic": now,
                        "start_q_rad": state.q_rad[joint_slice].copy(),
                    }
            if not active:
                trajectory = self._return_trajectories[arm_index]
                if trajectory is None:
                    target[joint_slice] = self._arm_hold_targets[arm_index]
                    hold_mask[joint_slice] = True
                else:
                    elapsed = max(0.0, now - trajectory["start_monotonic"])
                    progress = min(1.0, elapsed / self.return_duration)
                    blend = 0.5 - 0.5 * np.cos(np.pi * progress)
                    return_target = self._return_joint_targets[arm_index]
                    target[joint_slice] = trajectory["start_q_rad"] + blend * (
                        return_target - trajectory["start_q_rad"]
                    )
                    position_error = np.max(
                        np.abs(state.q_rad[joint_slice] - return_target)
                    )
                    command_error = np.max(
                        np.abs(self.limiter.last_command[joint_slice] - return_target)
                    )
                    max_speed = np.max(np.abs(state.dq_rad_s[joint_slice]))
                    if (
                        progress >= 1.0
                        and position_error <= self._return_completion_tolerance_rad
                        and command_error <= self._return_completion_tolerance_rad
                        and max_speed <= self.startup_max_joint_speed_rad_s
                    ):
                        self._return_trajectories[arm_index] = None
                        self._arm_hold_targets[arm_index] = return_target.copy()
        self._previous_active_arms = active_arms
        self.limiter.hold(target, hold_mask)
        target = self.limiter.limit(
            target,
            state.q_rad,
            state.dq_rad_s,
            target_guard_mask=~hold_mask,
        )
        returning_arms = tuple(
            trajectory is not None for trajectory in self._return_trajectories
        )
        self._command_sequence += 1
        self._command_slot.set(
            MarvinJointCommand(
                self._command_sequence,
                time.monotonic_ns(),
                target,
                active_arms,
                returning_arms,
            )
        )

    def _pre_solve_ik(self):
        """Reject XR jumps and rate-limit TCP targets before Placo sees them."""
        for name, config in self.manipulator_config.items():
            task = self.effector_task[name]
            actual = self.placo_robot.get_T_world_frame(config["link_name"])
            active = bool(self.active.get(name, False))
            if not active:
                self._target_guards[name].reset(actual)
                if self.effector_control_mode[name] == "position":
                    task.target_world = actual[:3, 3]
                else:
                    task.T_world_frame = actual
                self._guard_previous_active[name] = False
                self._raw_tcp_targets[name] = actual[:3, 3].copy()
                self._limited_tcp_targets[name] = actual[:3, 3].copy()
                self._raw_tcp_transforms[name] = actual.copy()
                self._limited_tcp_transforms[name] = actual.copy()
                continue

            if self.effector_control_mode[name] == "position":
                raw = actual.copy()
                raw[:3, 3] = task.target_world
            else:
                raw = task.T_world_frame.copy()
            if not self._guard_previous_active[name]:
                self._target_guards[name].reset(actual)
                self._guard_previous_active[name] = True

            arm_suffix = "L" if name == "left_hand" else "R"
            velocity_offsets = [
                self.placo_robot.get_joint_v_offset(f"Joint{index}_{arm_suffix}")
                for index in range(1, 8)
            ]
            jacobian = self.placo_robot.frame_jacobian(
                config["link_name"], "local_world_aligned"
            )[:3, velocity_offsets]
            minimum_singular_value = float(np.linalg.svd(jacobian, compute_uv=False)[-1])
            self._singular_values[name] = minimum_singular_value
            fault_sigma = self._tcp_guard_config["singularity_fault_sigma"]
            full_speed_sigma = self._tcp_guard_config["singularity_full_speed_sigma"]
            if minimum_singular_value <= fault_sigma:
                raise RuntimeError(
                    f"{name} translational Jacobian is singular "
                    f"(sigma_min={minimum_singular_value:.5f})"
                )
            speed_scale = min(
                1.0,
                max(
                    0.1,
                    (minimum_singular_value - fault_sigma)
                    / (full_speed_sigma - fault_sigma),
                ),
            )
            limited = self._target_guards[name].filter(raw, speed_scale=speed_scale)
            if self.effector_control_mode[name] == "position":
                task.target_world = limited[:3, 3]
            else:
                task.T_world_frame = limited
            self._raw_tcp_targets[name] = raw[:3, 3].copy()
            self._limited_tcp_targets[name] = limited[:3, 3].copy()
            self._raw_tcp_transforms[name] = raw.copy()
            self._limited_tcp_transforms[name] = limited.copy()

    def _verify_parameter_configuration(self):
        raw = self.adapter.last_raw_state
        for index, arm in enumerate(("A", "B")):
            inputs = raw["inputs"][index]
            if int(inputs["joint_vel_ratio"]) != self.velocity_ratio:
                raise RuntimeError(f"arm {arm} velocity ratio readback mismatch")
            if int(inputs["joint_acc_ratio"]) != self.acceleration_ratio:
                raise RuntimeError(f"arm {arm} acceleration ratio readback mismatch")
            expected_k = self.left_k if index == 0 else self.right_k
            expected_d = self.left_d if index == 0 else self.right_d
            np.testing.assert_allclose(inputs["joint_k"], expected_k, rtol=0.0, atol=1e-6)
            np.testing.assert_allclose(inputs["joint_d"], expected_d, rtol=0.0, atol=1e-6)
        if self.configure_tools:
            for index, expected in enumerate((self.left_tool, self.right_tool)):
                if expected is None:
                    continue
                inputs = raw["inputs"][index]
                np.testing.assert_allclose(
                    inputs["tool_kine"], expected.kinematics_mm_deg, rtol=0.0, atol=1e-6
                )
                np.testing.assert_allclose(
                    inputs["tool_dyn"], expected.dynamics_vendor_units, rtol=0.0, atol=1e-6
                )

    def _verify_configuration(self, state):
        self._verify_parameter_configuration()
        raw = self.adapter.last_raw_state
        if state.arm_state != (3, 3):
            raise RuntimeError(f"joint impedance state readback mismatch: {state.arm_state}")
        impedance = tuple(int(item["imp_type"]) for item in raw["inputs"][:2])
        if impedance != (1, 1):
            raise RuntimeError(f"joint impedance type readback mismatch: {impedance}")

    def _require_matching_tool_or_opt_in(self):
        if self.configure_tools:
            return
        raw = self.adapter.last_raw_state
        for index, expected in enumerate((self.left_tool, self.right_tool)):
            if expected is None:
                continue
            actual_kine = np.asarray(raw["inputs"][index]["tool_kine"], dtype=float)
            actual_dyn = np.asarray(raw["inputs"][index]["tool_dyn"], dtype=float)
            if not (
                np.allclose(actual_kine, expected.kinematics_mm_deg, rtol=0.0, atol=1e-6)
                and np.allclose(actual_dyn, expected.dynamics_vendor_units, rtol=0.0, atol=1e-6)
            ):
                arm = "A/left" if index == 0 else "B/right"
                raise RuntimeError(
                    f"{arm} active Tool does not match tools_cfg.json; verify it in "
                    "MarvinPlatform or explicitly opt in to configure_tools"
                )

    def _require_stationary_healthy(self, state, phase):
        max_speed = self._require_healthy_within_startup_speed(state, phase)
        if not all(state.low_speed):
            raise RuntimeError(
                f"Marvin must be stationary during {phase}: low_speed={state.low_speed}, "
                f"max_joint_speed={max_speed:.4f} rad/s"
            )

    def _require_healthy_within_startup_speed(self, state, phase):
        """Reject controller faults and motion beyond the bounded startup envelope."""
        if any(state.error_code) or any(value == 100 for value in state.arm_state):
            raise RuntimeError(
                f"Marvin is unhealthy during {phase}: states={state.arm_state}, "
                f"errors={state.error_code}"
            )
        max_speed = float(np.max(np.abs(state.dq_rad_s)))
        if max_speed > self.startup_max_joint_speed_rad_s:
            raise RuntimeError(
                f"Marvin exceeds the startup motion bound during {phase}: "
                f"low_speed={state.low_speed}, max_joint_speed={max_speed:.4f} rad/s, "
                f"limit={self.startup_max_joint_speed_rad_s:.4f} rad/s"
            )
        return max_speed

    def _wait_stationary_healthy_for(self, duration_s, phase):
        """Observe feedback throughout a vendor-required settling interval."""
        deadline = time.monotonic() + duration_s
        while True:
            state = self.adapter.read_state()
            self._robot_state_slot.set(state)
            self._require_stationary_healthy(state, phase)
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return state
            time.sleep(min(0.02, remaining))

    def _move_to_configured_initial_pose(self, state):
        if self.initial_pose_q_rad is None:
            return state
        pressed = [
            self.xr_client.get_key_value_by_name(config["control_trigger"])
            for config in self.manipulator_config.values()
        ]
        if any(value > 0.1 for value in pressed):
            raise RuntimeError(
                "release both Grip controls before the startup move to the initial pose"
            )

        start = state.q_rad.copy()
        target = self.initial_pose_q_rad
        distance = np.abs(target - start)
        duration_s = self.startup_move_duration_s
        if (
            float(np.max(distance)) <= self.startup_pose_tolerance_rad
            and float(np.max(np.abs(state.dq_rad_s)))
            <= self.startup_max_joint_speed_rad_s
            and all(state.low_speed)
        ):
            return state

        # A quintic smoothstep has zero velocity and acceleration at both ends.
        # Validate its continuous extrema before sending any motion command so an
        # arbitrary detected pose is accepted only when the requested three-second
        # move is physically compatible with the configured robot limits.
        peak_velocity = 1.875 * distance / duration_s
        peak_acceleration = (10.0 / np.sqrt(3.0)) * distance / duration_s**2
        peak_jerk = 60.0 * distance / duration_s**3
        infeasible = (
            (peak_velocity > self.startup_motion_max_velocity + 1e-12)
            | (peak_acceleration > self.startup_motion_max_acceleration + 1e-12)
            | (peak_jerk > self.startup_motion_max_jerk + 1e-12)
        )
        if np.any(infeasible):
            joints = np.flatnonzero(infeasible).tolist()
            raise RuntimeError(
                f"detected startup pose cannot reach natural rest safely within "
                f"{duration_s:g} seconds at joint indices {joints}; "
                f"start_deg={np.rad2deg(start).round(2).tolist()}, "
                f"target_deg={np.rad2deg(target).round(2).tolist()}, "
                f"peak_velocity_rad_s={peak_velocity.round(3).tolist()}"
            )

        self.limiter.reset(start)
        started = time.monotonic()
        deadline = started + duration_s
        next_cycle = time.monotonic()
        if self._logger is not None:
            self._logger.record(
                "startup_pose_move_started",
                start_q_rad=start,
                target_q_rad=target,
                duration_s=duration_s,
                peak_velocity_rad_s=peak_velocity,
                peak_acceleration_rad_s2=peak_acceleration,
                peak_jerk_rad_s3=peak_jerk,
            )

        while True:
            state = self.adapter.read_state()
            self._robot_state_slot.set(state)
            if any(state.error_code) or state.arm_state != (3, 3):
                raise RuntimeError(
                    "Marvin became unhealthy during startup pose motion: "
                    f"states={state.arm_state}, errors={state.error_code}"
                )
            if state.age_ms() > self.safety.config.feedback_fault_ms:
                raise RuntimeError(
                    f"Marvin feedback stalled for {state.age_ms():.1f} ms during "
                    "startup pose motion"
                )

            now = time.monotonic()
            progress = min(1.0, max(0.0, (now - started) / duration_s))
            blend = 10.0 * progress**3 - 15.0 * progress**4 + 6.0 * progress**5
            command = start + blend * (target - start)
            tracking_error = float(np.max(np.abs(command - state.q_rad)))
            if tracking_error > self.safety.config.tracking_error_fault_rad:
                raise RuntimeError(
                    "startup pose tracking error exceeded the FAULT threshold: "
                    f"{np.rad2deg(tracking_error):.2f} deg"
                )
            self.adapter.send_joint_command(command)
            if progress >= 1.0:
                break
            next_cycle += 1.0 / self.control_hz
            delay = next_cycle - time.monotonic()
            if delay > 0.0:
                time.sleep(delay)
            else:
                next_cycle = time.monotonic()

        # Read one fresh frame after the final target. A robot that cannot follow
        # the prevalidated trajectory within tolerance is not armed.
        state = self.adapter.wait_for_fresh_feedback(timeout=0.1, required_updates=1)
        self._robot_state_slot.set(state)
        position_error = float(np.max(np.abs(target - state.q_rad)))
        max_speed = float(np.max(np.abs(state.dq_rad_s)))
        if (
            position_error > self.startup_pose_tolerance_rad
            or max_speed > self.startup_max_joint_speed_rad_s
            or not all(state.low_speed)
        ):
            raise TimeoutError(
                f"Marvin did not reach natural rest within {duration_s:g} seconds; "
                f"maximum_error_deg={np.rad2deg(position_error):.2f}, "
                f"max_joint_speed_rad_s={max_speed:.4f}, low_speed={state.low_speed}"
            )
        if self._logger is not None:
            self._logger.record(
                "startup_pose_move_completed",
                target_q_rad=target,
                feedback_q_rad=state.q_rad,
                maximum_error_rad=position_error,
                duration_s=duration_s,
            )
        return state

    def _adopt_feedback_as_start(self, state):
        self._robot_state_slot.set(state)
        self._arm_hold_targets[0] = state.q_rad[:7].copy()
        self._arm_hold_targets[1] = state.q_rad[7:].copy()
        return_target = (
            state.q_rad if self.initial_pose_q_rad is None else self.initial_pose_q_rad
        )
        self._return_joint_targets[0] = return_target[:7].copy()
        self._return_joint_targets[1] = return_target[7:].copy()
        self._return_trajectories = [None, None]
        self._previous_active_arms = (False, False)
        # The startup hold command is stationary even when measured dq contains
        # small estimator noise. Keep measured dq only for predictive braking.
        self.limiter.reset(state.q_rad)
        self._update_robot_state()
        self.sync_end_effector_poses_to_placo_tasks()
        for name, config in self.manipulator_config.items():
            actual = self.placo_robot.get_T_world_frame(config["link_name"])
            self._target_guards[name].rebase(actual)
            self._guard_previous_active[name] = False

    def _scale_calibration_sample_rejection_reason(self):
        if any(trajectory is not None for trajectory in self._return_trajectories):
            return "wait for both arms to finish automatic reset"
        state = self._robot_state_slot.get()
        if state is None:
            return "no robot feedback"
        max_speed = float(np.max(np.abs(state.dq_rad_s)))
        if not all(state.low_speed) or max_speed > self.startup_max_joint_speed_rad_s:
            return "wait until both arms are stationary"
        return None

    def _on_scale_calibration_completed(self, result):
        old_scale_factor = self.scale_factor
        current_path, history_path = save_scale_calibration(
            self.scale_calibration_path,
            result,
            self.calibration_workspace_margin,
        )
        # The calibration gate guarantees that both Grips are released, both
        # return trajectories have completed, and feedback is stationary. Apply
        # only in that safe window, then force the next Grip engagement to latch
        # fresh controller/TCP references so changing scale cannot move a target.
        self.scale_factor = float(result.scale_factor)
        for name in self.manipulator_config:
            self.ref_ee_xyz[name] = None
            self.ref_ee_quat[name] = None
            self.ref_controller_xyz[name] = None
            self.ref_controller_quat[name] = None
        if self._logger is not None:
            self._logger.record(
                "scale_calibration_saved",
                old_scale_factor=old_scale_factor,
                scale_factor=result.scale_factor,
                unclamped_scale_factor=result.unclamped_scale_factor,
                current_path=str(current_path),
                history_path=str(history_path),
                apply_policy="safe_idle_immediate_next_grip",
            )
        print(
            f"Saved and applied scale_factor: {old_scale_factor:.6f} -> "
            f"{self.scale_factor:.6f} ({current_path})"
        )
        print("Press Grip to latch a fresh zero and teleoperate with the new scale.")

    def _configure_hardware(self):
        state = self.adapter.wait_for_fresh_feedback(timeout=1.0, required_updates=2)
        self._require_stationary_healthy(state, "pre-enable validation")
        self._adopt_feedback_as_start(state)
        self._require_matching_tool_or_opt_in()
        try:
            self.adapter.configure_parameters(
                self.velocity_ratio,
                self.acceleration_ratio,
                self.left_k,
                self.left_d,
                self.right_k,
                self.right_d,
                self.left_tool if self.configure_tools else None,
                self.right_tool if self.configure_tools else None,
            )
        except TimeoutError as response_timeout:
            if "parameter configuration response timed out" not in str(response_timeout):
                raise
            # The settings are idempotent and are exposed in controller feedback.
            # An acknowledgement packet can be lost after they were applied, so
            # reconcile against fresh authoritative readback instead of resending.
            state = self.adapter.wait_for_fresh_feedback(timeout=1.0, required_updates=2)
            self._require_stationary_healthy(state, "parameter response reconciliation")
            try:
                self._verify_parameter_configuration()
            except (AssertionError, KeyError, TypeError, ValueError, RuntimeError):
                raise TimeoutError(
                    "Marvin parameter configuration response timed out and fresh "
                    "controller readback does not match the requested settings"
                ) from response_timeout
            print(
                "WARNING: Marvin parameter response timed out, but fresh controller "
                "readback matches all requested settings; continuing without resend."
            )
        state = self._wait_stationary_healthy_for(
            self.parameter_settle_s, "parameter settling"
        )
        self._verify_parameter_configuration()
        self._adopt_feedback_as_start(state)
        # Set the target before entering torque/impedance mode, preventing a
        # transition toward an old controller-side joint target.
        # Startup is not a real-time path: wait until the feedback-equal hold
        # target is acknowledged before opening the next SDK transaction for
        # the impedance-mode switch. Periodic commands stay non-blocking.
        self.adapter.send_joint_command(state.q_rad, wait_response=True)
        transition_started = time.monotonic()
        self.adapter.enter_joint_impedance()
        # Entering torque/impedance mode can briefly clear the vendor's
        # low-speed flag while gravity compensation settles. Permit only that
        # bounded transient, then require a continuous stationary dwell before
        # enabling PD feedforward. Any controller fault or speed above the
        # startup envelope still fails immediately.
        transition_timeout_s = max(5.0, self.mode_settle_s + 2.0)
        deadline = transition_started + transition_timeout_s
        stationary_since = None
        while time.monotonic() < deadline:
            state = self.adapter.read_state()
            self._robot_state_slot.set(state)
            self._require_healthy_within_startup_speed(
                state, "joint impedance settling"
            )
            now = time.monotonic()
            if state.arm_state == (3, 3) and all(state.low_speed):
                if stationary_since is None:
                    stationary_since = now
                if now - stationary_since >= self.mode_settle_s:
                    self._verify_configuration(state)
                    break
            else:
                stationary_since = None
            time.sleep(0.02)
        else:
            raise TimeoutError(
                "Marvin did not enter joint impedance mode and remain stationary "
                f"for {self.mode_settle_s:g} seconds within the "
                f"{transition_timeout_s:g}-second startup timeout; "
                f"last_state={state.arm_state}, low_speed={state.low_speed}, "
                f"max_joint_speed={float(np.max(np.abs(state.dq_rad_s))):.4f} rad/s"
            )

        self.adapter.enable_pd_feedforward(self.pd_period_ms)
        state = self._wait_stationary_healthy_for(
            self.pd_settle_s, "PD feedforward settling"
        )
        self._verify_configuration(state)
        state = self._move_to_configured_initial_pose(state)
        self._adopt_feedback_as_start(state)
        self.safety.arm()

    def _log_safety_transitions(self, previous_count):
        transitions = self.safety.transitions[previous_count:]
        for transition in transitions:
            self._logger.record("safety_transition", **transition)
        return previous_count + len(transitions)

    def _io_loop(self):
        feedback_period = 1.0 / self.feedback_hz
        command_period = 1.0 / self.command_hz
        next_feedback = time.monotonic()
        next_command = next_feedback
        transition_count = 0
        idle_sent = False
        last_sent_sequence = None
        try:
            while not self._stop_event.is_set():
                read_start_ns = time.monotonic_ns()
                state = self.adapter.read_state()
                read_duration_ms = (time.monotonic_ns() - read_start_ns) / 1e6
                self._robot_state_slot.set(state)
                self._logger.record(
                    "robot_state",
                    sdk_read_duration_ms=read_duration_ms,
                    frame_serial=state.frame_serial,
                    q_rad=state.q_rad,
                    dq_rad_s=state.dq_rad_s,
                    torque_nm=state.torque_nm,
                    controller_commanded_q_rad=state.commanded_q_rad,
                    arm_state=state.arm_state,
                    error_code=state.error_code,
                    low_speed=state.low_speed,
                    frame_miss_count=state.frame_miss_count,
                    system_cycle_miss_count=state.system_cycle_miss_count,
                )

                xr = self.xr_client.get_diagnostics()
                command = self._command_slot.get()
                control_observation = self._control_observation_slot.get()
                now = time.monotonic()
                stop_after_observation = False
                if now >= next_command:
                    external_fault = self._control_fault_slot.get()
                    if external_fault:
                        self.safety.fault(external_fault)
                    decision = self.safety.evaluate(
                        state,
                        command,
                        xr["source_age_ms"],
                    )
                    if (
                        not decision.request_idle
                        and command is not None
                        and last_sent_sequence is not None
                    ):
                        if command.sequence < last_sent_sequence:
                            decision = self.safety.fault(
                                f"command sequence regressed from {last_sent_sequence} "
                                f"to {command.sequence}"
                            )
                        # At equal 200 Hz producer/consumer rates, thread jitter can
                        # legitimately expose the same latest-value command twice.
                        # Its monotonic age is already guarded by command_validity_ms;
                        # resend it for the vendor's 5 ms PD stream continuity.
                    transition_count = self._log_safety_transitions(transition_count)
                    self._logger.record(
                        "hardware_command_decision",
                        safety_state=decision.state.value,
                        reason=decision.reason,
                        command_sequence=None if command is None else command.sequence,
                        returning_arms=(
                            None if command is None else command.returning_arms
                        ),
                        command_age_ms=None if command is None else command.age_ms(),
                        feedback_age_ms=state.age_ms(),
                        xr_source_age_ms=xr["source_age_ms"],
                    )
                    if decision.request_idle:
                        self.adapter.set_idle(wait_response=False)
                        idle_sent = True
                        self._stop_event.set()
                        stop_after_observation = True
                    elif decision.send_command:
                        if decision.use_feedback_hold or command is None:
                            if self._io_hold_target is None:
                                self._io_hold_target = state.q_rad.copy()
                            q_command = self._io_hold_target
                        else:
                            self._io_hold_target = None
                            q_command = command.q_rad
                        send_start_ns = time.monotonic_ns()
                        self.adapter.send_joint_command(q_command)
                        if not decision.use_feedback_hold and command is not None:
                            last_sent_sequence = command.sequence
                        self._logger.record(
                            "hardware_command_sent",
                            duration_ms=(time.monotonic_ns() - send_start_ns) / 1e6,
                            q_command_rad=q_command,
                            command_sequence=None if command is None else command.sequence,
                            safety_state=decision.state.value,
                        )
                    next_command += command_period
                    if next_command < now - command_period:
                        next_command = now + command_period

                if self._calibration_recorder is not None:
                    self._calibration_recorder.record(
                        state=state,
                        command=command,
                        control=control_observation,
                        safety_state=self.safety.state.value,
                        safety_reason=self.safety.reason,
                        sdk_read_duration_ms=read_duration_ms,
                        scale_factor=self.scale_factor,
                    )
                if self._ros2_observer is not None:
                    self._ros2_observer.update_robot_state(state)
                    self._ros2_observer.update_safety(
                        self.safety.state.value,
                        self.safety.reason,
                    )
                    self._ros2_observer.update_diagnostics(
                        {
                            "safety_state": self.safety.state.value,
                            "safety_reason": self.safety.reason,
                            "frame_serial_A": state.frame_serial[0],
                            "frame_serial_B": state.frame_serial[1],
                            "feedback_age_ms": state.age_ms(),
                            "xr_source_age_ms": xr["source_age_ms"],
                            "command_age_ms": (
                                "" if command is None else command.age_ms()
                            ),
                            "sdk_read_duration_ms": read_duration_ms,
                            "error_code_A": state.error_code[0],
                            "error_code_B": state.error_code[1],
                        }
                    )
                if stop_after_observation:
                    break

                next_feedback += feedback_period
                delay = next_feedback - time.monotonic()
                if delay > 0.0:
                    self._stop_event.wait(delay)
                else:
                    next_feedback = time.monotonic()
        except Exception as error:
            self._thread_errors.set(error)
            self._logger.record("io_exception", error=repr(error))
            self._stop_event.set()
        finally:
            if not idle_sent:
                try:
                    self.adapter.set_idle(wait_response=False)
                except Exception as error:
                    self._logger.record("idle_exception", error=repr(error))

    def _control_loop(self):
        period = 1.0 / self.control_hz
        next_deadline = time.monotonic()
        try:
            while not self._stop_event.is_set():
                cycle_start = time.monotonic_ns()
                deadline_lateness_ms = max(
                    0.0,
                    (time.monotonic() - next_deadline) * 1000.0,
                )
                begin_cycle = getattr(self.xr_client, "begin_cycle", None)
                end_cycle = getattr(self.xr_client, "end_cycle", None)
                if begin_cycle is not None:
                    begin_cycle()
                try:
                    self._update_ik()
                    if self._last_ik_success:
                        self._ik_failure_count = 0
                    else:
                        self._ik_failure_count += 1
                    if self._ik_failure_count >= 3:
                        self._control_fault_slot.set("three consecutive IK failures")
                        self._stop_event.wait(period)
                        continue
                    self._send_command()
                    command = self._command_slot.get()
                    state = self._robot_state_slot.get()
                    q_ik = self.placo_robot.state.q[self._joint_offsets].copy()
                    actual_tcp_transforms = self._actual_tcp_transforms(state)
                    xr = self.xr_client.get_diagnostics()
                    arm_names = ("left_hand", "right_hand")
                    control_duration_ms = (time.monotonic_ns() - cycle_start) / 1e6
                    control_observation = MarvinControlObservation(
                        sequence=command.sequence,
                        monotonic_ns=time.monotonic_ns(),
                        duration_ms=control_duration_ms,
                        deadline_lateness_ms=deadline_lateness_ms,
                        deadline_miss=deadline_lateness_ms > period * 1000.0,
                        xr_sequence=xr["sequence"],
                        xr_source_timestamp_ns=xr["source_timestamp_ns"],
                        xr_poll_age_ms=xr["poll_age_ms"],
                        xr_source_age_ms=xr["source_age_ms"],
                        q_ik_rad=q_ik,
                        q_command_rad=command.q_rad,
                        active_arms=command.active_arms,
                        raw_tcp_transforms=np.stack(
                            [self._raw_tcp_transforms[name] for name in arm_names]
                        ),
                        limited_tcp_transforms=np.stack(
                            [self._limited_tcp_transforms[name] for name in arm_names]
                        ),
                        actual_tcp_transforms=np.stack(
                            [actual_tcp_transforms[name] for name in arm_names]
                        ),
                        translational_sigma_min=np.asarray(
                            [self._singular_values.get(name, np.nan) for name in arm_names]
                        ),
                    )
                    self._control_observation_slot.set(control_observation)
                    if self._ros2_observer is not None:
                        self._ros2_observer.update_control(control_observation)
                    self._logger.record(
                        "control_cycle",
                        duration_ms=control_duration_ms,
                        deadline_lateness_ms=deadline_lateness_ms,
                        deadline_miss=deadline_lateness_ms > period * 1000.0,
                        xr=xr,
                        raw_tcp_targets=self._raw_tcp_targets,
                        limited_tcp_targets=self._limited_tcp_targets,
                        raw_tcp_transforms=self._raw_tcp_transforms,
                        limited_tcp_transforms=self._limited_tcp_transforms,
                        actual_tcp_transforms=actual_tcp_transforms,
                        actual_tcp_positions={
                            name: transform[:3, 3]
                            for name, transform in actual_tcp_transforms.items()
                        },
                        translational_sigma_min=self._singular_values,
                        q_ik_rad=q_ik,
                        q_command_rad=command.q_rad,
                        command_sequence=command.sequence,
                        active_arms=command.active_arms,
                        returning_arms=command.returning_arms,
                        scale_factor=self.scale_factor,
                    )
                finally:
                    if end_cycle is not None:
                        end_cycle()

                if self.visualize_placo:
                    self._update_placo_viz()
                next_deadline += period
                delay = next_deadline - time.monotonic()
                if delay > 0.0:
                    self._stop_event.wait(delay)
                elif delay < -period:
                    next_deadline = time.monotonic()
        except Exception as error:
            self._thread_errors.set(error)
            self._control_fault_slot.set(f"control loop failed: {error!r}")
            self._logger.record("control_exception", error=repr(error))
            self._stop_event.set()

    def run(self):
        if not self.enable_hardware:
            self.adapter.release()
            self.xr_client.close()
            raise PermissionError("hardware motion requires explicit enable_hardware=True")
        metadata = {
            "robot_ip": self.adapter.robot_ip,
            "sdk_version": self.adapter.sdk_version(),
            "expected_sdk_version": self.expected_sdk_version,
            "robot_name_reported_by_sdk": self.adapter.robot_name(),
            "joint_names": self.joint_names,
            "scale_factor": self.scale_factor,
            "release_return": {
                "enabled": self.enable_release_return,
                "target": (
                    "measured_startup_joint_pose"
                    if self.initial_pose_q_rad is None
                    else "configured_initial_joint_pose"
                ),
                "nominal_duration_s": self.return_duration,
                "completion_tolerance_rad": self._return_completion_tolerance_rad,
            },
            "initial_pose": {
                "q_rad": self.initial_pose_q_rad,
                "startup_move_enabled": self.initial_pose_q_rad is not None,
                "source": "detected_feedback_pose",
                "duration_s": self.startup_move_duration_s,
                "completion_tolerance_rad": self.startup_pose_tolerance_rad,
                "max_velocity_rad_s": self.startup_motion_max_velocity,
                "max_acceleration_rad_s2": self.startup_motion_max_acceleration,
                "max_jerk_rad_s3": self.startup_motion_max_jerk,
            },
            "runtime_scale_calibration": {
                "enabled": self.enable_arm_length_calibration,
                "path": self.scale_calibration_path,
                "workspace_margin": self.calibration_workspace_margin,
                "apply_policy": "safe_idle_immediate_next_grip",
            },
            "control_hz": self.control_hz,
            "feedback_hz": self.feedback_hz,
            "command_hz": self.command_hz,
            "xr_poll_hz": self.xr_poll_hz,
            "pd_period_ms": self.pd_period_ms,
            "startup_settle_s": {
                "parameters": self.parameter_settle_s,
                "joint_impedance": self.mode_settle_s,
                "pd_feedforward": self.pd_settle_s,
            },
            "velocity_ratio": self.velocity_ratio,
            "acceleration_ratio": self.acceleration_ratio,
            "left_k": self.left_k,
            "left_d": self.left_d,
            "right_k": self.right_k,
            "right_d": self.right_d,
            "configure_tools": self.configure_tools,
            "left_tool": (
                None
                if self.left_tool is None
                else {
                    "kinematics_mm_deg": self.left_tool.kinematics_mm_deg,
                    "dynamics_vendor_units": self.left_tool.dynamics_vendor_units,
                }
            ),
            "right_tool": (
                None
                if self.right_tool is None
                else {
                    "kinematics_mm_deg": self.right_tool.kinematics_mm_deg,
                    "dynamics_vendor_units": self.right_tool.dynamics_vendor_units,
                }
            ),
            "startup_max_joint_speed_rad_s": self.startup_max_joint_speed_rad_s,
            "max_joint_velocity_rad_s": self.limiter.max_velocity,
            "max_joint_acceleration_rad_s2": self.limiter.max_acceleration,
            "max_joint_jerk_rad_s3": self.limiter.max_jerk,
            "target_braking_guard_rad": self.limiter.target_braking_guard,
            "jerk_braking_extra_distance_at_max_velocity_rad": (
                self.limiter.jerk_braking_extra_distance_at_max_velocity
            ),
            "joint_target_natural_frequency_rad_s": (
                self.limiter.target_natural_frequency
            ),
            "joint_limit_margin_rad": self._joint_limit_margin,
            "safety": vars(self.safety.config),
            "tcp_guard": self._tcp_guard_config,
            "ros2_observation": {
                "enabled": self.enable_ros2_observation,
                "namespace": self.ros2_namespace,
                "publish_hz": self.ros2_publish_hz,
                "control_authority": False,
            },
            "calibration_recording": {
                "enabled": True,
                "feedback_sample_hz": self.feedback_hz,
                "format": "csv+metadata.json",
            },
            **self.session_metadata,
        }
        threads = []
        ros2_error_reported = False
        calibration_error_reported = False
        try:
            self._logger = MarvinSessionLogger(self.log_dir, metadata)
            self._calibration_recorder = MarvinCalibrationRecorder(
                self.log_dir,
                metadata,
                self.joint_names,
            )
            if self.enable_ros2_observation:
                self._ros2_observer = MarvinRos2Observer(
                    self.joint_names,
                    namespace=self.ros2_namespace,
                    publish_hz=self.ros2_publish_hz,
                )
                self._ros2_observer.start()
            self.xr_client.start()
            if not self.xr_client.wait_until_ready(timeout=2.0):
                raise RuntimeError("XR worker produced no valid snapshot within 2 seconds")
            self._configure_hardware()
            threads = [
                threading.Thread(target=self._io_loop, name="marvin-sdk-io", daemon=True),
                threading.Thread(target=self._control_loop, name="marvin-control", daemon=True),
            ]
            for thread in threads:
                thread.start()
            print(
                "Marvin hardware teleoperation armed. Grip enables each arm; release returns "
                "that arm to the configured natural-rest pose. A/A calibrates scale, B resets; "
                "a completed calibration is saved and applies on the next Grip engagement."
            )
            while not self._stop_event.wait(0.1):
                if (
                    self._ros2_observer is not None
                    and self._ros2_observer.error is not None
                    and not ros2_error_reported
                ):
                    self._logger.record(
                        "ros2_observer_exception",
                        error=repr(self._ros2_observer.error),
                        control_action="none; ROS 2 is observation-only",
                    )
                    print(f"ROS 2 observer stopped: {self._ros2_observer.error!r}")
                    ros2_error_reported = True
                if (
                    self._calibration_recorder is not None
                    and self._calibration_recorder.error is not None
                    and not calibration_error_reported
                ):
                    self._logger.record(
                        "calibration_recorder_exception",
                        error=repr(self._calibration_recorder.error),
                        control_action="none; session is invalid for calibration",
                    )
                    print(
                        "Calibration recorder stopped; this session is invalid: "
                        f"{self._calibration_recorder.error!r}"
                    )
                    calibration_error_reported = True
                if any(not thread.is_alive() for thread in threads):
                    self._stop_event.set()
                    break
        except KeyboardInterrupt:
            print("\nMarvin teleoperation stop requested.")
        finally:
            self._stop_event.set()
            for thread in reversed(threads):
                thread.join(timeout=5.0)
            terminal_state = {
                "state_before_shutdown": self.safety.state.value,
                "reason_before_shutdown": self.safety.reason,
                "final_scale_factor": self.scale_factor,
            }
            self.safety.shutdown()
            live_threads = [thread.name for thread in threads if thread.is_alive()]
            if live_threads:
                if self._logger is not None:
                    self._logger.record(
                        "shutdown_incomplete",
                        live_threads=live_threads,
                        action="use physical emergency stop; SDK ownership thread did not exit",
                    )
            else:
                try:
                    self.adapter.set_idle(wait_response=True)
                    deadline = time.monotonic() + 1.0
                    while time.monotonic() < deadline:
                        state = self.adapter.read_state()
                        if state.arm_state == (0, 0):
                            break
                        time.sleep(0.02)
                except Exception as error:
                    if self._logger is not None:
                        self._logger.record("shutdown_exception", error=repr(error))
                finally:
                    self.adapter.release()
            self.xr_client.close()
            if self._ros2_observer is not None:
                self._ros2_observer.close()
            if self._calibration_recorder is not None:
                self._calibration_recorder.close(terminal_state=terminal_state)
                print(f"Calibration CSV: {self._calibration_recorder.csv_path.resolve()}")
                print(
                    "Calibration metadata: "
                    f"{self._calibration_recorder.metadata_path.resolve()}"
                )
            if self._logger is not None:
                self._logger.close(final_state=terminal_state)
                print(f"Hardware log: {self._logger.events_path.resolve()}")
            if live_threads:
                raise RuntimeError(
                    "hardware threads did not stop; use the physical emergency stop: "
                    + ", ".join(live_threads)
                )
        thread_error = self._thread_errors.get()
        if thread_error is not None:
            raise RuntimeError("Marvin hardware teleoperation thread failed") from thread_error
