"""Minimal synchronous XR-to-Marvin teleoperation controller."""

import time

import numpy as np

from xr_marvin_teleop.common.marvin_scale_calibration import (
    ArmLengthScaleCalibrator,
    resolve_scale_factor,
    save_scale_calibration,
)
from xr_marvin_teleop.common.marvin_postures import MARVIN_INITIAL_POSE_Q_RAD
from xr_marvin_teleop.common.xr_target_mapper import (
    XrTargetMapper,
    transform_controller_poses_to_marvin_frame,
)


DEFAULT_JOINT_K = (5.0, 5.0, 5.0, 4.0, 4.0, 3.0, 3.0)
DEFAULT_JOINT_D = (0.4, 0.4, 0.4, 0.4, 0.3, 0.3, 0.3)
DEFAULT_CONTROL_HZ = 50
DEFAULT_JOINT_VELOCITY_RATIO = 10
DEFAULT_JOINT_ACCELERATION_RATIO = 10
MAX_CONSECUTIVE_STALE_FEEDBACK_CYCLES = 3
DEFAULT_GRIPPER_RATE = 0.5
DEFAULT_GRIPPER_COMMAND_HZ = 20.0


class MarvinHardwareTeleopController:
    def __init__(
        self,
        xr_client,
        adapter,
        kinematics,
        scale_calibration_path,
        initial_pose_q_rad=MARVIN_INITIAL_POSE_Q_RAD,
        tool_configurations=None,
        left_k=DEFAULT_JOINT_K,
        left_d=DEFAULT_JOINT_D,
        right_k=DEFAULT_JOINT_K,
        right_d=DEFAULT_JOINT_D,
        joint_velocity_ratio=DEFAULT_JOINT_VELOCITY_RATIO,
        joint_acceleration_ratio=DEFAULT_JOINT_ACCELERATION_RATIO,
        requested_scale_factor=None,
        control_hz=DEFAULT_CONTROL_HZ,
        return_duration=3.0,
        grip_activation_threshold=0.9,
        expected_sdk_version=None,
        control_parameter_settle_seconds=0.2,
        mode_settle_seconds=1.0,
        pd_settle_seconds=1.0,
        session_logger=None,
        gripper_control_enabled=False,
        initial_gripper_closedness=(0.0, 0.0),
        gripper_rate=DEFAULT_GRIPPER_RATE,
        gripper_command_hz=DEFAULT_GRIPPER_COMMAND_HZ,
        trigger_deadzone=0.08,
        thumbstick_deadzone=0.20,
        thumbstick_y_sign=1.0,
    ):
        control_hz = float(control_hz)
        if not 50.0 <= control_hz <= 200.0:
            raise ValueError("control_hz must be within [50, 200]")
        pd_period_milliseconds = 1000.0 / control_hz
        if not np.isclose(pd_period_milliseconds, round(pd_period_milliseconds)):
            raise ValueError("control_hz must produce an integer PD period")
        if return_duration <= 0.0:
            raise ValueError("return_duration must be positive")
        if not 0.0 < grip_activation_threshold <= 1.0:
            raise ValueError("grip_activation_threshold must be within (0, 1]")
        if (
            control_parameter_settle_seconds < 0.0
            or mode_settle_seconds < 0.0
            or pd_settle_seconds < 0.0
        ):
            raise ValueError("settle durations must be non-negative")
        gripper_rate = float(gripper_rate)
        gripper_command_hz = float(gripper_command_hz)
        if not np.isfinite(gripper_rate) or gripper_rate <= 0.0:
            raise ValueError("gripper_rate must be positive")
        if (
            not np.isfinite(gripper_command_hz)
            or not 0.0 < gripper_command_hz <= control_hz
        ):
            raise ValueError("gripper_command_hz must be within (0, control_hz]")
        for field_name, value in (
            ("trigger_deadzone", trigger_deadzone),
            ("thumbstick_deadzone", thumbstick_deadzone),
        ):
            if not np.isfinite(value) or not 0.0 <= value < 1.0:
                raise ValueError(f"{field_name} must be within [0, 1)")
        if thumbstick_y_sign not in (-1, 1):
            raise ValueError("thumbstick_y_sign must be -1 or 1")
        initial_gripper_closedness = np.asarray(
            initial_gripper_closedness, dtype=float
        ).reshape(-1)
        if (
            initial_gripper_closedness.shape != (2,)
            or not np.all(np.isfinite(initial_gripper_closedness))
            or np.any(initial_gripper_closedness < 0.0)
            or np.any(initial_gripper_closedness > 1.0)
        ):
            raise ValueError(
                "initial_gripper_closedness must contain two values within [0, 1]"
            )
        if gripper_control_enabled and not hasattr(adapter, "send_gripper_command"):
            raise TypeError("adapter must provide send_gripper_command()")
        initial_pose_q_rad = np.asarray(
            initial_pose_q_rad, dtype=float
        ).reshape(-1)
        if initial_pose_q_rad.shape != (14,) or not np.all(
            np.isfinite(initial_pose_q_rad)
        ):
            raise ValueError("initial_pose_q_rad must be a finite 14-joint vector")

        self.xr_client = xr_client
        self.adapter = adapter
        self.kinematics = kinematics
        self.scale_calibration_path = scale_calibration_path
        self.initial_pose_q_rad = initial_pose_q_rad.copy()
        self.tool_configurations = tool_configurations
        self.left_k = tuple(left_k)
        self.left_d = tuple(left_d)
        self.right_k = tuple(right_k)
        self.right_d = tuple(right_d)
        for field_name, value in (
            ("joint_velocity_ratio", joint_velocity_ratio),
            ("joint_acceleration_ratio", joint_acceleration_ratio),
        ):
            numeric_value = float(value)
            if (
                isinstance(value, (bool, np.bool_))
                or not np.isfinite(numeric_value)
                or not numeric_value.is_integer()
                or not 0.0 <= numeric_value <= 100.0
            ):
                raise ValueError(f"{field_name} must be an integer within [0, 100]")
            setattr(self, field_name, int(numeric_value))
        self.control_hz = control_hz
        self.control_period_seconds = 1.0 / control_hz
        self.pd_period_milliseconds = int(round(pd_period_milliseconds))
        self.return_duration = float(return_duration)
        self.grip_activation_threshold = float(grip_activation_threshold)
        self.expected_sdk_version = expected_sdk_version
        self.control_parameter_settle_seconds = float(
            control_parameter_settle_seconds
        )
        self.mode_settle_seconds = float(mode_settle_seconds)
        self.pd_settle_seconds = float(pd_settle_seconds)
        self.session_logger = session_logger
        self.gripper_control_enabled = bool(gripper_control_enabled)
        self.gripper_rate = gripper_rate
        self.gripper_command_period_seconds = 1.0 / gripper_command_hz
        self.trigger_deadzone = float(trigger_deadzone)
        self.thumbstick_deadzone = float(thumbstick_deadzone)
        self.thumbstick_y_sign = int(thumbstick_y_sign)
        self._gripper_closedness = initial_gripper_closedness.copy()
        self._last_sent_gripper_closedness = initial_gripper_closedness.copy()
        self._last_gripper_update_time = None
        self._last_gripper_command_time = None

        scale_factor = resolve_scale_factor(
            requested_scale_factor, scale_calibration_path
        )
        self.pose_mapper = XrTargetMapper(scale_factor)
        self.scale_calibrator = ArmLengthScaleCalibrator()
        self._previous_grip_states = (False, False)
        self._previous_button_a = False
        self._previous_button_b = False
        self._return_start_times = [None, None]
        self._return_start_q_rad = [None, None]
        self._last_commanded_q_rad = None
        self._last_ik_q_rad = [None, None]
        self._last_feedback_frame_serial = None
        self._stale_feedback_cycle_counts = [0, 0]
        self._xr_frame_available = False
        self._hardware_prepared = False

    @property
    def scale_factor(self):
        return self.pose_mapper.scale_factor

    @property
    def gripper_closedness(self):
        return tuple(self._gripper_closedness)

    @staticmethod
    def _arm_joint_slice(arm_index):
        return slice(arm_index * 7, (arm_index + 1) * 7)

    @staticmethod
    def _require_healthy_feedback(robot_feedback, require_impedance_mode):
        if any(robot_feedback.error_code):
            raise RuntimeError(
                f"Marvin reported error codes {robot_feedback.error_code}"
            )
        if any(arm_state == 100 for arm_state in robot_feedback.arm_state):
            raise RuntimeError(
                f"Marvin reported error states {robot_feedback.arm_state}"
            )
        if require_impedance_mode and robot_feedback.arm_state != (3, 3):
            raise RuntimeError(
                "Marvin did not enter dual-arm joint impedance mode: "
                f"{robot_feedback.arm_state}"
            )

    def _require_advancing_feedback(self, robot_feedback):
        current_serials = robot_feedback.frame_serial
        if self._last_feedback_frame_serial is None:
            self._last_feedback_frame_serial = current_serials
            return
        for arm_index, frame_serial in enumerate(current_serials):
            if (
                frame_serial == 0
                or frame_serial == self._last_feedback_frame_serial[arm_index]
            ):
                self._stale_feedback_cycle_counts[arm_index] += 1
            else:
                self._stale_feedback_cycle_counts[arm_index] = 0
        self._last_feedback_frame_serial = current_serials
        if max(self._stale_feedback_cycle_counts) >= (
            MAX_CONSECUTIVE_STALE_FEEDBACK_CYCLES
        ):
            raise TimeoutError(
                "Marvin feedback stopped advancing: "
                f"frame_serial={current_serials}"
            )

    def prepare_hardware(self):
        if self._hardware_prepared:
            return
        wait_for_fresh_snapshot = getattr(
            self.xr_client, "wait_for_fresh_snapshot", None
        )
        startup_xr_snapshot = (
            self.xr_client.read_snapshot()
            if wait_for_fresh_snapshot is None
            else wait_for_fresh_snapshot(timeout_seconds=2.0)
        )
        if any(value > 0.1 for value in startup_xr_snapshot.grip_values):
            raise RuntimeError("release both Grip controls before enabling Marvin")
        if self.gripper_control_enabled and (
            any(value > 0.1 for value in startup_xr_snapshot.trigger_values)
            or any(
                abs(value) > self.thumbstick_deadzone
                for value in startup_xr_snapshot.thumbstick_y_values
            )
        ):
            raise RuntimeError(
                "release both Triggers and center both thumbsticks before "
                "enabling grippers"
            )

        self.adapter.connect()
        actual_sdk_version = self.adapter.sdk_version()
        if (
            self.expected_sdk_version is not None
            and actual_sdk_version != self.expected_sdk_version
        ):
            raise RuntimeError(
                "Marvin control SDK version mismatch: "
                f"expected {self.expected_sdk_version}, "
                f"got {actual_sdk_version}"
            )
        robot_feedback = self.adapter.wait_for_fresh_feedback()
        self._require_healthy_feedback(robot_feedback, False)
        if not all(robot_feedback.low_speed):
            raise RuntimeError(
                "Marvin must be stationary before enabling teleoperation"
            )

        self.adapter.configure_control_parameters(
            self.left_k,
            self.left_d,
            self.right_k,
            self.right_d,
            self.tool_configurations,
            joint_velocity_ratio=self.joint_velocity_ratio,
            joint_acceleration_ratio=self.joint_acceleration_ratio,
        )
        if self.control_parameter_settle_seconds > 0.0:
            time.sleep(self.control_parameter_settle_seconds)

        startup_q_rad = robot_feedback.q_rad.copy()
        self._last_commanded_q_rad = startup_q_rad.copy()
        self._last_ik_q_rad = [
            startup_q_rad[:7].copy(),
            startup_q_rad[7:].copy(),
        ]
        self.adapter.enter_joint_impedance()
        if self.mode_settle_seconds > 0.0:
            time.sleep(self.mode_settle_seconds)
        robot_feedback = self.adapter.wait_for_fresh_feedback(
            timeout_seconds=max(1.0, self.mode_settle_seconds + 0.5),
            required_updates=1,
        )
        self._require_healthy_feedback(robot_feedback, True)
        self.adapter.enable_pd_feedforward(self.pd_period_milliseconds)
        if self.pd_settle_seconds > 0.0:
            time.sleep(self.pd_settle_seconds)
        robot_feedback = self.adapter.wait_for_fresh_feedback(
            timeout_seconds=max(1.0, self.pd_settle_seconds + 0.5),
            required_updates=1,
        )
        self._require_healthy_feedback(robot_feedback, True)
        self.adapter.send_joint_command(startup_q_rad, wait_response=True)
        self._last_feedback_frame_serial = robot_feedback.frame_serial
        self._stale_feedback_cycle_counts = [0, 0]
        self._previous_button_a = startup_xr_snapshot.button_a
        self._previous_button_b = startup_xr_snapshot.button_b
        self._xr_frame_available = True
        self._hardware_prepared = True

    def _process_controls(
        self, xr_snapshot, controller_poses_marvin, grip_states, robot_feedback
    ):
        button_a_pressed = xr_snapshot.button_a and not self._previous_button_a
        button_b_pressed = xr_snapshot.button_b and not self._previous_button_b
        self._previous_button_a = xr_snapshot.button_a
        self._previous_button_b = xr_snapshot.button_b

        reset_requested = button_b_pressed and not any(grip_states)
        if button_b_pressed:
            print(
                "Robot reset requested."
                if reset_requested
                else "Robot reset ignored: release both Grip controls."
            )
        if not button_a_pressed:
            return reset_requested
        if (
            any(grip_states)
            or any(start_time is not None for start_time in self._return_start_times)
            or not all(robot_feedback.low_speed)
        ):
            print(
                "Scale calibration ignored: release Grip and wait for return "
                "to finish."
            )
            return reset_requested
        controller_positions = {
            "left": controller_poses_marvin[0][0],
            "right": controller_poses_marvin[1][0],
        }
        try:
            calibration_result = self.scale_calibrator.capture(
                controller_positions
            )
        except ValueError as calibration_error:
            print(f"Scale calibration sample rejected: {calibration_error}")
            return reset_requested
        if calibration_result.status == "down_captured":
            print(
                "Scale calibration: down pose captured; extend both arms and "
                "press A."
            )
            return reset_requested
        save_scale_calibration(self.scale_calibration_path, calibration_result)
        self.pose_mapper.scale_factor = calibration_result.scale_factor
        print(f"Scale calibration applied: {calibration_result.scale_factor:.6f}")
        return reset_requested

    def _compute_q_command(
        self,
        xr_snapshot,
        robot_feedback,
        cycle_time_seconds,
    ):
        controller_poses_marvin = transform_controller_poses_to_marvin_frame(
            xr_snapshot
        )
        grip_states = tuple(
            value > self.grip_activation_threshold
            for value in xr_snapshot.grip_values
        )
        reset_requested = self._process_controls(
            xr_snapshot, controller_poses_marvin, grip_states, robot_feedback
        )

        q_command_rad = self._last_commanded_q_rad.copy()
        for arm_index, is_grip_active in enumerate(grip_states):
            arm_joint_slice = self._arm_joint_slice(arm_index)
            arm_q_rad = robot_feedback.q_rad[arm_joint_slice]
            current_tcp_transform = (
                self.kinematics.fk_world(arm_index, arm_q_rad)
            )

            if is_grip_active:
                self._return_start_times[arm_index] = None
                self._return_start_q_rad[arm_index] = None
                target_tcp_transform = self.pose_mapper.map_arm(
                    arm_index,
                    controller_poses_marvin[arm_index],
                    current_tcp_transform,
                    True,
                )
                q_ref_rad = (
                    arm_q_rad
                    if not self._previous_grip_states[arm_index]
                    else self._last_ik_q_rad[arm_index]
                )
                inverse_kinematics_result = (
                    self.kinematics.ik_world(
                        arm_index,
                        target_tcp_transform,
                        q_ref_rad,
                    )
                )
                if inverse_kinematics_result.success:
                    q_command_rad[arm_joint_slice] = (
                        inverse_kinematics_result.q_rad
                    )
                    self._last_ik_q_rad[arm_index] = (
                        inverse_kinematics_result.q_rad.copy()
                    )
                continue

            self.pose_mapper.map_arm(
                arm_index,
                controller_poses_marvin[arm_index],
                current_tcp_transform,
                False,
            )
            if reset_requested:
                self._return_start_times[arm_index] = cycle_time_seconds
                self._return_start_q_rad[arm_index] = arm_q_rad.copy()
            elif self._previous_grip_states[arm_index]:
                q_command_rad[arm_joint_slice] = arm_q_rad
                self._last_ik_q_rad[arm_index] = arm_q_rad.copy()
            return_start_time = self._return_start_times[arm_index]
            if return_start_time is None:
                continue

            return_progress = min(
                1.0,
                max(
                    0.0,
                    (cycle_time_seconds - return_start_time)
                    / self.return_duration,
                ),
            )
            cosine_blend = 0.5 - 0.5 * np.cos(np.pi * return_progress)
            return_start_q_rad = self._return_start_q_rad[arm_index]
            return_target_q_rad = self.initial_pose_q_rad[arm_joint_slice]
            q_command_rad[arm_joint_slice] = (
                return_start_q_rad
                + cosine_blend
                * (
                    return_target_q_rad
                    - return_start_q_rad
                )
            )
            if return_progress >= 1.0:
                self._return_start_times[arm_index] = None
                self._return_start_q_rad[arm_index] = None

        self._previous_grip_states = grip_states
        self._last_commanded_q_rad = q_command_rad
        return q_command_rad.copy()

    @staticmethod
    def _deadzone(value, threshold):
        return max(0.0, (value - threshold) / (1.0 - threshold))

    def _update_gripper_command(self, xr_snapshot, cycle_time_seconds):
        if not self.gripper_control_enabled:
            return
        previous_time = self._last_gripper_update_time
        self._last_gripper_update_time = cycle_time_seconds
        if previous_time is None:
            return
        elapsed_seconds = cycle_time_seconds - previous_time
        if elapsed_seconds < 0.0:
            raise ValueError("control cycle time regressed")
        elapsed_seconds = min(elapsed_seconds, 2.0 * self.control_period_seconds)
        for arm_index, (trigger, raw_stick_y) in enumerate(
            zip(xr_snapshot.trigger_values, xr_snapshot.thumbstick_y_values)
        ):
            stick_y = self.thumbstick_y_sign * raw_stick_y
            close_input = max(
                self._deadzone(trigger, self.trigger_deadzone),
                self._deadzone(max(-stick_y, 0.0), self.thumbstick_deadzone),
            )
            open_input = self._deadzone(
                max(stick_y, 0.0), self.thumbstick_deadzone
            )
            if close_input > 0.0:
                self._gripper_closedness[arm_index] += (
                    self.gripper_rate * close_input * elapsed_seconds
                )
            elif open_input > 0.0:
                self._gripper_closedness[arm_index] -= (
                    self.gripper_rate * open_input * elapsed_seconds
                )
        np.clip(self._gripper_closedness, 0.0, 1.0, out=self._gripper_closedness)
        if (
            self._last_gripper_command_time is not None
            and cycle_time_seconds - self._last_gripper_command_time
            < self.gripper_command_period_seconds
        ):
            return
        if np.max(
            np.abs(
                self._gripper_closedness
                - self._last_sent_gripper_closedness
            )
        ) < 0.01:
            return
        self.adapter.send_gripper_command(tuple(self._gripper_closedness))
        self._last_sent_gripper_closedness = self._gripper_closedness.copy()
        self._last_gripper_command_time = cycle_time_seconds

    def execute_control_cycle(self, cycle_time_seconds=None):
        if not self._hardware_prepared:
            raise RuntimeError("prepare_hardware() must run before control cycles")
        if cycle_time_seconds is None:
            cycle_time_seconds = time.monotonic()
        xr_snapshot = self.xr_client.read_snapshot()
        robot_feedback = self.adapter.read_state()
        self._require_healthy_feedback(robot_feedback, True)
        self._require_advancing_feedback(robot_feedback)
        if xr_snapshot is None:
            if self._xr_frame_available:
                print("PICO XR frame stale; holding joint targets.")
            self._xr_frame_available = False
            self.pose_mapper.reset_arm()
            self._previous_grip_states = (False, False)
            self._previous_button_a = True
            self._previous_button_b = True
            q_command_rad = self._last_commanded_q_rad.copy()
            self.adapter.send_joint_command(q_command_rad)
            if self.session_logger is not None:
                self.session_logger.record_control_cycle(
                    None,
                    robot_feedback,
                    q_command_rad,
                    self.scale_factor,
                    self.gripper_closedness,
                )
            return q_command_rad
        if not self._xr_frame_available:
            print("PICO XR stream recovered; Grip origins will be re-anchored.")
        self._xr_frame_available = True
        q_command_rad = self._compute_q_command(
            xr_snapshot, robot_feedback, float(cycle_time_seconds)
        )
        self.adapter.send_joint_command(q_command_rad)
        self._update_gripper_command(xr_snapshot, float(cycle_time_seconds))
        if self.session_logger is not None:
            self.session_logger.record_control_cycle(
                xr_snapshot,
                robot_feedback,
                q_command_rad,
                self.scale_factor,
                self.gripper_closedness,
            )
        return q_command_rad

    def shutdown_hardware(self):
        try:
            idle_command_sent = self.adapter.set_idle()
            if idle_command_sent:
                deadline = time.monotonic() + 2.0
                while time.monotonic() < deadline:
                    if self.adapter.read_state().arm_state == (0, 0):
                        break
                    time.sleep(0.01)
                else:
                    raise TimeoutError("Marvin did not enter idle state")
        finally:
            self.adapter.release()
            self.xr_client.close()
            if self.session_logger is not None:
                self.session_logger.close()
            self._hardware_prepared = False

    def run(self, maximum_cycles=None):
        completed_cycles = 0
        next_cycle_time = time.monotonic()
        try:
            self.prepare_hardware()
            print(
                "Marvin teleoperation active: Grip controls each arm; "
                "Trigger/stick Y controls each gripper; "
                "A/A calibrates scale; B resets both arms."
            )
            while maximum_cycles is None or completed_cycles < maximum_cycles:
                is_running = getattr(self.adapter, "is_running", None)
                if is_running is not None and not is_running():
                    break
                self.execute_control_cycle()
                completed_cycles += 1
                next_cycle_time += self.control_period_seconds
                remaining_seconds = next_cycle_time - time.monotonic()
                if remaining_seconds > 0.0:
                    time.sleep(remaining_seconds)
                else:
                    next_cycle_time = time.monotonic()
        except KeyboardInterrupt:
            print("\nMarvin teleoperation stop requested.")
        finally:
            self.shutdown_hardware()
