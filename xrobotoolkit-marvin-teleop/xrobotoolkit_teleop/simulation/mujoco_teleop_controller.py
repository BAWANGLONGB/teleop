import csv
import json
import queue
import threading
import time
from collections import deque
from datetime import datetime
from pathlib import Path
from typing import Any, Dict

import mujoco
from meshcat import transformations as tf
from mujoco import viewer as mj_viewer
import numpy as np

from xrobotoolkit_teleop.common.base_teleop_controller import BaseTeleopController
from xrobotoolkit_teleop.common.buffered_xr_client import BufferedXrClient
from xrobotoolkit_teleop.utils.geometry import (
    R_HEADSET_TO_WORLD,
)
from xrobotoolkit_teleop.utils.mujoco_utils import (
    calc_mujoco_ctrl_from_qpos,
    calc_mujoco_qpos_from_placo_q,
    calc_placo_q_from_mujoco_qpos,
    set_mujoco_joint_pos_by_name,
)


class MujocoTeleopController(BaseTeleopController):
    def __init__(
        self,
        xml_path: str,
        robot_urdf_path: str,
        manipulator_config: Dict[str, Dict[str, Any]],
        floating_base=False,
        R_headset_world=R_HEADSET_TO_WORLD,
        visualize_placo=False,
        scale_factor=1.0,
        dt=0.01,
        mj_qpos_init=None,
        viewer_camera=None,
        reference_mode="world",
        return_joint_positions=None,
        return_duration=3.0,
        scale_calibration_config=None,
        max_joint_speed=None,
        max_joint_acceleration=None,
        joint_limit_margin=None,
        target_velocity_feedforward=0.8,
        target_velocity_filter_time_constant=0.04,
        render_hz=60.0,
        xr_poll_hz=None,
        telemetry_report_interval=2.0,
        telemetry_output_dir=None,
        telemetry_session_name="mujoco_latency",
    ):
        if dt <= 0.0:
            raise ValueError("dt must be positive")
        if return_duration <= 0.0:
            raise ValueError("return_duration must be positive")
        if joint_limit_margin is not None and joint_limit_margin < 0.0:
            raise ValueError("joint_limit_margin must be non-negative when provided")
        if not 0.0 <= target_velocity_feedforward <= 1.0:
            raise ValueError("target_velocity_feedforward must be in [0, 1]")
        if target_velocity_filter_time_constant < 0.0:
            raise ValueError("target_velocity_filter_time_constant must be non-negative")
        if render_hz <= 0.0:
            raise ValueError("render_hz must be positive")
        if xr_poll_hz is not None and xr_poll_hz <= 0.0:
            raise ValueError("xr_poll_hz must be positive when provided")
        if telemetry_report_interval < 0.0:
            raise ValueError("telemetry_report_interval must be non-negative")
        if telemetry_output_dir is not None and not str(telemetry_output_dir).strip():
            raise ValueError("telemetry_output_dir must not be empty when provided")
        if not telemetry_session_name or not telemetry_session_name.strip():
            raise ValueError("telemetry_session_name must not be empty")
        self.visualize_placo = visualize_placo
        self.xml_path = xml_path
        self.mj_qpos_init = mj_qpos_init
        self.viewer_camera = viewer_camera
        self.return_joint_positions = return_joint_positions or {}
        self.return_duration = return_duration
        self.max_joint_speed = max_joint_speed
        self.max_joint_acceleration = max_joint_acceleration
        self.joint_limit_margin = joint_limit_margin
        self.target_velocity_feedforward = target_velocity_feedforward
        self.target_velocity_filter_time_constant = target_velocity_filter_time_constant
        self.render_hz = render_hz
        self.xr_poll_hz = xr_poll_hz
        self.telemetry_report_interval = telemetry_report_interval
        self.telemetry_output_dir = telemetry_output_dir
        self.telemetry_session_name = telemetry_session_name
        self._last_ctrl_command = None
        self._last_ctrl_velocity = None
        self._last_ctrl_target = None
        self._filtered_target_velocity = None
        self._ctrl_lower_limits = None
        self._ctrl_upper_limits = None
        self._max_joint_speed = None
        self._max_joint_acceleration = None
        self._previous_active = {name: False for name in manipulator_config}
        self._return_trajectory = {}
        self._simulation_lock = threading.Lock()
        self._control_exception = None
        self._latency_samples = deque(maxlen=1000)
        self._deadline_miss_count = 0
        self._last_telemetry_report = time.monotonic()
        self._telemetry_queue = None
        self._telemetry_writer_thread = None
        self._telemetry_writer_error = None
        self._telemetry_stop_token = object()
        self._telemetry_sample_count = 0
        self._telemetry_started_at = None
        self._telemetry_started_monotonic = None
        self.telemetry_csv_path = None
        self.telemetry_summary_path = None

        # To be initialized later
        self.mj_model = None
        self.mj_data = None
        self.target_mocap_idx = {name: -1 for name in manipulator_config.keys()}
        self.commanded_mocap_idx = {name: -1 for name in manipulator_config.keys()}

        super().__init__(
            robot_urdf_path,
            manipulator_config,
            floating_base,
            R_headset_world,
            scale_factor,
            q_init=None,
            dt=dt,
            reference_mode=reference_mode,
            scale_calibration_config=scale_calibration_config,
        )

        # Placo tasks are initially created from its neutral configuration.  A
        # MuJoCo scene may start from a non-zero keyframe/qpos, so synchronize
        # both states before accepting the first XR command.  Without this,
        # the robot can jump toward the URDF neutral pose on startup.
        self._update_robot_state()
        self.sync_end_effector_poses_to_placo_tasks()
        self._last_ctrl_command = calc_mujoco_ctrl_from_qpos(self.mj_model, self.mj_data.qpos)
        self._last_ctrl_velocity = np.zeros_like(self._last_ctrl_command)
        self._last_ctrl_target = self._last_ctrl_command.copy()
        self._filtered_target_velocity = np.zeros_like(self._last_ctrl_command)
        self._validate_initial_joint_command()

        if self.xr_poll_hz is not None:
            pose_names = ["headset"]
            key_names = []
            include_motion_trackers = False
            for config in self.manipulator_config.values():
                pose_names.append(config["pose_source"])
                key_names.append(config["control_trigger"])
                if "gripper_config" in config:
                    key_names.append(config["gripper_config"]["gripper_trigger"])
                include_motion_trackers |= "motion_tracker" in config
            self.xr_client = BufferedXrClient(
                self.xr_client,
                pose_names=pose_names,
                key_names=key_names,
                button_names=[self._calibration_button, self._calibration_cancel_button],
                poll_hz=self.xr_poll_hz,
                include_motion_trackers=include_motion_trackers,
            )

        if visualize_placo:
            self._init_placo_viz()

    def _robot_setup(self):
        self.mj_model = mujoco.MjModel.from_xml_path(self.xml_path)
        self.mj_data = mujoco.MjData(self.mj_model)
        self._command_fk_data = mujoco.MjData(self.mj_model)

        physics_timestep = float(self.mj_model.opt.timestep)
        substep_ratio = self.dt / physics_timestep
        self.physics_substeps = int(round(substep_ratio))
        if self.physics_substeps < 1 or not np.isclose(
            self.physics_substeps * physics_timestep,
            self.dt,
            rtol=0.0,
            atol=1e-12,
        ):
            raise ValueError(
                "Control dt must be an integer multiple of the MuJoCo physics timestep: "
                f"dt={self.dt}, physics_timestep={physics_timestep}"
            )

        print("Joint names in the Mujoco model:")
        for i in range(self.mj_model.njnt):
            joint_name = mujoco.mj_id2name(self.mj_model, mujoco.mjtObj.mjOBJ_JOINT, i)
            print(f"  {joint_name}")

        # Configure scene lighting
        self.mj_model.vis.headlight.ambient = [0.4, 0.4, 0.4]
        self.mj_model.vis.headlight.diffuse = [0.8, 0.8, 0.8]
        self.mj_model.vis.headlight.specular = [0.6, 0.6, 0.6]

        mujoco.mj_resetData(self.mj_model, self.mj_data)
        if self.mj_qpos_init is None:
            mujoco.mj_resetDataKeyframe(self.mj_model, self.mj_data, self.mj_model.key("home").id)
        else:
            self.mj_data.qpos[:] = self.mj_qpos_init
            self.mj_data.ctrl[:] = calc_mujoco_ctrl_from_qpos(self.mj_model, self.mj_qpos_init)
        mujoco.mj_forward(self.mj_model, self.mj_data)

        self._configure_actuator_joint_limits()
        self._max_joint_speed = self._resolve_actuator_parameter(
            self.max_joint_speed, "max_joint_speed"
        )
        self._max_joint_acceleration = self._resolve_actuator_parameter(
            self.max_joint_acceleration, "max_joint_acceleration"
        )

        # setup mocap target
        for name, config in self.manipulator_config.items():
            if "vis_target" not in config:
                print(f"Warning: 'vis_target' not found in config for {name}. Skipping mocap setup.")
                continue
            vis_target = config["vis_target"]
            mocap_id = mujoco.mj_name2id(self.mj_model, mujoco.mjtObj.mjOBJ_BODY, vis_target)
            if mocap_id == -1:
                raise ValueError(f"Mocap body '{vis_target}' not found in the model.")

            if self.mj_model.body_mocapid[mocap_id] == -1:
                raise ValueError(f"Body '{self.vis_target}' is not configured for mocap.")
            else:
                self.target_mocap_idx[name] = self.mj_model.body_mocapid[mocap_id]

            print(f"Mocap ID for '{vis_target}' body: {self.target_mocap_idx[name]}")

            commanded_target = config.get("vis_commanded_target")
            if commanded_target is not None:
                body_id = mujoco.mj_name2id(
                    self.mj_model, mujoco.mjtObj.mjOBJ_BODY, commanded_target
                )
                if body_id == -1 or self.mj_model.body_mocapid[body_id] == -1:
                    raise ValueError(
                        f"Commanded target body '{commanded_target}' is missing or not mocap"
                    )
                self.commanded_mocap_idx[name] = self.mj_model.body_mocapid[body_id]

    def _resolve_actuator_parameter(self, value, parameter_name):
        if value is None:
            return None
        if np.isscalar(value):
            result = np.full(self.mj_model.nu, float(value))
        elif isinstance(value, dict):
            result = np.empty(self.mj_model.nu)
            for actuator_id in range(self.mj_model.nu):
                joint_id = int(self.mj_model.actuator_trnid[actuator_id, 0])
                joint_name = mujoco.mj_id2name(
                    self.mj_model, mujoco.mjtObj.mjOBJ_JOINT, joint_id
                )
                actuator_name = mujoco.mj_id2name(
                    self.mj_model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_id
                )
                if joint_name in value:
                    result[actuator_id] = value[joint_name]
                elif actuator_name in value:
                    result[actuator_id] = value[actuator_name]
                elif "default" in value:
                    result[actuator_id] = value["default"]
                else:
                    raise ValueError(
                        f"{parameter_name} has no value for joint '{joint_name}'"
                    )
        else:
            result = np.asarray(value, dtype=float)
            if result.shape != (self.mj_model.nu,):
                raise ValueError(
                    f"{parameter_name} must be scalar, a mapping, or length {self.mj_model.nu}"
                )
        if not np.all(np.isfinite(result)) or np.any(result <= 0.0):
            raise ValueError(f"{parameter_name} values must all be finite and positive")
        return result

    def _configure_actuator_joint_limits(self):
        """Map MuJoCo joint ranges to actuator-space soft limits."""
        self._ctrl_lower_limits = np.full(self.mj_model.nu, -np.inf)
        self._ctrl_upper_limits = np.full(self.mj_model.nu, np.inf)
        if self.joint_limit_margin is None:
            return

        for actuator_id in range(self.mj_model.nu):
            joint_id = int(self.mj_model.actuator_trnid[actuator_id, 0])
            if joint_id < 0 or not self.mj_model.jnt_limited[joint_id]:
                continue

            lower, upper = self.mj_model.jnt_range[joint_id]
            soft_lower = float(lower + self.joint_limit_margin)
            soft_upper = float(upper - self.joint_limit_margin)
            if soft_lower >= soft_upper:
                joint_name = mujoco.mj_id2name(
                    self.mj_model, mujoco.mjtObj.mjOBJ_JOINT, joint_id
                )
                raise ValueError(
                    f"joint_limit_margin leaves no safe range for joint '{joint_name}': "
                    f"range=[{lower}, {upper}], margin={self.joint_limit_margin}"
                )
            self._ctrl_lower_limits[actuator_id] = soft_lower
            self._ctrl_upper_limits[actuator_id] = soft_upper

    def _validate_initial_joint_command(self):
        if self.joint_limit_margin is None:
            return
        outside = np.flatnonzero(
            (self._last_ctrl_command < self._ctrl_lower_limits)
            | (self._last_ctrl_command > self._ctrl_upper_limits)
        )
        if outside.size:
            actuator_names = [
                mujoco.mj_id2name(self.mj_model, mujoco.mjtObj.mjOBJ_ACTUATOR, int(index))
                for index in outside
            ]
            raise ValueError(
                "Initial joint command is outside the configured soft limits for actuators: "
                + ", ".join(actuator_names)
            )

    def _limit_joint_command(self, ctrl_desired):
        """Apply command velocity, acceleration, and predictive limit braking."""
        target = np.clip(ctrl_desired, self._ctrl_lower_limits, self._ctrl_upper_limits)
        position_error = target - self._last_ctrl_command
        raw_target_velocity = (target - self._last_ctrl_target) / self.dt
        if self._max_joint_speed is not None:
            raw_target_velocity = np.clip(
                raw_target_velocity, -self._max_joint_speed, self._max_joint_speed
            )
        filter_alpha = (
            1.0
            if self.target_velocity_filter_time_constant == 0.0
            else self.dt / (self.target_velocity_filter_time_constant + self.dt)
        )
        moving_target = np.abs(raw_target_velocity) > 1e-6
        self._filtered_target_velocity[moving_target] += filter_alpha * (
            raw_target_velocity[moving_target] - self._filtered_target_velocity[moving_target]
        )
        # A stopped target must immediately request a zero terminal velocity;
        # otherwise the filter itself creates overshoot after the operator stops.
        self._filtered_target_velocity[~moving_target] = 0.0
        self._last_ctrl_target = target.copy()

        velocity = position_error / self.dt

        if self._max_joint_speed is not None:
            velocity = np.clip(velocity, -self._max_joint_speed, self._max_joint_speed)

        if self._max_joint_acceleration is not None:
            max_velocity_change = self._max_joint_acceleration * self.dt

            # Plan a zero-velocity arrival at a stationary joint target. A
            # plain acceleration slew limiter starts braking only after it has
            # crossed the target, which adds visible lag and oscillation. The
            # one-tick conservative bound below starts braking early enough at
            # the discrete control rate.
            target_braking_speed = np.maximum(
                0.0,
                np.sqrt(
                    max_velocity_change**2
                    + 2.0 * self._max_joint_acceleration * np.abs(position_error)
                )
                - max_velocity_change,
            )
            target_velocity = (
                self.target_velocity_feedforward * self._filtered_target_velocity
            )
            correction_velocity = np.sign(position_error) * target_braking_speed
            velocity = target_velocity + correction_velocity
            if self._max_joint_speed is not None:
                velocity = np.clip(velocity, -self._max_joint_speed, self._max_joint_speed)
            velocity = np.clip(
                velocity,
                self._last_ctrl_velocity - max_velocity_change,
                self._last_ctrl_velocity + max_velocity_change,
            )

            if self.joint_limit_margin is not None:
                # Include the next control interval in the stopping-distance
                # bound: v*dt + v^2/(2*a) <= distance. This makes the
                # continuous braking rule conservative at the 100 Hz command
                # rate and starts braking before the soft limit is reached.
                acceleration = self._max_joint_acceleration
                control_tick_velocity = acceleration * self.dt
                upper_distance = np.maximum(
                    0.0, self._ctrl_upper_limits - self._last_ctrl_command
                )
                lower_distance = np.maximum(
                    0.0, self._last_ctrl_command - self._ctrl_lower_limits
                )
                upper_braking_speed = (
                    np.sqrt(control_tick_velocity**2 + 2.0 * acceleration * upper_distance)
                    - control_tick_velocity
                )
                lower_braking_speed = (
                    np.sqrt(control_tick_velocity**2 + 2.0 * acceleration * lower_distance)
                    - control_tick_velocity
                )
                velocity = np.clip(velocity, -lower_braking_speed, upper_braking_speed)

        command = self._last_ctrl_command + velocity * self.dt
        command = np.clip(command, self._ctrl_lower_limits, self._ctrl_upper_limits)
        self._last_ctrl_velocity = (command - self._last_ctrl_command) / self.dt
        return command

    def _send_command(self):
        qpos_desired = calc_mujoco_qpos_from_placo_q(
            self.mj_model,
            self.placo_robot,
            self.placo_robot.state.q,
            floating_base=self.floating_base,
        )

        for gripper_name, gripper_target in self.gripper_pos_target.items():
            for joint_name, joint_pos in gripper_target.items():
                success = set_mujoco_joint_pos_by_name(
                    self.mj_model,
                    qpos_desired,
                    joint_name,
                    joint_pos,
                )
                if not success:
                    raise ValueError(f"Joint '{gripper_name}' not found in MuJoCo model.")

        ctrl_desired = calc_mujoco_ctrl_from_qpos(self.mj_model, qpos_desired)
        if (
            self._max_joint_speed is not None
            or self._max_joint_acceleration is not None
            or self.joint_limit_margin is not None
        ):
            # Limit only the robot command. The mocap target is updated from
            # the unsmoothed Cartesian task, so the target ball remains
            # immediate while the simulated arm follows a safe trajectory.
            ctrl_desired = self._limit_joint_command(ctrl_desired)

        self.mj_data.ctrl[:] = ctrl_desired
        self._last_ctrl_command = ctrl_desired.copy()

        if self.visualize_placo:
            self._update_placo_viz()

    def _update_robot_state(self):
        mj_qpos = self.mj_data.qpos.copy()
        self.placo_robot.state.q = calc_placo_q_from_mujoco_qpos(
            self.mj_model,
            self.placo_robot,
            mj_qpos,
            floating_base=self.floating_base,
        )
        self.placo_robot.update_kinematics()

    def _update_mocap_target(self):
        for name, task in self.effector_task.items():
            mocap_idx = self.target_mocap_idx.get(name)
            if mocap_idx is None or mocap_idx == -1:
                continue
            if self.effector_control_mode[name] == "position":
                target_position = task.target_world
                target_quaternion = self.mj_data.mocap_quat[mocap_idx]
            else:
                T_world_target = task.T_world_frame
                target_position = T_world_target[:3, 3]
                target_quaternion = tf.quaternion_from_matrix(T_world_target)
            self.mj_data.mocap_pos[mocap_idx] = target_position
            self.mj_data.mocap_quat[mocap_idx] = target_quaternion

    def _update_commanded_mocap_target(self):
        """Show the TCP implied by the limited joint command."""
        self._command_fk_data.qpos[:] = self.mj_data.qpos
        for actuator_id, command in enumerate(self._last_ctrl_command):
            joint_id = int(self.mj_model.actuator_trnid[actuator_id, 0])
            qpos_address = int(self.mj_model.jnt_qposadr[joint_id])
            self._command_fk_data.qpos[qpos_address] = command
        mujoco.mj_forward(self.mj_model, self._command_fk_data)

        for name, config in self.manipulator_config.items():
            mocap_idx = self.commanded_mocap_idx.get(name, -1)
            if mocap_idx == -1:
                continue
            position, quaternion = self._get_mujoco_frame_pose(
                self._command_fk_data, config
            )
            self.mj_data.mocap_pos[mocap_idx] = position
            self.mj_data.mocap_quat[mocap_idx] = quaternion

    def _get_mujoco_frame_pose(self, data, config):
        site_name = config.get("mujoco_site_name")
        if site_name is not None:
            site_id = mujoco.mj_name2id(
                self.mj_model, mujoco.mjtObj.mjOBJ_SITE, site_name
            )
            if site_id == -1:
                raise ValueError(f"MuJoCo site '{site_name}' not found")
            rotation = data.site_xmat[site_id].reshape(3, 3)
            transform = np.eye(4)
            transform[:3, :3] = rotation
            return data.site_xpos[site_id].copy(), tf.quaternion_from_matrix(transform)

        body_name = config["link_name"]
        body_id = mujoco.mj_name2id(
            self.mj_model, mujoco.mjtObj.mjOBJ_BODY, body_name
        )
        if body_id == -1:
            raise ValueError(f"MuJoCo body '{body_name}' not found")
        return data.xpos[body_id].copy(), data.xquat[body_id].copy()

    def _tcp_tracking_errors(self):
        errors = {}
        for name, config in self.manipulator_config.items():
            if self.effector_control_mode[name] == "position":
                raw_position = self.effector_task[name].target_world
            else:
                raw_position = self.effector_task[name].T_world_frame[:3, 3]
            commanded_position, _ = self._get_mujoco_frame_pose(
                self._command_fk_data, config
            )
            actual_position, _ = self._get_mujoco_frame_pose(self.mj_data, config)
            errors[f"{name}_raw_to_command_m"] = float(
                np.linalg.norm(raw_position - commanded_position)
            )
            errors[f"{name}_command_to_actual_m"] = float(
                np.linalg.norm(commanded_position - actual_position)
            )
        return errors

    def _xr_diagnostics(self):
        get_diagnostics = getattr(self.xr_client, "get_diagnostics", None)
        if get_diagnostics is None:
            return {
                "poll_age_ms": float("nan"),
                "source_age_ms": float("nan"),
                "source_timestamp_ns": self.xr_client.get_timestamp_ns(),
            }
        return get_diagnostics()

    def _record_latency_sample(self, sample):
        self._latency_samples.append(sample)
        self._telemetry_sample_count += 1
        if self._telemetry_queue is not None:
            row = {
                "sample_index": self._telemetry_sample_count,
                "wall_time_ns": time.time_ns(),
                "monotonic_time_ns": time.monotonic_ns(),
                "simulation_time_s": float(self.mj_data.time),
                **sample,
            }
            # SimpleQueue is unbounded and put() never blocks. File I/O stays
            # on the writer thread and cannot stall the 100 Hz control loop.
            self._telemetry_queue.put(row)
        if self.telemetry_report_interval == 0.0:
            return
        now = time.monotonic()
        if now - self._last_telemetry_report < self.telemetry_report_interval:
            return
        self._last_telemetry_report = now
        diagnostics = self.get_latency_diagnostics()
        p95 = diagnostics["p95"]
        print(
            "[latency p95] "
            f"cycle={p95.get('cycle_ms', float('nan')):.2f} ms, "
            f"xr+ik={p95.get('ik_and_xr_ms', float('nan')):.2f} ms, "
            f"physics={p95.get('physics_ms', float('nan')):.2f} ms, "
            f"render={p95.get('render_ms', float('nan')):.2f} ms, "
            f"XR-age={p95.get('xr_source_age_ms', float('nan')):.1f} ms, "
            f"raw->cmd={1000.0 * p95.get('max_raw_to_command_m', float('nan')):.1f} mm, "
            f"cmd->actual={1000.0 * p95.get('max_command_to_actual_m', float('nan')):.1f} mm, "
            f"deadline-misses={self._deadline_miss_count}"
        )

    def get_latency_diagnostics(self):
        samples = list(self._latency_samples)
        if not samples:
            return {
                "latest": {},
                "p50": {},
                "p95": {},
                "p99": {},
                "deadline_misses": 0,
            }
        keys = set().union(*(sample.keys() for sample in samples))

        def percentile(percent):
            result = {}
            for key in keys:
                values = np.asarray(
                    [sample[key] for sample in samples if key in sample], dtype=float
                )
                values = values[np.isfinite(values)]
                if values.size:
                    result[key] = float(np.percentile(values, percent))
            return result

        return {
            "latest": samples[-1].copy(),
            "p50": percentile(50),
            "p95": percentile(95),
            "p99": percentile(99),
            "deadline_misses": self._deadline_miss_count,
        }

    def _telemetry_fieldnames(self):
        fields = [
            "sample_index",
            "wall_time_ns",
            "monotonic_time_ns",
            "simulation_time_s",
            "cycle_ms",
            "ik_and_xr_ms",
            "command_ms",
            "physics_ms",
            "render_ms",
            "deadline_late_ms",
            "xr_poll_age_ms",
            "xr_source_age_ms",
            "xr_source_timestamp_ns",
            "max_raw_to_command_m",
            "max_command_to_actual_m",
        ]
        for name in self.manipulator_config:
            fields.extend(
                [
                    f"{name}_raw_to_command_m",
                    f"{name}_command_to_actual_m",
                ]
            )
        return fields

    def _telemetry_writer(self, fieldnames):
        try:
            with self.telemetry_csv_path.open("w", newline="", encoding="utf-8") as csv_file:
                writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
                writer.writeheader()
                last_flush = time.monotonic()
                while True:
                    row = self._telemetry_queue.get()
                    if row is self._telemetry_stop_token:
                        break
                    writer.writerow(row)
                    now = time.monotonic()
                    if now - last_flush >= 1.0:
                        csv_file.flush()
                        last_flush = now
                csv_file.flush()
        except Exception as error:
            self._telemetry_writer_error = error

    def _start_telemetry_logging(self):
        if self.telemetry_output_dir is None or self._telemetry_writer_thread is not None:
            return

        output_dir = Path(self.telemetry_output_dir).expanduser()
        output_dir.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S_%f")
        safe_name = "".join(
            character if character.isalnum() or character in "-_" else "_"
            for character in self.telemetry_session_name
        )
        session_stem = f"{safe_name}_{timestamp}"
        self.telemetry_csv_path = output_dir / f"{session_stem}.csv"
        self.telemetry_summary_path = output_dir / f"{session_stem}.summary.json"
        self._telemetry_queue = queue.SimpleQueue()
        self._telemetry_started_at = datetime.now().astimezone()
        self._telemetry_started_monotonic = time.monotonic()
        self._telemetry_writer_error = None
        self._telemetry_writer_thread = threading.Thread(
            target=self._telemetry_writer,
            args=(self._telemetry_fieldnames(),),
            name="mujoco-telemetry-writer",
            daemon=True,
        )
        self._telemetry_writer_thread.start()
        print(f"Latency samples: {self.telemetry_csv_path.resolve()}")

    def _full_session_statistics(self, fieldnames):
        metric_fields = fieldnames[4:]
        if self._telemetry_sample_count == 0:
            return {"latest": {}, "p50": {}, "p95": {}, "p99": {}}

        values = np.loadtxt(
            self.telemetry_csv_path,
            delimiter=",",
            skiprows=1,
            usecols=tuple(range(4, len(fieldnames))),
            ndmin=2,
        )
        statistics = {"latest": {}, "p50": {}, "p95": {}, "p99": {}}
        for column, name in enumerate(metric_fields):
            finite_values = values[:, column]
            finite_values = finite_values[np.isfinite(finite_values)]
            statistics["latest"][name] = (
                float(values[-1, column]) if np.isfinite(values[-1, column]) else None
            )
            if finite_values.size:
                for percentile in (50, 95, 99):
                    statistics[f"p{percentile}"][name] = float(
                        np.percentile(finite_values, percentile)
                    )
        return statistics

    def _stop_telemetry_logging(self):
        writer_thread = self._telemetry_writer_thread
        if writer_thread is None:
            return

        self._telemetry_queue.put(self._telemetry_stop_token)
        writer_thread.join(timeout=5.0)
        if writer_thread.is_alive():
            print("Warning: telemetry writer did not stop within 5 seconds")
            return

        ended_at = datetime.now().astimezone()
        duration_s = time.monotonic() - self._telemetry_started_monotonic
        fieldnames = self._telemetry_fieldnames()
        writer_error = self._telemetry_writer_error
        statistics = (
            self._full_session_statistics(fieldnames)
            if writer_error is None
            else {"latest": {}, "p50": {}, "p95": {}, "p99": {}}
        )
        summary = {
            "schema_version": 1,
            "session": {
                "started_at": self._telemetry_started_at.isoformat(),
                "ended_at": ended_at.isoformat(),
                "duration_s": duration_s,
                "sample_count": self._telemetry_sample_count,
                "deadline_misses": self._deadline_miss_count,
                "control_hz": 1.0 / self.dt,
                "physics_hz": 1.0 / float(self.mj_model.opt.timestep),
                "physics_substeps": self.physics_substeps,
                "csv_path": str(self.telemetry_csv_path.resolve()),
                "writer_error": repr(writer_error) if writer_error is not None else None,
            },
            "metrics": statistics,
        }
        with self.telemetry_summary_path.open("w", encoding="utf-8") as summary_file:
            json.dump(summary, summary_file, indent=2, ensure_ascii=False, allow_nan=False)
            summary_file.write("\n")
        print(f"Latency summary: {self.telemetry_summary_path.resolve()}")
        if writer_error is not None:
            print(f"Warning: telemetry CSV writer failed: {writer_error!r}")

        self._telemetry_writer_thread = None
        self._telemetry_queue = None

    def _step_physics(self):
        """Advance one control period using fixed-rate MuJoCo substeps."""
        for _ in range(self.physics_substeps):
            mujoco.mj_step(self.mj_model, self.mj_data)

    def _control_cycle(self, deadline_late_ms=0.0):
        cycle_start_ns = time.perf_counter_ns()
        ik_start_ns = cycle_start_ns
        begin_xr_cycle = getattr(self.xr_client, "begin_cycle", None)
        end_xr_cycle = getattr(self.xr_client, "end_cycle", None)
        if begin_xr_cycle is not None:
            begin_xr_cycle()
        try:
            self._update_ik()
            ik_end_ns = time.perf_counter_ns()
            self._apply_release_return()
            self._update_gripper_target()
            xr = self._xr_diagnostics()
        finally:
            if end_xr_cycle is not None:
                end_xr_cycle()
        self._update_mocap_target()

        command_start_ns = time.perf_counter_ns()
        self._send_command()
        self._update_commanded_mocap_target()
        command_end_ns = time.perf_counter_ns()

        physics_start_ns = time.perf_counter_ns()
        self._step_physics()
        physics_end_ns = time.perf_counter_ns()

        tcp_errors = self._tcp_tracking_errors()
        sample = {
            "cycle_ms": (physics_end_ns - cycle_start_ns) / 1e6,
            "ik_and_xr_ms": (ik_end_ns - ik_start_ns) / 1e6,
            "command_ms": (command_end_ns - command_start_ns) / 1e6,
            "physics_ms": (physics_end_ns - physics_start_ns) / 1e6,
            "render_ms": getattr(self, "_last_render_ms", float("nan")),
            "deadline_late_ms": deadline_late_ms,
            "xr_poll_age_ms": xr["poll_age_ms"],
            "xr_source_age_ms": xr["source_age_ms"],
            "xr_source_timestamp_ns": xr["source_timestamp_ns"],
            "max_raw_to_command_m": max(
                value for key, value in tcp_errors.items() if "raw_to_command" in key
            ),
            "max_command_to_actual_m": max(
                value for key, value in tcp_errors.items() if "command_to_actual" in key
            ),
            **tcp_errors,
        }
        self._record_latency_sample(sample)
        return sample

    def _apply_release_return(self):
        """Smoothly return each inactive manipulator to its configured joint pose."""
        if not self.return_joint_positions:
            return

        qpos_desired = calc_mujoco_qpos_from_placo_q(
            self.mj_model,
            self.placo_robot,
            self.placo_robot.state.q,
            floating_base=self.floating_base,
        )
        returning = False

        for name, joint_targets in self.return_joint_positions.items():
            active = self.active.get(name, False)
            if active:
                self._return_trajectory.pop(name, None)
            elif self._previous_active.get(name, False):
                start_positions = {}
                for joint_name in joint_targets:
                    joint_id = mujoco.mj_name2id(self.mj_model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
                    if joint_id < 0:
                        raise ValueError(f"Return joint '{joint_name}' not found in MuJoCo model")
                    qpos_address = int(self.mj_model.jnt_qposadr[joint_id])
                    start_positions[joint_name] = float(self.mj_data.qpos[qpos_address])
                self._return_trajectory[name] = {
                    "start_time": float(self.mj_data.time),
                    "start_positions": start_positions,
                }

            trajectory = self._return_trajectory.get(name)
            if not active and trajectory is not None:
                elapsed = max(0.0, float(self.mj_data.time) - trajectory["start_time"])
                progress = min(1.0, elapsed / self.return_duration)
                blend = 0.5 - 0.5 * np.cos(np.pi * progress)
                for joint_name, target in joint_targets.items():
                    start = trajectory["start_positions"][joint_name]
                    command = start + blend * (target - start)
                    if not set_mujoco_joint_pos_by_name(self.mj_model, qpos_desired, joint_name, command):
                        raise ValueError(f"Return joint '{joint_name}' not found in MuJoCo model")
                returning = True

            self._previous_active[name] = active

        if not returning:
            return

        self.placo_robot.state.q = calc_placo_q_from_mujoco_qpos(
            self.mj_model,
            self.placo_robot,
            qpos_desired,
            floating_base=self.floating_base,
        )
        self.placo_robot.update_kinematics()

        # Keep the inactive Cartesian task synchronized with the return
        # trajectory. This makes the target marker follow the commanded TCP
        # and guarantees that the next Grip press starts from a fresh target.
        for name in self._return_trajectory:
            if self.active.get(name, False):
                continue
            link_name = self.manipulator_config[name]["link_name"]
            target_pose = self.placo_robot.get_T_world_frame(link_name)
            if self.effector_control_mode[name] == "position":
                self.effector_task[name].target_world = target_pose[:3, 3]
            else:
                self.effector_task[name].T_world_frame = target_pose

    def _get_link_pose(self, ee_name):
        """Get the end effector position and orientation."""
        # MuJoCo fuses fixed URDF links into their parent body.  Allow a model
        # to expose such an end effector as a site while Placo continues to use
        # the original fixed link as its IK frame.
        site_name = None
        for config in self.manipulator_config.values():
            if config.get("link_name") == ee_name:
                site_name = config.get("mujoco_site_name")
                break

        if site_name is not None:
            site_id = mujoco.mj_name2id(self.mj_model, mujoco.mjtObj.mjOBJ_SITE, site_name)
            if site_id == -1:
                raise ValueError(f"End effector site '{site_name}' not found in the MuJoCo model.")
            ee_xyz = self.mj_data.site_xpos[site_id].copy()
            ee_quat = tf.quaternion_from_matrix(
                np.block(
                    [
                        [self.mj_data.site_xmat[site_id].reshape(3, 3), np.zeros((3, 1))],
                        [np.zeros((1, 3)), np.ones((1, 1))],
                    ]
                )
            )
            return ee_xyz, ee_quat

        ee_id = mujoco.mj_name2id(self.mj_model, mujoco.mjtObj.mjOBJ_BODY, ee_name)
        if ee_id == -1:
            raise ValueError(f"End effector body '{ee_name}' not found in the model.")

        ee_xyz = self.mj_data.xpos[ee_id].copy()
        ee_quat = self.mj_data.xquat[ee_id].copy()

        return ee_xyz, ee_quat

    def run(self):
        control_thread = None
        try:
            self._start_telemetry_logging()
            start_xr = getattr(self.xr_client, "start", None)
            if start_xr is not None:
                start_xr()
                if not self.xr_client.wait_until_ready(timeout=2.0):
                    raise RuntimeError(
                        "XR latest-value worker did not produce a valid snapshot within 2 seconds"
                    )

            # The viewer owns a separate model/data pair. Copying one snapshot
            # under the simulation lock is fast; potentially blocking GPU/UI
            # synchronization then happens without delaying the control loop.
            render_model = mujoco.MjModel.from_xml_path(self.xml_path)
            render_data = mujoco.MjData(render_model)
            render_model.vis.headlight.ambient = self.mj_model.vis.headlight.ambient
            render_model.vis.headlight.diffuse = self.mj_model.vis.headlight.diffuse
            render_model.vis.headlight.specular = self.mj_model.vis.headlight.specular
            with self._simulation_lock:
                mujoco.mj_copyData(render_data, render_model, self.mj_data)

            with mj_viewer.launch_passive(render_model, render_data) as viewer:
                # Set up viewer camera
                camera_id = (
                    mujoco.mj_name2id(render_model, mujoco.mjtObj.mjOBJ_CAMERA, self.viewer_camera)
                    if self.viewer_camera is not None
                    else -1
                )
                if camera_id >= 0:
                    viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
                    viewer.cam.fixedcamid = camera_id
                else:
                    viewer.cam.azimuth = 0
                    viewer.cam.elevation = -50
                    viewer.cam.distance = 2.0
                    viewer.cam.lookat = [0.2, 0, 0]

                control_thread = threading.Thread(
                    target=self._control_loop,
                    name="mujoco-control-100hz",
                    daemon=True,
                )
                control_thread.start()

                render_period = 1.0 / self.render_hz
                next_render_deadline = time.monotonic()
                while not self._stop_event.is_set() and viewer.is_running():
                    render_start_ns = time.perf_counter_ns()
                    with self._simulation_lock:
                        mujoco.mj_copyData(render_data, render_model, self.mj_data)
                    viewer.sync()
                    self._last_render_ms = (
                        time.perf_counter_ns() - render_start_ns
                    ) / 1e6

                    next_render_deadline += render_period
                    sleep_duration = next_render_deadline - time.monotonic()
                    if sleep_duration > 0.0:
                        self._stop_event.wait(sleep_duration)
                    else:
                        next_render_deadline = time.monotonic()

                self._stop_event.set()
                control_thread.join(timeout=2.0)
                if control_thread.is_alive():
                    raise RuntimeError("MuJoCo control thread did not stop within 2 seconds")
                if self._control_exception is not None:
                    raise RuntimeError("MuJoCo control loop failed") from self._control_exception
        except KeyboardInterrupt:
            print("\nTeleoperation stopped.")
            self._stop_event.set()
        finally:
            self._stop_event.set()
            if control_thread is not None and control_thread.is_alive():
                control_thread.join(timeout=2.0)
            try:
                self.xr_client.close()
            finally:
                self._stop_telemetry_logging()

    def _control_loop(self):
        next_control_deadline = time.monotonic()
        try:
            while not self._stop_event.is_set():
                cycle_start = time.monotonic()
                deadline_late_ms = max(0.0, (cycle_start - next_control_deadline) * 1000.0)
                if deadline_late_ms > self.dt * 1000.0 * 0.1:
                    self._deadline_miss_count += 1

                with self._simulation_lock:
                    self._control_cycle(deadline_late_ms=deadline_late_ms)

                next_control_deadline += self.dt
                sleep_duration = next_control_deadline - time.monotonic()
                if sleep_duration > 0.0:
                    self._stop_event.wait(sleep_duration)
                elif sleep_duration < -self.dt:
                    # Never issue a burst of stale commands after an IK or OS
                    # scheduling stall. Resume from the current monotonic time.
                    next_control_deadline = time.monotonic()
        except Exception as error:
            self._control_exception = error
            self._stop_event.set()
