"""Lossless background CSV recording for real-to-MuJoCo calibration."""

from __future__ import annotations

import csv
import json
import queue
import threading
import time
from datetime import datetime
from pathlib import Path

import numpy as np

from xrobotoolkit_teleop.common.marvin_observation import MarvinControlObservation
from xrobotoolkit_teleop.common.marvin_types import MarvinJointCommand, MarvinRobotState


def _json_value(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {str(key): _json_value(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_value(item) for item in value]
    return value


class MarvinCalibrationRecorder:
    """Record every hardware feedback sample with its latest control context."""

    def __init__(self, output_dir, metadata, joint_names):
        if len(joint_names) != 14:
            raise ValueError("joint_names must contain 14 names")
        timestamp = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S_%f")
        output_dir = Path(output_dir).expanduser()
        output_dir.mkdir(parents=True, exist_ok=True)
        self.csv_path = output_dir / f"marvin_calibration_{timestamp}.csv"
        self.metadata_path = output_dir / f"marvin_calibration_{timestamp}.metadata.json"
        self._metadata = _json_value(metadata)
        self._joint_names = tuple(joint_names)
        self._fieldnames = self._make_fieldnames()
        self._queue = queue.SimpleQueue()
        self._stop_token = object()
        self._error = None
        self._sample_count = 0
        self._started_at = datetime.now().astimezone()
        self._started_monotonic = time.monotonic()
        self._q_min = np.full(14, np.inf)
        self._q_max = np.full(14, -np.inf)
        self._max_abs_dq = np.zeros(14)
        self._max_abs_tau = np.zeros(14)
        self._first_frame_serial = None
        self._last_frame_serial = None
        self._final_frame_miss_count = None
        self._final_system_cycle_miss_count = None
        # Open synchronously: requested recording must be writable before the
        # controller is allowed to enter an enabled hardware mode.
        self._csv_file = self.csv_path.open("w", newline="", encoding="utf-8")
        self._writer_object = csv.DictWriter(self._csv_file, fieldnames=self._fieldnames)
        self._writer_object.writeheader()
        self._csv_file.flush()
        self._thread = threading.Thread(
            target=self._writer,
            name="marvin-calibration-recorder",
            daemon=True,
        )
        self._thread.start()

    @property
    def error(self):
        return self._error

    def _make_fieldnames(self):
        fields = [
            "sample_index",
            "wall_time_ns",
            "host_monotonic_ns",
            "robot_receipt_monotonic_ns",
            "sdk_read_duration_ms",
            "frame_serial_A",
            "frame_serial_B",
            "input_frame_serial_A",
            "input_frame_serial_B",
            "frame_miss_count_A",
            "frame_miss_count_B",
            "system_cycle_miss_count_A",
            "system_cycle_miss_count_B",
            "arm_state_A",
            "arm_state_B",
            "command_state_A",
            "command_state_B",
            "error_code_A",
            "error_code_B",
            "low_speed_A",
            "low_speed_B",
            "safety_state",
            "safety_reason",
            "software_command_sequence",
            "software_command_age_ms",
            "active_left",
            "active_right",
            "returning_left",
            "returning_right",
            "scale_factor",
            "control_observation_sequence",
            "control_observation_age_ms",
            "control_duration_ms",
            "control_deadline_lateness_ms",
            "control_deadline_miss",
            "xr_sequence",
            "xr_source_timestamp_ns",
            "xr_poll_age_ms",
            "xr_source_age_ms",
            "sigma_min_left",
            "sigma_min_right",
        ]
        for prefix in (
            "q_rad",
            "dq_rad_s",
            "tau_nm",
            "controller_q_command_rad",
            "software_q_command_rad",
            "q_ik_rad",
        ):
            fields.extend(f"{prefix}_{name}" for name in self._joint_names)
        for pose_kind in ("raw_target", "limited_target", "actual"):
            for arm in ("left", "right"):
                fields.extend(
                    f"{pose_kind}_{arm}_T_{row}{column}"
                    for row in range(4)
                    for column in range(4)
                )
        return fields

    @staticmethod
    def _put_vector(row, prefix, values, joint_names):
        if values is None:
            values = [""] * len(joint_names)
        for name, value in zip(joint_names, values):
            row[f"{prefix}_{name}"] = value

    @staticmethod
    def _put_transform(row, prefix, transform):
        values = [""] * 16 if transform is None else np.asarray(transform).reshape(-1)
        for index, value in enumerate(values):
            row[f"{prefix}_T_{index // 4}{index % 4}"] = value

    def record(
        self,
        state: MarvinRobotState,
        command: MarvinJointCommand | None,
        control: MarvinControlObservation | None,
        safety_state,
        safety_reason,
        sdk_read_duration_ms,
        scale_factor=None,
    ):
        if self._error is not None:
            return
        now_ns = time.monotonic_ns()
        row = {
            "wall_time_ns": time.time_ns(),
            "host_monotonic_ns": now_ns,
            "robot_receipt_monotonic_ns": state.receipt_monotonic_ns,
            "sdk_read_duration_ms": sdk_read_duration_ms,
            "frame_serial_A": state.frame_serial[0],
            "frame_serial_B": state.frame_serial[1],
            "input_frame_serial_A": state.input_frame_serial[0],
            "input_frame_serial_B": state.input_frame_serial[1],
            "frame_miss_count_A": state.frame_miss_count[0],
            "frame_miss_count_B": state.frame_miss_count[1],
            "system_cycle_miss_count_A": state.system_cycle_miss_count[0],
            "system_cycle_miss_count_B": state.system_cycle_miss_count[1],
            "arm_state_A": state.arm_state[0],
            "arm_state_B": state.arm_state[1],
            "command_state_A": state.command_state[0],
            "command_state_B": state.command_state[1],
            "error_code_A": state.error_code[0],
            "error_code_B": state.error_code[1],
            "low_speed_A": int(state.low_speed[0]),
            "low_speed_B": int(state.low_speed[1]),
            "safety_state": safety_state,
            "safety_reason": safety_reason,
            "software_command_sequence": "" if command is None else command.sequence,
            "software_command_age_ms": "" if command is None else command.age_ms(now_ns),
            "active_left": "" if command is None else int(command.active_arms[0]),
            "active_right": "" if command is None else int(command.active_arms[1]),
            "returning_left": "" if command is None else int(command.returning_arms[0]),
            "returning_right": "" if command is None else int(command.returning_arms[1]),
            "scale_factor": "" if scale_factor is None else float(scale_factor),
            "control_observation_sequence": "" if control is None else control.sequence,
            "control_observation_age_ms": "" if control is None else control.age_ms(now_ns),
            "control_duration_ms": "" if control is None else control.duration_ms,
            "control_deadline_lateness_ms": (
                "" if control is None else control.deadline_lateness_ms
            ),
            "control_deadline_miss": "" if control is None else int(control.deadline_miss),
            "xr_sequence": "" if control is None else control.xr_sequence,
            "xr_source_timestamp_ns": (
                ""
                if control is None or control.xr_source_timestamp_ns is None
                else control.xr_source_timestamp_ns
            ),
            "xr_poll_age_ms": "" if control is None else control.xr_poll_age_ms,
            "xr_source_age_ms": "" if control is None else control.xr_source_age_ms,
            "sigma_min_left": (
                "" if control is None else control.translational_sigma_min[0]
            ),
            "sigma_min_right": (
                "" if control is None else control.translational_sigma_min[1]
            ),
        }
        self._put_vector(row, "q_rad", state.q_rad, self._joint_names)
        self._put_vector(row, "dq_rad_s", state.dq_rad_s, self._joint_names)
        self._put_vector(row, "tau_nm", state.torque_nm, self._joint_names)
        self._put_vector(
            row,
            "controller_q_command_rad",
            state.commanded_q_rad,
            self._joint_names,
        )
        self._put_vector(
            row,
            "software_q_command_rad",
            None if command is None else command.q_rad,
            self._joint_names,
        )
        self._put_vector(
            row,
            "q_ik_rad",
            None if control is None else control.q_ik_rad,
            self._joint_names,
        )
        transforms = (
            None if control is None else control.raw_tcp_transforms,
            None if control is None else control.limited_tcp_transforms,
            None if control is None else control.actual_tcp_transforms,
        )
        for pose_kind, pose_transforms in zip(
            ("raw_target", "limited_target", "actual"), transforms
        ):
            for arm_index, arm in enumerate(("left", "right")):
                self._put_transform(
                    row,
                    f"{pose_kind}_{arm}",
                    None if pose_transforms is None else pose_transforms[arm_index],
                )
        self._queue.put((row, state))

    def _writer(self):
        last_flush = time.monotonic()
        try:
            while True:
                item = self._queue.get()
                if item is self._stop_token:
                    break
                row, state = item
                row["sample_index"] = self._sample_count
                self._writer_object.writerow(row)
                self._sample_count += 1
                self._q_min = np.minimum(self._q_min, state.q_rad)
                self._q_max = np.maximum(self._q_max, state.q_rad)
                self._max_abs_dq = np.maximum(self._max_abs_dq, np.abs(state.dq_rad_s))
                self._max_abs_tau = np.maximum(self._max_abs_tau, np.abs(state.torque_nm))
                if self._first_frame_serial is None:
                    self._first_frame_serial = state.frame_serial
                self._last_frame_serial = state.frame_serial
                self._final_frame_miss_count = state.frame_miss_count
                self._final_system_cycle_miss_count = state.system_cycle_miss_count
                if time.monotonic() - last_flush >= 1.0:
                    self._csv_file.flush()
                    last_flush = time.monotonic()
            self._csv_file.flush()
        except Exception as error:
            self._error = error
        finally:
            self._csv_file.close()

    def close(self, terminal_state=None):
        if self._thread is None:
            return
        self._queue.put(self._stop_token)
        self._thread.join(timeout=10.0)
        if self._thread.is_alive():
            self._error = RuntimeError("calibration writer did not stop within 10 seconds")
        ended_at = datetime.now().astimezone()
        valid_samples = self._sample_count > 0
        summary = {
            "schema_version": 1,
            "purpose": "Marvin real-hardware data for MuJoCo parameter calibration",
            "data_contract": {
                "joint_order": self._joint_names,
                "position_unit": "rad",
                "velocity_unit": "rad/s",
                "acceleration_unit_after_npz_conversion": "rad/s^2",
                "torque_unit": "N*m",
                "tcp_transform": "T_dual_origin_tcp, row-major 4x4",
                "host_timing": "time.monotonic_ns",
                "control_alignment": "latest complete control snapshot at feedback receipt",
                "actual_tcp_source": "URDF FK evaluated at measured joint position",
            },
            "started_at": self._started_at.isoformat(),
            "ended_at": ended_at.isoformat(),
            "duration_s": time.monotonic() - self._started_monotonic,
            "sample_count": self._sample_count,
            "csv_path": str(self.csv_path.resolve()),
            "terminal_state": terminal_state,
            "writer_error": None if self._error is None else repr(self._error),
            "first_frame_serial": self._first_frame_serial,
            "last_frame_serial": self._last_frame_serial,
            "final_frame_miss_count": self._final_frame_miss_count,
            "final_system_cycle_miss_count": self._final_system_cycle_miss_count,
            "q_min_rad": self._q_min if valid_samples else None,
            "q_max_rad": self._q_max if valid_samples else None,
            "max_abs_dq_rad_s": self._max_abs_dq if valid_samples else None,
            "max_abs_tau_nm": self._max_abs_tau if valid_samples else None,
            "configuration": self._metadata,
        }
        with self.metadata_path.open("w", encoding="utf-8") as metadata_file:
            json.dump(
                _json_value(summary),
                metadata_file,
                ensure_ascii=False,
                indent=2,
                allow_nan=False,
            )
            metadata_file.write("\n")
        self._thread = None
