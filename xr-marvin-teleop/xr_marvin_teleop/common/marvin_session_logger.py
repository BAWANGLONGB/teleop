"""JSONL logging for XR-to-Marvin control cycles."""

import json
import queue
import threading
import time
from datetime import datetime
from pathlib import Path

import numpy as np


def _json_value(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, (list, tuple)):
        return [_json_value(item) for item in value]
    if isinstance(value, dict):
        return {str(key): _json_value(item) for key, item in value.items()}
    return value


class MarvinSessionLogger:
    """Write replayable control-cycle records without a logging dependency."""

    def __init__(self, output_directory, session_name):
        output_directory = Path(output_directory).expanduser()
        output_directory.mkdir(parents=True, exist_ok=True)
        session_name = "".join(
            character if character.isalnum() or character in "-_" else "_"
            for character in str(session_name)
        ).strip("_")
        if not session_name:
            raise ValueError("session_name must contain a letter or number")
        timestamp = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S_%f")
        self.path = output_directory / f"marvin_{session_name}_{timestamp}.jsonl"
        self._queue = queue.SimpleQueue()
        self._stop_token = object()
        self._writer_error = None
        self._thread = threading.Thread(
            target=self._write_records,
            name="marvin-session-logger",
            daemon=True,
        )
        self._thread.start()

    def _write_records(self):
        try:
            with self.path.open("w", encoding="utf-8") as output_file:
                last_flush_time = time.monotonic()
                while True:
                    record = self._queue.get()
                    if record is self._stop_token:
                        break
                    json.dump(
                        record,
                        output_file,
                        ensure_ascii=False,
                        allow_nan=False,
                    )
                    output_file.write("\n")
                    if time.monotonic() - last_flush_time >= 1.0:
                        output_file.flush()
                        last_flush_time = time.monotonic()
        except Exception as error:
            self._writer_error = error

    def record_control_cycle(
        self,
        xr_snapshot,
        robot_feedback,
        q_command_rad,
        scale_factor,
    ):
        if self._thread is None:
            raise RuntimeError("Marvin session logger is closed")
        record = {
            "schema_version": 1,
            "event": "control_cycle",
            "monotonic_time_ns": time.monotonic_ns(),
            "xr_timestamp_ns": xr_snapshot.timestamp_ns,
            "headset_pose": xr_snapshot.headset_pose,
            "left_controller_pose": xr_snapshot.left_controller_pose,
            "right_controller_pose": xr_snapshot.right_controller_pose,
            "grip_values": xr_snapshot.grip_values,
            "button_a": xr_snapshot.button_a,
            "button_b": xr_snapshot.button_b,
            "scale_factor": scale_factor,
            "frame_serial": robot_feedback.frame_serial,
            "arm_state": robot_feedback.arm_state,
            "error_code": robot_feedback.error_code,
            "q_feedback_rad": robot_feedback.q_rad,
            "dq_feedback_rad_s": robot_feedback.dq_rad_s,
            "q_command_rad": np.asarray(q_command_rad, dtype=float),
        }
        self._queue.put(_json_value(record))

    def close(self):
        if self._thread is not None:
            self._queue.put(self._stop_token)
            self._thread.join(timeout=5.0)
            if self._thread.is_alive():
                raise RuntimeError("Marvin session logger did not stop")
            self._thread = None
            if self._writer_error is not None:
                raise RuntimeError("Marvin session log writer failed") from self._writer_error


def read_marvin_session(path):
    """Return validated control-cycle records from a Marvin JSONL log."""
    records = []
    with Path(path).expanduser().open(encoding="utf-8") as source:
        for line_number, line in enumerate(source, 1):
            try:
                record = json.loads(line)
            except json.JSONDecodeError as error:
                raise ValueError(f"invalid JSON at line {line_number}") from error
            if record.get("event") != "control_cycle":
                continue
            for field_name in ("q_feedback_rad", "q_command_rad"):
                values = np.asarray(record.get(field_name), dtype=float)
                if values.shape != (14,) or not np.all(np.isfinite(values)):
                    raise ValueError(
                        f"{field_name} at line {line_number} must contain 14 finite values"
                    )
            records.append(record)
    return records
