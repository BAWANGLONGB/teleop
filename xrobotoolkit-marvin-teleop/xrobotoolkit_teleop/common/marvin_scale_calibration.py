"""Persistent, PICO-only operator scale calibration for Marvin teleoperation."""

from __future__ import annotations

import hashlib
import json
import os
import tempfile
from datetime import datetime
from pathlib import Path

import numpy as np

from xrobotoolkit_teleop.utils.geometry import pose_in_head_yaw_frame


SCHEMA_VERSION = 1
DEFAULT_SCALE_FACTOR = 0.5
MIN_SAVED_SCALE_FACTOR = 0.25
MAX_SAVED_SCALE_FACTOR = 1.5

# FK difference between the configured Marvin human-rest pose and the
# symmetric forward-extension pose. This is a pose-pair travel, not total arm
# reach, and is shared with the MuJoCo two-point calibration.
MARVIN_REST_TO_FORWARD_TCP_DELTA = np.array([-0.558866, 0.0, 0.664989])
MARVIN_REST_TO_FORWARD_TCP_TRAVEL = float(
    np.linalg.norm(MARVIN_REST_TO_FORWARD_TCP_DELTA)
)

# OpenXR right/up/back -> Marvin +Y/+Z/+X.
R_PICO_TO_MARVIN_WORLD = np.array(
    [
        [0.0, 0.0, 1.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
    ]
)


def make_marvin_scale_calibration_config(workspace_margin=0.95):
    """Build the shared two-point calibration limits for simulation and hardware."""
    if not 0.0 < workspace_margin <= 1.0:
        raise ValueError("workspace_margin must be in (0, 1]")
    return {
        "robot_motion_range": MARVIN_REST_TO_FORWARD_TCP_TRAVEL,
        "expected_motion_direction": MARVIN_REST_TO_FORWARD_TCP_DELTA,
        "workspace_margin": float(workspace_margin),
        "min_arm_length": 0.35,
        "max_arm_length": 1.0,
        "max_bilateral_difference_ratio": 0.15,
        "max_direction_difference_deg": 20.0,
        "max_expected_direction_error_deg": 25.0,
        "min_scale_factor": MIN_SAVED_SCALE_FACTOR,
        "max_scale_factor": MAX_SAVED_SCALE_FACTOR,
    }


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source_file:
        for chunk in iter(lambda: source_file.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def controller_positions_in_marvin_head_yaw_frame(
    headset_pose,
    left_controller_pose,
    right_controller_pose,
    fallback_yaw_rotation=None,
):
    """Return both controller positions in the real teleop translation frame."""
    headset_pose = np.asarray(headset_pose, dtype=float)
    if headset_pose.shape != (7,) or not np.all(np.isfinite(headset_pose)):
        raise ValueError("headset pose must be a finite 7-vector")
    headset_quaternion = np.array(
        [headset_pose[6], headset_pose[3], headset_pose[4], headset_pose[5]],
        dtype=float,
    )
    positions = {}
    yaw_rotation = fallback_yaw_rotation
    for name, pose in (
        ("left_hand", left_controller_pose),
        ("right_hand", right_controller_pose),
    ):
        pose = np.asarray(pose, dtype=float)
        if pose.shape != (7,) or not np.all(np.isfinite(pose)):
            raise ValueError(f"{name} controller pose must be a finite 7-vector")
        quaternion = np.array([pose[6], pose[3], pose[4], pose[5]], dtype=float)
        position, _, yaw_rotation = pose_in_head_yaw_frame(
            pose[:3],
            quaternion,
            headset_pose[:3],
            headset_quaternion,
            yaw_rotation,
        )
        positions[name] = R_PICO_TO_MARVIN_WORLD @ position
    return positions, yaw_rotation


def _validate_record(record, path):
    if not isinstance(record, dict) or record.get("schema_version") != SCHEMA_VERSION:
        raise ValueError(f"unsupported Marvin scale calibration schema in {path}")
    value = record.get("scale_factor")
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not np.isfinite(value)
        or not MIN_SAVED_SCALE_FACTOR <= float(value) <= MAX_SAVED_SCALE_FACTOR
    ):
        raise ValueError(
            f"saved scale_factor in {path} must be within "
            f"[{MIN_SAVED_SCALE_FACTOR}, {MAX_SAVED_SCALE_FACTOR}]"
        )
    record = dict(record)
    record["scale_factor"] = float(value)
    return record


def load_scale_calibration(path):
    """Load and validate one persisted calibration record."""
    path = Path(path).expanduser().resolve()
    with path.open(encoding="utf-8") as calibration_file:
        record = json.load(calibration_file)
    record = _validate_record(record, path)
    record["path"] = str(path)
    record["sha256"] = _sha256(path)
    return record


def resolve_scale_factor(requested_scale_factor, calibration_path):
    """Resolve explicit CLI > saved PICO calibration > conservative default."""
    if requested_scale_factor is not None:
        value = float(requested_scale_factor)
        if not np.isfinite(value) or value <= 0.0:
            raise ValueError("scale_factor must be a positive finite value")
        return value, {"source": "cli", "path": None, "sha256": None}

    calibration_path = Path(calibration_path).expanduser().resolve()
    if calibration_path.is_file():
        record = load_scale_calibration(calibration_path)
        return record["scale_factor"], {
            "source": "pico_calibration",
            "path": record["path"],
            "sha256": record["sha256"],
            "created_at": record.get("created_at"),
        }
    return DEFAULT_SCALE_FACTOR, {
        "source": "code_default",
        "path": str(calibration_path),
        "sha256": None,
    }


def _write_json_atomic(path: Path, record):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary_path = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            dir=path.parent,
            prefix=f".{path.name}.",
            suffix=".tmp",
            delete=False,
        ) as temporary_file:
            temporary_path = Path(temporary_file.name)
            json.dump(record, temporary_file, indent=2, ensure_ascii=False, allow_nan=False)
            temporary_file.write("\n")
            temporary_file.flush()
            os.fsync(temporary_file.fileno())
        temporary_path.replace(path)
    finally:
        if temporary_path is not None and temporary_path.exists():
            temporary_path.unlink()


def save_scale_calibration(path, result, workspace_margin):
    """Save an immutable history record and atomically update the current record."""
    if result.status != "completed" or result.scale_factor is None:
        raise ValueError("only a completed arm-length calibration can be saved")
    path = Path(path).expanduser().resolve()
    now = datetime.now().astimezone()
    record = {
        "schema_version": SCHEMA_VERSION,
        "created_at": now.isoformat(),
        "source": "pico_two_point_down_to_forward",
        "coordinate_frame": "marvin_head_yaw",
        "scale_factor": float(result.scale_factor),
        "unclamped_scale_factor": float(result.unclamped_scale_factor),
        "workspace_margin": float(workspace_margin),
        "robot_motion_range_m": MARVIN_REST_TO_FORWARD_TCP_TRAVEL,
        "controller_travels_m": result.controller_travels,
        "arm_lengths_m": result.arm_lengths,
        "mean_arm_length_m": float(result.mean_arm_length),
    }
    _validate_record(record, path)
    history_path = path.with_name(
        f"{path.stem}_{now.strftime('%Y%m%d_%H%M%S_%f')}{path.suffix}"
    )
    _write_json_atomic(history_path, record)
    _write_json_atomic(path, record)
    return path, history_path
