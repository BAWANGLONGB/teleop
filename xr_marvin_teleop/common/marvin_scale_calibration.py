import json
import os
import tempfile
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

import numpy as np


DEFAULT_SCALE_FACTOR = 1.0
MIN_SCALE_FACTOR = 0.5
MAX_SCALE_FACTOR = 1.5
MARVIN_REST_TO_FORWARD_TCP_DELTA = np.array([-0.558866, 0.0, 0.664989])
MARVIN_REST_TO_FORWARD_TCP_TRAVEL = float(
    np.linalg.norm(MARVIN_REST_TO_FORWARD_TCP_DELTA)
)


def resolve_scale_factor(requested_scale_factor, calibration_path):
    if requested_scale_factor is not None:
        value = float(requested_scale_factor)
        if not np.isfinite(value) or value <= 0.0:
            raise ValueError("scale_factor must be positive and finite")
        return value
    calibration_path = Path(calibration_path).expanduser()
    if not calibration_path.is_file():
        return DEFAULT_SCALE_FACTOR
    with calibration_path.open(encoding="utf-8") as calibration_file:
        calibration_record = json.load(calibration_file)
    if (
        not isinstance(calibration_record, dict)
        or calibration_record.get("schema_version") != 1
    ):
        raise ValueError(f"unsupported scale calibration: {calibration_path}")
    value = calibration_record.get("scale_factor")
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not np.isfinite(value)
        or not MIN_SCALE_FACTOR <= float(value) <= MAX_SCALE_FACTOR
    ):
        raise ValueError(f"invalid scale calibration: {calibration_path}")
    return float(value)


def save_scale_calibration(calibration_path, calibration_result):
    if calibration_result.scale_factor is None:
        raise ValueError("only a completed calibration can be saved")
    calibration_path = Path(calibration_path).expanduser().resolve()
    calibration_path.parent.mkdir(parents=True, exist_ok=True)
    calibration_record = {
        "schema_version": 1,
        "created_at": datetime.now().astimezone().isoformat(),
        "scale_factor": calibration_result.scale_factor,
        "controller_travels_m": calibration_result.controller_travels,
        "arm_lengths_m": calibration_result.arm_lengths,
    }
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(
            "w", encoding="utf-8", dir=calibration_path.parent, delete=False
        ) as output:
            temporary = Path(output.name)
            json.dump(
                calibration_record,
                output,
                indent=2,
                ensure_ascii=False,
                allow_nan=False,
            )
            output.write("\n")
            output.flush()
            os.fsync(output.fileno())
        temporary.replace(calibration_path)
    finally:
        if temporary is not None and temporary.exists():
            temporary.unlink()


@dataclass(frozen=True)
class ArmLengthCalibrationResult:
    status: str
    scale_factor: float | None = None
    controller_travels: dict[str, float] | None = None
    arm_lengths: dict[str, float] | None = None


class ArmLengthScaleCalibrator:
    """Two A presses: natural-down, then horizontal-forward."""

    def __init__(self, workspace_margin=0.95):
        if not 0.0 < workspace_margin <= 1.0:
            raise ValueError("workspace_margin must be in (0, 1]")
        self.workspace_margin = float(workspace_margin)
        self._down_pose_positions = None

    def reset(self):
        self._down_pose_positions = None

    @staticmethod
    def _validate_controller_positions(controller_positions):
        validated_positions = {
            arm_name: np.asarray(position, dtype=float).copy()
            for arm_name, position in controller_positions.items()
        }
        if set(validated_positions) != {"left", "right"}:
            raise ValueError("calibration requires left and right positions")
        if any(
            position.shape != (3,) or not np.all(np.isfinite(position))
            for position in validated_positions.values()
        ):
            raise ValueError("calibration positions must be finite 3-vectors")
        return validated_positions

    def capture(self, controller_positions):
        controller_positions = self._validate_controller_positions(
            controller_positions
        )
        if self._down_pose_positions is None:
            self._down_pose_positions = controller_positions
            return ArmLengthCalibrationResult("down_captured")

        controller_displacements = {
            arm_name: controller_positions[arm_name]
            - self._down_pose_positions[arm_name]
            for arm_name in controller_positions
        }
        controller_travels = {
            arm_name: float(np.linalg.norm(displacement))
            for arm_name, displacement in controller_displacements.items()
        }
        arm_lengths = {
            arm_name: travel / np.sqrt(2.0)
            for arm_name, travel in controller_travels.items()
        }
        if any(not 0.35 <= length <= 1.0 for length in arm_lengths.values()):
            raise ValueError("measured arm length must be within [0.35, 1.0] m")
        mean_arm_length = float(np.mean(list(arm_lengths.values())))
        if np.ptp(list(arm_lengths.values())) / mean_arm_length > 0.15:
            raise ValueError("left/right arm-length difference exceeds 15%")

        motion_directions = {
            arm_name: controller_displacements[arm_name]
            / controller_travels[arm_name]
            for arm_name in controller_displacements
        }
        left_right_angle = np.rad2deg(
            np.arccos(
                np.clip(
                    np.dot(
                        motion_directions["left"], motion_directions["right"]
                    ),
                    -1.0,
                    1.0,
                )
            )
        )
        if left_right_angle > 20.0:
            raise ValueError(
                "left/right calibration directions differ by more than 20 degrees"
            )
        expected_motion_direction = (
            MARVIN_REST_TO_FORWARD_TCP_DELTA
            / MARVIN_REST_TO_FORWARD_TCP_TRAVEL
        )
        if any(
            np.rad2deg(
                np.arccos(
                    np.clip(
                        np.dot(direction, expected_motion_direction), -1.0, 1.0
                    )
                )
            )
            > 25.0
            for direction in motion_directions.values()
        ):
            raise ValueError(
                "calibration motion does not point in the Marvin forward direction"
            )

        scale_factor = (
            self.workspace_margin
            * MARVIN_REST_TO_FORWARD_TCP_TRAVEL
            / np.mean(list(controller_travels.values()))
        )
        self.reset()
        return ArmLengthCalibrationResult(
            "completed",
            float(np.clip(scale_factor, MIN_SCALE_FACTOR, MAX_SCALE_FACTOR)),
            controller_travels,
            arm_lengths,
        )
