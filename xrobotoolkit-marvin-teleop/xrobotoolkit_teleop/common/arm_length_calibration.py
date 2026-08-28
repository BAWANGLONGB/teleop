"""Two-point operator arm-length calibration for Cartesian teleoperation."""

from dataclasses import dataclass
from typing import Mapping

import numpy as np


@dataclass(frozen=True)
class ArmLengthCalibrationResult:
    """Result of one calibration button press."""

    status: str
    controller_travels: dict[str, float] | None = None
    arm_lengths: dict[str, float] | None = None
    mean_arm_length: float | None = None
    scale_factor: float | None = None
    unclamped_scale_factor: float | None = None
    message: str = ""


class ArmLengthScaleCalibrator:
    """Estimate a scalar motion gain from down/forward arm-pose samples.

    The first sample is captured with both arms hanging naturally. The second
    is captured with both arms straight forward. These human poses correspond
    to the robot's configured down and forward-extension poses, so the gain is
    based on like-for-like TCP travel instead of total kinematic arm length.
    """

    def __init__(
        self,
        robot_motion_range: float,
        expected_motion_direction: np.ndarray | None = None,
        workspace_margin: float = 0.95,
        min_arm_length: float = 0.35,
        max_arm_length: float = 1.0,
        max_bilateral_difference_ratio: float = 0.15,
        max_direction_difference_deg: float = 20.0,
        max_expected_direction_error_deg: float = 25.0,
        min_scale_factor: float = 0.25,
        max_scale_factor: float = 1.5,
    ):
        if robot_motion_range <= 0.0:
            raise ValueError("robot_motion_range must be positive")
        if not 0.0 < workspace_margin <= 1.0:
            raise ValueError("workspace_margin must be in (0, 1]")
        if not 0.0 < min_arm_length < max_arm_length:
            raise ValueError("arm-length limits must satisfy 0 < min < max")
        if not 0.0 <= max_bilateral_difference_ratio < 1.0:
            raise ValueError("max_bilateral_difference_ratio must be in [0, 1)")
        if not 0.0 < max_direction_difference_deg < 180.0:
            raise ValueError("max_direction_difference_deg must be in (0, 180)")
        if not 0.0 < max_expected_direction_error_deg < 180.0:
            raise ValueError("max_expected_direction_error_deg must be in (0, 180)")
        if not 0.0 < min_scale_factor <= max_scale_factor:
            raise ValueError("scale-factor limits must satisfy 0 < min <= max")

        self.robot_motion_range = float(robot_motion_range)
        self.expected_motion_direction = None
        if expected_motion_direction is not None:
            direction = np.asarray(expected_motion_direction, dtype=float)
            if direction.shape != (3,) or not np.all(np.isfinite(direction)):
                raise ValueError("expected_motion_direction must be a finite 3-vector")
            direction_norm = float(np.linalg.norm(direction))
            if direction_norm <= 1e-9:
                raise ValueError("expected_motion_direction must be non-zero")
            self.expected_motion_direction = direction / direction_norm
        self.workspace_margin = float(workspace_margin)
        self.min_arm_length = float(min_arm_length)
        self.max_arm_length = float(max_arm_length)
        self.max_bilateral_difference_ratio = float(max_bilateral_difference_ratio)
        self.max_direction_difference_deg = float(max_direction_difference_deg)
        self.max_expected_direction_error_deg = float(max_expected_direction_error_deg)
        self.min_scale_factor = float(min_scale_factor)
        self.max_scale_factor = float(max_scale_factor)
        self._retracted_positions: dict[str, np.ndarray] | None = None

    @property
    def awaiting_extended_sample(self) -> bool:
        return self._retracted_positions is not None

    def reset(self) -> None:
        self._retracted_positions = None

    @staticmethod
    def _copy_positions(positions: Mapping[str, np.ndarray]) -> dict[str, np.ndarray]:
        copied = {name: np.asarray(value, dtype=float).copy() for name, value in positions.items()}
        if len(copied) < 2:
            raise ValueError("arm-length calibration requires two controller positions")
        for name, position in copied.items():
            if position.shape != (3,) or not np.all(np.isfinite(position)):
                raise ValueError(f"invalid controller position for {name}: {position}")
        return copied

    def capture(self, positions: Mapping[str, np.ndarray]) -> ArmLengthCalibrationResult:
        """Capture one stage; returns a result without changing scale itself."""
        current_positions = self._copy_positions(positions)

        if self._retracted_positions is None:
            self._retracted_positions = current_positions
            return ArmLengthCalibrationResult(
                status="down_pose_captured",
                message="自然下垂起点已记录；请保持躯干朝向不变，双臂水平向前伸直后再次按标定键。",
            )

        if set(current_positions) != set(self._retracted_positions):
            return ArmLengthCalibrationResult(
                status="rejected",
                message="两次采样的控制器集合不一致；保留下垂起点，请重新采集前伸点。",
            )

        controller_deltas = {
            name: current_positions[name] - self._retracted_positions[name] for name in current_positions
        }
        controller_travels = {
            name: float(np.linalg.norm(delta))
            for name, delta in controller_deltas.items()
        }
        # Natural-down and horizontal-forward poses are approximately 90
        # degrees apart around the shoulder, so their chord is sqrt(2) times
        # the shoulder-to-controller radius.
        arm_lengths = {
            name: travel / np.sqrt(2.0) for name, travel in controller_travels.items()
        }
        invalid = {
            name: length
            for name, length in arm_lengths.items()
            if not self.min_arm_length <= length <= self.max_arm_length
        }
        if invalid:
            details = ", ".join(f"{name}={length:.3f} m" for name, length in invalid.items())
            return ArmLengthCalibrationResult(
                status="rejected",
                controller_travels=controller_travels,
                arm_lengths=arm_lengths,
                message=(
                    f"臂长超出允许范围 [{self.min_arm_length:.2f}, {self.max_arm_length:.2f}] m: "
                    f"{details}；保留下垂起点，请重新采集前伸点。"
                ),
            )

        lengths = np.asarray(list(arm_lengths.values()), dtype=float)
        mean_arm_length = float(np.mean(lengths))
        bilateral_difference_ratio = float(np.ptp(lengths) / mean_arm_length)
        if bilateral_difference_ratio > self.max_bilateral_difference_ratio:
            details = ", ".join(f"{name}={length:.3f} m" for name, length in arm_lengths.items())
            return ArmLengthCalibrationResult(
                status="rejected",
                controller_travels=controller_travels,
                arm_lengths=arm_lengths,
                mean_arm_length=mean_arm_length,
                message=(
                    f"左右测量差异 {bilateral_difference_ratio:.1%} 过大: {details}；"
                    "保留下垂起点，请对称前伸后重新采样。"
                ),
            )

        directions = {
            name: controller_deltas[name] / controller_travels[name] for name in current_positions
        }
        direction_values = list(directions.values())
        direction_cosine = float(np.clip(np.dot(direction_values[0], direction_values[1]), -1.0, 1.0))
        direction_difference_deg = float(np.rad2deg(np.arccos(direction_cosine)))
        if direction_difference_deg > self.max_direction_difference_deg:
            return ArmLengthCalibrationResult(
                status="rejected",
                controller_travels=controller_travels,
                arm_lengths=arm_lengths,
                mean_arm_length=mean_arm_length,
                message=(
                    f"左右手柄运动方向相差 {direction_difference_deg:.1f}°，超过 "
                    f"{self.max_direction_difference_deg:.1f}°；请双臂同向水平前伸后重新采样。"
                ),
            )

        if self.expected_motion_direction is not None:
            direction_errors = {
                name: float(
                    np.rad2deg(
                        np.arccos(np.clip(np.dot(direction, self.expected_motion_direction), -1.0, 1.0))
                    )
                )
                for name, direction in directions.items()
            }
            if max(direction_errors.values()) > self.max_expected_direction_error_deg:
                details = ", ".join(f"{name}={error:.1f}°" for name, error in direction_errors.items())
                return ArmLengthCalibrationResult(
                    status="rejected",
                    controller_travels=controller_travels,
                    arm_lengths=arm_lengths,
                    mean_arm_length=mean_arm_length,
                    message=(
                        f"下垂到前伸的运动方向与 Marvin 标定方向不一致: {details}；"
                        "请保持躯干正对机器人，手臂从自然下垂移动到水平正前方。"
                    ),
                )

        mean_controller_travel = float(np.mean(list(controller_travels.values())))
        unclamped_scale = self.workspace_margin * self.robot_motion_range / mean_controller_travel
        scale_factor = float(np.clip(unclamped_scale, self.min_scale_factor, self.max_scale_factor))
        self.reset()
        return ArmLengthCalibrationResult(
            status="completed",
            controller_travels=controller_travels,
            arm_lengths=arm_lengths,
            mean_arm_length=mean_arm_length,
            scale_factor=scale_factor,
            unclamped_scale_factor=unclamped_scale,
            message="操作者臂长标定完成。",
        )
