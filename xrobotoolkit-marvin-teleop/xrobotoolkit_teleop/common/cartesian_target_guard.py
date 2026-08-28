"""Cartesian workspace, rate, and single-frame jump protection."""

from __future__ import annotations

import meshcat.transformations as tf
import numpy as np


class CartesianTargetGuard:
    def __init__(
        self,
        initial_transform,
        dt,
        max_displacement_m=0.25,
        max_linear_speed_m_s=0.1,
        max_angular_speed_rad_s=0.5,
        max_frame_translation_m=0.15,
        max_frame_rotation_rad=np.deg2rad(45.0),
    ):
        initial_transform = np.asarray(initial_transform, dtype=float)
        if initial_transform.shape != (4, 4) or not np.all(np.isfinite(initial_transform)):
            raise ValueError("initial_transform must be a finite 4x4 transform")
        for name, value in (
            ("dt", dt),
            ("max_displacement_m", max_displacement_m),
            ("max_linear_speed_m_s", max_linear_speed_m_s),
            ("max_angular_speed_rad_s", max_angular_speed_rad_s),
            ("max_frame_translation_m", max_frame_translation_m),
            ("max_frame_rotation_rad", max_frame_rotation_rad),
        ):
            if value <= 0.0:
                raise ValueError(f"{name} must be positive")
        self.initial_transform = initial_transform.copy()
        self.safe_transform = initial_transform.copy()
        self.last_raw_transform = initial_transform.copy()
        self.dt = float(dt)
        self.max_displacement_m = float(max_displacement_m)
        self.max_linear_speed_m_s = float(max_linear_speed_m_s)
        self.max_angular_speed_rad_s = float(max_angular_speed_rad_s)
        self.max_frame_translation_m = float(max_frame_translation_m)
        self.max_frame_rotation_rad = float(max_frame_rotation_rad)

    @staticmethod
    def _rotation_angle(left, right):
        relative = left[:3, :3].T @ right[:3, :3]
        cosine = np.clip((np.trace(relative) - 1.0) * 0.5, -1.0, 1.0)
        return float(np.arccos(cosine))

    def reset(self, transform):
        transform = np.asarray(transform, dtype=float)
        if transform.shape != (4, 4) or not np.all(np.isfinite(transform)):
            raise ValueError("reset transform must be finite 4x4")
        self.safe_transform = transform.copy()
        self.last_raw_transform = transform.copy()

    def rebase(self, transform):
        """Set a newly measured startup pose as the workspace and rate-limit origin."""
        self.reset(transform)
        self.initial_transform = np.asarray(transform, dtype=float).copy()

    def filter(self, raw_transform, speed_scale=1.0):
        raw = np.asarray(raw_transform, dtype=float)
        if raw.shape != (4, 4) or not np.all(np.isfinite(raw)):
            raise RuntimeError("TCP target contains NaN, Inf, or has an invalid shape")
        if not 0.0 < speed_scale <= 1.0:
            raise ValueError("speed_scale must be in (0, 1]")

        frame_translation = np.linalg.norm(
            raw[:3, 3] - self.last_raw_transform[:3, 3]
        )
        frame_rotation = self._rotation_angle(self.last_raw_transform, raw)
        if frame_translation > self.max_frame_translation_m:
            raise RuntimeError(
                f"single-frame TCP translation jump {frame_translation:.3f} m exceeds "
                f"{self.max_frame_translation_m:.3f} m"
            )
        if frame_rotation > self.max_frame_rotation_rad:
            raise RuntimeError(
                f"single-frame TCP rotation jump {np.rad2deg(frame_rotation):.1f} deg exceeds "
                f"{np.rad2deg(self.max_frame_rotation_rad):.1f} deg"
            )
        self.last_raw_transform = raw.copy()

        target = raw.copy()
        displacement = target[:3, 3] - self.initial_transform[:3, 3]
        displacement_norm = np.linalg.norm(displacement)
        if displacement_norm > self.max_displacement_m:
            target[:3, 3] = self.initial_transform[:3, 3] + (
                displacement * self.max_displacement_m / displacement_norm
            )

        delta = target[:3, 3] - self.safe_transform[:3, 3]
        max_translation = self.max_linear_speed_m_s * speed_scale * self.dt
        delta_norm = np.linalg.norm(delta)
        if delta_norm > max_translation:
            target[:3, 3] = self.safe_transform[:3, 3] + delta * max_translation / delta_norm

        angle = self._rotation_angle(self.safe_transform, target)
        max_angle = self.max_angular_speed_rad_s * speed_scale * self.dt
        if angle > max_angle:
            start_quaternion = tf.quaternion_from_matrix(self.safe_transform)
            target_quaternion = tf.quaternion_from_matrix(target)
            quaternion = tf.quaternion_slerp(
                start_quaternion,
                target_quaternion,
                max_angle / angle,
            )
            rotation = tf.quaternion_matrix(quaternion)
            target[:3, :3] = rotation[:3, :3]

        self.safe_transform = target.copy()
        return target
