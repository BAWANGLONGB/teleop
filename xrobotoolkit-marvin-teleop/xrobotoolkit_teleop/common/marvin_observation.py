"""Synchronized observation contracts for ROS 2 and MuJoCo calibration logs."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from xrobotoolkit_teleop.common.marvin_types import MARVIN_JOINT_COUNT


def _readonly_array(value, shape, name):
    result = np.asarray(value, dtype=float).copy()
    if result.shape != shape or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must be a finite array with shape {shape}")
    result.setflags(write=False)
    return result


@dataclass(frozen=True)
class MarvinControlObservation:
    """One complete 100 Hz control snapshot, ordered left then right."""

    sequence: int
    monotonic_ns: int
    duration_ms: float
    deadline_lateness_ms: float
    deadline_miss: bool
    xr_sequence: int
    xr_source_timestamp_ns: int | None
    xr_poll_age_ms: float
    xr_source_age_ms: float
    q_ik_rad: np.ndarray
    q_command_rad: np.ndarray
    active_arms: tuple[bool, bool]
    raw_tcp_transforms: np.ndarray
    limited_tcp_transforms: np.ndarray
    actual_tcp_transforms: np.ndarray
    translational_sigma_min: np.ndarray

    def __post_init__(self):
        if self.sequence <= 0 or self.monotonic_ns <= 0:
            raise ValueError("observation sequence and monotonic timestamp must be positive")
        if len(self.active_arms) != 2:
            raise ValueError("active_arms must contain left/right values")
        object.__setattr__(
            self,
            "q_ik_rad",
            _readonly_array(self.q_ik_rad, (MARVIN_JOINT_COUNT,), "q_ik_rad"),
        )
        object.__setattr__(
            self,
            "q_command_rad",
            _readonly_array(
                self.q_command_rad,
                (MARVIN_JOINT_COUNT,),
                "q_command_rad",
            ),
        )
        for name in (
            "raw_tcp_transforms",
            "limited_tcp_transforms",
            "actual_tcp_transforms",
        ):
            object.__setattr__(self, name, _readonly_array(getattr(self, name), (2, 4, 4), name))
        sigma = np.asarray(self.translational_sigma_min, dtype=float).reshape(-1).copy()
        if sigma.size != 2 or np.any(np.isinf(sigma)):
            raise ValueError("translational_sigma_min must contain two finite-or-NaN values")
        sigma.setflags(write=False)
        object.__setattr__(self, "translational_sigma_min", sigma)

    def age_ms(self, now_ns):
        return max(0.0, (now_ns - self.monotonic_ns) / 1e6)
