"""Thread-safe data contracts shared by the Marvin hardware pipeline."""

from __future__ import annotations

import threading
from dataclasses import dataclass
from typing import Generic, TypeVar

import numpy as np


MARVIN_ARM_NAMES = ("left", "right")
MARVIN_SDK_ARMS = ("A", "B")
MARVIN_JOINT_COUNT = 14


def _vector(value, length, name):
    result = np.asarray(value, dtype=float).reshape(-1).copy()
    if result.size != length:
        raise ValueError(f"{name} must have length {length}, got {result.size}")
    if not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain only finite values")
    result.setflags(write=False)
    return result


@dataclass(frozen=True)
class MarvinRobotState:
    receipt_monotonic_ns: int
    frame_serial: tuple[int, int]
    q_rad: np.ndarray
    dq_rad_s: np.ndarray
    torque_nm: np.ndarray
    arm_state: tuple[int, int]
    command_state: tuple[int, int]
    error_code: tuple[int, int]
    low_speed: tuple[bool, bool]
    input_frame_serial: tuple[int, int] = (0, 0)
    frame_miss_count: tuple[int, int] = (0, 0)
    system_cycle_miss_count: tuple[int, int] = (0, 0)
    commanded_q_rad: np.ndarray | None = None

    def __post_init__(self):
        if self.receipt_monotonic_ns <= 0:
            raise ValueError("receipt_monotonic_ns must be positive")
        pair_fields = (
            "frame_serial",
            "arm_state",
            "command_state",
            "error_code",
            "low_speed",
            "input_frame_serial",
            "frame_miss_count",
            "system_cycle_miss_count",
        )
        for name in pair_fields:
            if len(getattr(self, name)) != 2:
                raise ValueError(f"{name} must contain left/right values")
        if any(value < 0 for value in self.frame_serial):
            raise ValueError("frame_serial values must be non-negative")
        object.__setattr__(self, "q_rad", _vector(self.q_rad, MARVIN_JOINT_COUNT, "q_rad"))
        object.__setattr__(self, "dq_rad_s", _vector(self.dq_rad_s, MARVIN_JOINT_COUNT, "dq_rad_s"))
        object.__setattr__(self, "torque_nm", _vector(self.torque_nm, MARVIN_JOINT_COUNT, "torque_nm"))
        if self.commanded_q_rad is not None:
            object.__setattr__(
                self,
                "commanded_q_rad",
                _vector(self.commanded_q_rad, MARVIN_JOINT_COUNT, "commanded_q_rad"),
            )

    def age_ms(self, now_ns=None):
        import time

        if now_ns is None:
            now_ns = time.monotonic_ns()
        return max(0.0, (now_ns - self.receipt_monotonic_ns) / 1e6)


@dataclass(frozen=True)
class MarvinJointCommand:
    sequence: int
    created_monotonic_ns: int
    q_rad: np.ndarray
    active_arms: tuple[bool, bool]
    returning_arms: tuple[bool, bool] = (False, False)

    def __post_init__(self):
        if self.sequence <= 0:
            raise ValueError("command sequence must be positive")
        if self.created_monotonic_ns <= 0:
            raise ValueError("created_monotonic_ns must be positive")
        if len(self.active_arms) != 2:
            raise ValueError("active_arms must contain left/right values")
        if len(self.returning_arms) != 2:
            raise ValueError("returning_arms must contain left/right values")
        object.__setattr__(self, "q_rad", _vector(self.q_rad, MARVIN_JOINT_COUNT, "q_rad"))

    def age_ms(self, now_ns=None):
        import time

        if now_ns is None:
            now_ns = time.monotonic_ns()
        return max(0.0, (now_ns - self.created_monotonic_ns) / 1e6)


T = TypeVar("T")


class LatestValue(Generic[T]):
    """A one-element atomic slot; consumers never build a stale FIFO backlog."""

    def __init__(self, value: T | None = None):
        self._value = value
        self._lock = threading.Lock()

    def set(self, value: T):
        with self._lock:
            self._value = value

    def get(self) -> T | None:
        with self._lock:
            return self._value
