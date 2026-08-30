import time
from dataclasses import dataclass

import numpy as np


def _validate_openxr_pose(value, field_name):
    pose = np.asarray(value, dtype=float).reshape(-1).copy()
    if pose.shape != (7,) or not np.all(np.isfinite(pose)):
        raise ValueError(f"{field_name} must be finite [x,y,z,qx,qy,qz,qw]")
    quaternion_norm = np.linalg.norm(pose[3:])
    if not np.isclose(quaternion_norm, 1.0, atol=1e-3):
        raise ValueError(f"{field_name} quaternion must be normalized")
    pose[3:] /= quaternion_norm
    return pose


@dataclass(frozen=True)
class XrSnapshot:
    timestamp_ns: int
    headset_pose: np.ndarray
    left_controller_pose: np.ndarray
    right_controller_pose: np.ndarray
    grip_values: tuple[float, float]
    button_a: bool
    button_b: bool

    def __post_init__(self):
        timestamp_ns = int(self.timestamp_ns)
        if timestamp_ns <= 0:
            raise ValueError("timestamp_ns must be positive")
        object.__setattr__(self, "timestamp_ns", timestamp_ns)
        for field_name in (
            "headset_pose",
            "left_controller_pose",
            "right_controller_pose",
        ):
            object.__setattr__(
                self,
                field_name,
                _validate_openxr_pose(getattr(self, field_name), field_name),
            )
        grip_values = tuple(float(value) for value in self.grip_values)
        if (
            len(grip_values) != 2
            or not np.all(np.isfinite(grip_values))
            or any(value < 0.0 or value > 1.0 for value in grip_values)
        ):
            raise ValueError("grip_values must contain two values within [0, 1]")
        object.__setattr__(self, "grip_values", grip_values)


class XrClient:
    """Synchronous XRoboToolkit client for the minimal PICO input set."""

    def __init__(self, xr_sdk=None, max_source_age_seconds=0.1):
        if max_source_age_seconds <= 0.0:
            raise ValueError("max_source_age_seconds must be positive")
        if xr_sdk is None:
            import xrobotoolkit_sdk as xr_sdk

        self._xr_sdk = xr_sdk
        self._max_source_age_ns = int(max_source_age_seconds * 1e9)
        self._last_source_timestamp_ns = None
        self._source_timestamp_change_monotonic_ns = None
        self._is_closed = False
        self._xr_sdk.init()
        print("XRoboToolkit SDK initialized; waiting for PICO data.")

    def get_pose_by_name(self, name):
        getters = {
            "headset": self._xr_sdk.get_headset_pose,
            "left_controller": self._xr_sdk.get_left_controller_pose,
            "right_controller": self._xr_sdk.get_right_controller_pose,
        }
        if name not in getters:
            raise ValueError(f"unsupported XR pose: {name}")
        return getters[name]()

    def get_key_value_by_name(self, name):
        getters = {
            "left_grip": self._xr_sdk.get_left_grip,
            "right_grip": self._xr_sdk.get_right_grip,
        }
        if name not in getters:
            raise ValueError(f"unsupported XR key: {name}")
        return float(getters[name]())

    def get_button_state_by_name(self, name):
        getters = {
            "A": self._xr_sdk.get_A_button,
            "B": self._xr_sdk.get_B_button,
        }
        if name not in getters:
            raise ValueError(f"unsupported XR button: {name}")
        return bool(getters[name]())

    def get_timestamp_ns(self):
        return int(self._xr_sdk.get_time_stamp_ns())

    def _capture_consistent_snapshot(self):
        for _ in range(3):
            timestamp_before_ns = self.get_timestamp_ns()
            snapshot = XrSnapshot(
                timestamp_ns=timestamp_before_ns,
                headset_pose=self.get_pose_by_name("headset"),
                left_controller_pose=self.get_pose_by_name("left_controller"),
                right_controller_pose=self.get_pose_by_name("right_controller"),
                grip_values=(
                    self.get_key_value_by_name("left_grip"),
                    self.get_key_value_by_name("right_grip"),
                ),
                button_a=self.get_button_state_by_name("A"),
                button_b=self.get_button_state_by_name("B"),
            )
            timestamp_after_ns = self.get_timestamp_ns()
            if timestamp_before_ns > 0 and timestamp_before_ns == timestamp_after_ns:
                return snapshot
        raise RuntimeError("PICO XR frame changed while reading one snapshot")

    def _require_fresh_timestamp(self, timestamp_ns):
        now_ns = time.monotonic_ns()
        if (
            self._last_source_timestamp_ns is not None
            and timestamp_ns < self._last_source_timestamp_ns
        ):
            raise RuntimeError("PICO XR timestamp regressed")
        if timestamp_ns != self._last_source_timestamp_ns:
            self._last_source_timestamp_ns = timestamp_ns
            self._source_timestamp_change_monotonic_ns = now_ns
            return
        if (
            self._source_timestamp_change_monotonic_ns is None
            or now_ns - self._source_timestamp_change_monotonic_ns
            > self._max_source_age_ns
        ):
            raise TimeoutError("PICO XR stream is stale")

    def wait_for_fresh_snapshot(self, timeout_seconds=2.0):
        deadline = time.monotonic() + timeout_seconds
        previous_timestamp_ns = None
        last_error = None
        while time.monotonic() < deadline:
            try:
                snapshot = self._capture_consistent_snapshot()
                if (
                    previous_timestamp_ns is not None
                    and snapshot.timestamp_ns > previous_timestamp_ns
                ):
                    self._require_fresh_timestamp(snapshot.timestamp_ns)
                    print("PICO XR stream ready.")
                    return snapshot
                previous_timestamp_ns = snapshot.timestamp_ns
            except Exception as error:
                last_error = error
            time.sleep(0.01)
        raise TimeoutError(
            "PICO produced no advancing Head/Controller snapshot within "
            f"{timeout_seconds:g} seconds"
        ) from last_error

    def read_snapshot(self):
        snapshot = self._capture_consistent_snapshot()
        self._require_fresh_timestamp(snapshot.timestamp_ns)
        return snapshot

    def close(self):
        if not self._is_closed:
            self._is_closed = True
            self._xr_sdk.close()
