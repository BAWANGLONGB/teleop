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
    left_controller_pose: np.ndarray
    right_controller_pose: np.ndarray
    grip_values: tuple[float, float]
    button_a: bool
    button_b: bool
    trigger_values: tuple[float, float] = (0.0, 0.0)
    thumbstick_y_values: tuple[float, float] = (0.0, 0.0)

    def __post_init__(self):
        timestamp_ns = int(self.timestamp_ns)
        if timestamp_ns <= 0:
            raise ValueError("timestamp_ns must be positive")
        object.__setattr__(self, "timestamp_ns", timestamp_ns)
        for field_name in (
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
        for field_name, lower_bound, upper_bound in (
            ("trigger_values", 0.0, 1.0),
            ("thumbstick_y_values", -1.0, 1.0),
        ):
            values = tuple(float(value) for value in getattr(self, field_name))
            if (
                len(values) != 2
                or not np.all(np.isfinite(values))
                or any(
                    value < lower_bound or value > upper_bound
                    for value in values
                )
            ):
                raise ValueError(
                    f"{field_name} must contain two values within "
                    f"[{lower_bound:g}, {upper_bound:g}]"
                )
            object.__setattr__(self, field_name, values)


class XrClient:
    """Read atomic PICO frames from the project-owned XRoboToolkit binding."""

    def __init__(
        self,
        xr_sdk=None,
        max_source_age_seconds=0.5,
        source_disconnect_timeout_seconds=2.0,
    ):
        if max_source_age_seconds <= 0.0:
            raise ValueError("max_source_age_seconds must be positive")
        if source_disconnect_timeout_seconds <= max_source_age_seconds:
            raise ValueError(
                "source_disconnect_timeout_seconds must exceed "
                "max_source_age_seconds"
            )
        if xr_sdk is None:
            from xr_marvin_teleop import _xrobotoolkit_sdk as xr_sdk
        if not hasattr(xr_sdk, "get_snapshot"):
            raise TypeError("XR SDK must provide atomic get_snapshot()")

        self._xr_sdk = xr_sdk
        self._max_source_age_ns = int(max_source_age_seconds * 1e9)
        self._source_disconnect_timeout_ns = int(
            source_disconnect_timeout_seconds * 1e9
        )
        self._last_source_timestamp_ns = None
        self._source_timestamp_change_monotonic_ns = None
        self._is_closed = False
        self._xr_sdk.init()
        print("XRoboToolkit SDK initialized; waiting for PICO data.")

    def _capture_snapshot(self):
        values = self._xr_sdk.get_snapshot()
        return None if values is None else XrSnapshot(**values)

    def _timestamp_is_usable(self, timestamp_ns):
        now_ns = time.monotonic_ns()
        if (
            self._last_source_timestamp_ns is not None
            and timestamp_ns < self._last_source_timestamp_ns
        ):
            raise RuntimeError("PICO XR timestamp regressed")
        if timestamp_ns != self._last_source_timestamp_ns:
            self._last_source_timestamp_ns = timestamp_ns
            self._source_timestamp_change_monotonic_ns = now_ns
            return True
        source_age_ns = now_ns - self._source_timestamp_change_monotonic_ns
        if source_age_ns > self._source_disconnect_timeout_ns:
            raise TimeoutError("PICO XR stream disconnected")
        return source_age_ns <= self._max_source_age_ns

    def wait_for_fresh_snapshot(self, timeout_seconds=2.0):
        deadline = time.monotonic() + timeout_seconds
        previous_timestamp_ns = None
        last_error = None
        while time.monotonic() < deadline:
            try:
                snapshot = self.read_snapshot()
                if (
                    snapshot is not None
                    and previous_timestamp_ns is not None
                    and snapshot.timestamp_ns > previous_timestamp_ns
                ):
                    print("PICO XR stream ready.")
                    return snapshot
                if snapshot is not None:
                    previous_timestamp_ns = snapshot.timestamp_ns
            except Exception as error:
                last_error = error
            time.sleep(0.01)
        raise TimeoutError(
            "PICO produced no advancing Controller snapshot within "
            f"{timeout_seconds:g} seconds"
        ) from last_error

    def read_snapshot(self):
        snapshot = self._capture_snapshot()
        if snapshot is None:
            return None
        return snapshot if self._timestamp_is_usable(snapshot.timestamp_ns) else None

    def close(self):
        if not self._is_closed:
            self._is_closed = True
            self._xr_sdk.close()
