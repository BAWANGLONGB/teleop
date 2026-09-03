"""Minimal Gen Finger Controller adapter for the Marvin gripper interface."""

import importlib
import json
import struct
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np


MIN_DAS_DISTANCE_M = 0.0
MAX_DAS_DISTANCE_M = 0.2
ENCODER_ZERO_TOLERANCE_M = 0.001
DAS_ENCODER_CALIBRATION_CODE = -66.66
ARM_NAMES = ("left", "right")
DEFAULT_CAMERA_LATENCY_MS = {"640x480": 25.0}


class DASFingerCalibrationRequired(RuntimeError):
    """The controller needs its first safe distance command to calibrate."""


@dataclass(frozen=True)
class DASFingerConfiguration:
    """Per-arm DAS device and calibrated opening range."""

    serial_port: str
    camera_device: str
    closed_distance_m: float
    open_distance_m: float
    invert: bool = False
    camera_fps: int = 60
    camera_resolution: str = "640x480"
    startup_distance_m: float = 0.05
    tactile_hz: float = 30.0
    camera_latency_ms: float | None = None

    def __post_init__(self):
        serial_port = str(self.serial_port).strip()
        camera_device = str(self.camera_device).strip()
        if not serial_port or not camera_device:
            raise ValueError("DAS serial_port and camera_device are required")
        object.__setattr__(self, "serial_port", serial_port)
        object.__setattr__(self, "camera_device", camera_device)

        closed_distance_m = float(self.closed_distance_m)
        open_distance_m = float(self.open_distance_m)
        if (
            not np.isfinite(closed_distance_m)
            or not np.isfinite(open_distance_m)
            or not MIN_DAS_DISTANCE_M
            <= closed_distance_m
            < open_distance_m
            <= MAX_DAS_DISTANCE_M
        ):
            raise ValueError(
                "DAS distances must satisfy "
                f"{MIN_DAS_DISTANCE_M:g} <= closed < open "
                f"<= {MAX_DAS_DISTANCE_M:g} m"
            )
        object.__setattr__(self, "closed_distance_m", closed_distance_m)
        object.__setattr__(self, "open_distance_m", open_distance_m)
        startup_distance_m = float(self.startup_distance_m)
        if (
            not np.isfinite(startup_distance_m)
            or not closed_distance_m <= startup_distance_m <= open_distance_m
        ):
            raise ValueError("DAS startup distance must be within [closed, open]")
        object.__setattr__(self, "startup_distance_m", startup_distance_m)
        object.__setattr__(self, "invert", bool(self.invert))

        camera_fps = int(self.camera_fps)
        if camera_fps <= 0:
            raise ValueError("DAS camera_fps must be positive")
        object.__setattr__(self, "camera_fps", camera_fps)
        camera_resolution = str(self.camera_resolution).strip().lower()
        if not camera_resolution:
            raise ValueError("DAS camera_resolution must not be empty")
        object.__setattr__(self, "camera_resolution", camera_resolution)
        camera_latency_ms = (
            DEFAULT_CAMERA_LATENCY_MS.get(camera_resolution)
            if self.camera_latency_ms is None
            else float(self.camera_latency_ms)
        )
        if camera_latency_ms is not None and (
            not np.isfinite(camera_latency_ms)
            or not 0.0 <= camera_latency_ms <= 1000.0
        ):
            raise ValueError("DAS camera_latency_ms must be within [0, 1000]")
        object.__setattr__(self, "camera_latency_ms", camera_latency_ms)
        tactile_hz = float(self.tactile_hz)
        if not np.isfinite(tactile_hz) or tactile_hz <= 0.0:
            raise ValueError("DAS tactile_hz must be positive")
        object.__setattr__(self, "tactile_hz", tactile_hz)


def load_das_finger_configurations(configuration_path):
    """Load validated left/right DAS configuration from JSON."""

    with Path(configuration_path).expanduser().open(encoding="utf-8") as file:
        configuration = json.load(file)
    try:
        return tuple(
            DASFingerConfiguration(**configuration[arm_name])
            for arm_name in ("left", "right")
        )
    except (KeyError, TypeError) as error:
        raise ValueError(
            "DAS gripper config must contain left/right device settings"
        ) from error


def load_das_sdk(sdk_root_path):
    if sdk_root_path is None:
        raise ValueError(
            "das_sdk_root is required; clone gen_finger_con_python_sdk_release "
            "and pass its root directory"
        )
    sdk_root = str(Path(sdk_root_path).expanduser().resolve())
    if sdk_root not in sys.path:
        sys.path.insert(0, sdk_root)
    try:
        return importlib.import_module("scripts")
    except ImportError as error:
        raise ImportError(
            "DAS Python SDK must expose its scripts package"
        ) from error


def _load_finger_system(sdk_root_path):
    try:
        return load_das_sdk(sdk_root_path).FingerSystem
    except AttributeError as error:
        raise ImportError(
            "DAS Python SDK must expose FingerSystem from its scripts package"
        ) from error


def _decode_encoder_value(record_data):
    if isinstance(record_data, (bytes, bytearray, memoryview)):
        raw_data = bytes(record_data)
        if len(raw_data) != 4:
            raise ValueError("DAS encoder callback must contain one big-endian float")
        value = struct.unpack(">f", raw_data)[0]
    else:
        value = float(record_data)
    if np.isclose(value, DAS_ENCODER_CALIBRATION_CODE, atol=1e-3):
        raise DASFingerCalibrationRequired(
            "DAS encoder requires initial calibration (returned -66.66); "
            "clear the gripper and run the SDK calibration startup first"
        )
    if not np.isfinite(value):
        raise ValueError(f"invalid DAS encoder distance: {value!r}")
    if -ENCODER_ZERO_TOLERANCE_M <= value < MIN_DAS_DISTANCE_M:
        return MIN_DAS_DISTANCE_M
    if not MIN_DAS_DISTANCE_M <= value <= MAX_DAS_DISTANCE_M:
        raise ValueError(f"invalid DAS encoder distance: {value!r}")
    return float(value)


def closedness_to_das_distances(closedness, configurations):
    closedness = np.asarray(closedness, dtype=float).reshape(-1)
    if (
        closedness.shape != (2,)
        or not np.all(np.isfinite(closedness))
        or np.any(closedness < 0.0)
        or np.any(closedness > 1.0)
    ):
        raise ValueError("gripper closedness must contain two values within [0, 1]")
    targets = []
    for value, configuration in zip(closedness, configurations):
        if configuration.invert:
            target = configuration.closed_distance_m + value * (
                configuration.open_distance_m - configuration.closed_distance_m
            )
        else:
            target = configuration.open_distance_m - value * (
                configuration.open_distance_m - configuration.closed_distance_m
            )
        targets.append(
            float(np.clip(target, MIN_DAS_DISTANCE_M, MAX_DAS_DISTANCE_M))
        )
    return tuple(targets)


def das_distances_to_closedness(distances, configurations):
    closedness = []
    for distance, configuration in zip(distances, configurations):
        span = configuration.open_distance_m - configuration.closed_distance_m
        if configuration.invert:
            value = (distance - configuration.closed_distance_m) / span
        else:
            value = (configuration.open_distance_m - distance) / span
        closedness.append(float(np.clip(value, 0.0, 1.0)))
    return tuple(closedness)


class DASFingerAdapter:
    """Drive two Gen Finger Controllers through the Python SDK.

    ``send_gripper_command`` keeps the existing Marvin normalized closedness
    interface. SDK calls run in one small worker thread so a USB call cannot
    stall the Marvin 50 Hz joint-control loop.
    """

    def __init__(
        self,
        configurations,
        sdk_root_path=None,
        finger_system_factory=None,
        command_hz=20.0,
        ready_timeout_seconds=10.0,
        encoder_stale_timeout_seconds=0.5,
        state_callback=None,
        tactile_callback=None,
        frame_callback=None,
    ):
        configurations = tuple(configurations)
        if len(configurations) != 2 or not all(
            isinstance(config, DASFingerConfiguration) for config in configurations
        ):
            raise TypeError(
                "configurations must contain two DASFingerConfiguration values"
            )
        command_hz = float(command_hz)
        if not np.isfinite(command_hz) or command_hz <= 0.0:
            raise ValueError("DAS command_hz must be positive")
        ready_timeout_seconds = float(ready_timeout_seconds)
        if not np.isfinite(ready_timeout_seconds) or ready_timeout_seconds <= 0.0:
            raise ValueError("DAS ready_timeout_seconds must be positive")
        encoder_stale_timeout_seconds = float(encoder_stale_timeout_seconds)
        if (
            not np.isfinite(encoder_stale_timeout_seconds)
            or encoder_stale_timeout_seconds <= 0.0
        ):
            raise ValueError("DAS encoder_stale_timeout_seconds must be positive")

        self.configurations = configurations
        self.sdk_root_path = sdk_root_path
        self._finger_system_factory = finger_system_factory
        self.command_period_seconds = 1.0 / command_hz
        self.ready_timeout_seconds = ready_timeout_seconds
        self.encoder_stale_timeout_seconds = encoder_stale_timeout_seconds
        self._state_callback = state_callback
        self._tactile_callback = tactile_callback
        self._frame_callback = frame_callback
        self._systems = [None, None]
        self._system_threads = [None, None]
        self._system_errors = [None, None]
        self._calibration_required = [threading.Event(), threading.Event()]
        self._calibration_commands_sent = [False, False]
        self._encoder_distances = np.full(2, np.nan, dtype=float)
        self._encoder_monotonic_ns = np.zeros(2, dtype=np.uint64)
        self._encoder_wall_time_ns = np.zeros(2, dtype=np.uint64)
        self._encoder_received = [threading.Event(), threading.Event()]
        self._targets = np.asarray(
            [config.startup_distance_m for config in configurations], dtype=float
        )
        self._target_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._command_thread = None
        self._is_connected = False
        self._is_released = False

    def _make_system(self, arm_index):
        configuration = self.configurations[arm_index]
        if self._finger_system_factory is None:
            finger_system_factory = _load_finger_system(self.sdk_root_path)
        else:
            finger_system_factory = self._finger_system_factory

        def publish_state(state):
            if self._state_callback is not None:
                try:
                    self._state_callback(arm_index, state)
                except Exception:
                    # Collection must never break finger control.
                    pass

        def encoder_callback(record_data, arm_index=arm_index):
            monotonic_ns = time.monotonic_ns()
            wall_time_ns = time.time_ns()
            try:
                distance = _decode_encoder_value(record_data)
                with self._target_lock:
                    self._encoder_distances[arm_index] = distance
                    self._encoder_monotonic_ns[arm_index] = monotonic_ns
                    self._encoder_wall_time_ns[arm_index] = wall_time_ns
                    target = float(self._targets[arm_index])
                self._encoder_received[arm_index].set()
                publish_state(
                    {
                        "wall_time_ns": wall_time_ns,
                        "steady_ns": monotonic_ns,
                        "valid": True,
                        "distance_m": distance,
                        "target_distance_m": target,
                        "status_flags": 0,
                    }
                )
            except DASFingerCalibrationRequired:
                self._calibration_required[arm_index].set()
                with self._target_lock:
                    target = float(self._targets[arm_index])
                publish_state(
                    {
                        "wall_time_ns": wall_time_ns,
                        "steady_ns": monotonic_ns,
                        "valid": False,
                        "distance_m": float("nan"),
                        "target_distance_m": target,
                        "status_flags": 1,
                    }
                )
            except Exception as error:
                self._system_errors[arm_index] = error

        def tactile_callback(record_data, arm_index=arm_index):
            if self._tactile_callback is not None:
                self._tactile_callback(
                    arm_index, bytes(record_data), time.time_ns(), time.monotonic_ns()
                )

        def capture_frames(camera, arm_index=arm_index):
            camera.frame_callback = lambda _camera_id, frame, wall_time_ns: (
                self._frame_callback(
                    arm_index, frame, wall_time_ns, time.monotonic_ns()
                )
            )
            camera.capture_frames_callback()

        # The vendor SDK uses this constructor shape and starts all I/O from
        # system.start(); no camera preview is needed for control-only use.
        return finger_system_factory(
            serial_port=configuration.serial_port,
            camera_resolutions=configuration.camera_resolution,
            video_devices=[configuration.camera_device],
            show_preview=False,
            encoder_callback=encoder_callback,
            tactile_callback=(
                tactile_callback if self._tactile_callback is not None else None
            ),
            capture_frames_callback=(
                capture_frames if self._frame_callback is not None else None
            ),
            camera_fps=configuration.camera_fps,
            trigger_mode=True,
            tactile_freq=(
                configuration.tactile_hz
                if self._tactile_callback is not None
                else None
            ),
            initial_distance_m=configuration.startup_distance_m,
        )

    @staticmethod
    def _set_system_target(system, distance):
        set_distance = getattr(system, "set_finger_distance", None)
        if callable(set_distance):
            set_distance(float(distance))
            return
        data_bus = getattr(system, "databus", None)
        set_target = getattr(data_bus, "set_target_distance", None)
        if not callable(set_target):
            raise RuntimeError("DAS FingerSystem has no distance-control method")
        set_target(float(distance))

    def _run_system(self, arm_index):
        try:
            if self._systems[arm_index].start() is False:
                self._system_errors[arm_index] = RuntimeError(
                    "DAS Finger SDK returned startup failure"
                )
        except Exception as error:
            self._system_errors[arm_index] = error

    def connect(self):
        if self._is_connected:
            return
        if self._is_released:
            raise RuntimeError("a released DAS adapter cannot reconnect")

        self._stop_event.clear()
        self._system_errors = [None, None]
        self._calibration_required = [threading.Event(), threading.Event()]
        self._calibration_commands_sent = [False, False]
        self._encoder_received = [threading.Event(), threading.Event()]
        self._encoder_distances.fill(np.nan)
        self._encoder_monotonic_ns.fill(0)
        self._encoder_wall_time_ns.fill(0)
        with self._target_lock:
            self._targets[:] = [
                config.startup_distance_m for config in self.configurations
            ]
        try:
            for arm_index in (0, 1):
                self._systems[arm_index] = self._make_system(arm_index)
                thread = threading.Thread(
                    target=self._run_system,
                    args=(arm_index,),
                    name=f"das-finger-{arm_index}",
                    daemon=True,
                )
                self._system_threads[arm_index] = thread
                thread.start()
        except Exception:
            self.release()
            raise

        deadline = time.monotonic() + self.ready_timeout_seconds
        while time.monotonic() < deadline:
            if any(error is not None for error in self._system_errors):
                self.release()
                error_index = next(
                    index
                    for index, error in enumerate(self._system_errors)
                    if error is not None
                )
                startup_error = self._system_errors[error_index]
                raise RuntimeError(
                    f"DAS {ARM_NAMES[error_index]} Finger SDK failed during startup: "
                    f"{startup_error}"
                ) from startup_error
            for arm_index, system in enumerate(self._systems):
                if (
                    self._calibration_required[arm_index].is_set()
                    and not self._calibration_commands_sent[arm_index]
                    and getattr(system, "databus", None) is not None
                ):
                    self._set_system_target(
                        system,
                        self.configurations[arm_index].startup_distance_m,
                    )
                    self._calibration_commands_sent[arm_index] = True
            if all(
                getattr(system, "databus", None) is not None
                for system in self._systems
            ) and all(event.is_set() for event in self._encoder_received):
                break
            time.sleep(0.01)
        else:
            self.release()
            error_index = next(
                (
                    index
                    for index, error in enumerate(self._system_errors)
                    if error is not None
                ),
                None,
            )
            if error_index is not None:
                startup_error = self._system_errors[error_index]
                raise RuntimeError(
                    f"DAS {ARM_NAMES[error_index]} Finger SDK failed during startup: "
                    f"{startup_error}"
                ) from startup_error
            missing = [
                ARM_NAMES[index]
                for index, event in enumerate(self._encoder_received)
                if not event.is_set()
            ]
            calibration = [
                ARM_NAMES[index]
                for index in range(2)
                if self._calibration_required[index].is_set()
                and not self._encoder_received[index].is_set()
            ]
            if calibration:
                raise TimeoutError(
                    "DAS encoder calibration is still required for "
                    f"{', '.join(calibration)} (returned -66.66); clear that "
                    "gripper and run calibrate_das_finger.py"
                )
            raise TimeoutError(
                f"DAS Finger SDK did not provide encoder feedback for "
                f"{', '.join(missing)} within {self.ready_timeout_seconds:g} seconds"
            )

        with self._target_lock:
            self._targets[:] = self._encoder_distances
        self._is_connected = True
        self._command_thread = threading.Thread(
            target=self._command_loop,
            name="das-finger-command",
            daemon=True,
        )
        self._command_thread.start()

    def _command_loop(self):
        while not self._stop_event.wait(self.command_period_seconds):
            with self._target_lock:
                targets = self._targets.copy()
            for arm_index, (system, target) in enumerate(zip(self._systems, targets)):
                try:
                    self._set_system_target(system, target)
                except Exception as error:
                    # Do not raise on the Marvin control thread. The next
                    # adapter operation will surface the stored SDK failure.
                    self._system_errors[arm_index] = error

    def _require_connected(self):
        if not self._is_connected:
            raise RuntimeError("DAS Finger adapter is not connected")
        for arm_index, error in enumerate(self._system_errors):
            if error is not None:
                raise RuntimeError(
                    f"DAS {ARM_NAMES[arm_index]} Finger SDK reported an error"
                ) from error

    def check_health(self):
        self._require_connected()
        now_ns = time.monotonic_ns()
        with self._target_lock:
            timestamps = self._encoder_monotonic_ns.copy()
        stale = [
            ARM_NAMES[index]
            for index, timestamp_ns in enumerate(timestamps)
            if timestamp_ns == 0
            or now_ns - int(timestamp_ns)
            > self.encoder_stale_timeout_seconds * 1e9
        ]
        if stale:
            raise TimeoutError(
                f"DAS encoder feedback stale for {', '.join(stale)}"
            )

    def send_gripper_command(self, closedness):
        self._require_connected()
        targets = closedness_to_das_distances(closedness, self.configurations)
        with self._target_lock:
            self._targets[:] = targets
        return targets

    def get_initial_gripper_closedness(self):
        self._require_connected()
        with self._target_lock:
            distances = self._encoder_distances.copy()
        return das_distances_to_closedness(distances, self.configurations)

    def get_encoder_distances(self):
        return self.get_gripper_state()["distance_m"]

    def get_gripper_state(self):
        self.check_health()
        with self._target_lock:
            return {
                "distance_m": tuple(self._encoder_distances),
                "target_distance_m": tuple(self._targets),
                "encoder_monotonic_ns": tuple(
                    int(value) for value in self._encoder_monotonic_ns
                ),
                "encoder_wall_time_ns": tuple(
                    int(value) for value in self._encoder_wall_time_ns
                ),
                "encoder_valid": tuple(
                    event.is_set() for event in self._encoder_received
                ),
            }

    def set_idle(self):
        if not self._is_connected:
            return False
        self._stop_event.set()
        if self._command_thread is not None:
            self._command_thread.join(timeout=1.0)
            self._command_thread = None
        return True

    def release(self):
        if self._is_released:
            return
        self._is_released = True
        self._stop_event.set()
        if self._command_thread is not None:
            self._command_thread.join(timeout=1.0)
            self._command_thread = None
        for system in self._systems:
            stop = getattr(system, "stop", None)
            if callable(stop):
                try:
                    stop()
                except Exception:
                    pass
        for thread in self._system_threads:
            if thread is not None:
                thread.join(timeout=1.0)
        self._system_threads = [None, None]
        self._systems = [None, None]
        self._is_connected = False
