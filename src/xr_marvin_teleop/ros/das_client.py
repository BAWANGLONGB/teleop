"""Gripper-adapter client for a DAS source running in another ROS2 process."""

import math
import threading
import time
import uuid
from . import spin_until_stopped
from .protocol import (
    DAS_COMMAND_TOPIC,
    DAS_STATE_TOPICS,
    GRIPPER_NAMES,
    SampleJoiner,
    joint_positions,
    sample_status,
    stamp_ns,
    status_topic,
    trajectory,
)

from xr_marvin_teleop.hardware.interface.das_finger import (
    ARM_NAMES,
    DASFingerConfiguration,
    closedness_to_das_distances,
    das_distances_to_closedness,
    MIN_DAS_DISTANCE_M, MAX_DAS_DISTANCE_M,
)


class RosDasClient:
    def __init__(
        self,
        configurations,
        ready_timeout_seconds=10.0,
        encoder_stale_timeout_seconds=0.5,
    ):
        configurations = tuple(configurations)
        if len(configurations) != 2 or not all(
            isinstance(config, DASFingerConfiguration) for config in configurations
        ):
            raise TypeError(
                "configurations must contain two DASFingerConfiguration values"
            )
        if ready_timeout_seconds <= 0.0 or encoder_stale_timeout_seconds <= 0.0:
            raise ValueError("DAS ROS timeouts must be positive")
        try:
            import rclpy
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node
            from rclpy.qos import (
                DurabilityPolicy,
                HistoryPolicy,
                QoSProfile,
                ReliabilityPolicy,
            )
            from sensor_msgs.msg import JointState
            from trajectory_msgs.msg import JointTrajectory
            from diagnostic_msgs.msg import DiagnosticArray
        except (ImportError, OSError) as error:
            raise RuntimeError(
                "RosDasClient requires sourced ROS2"
            ) from error

        critical_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.configurations = configurations
        self.ready_timeout_seconds = float(ready_timeout_seconds)
        self.encoder_stale_timeout_seconds = float(
            encoder_stale_timeout_seconds
        )
        self._rclpy = rclpy
        self._session = uuid.uuid4().hex
        self._owns_context = not rclpy.ok()
        if self._owns_context:
            rclpy.init()
        try:
            self._node = Node("marvin_das_client")
        except Exception:
            if self._owns_context and rclpy.ok():
                rclpy.shutdown()
            raise
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._condition = threading.Condition()
        self._distances = [math.nan, math.nan]
        self._targets = [
            config.startup_distance_m for config in self.configurations
        ]
        self._encoder_monotonic_ns = [0, 0]
        self._encoder_wall_time_ns = [0, 0]
        self._valid = [False, False]
        self._status_flags = [0, 0]
        self._sequence_ids = [0, 0]
        self._update_ids = [0, 0]
        self._command_sequence = 0
        self._publisher = self._node.create_publisher(
            JointTrajectory, DAS_COMMAND_TOPIC, critical_qos
        )
        self._status_publisher = self._node.create_publisher(
            DiagnosticArray, status_topic(DAS_COMMAND_TOPIC), critical_qos)
        self._joiners = [
            SampleJoiner([topic], int(encoder_stale_timeout_seconds * 1e9))
            for topic in DAS_STATE_TOPICS
        ]
        self._subscriptions = tuple(
            self._node.create_subscription(
                message_type,
                DAS_STATE_TOPICS[index] + suffix,
                lambda message, index=index, suffix=suffix: self._callback(index, suffix, message),
                critical_qos,
            )
            for index, side in enumerate(ARM_NAMES)
            for message_type, suffix in ((JointState, ""), (DiagnosticArray, "/status"))
        )
        self._stop_event = threading.Event()
        self._thread = threading.Thread(
            target=self._spin, name="das-ros-client", daemon=True
        )
        self._is_connected = False
        self._is_released = False
        self._thread.start()

    def _callback(self, arm_index, suffix, message):
        topic = DAS_STATE_TOPICS[arm_index]
        try:
            joined = self._joiners[arm_index].push("status" if suffix else topic, message)
            if joined is None:
                return
            parts, values, received = joined
            distance = (joint_positions(parts[topic], [GRIPPER_NAMES[arm_index]])[0]
                        if values["valid"] else math.nan)
            if values["valid"]:
                if not MIN_DAS_DISTANCE_M <= distance <= MAX_DAS_DISTANCE_M:
                    raise ValueError("DAS feedback outside physical limits")
            target = float(values["target_distance_m"])
            flags = int(values["status_flags"])
            if not MIN_DAS_DISTANCE_M <= target <= MAX_DAS_DISTANCE_M or not 0 <= flags <= 0xFFFFFFFF:
                raise ValueError("invalid DAS feedback diagnostics")
        except (ValueError, TypeError, KeyError, AttributeError):
            with self._condition:
                self._valid[arm_index] = False
                self._condition.notify_all()
            return
        with self._condition:
            self._sequence_ids[arm_index] = values["sequence_id"]
            self._distances[arm_index] = distance
            self._targets[arm_index] = target
            self._encoder_monotonic_ns[arm_index] = received
            self._encoder_wall_time_ns[arm_index] = stamp_ns(parts["status"])
            self._status_flags[arm_index] = flags
            self._valid[arm_index] = values["valid"] and math.isfinite(distance) and not flags
            self._update_ids[arm_index] += 1
            self._condition.notify_all()

    def _spin(self):
        spin_until_stopped(self._executor, self._node.context, self._stop_event)

    def connect(self, timeout_seconds=None):
        if self._is_connected:
            return
        if self._is_released:
            raise RuntimeError("a released DAS ROS client cannot reconnect")
        timeout_seconds = (
            self.ready_timeout_seconds
            if timeout_seconds is None
            else float(timeout_seconds)
        )
        deadline = time.monotonic() + timeout_seconds
        with self._condition:
            initial_updates = tuple(self._update_ids)
            while time.monotonic() < deadline:
                if all(self._valid) and all(
                    current > initial
                    for current, initial in zip(
                        self._update_ids, initial_updates
                    )
                ):
                    self._is_connected = True
                    return
                self._condition.wait(
                    timeout=min(0.05, deadline - time.monotonic())
                )
        missing = [
            side for side, valid in zip(ARM_NAMES, self._valid) if not valid
        ]
        raise TimeoutError(
            "DAS ROS2 source did not provide fresh valid feedback for "
            f"{', '.join(missing or ARM_NAMES)} within {timeout_seconds:g} seconds"
        )

    def _require_connected(self):
        if not self._is_connected:
            raise RuntimeError("DAS ROS client is not connected")

    def check_health(self):
        self._require_connected()
        now_ns = time.monotonic_ns()
        with self._condition:
            invalid = [
                ARM_NAMES[index]
                for index, valid in enumerate(self._valid)
                if not valid
            ]
            stale = [
                ARM_NAMES[index]
                for index, timestamp_ns in enumerate(
                    self._encoder_monotonic_ns
                )
                if timestamp_ns == 0
                or now_ns - timestamp_ns
                > self.encoder_stale_timeout_seconds * 1e9
            ]
        if invalid:
            raise RuntimeError(
                f"DAS ROS feedback invalid for {', '.join(invalid)}"
            )
        if stale:
            raise TimeoutError(
                f"DAS ROS encoder feedback stale for {', '.join(stale)}"
            )

    def send_gripper_command(self, closedness):
        self._require_connected()
        targets = closedness_to_das_distances(closedness, self.configurations)
        wall_time_ns = time.time_ns()
        steady_ns = time.monotonic_ns()
        message = trajectory([1.0 - value for value in closedness], GRIPPER_NAMES, wall_time_ns)
        self._command_sequence += 1
        self._publisher.publish(message)
        self._status_publisher.publish(sample_status(
            [DAS_COMMAND_TOPIC], wall_time_ns, self._session, self._command_sequence,
            steady_ns, command=True))
        with self._condition:
            self._targets[:] = targets
        return targets

    def get_initial_gripper_closedness(self):
        self.check_health()
        with self._condition:
            distances = tuple(self._distances)
        return das_distances_to_closedness(distances, self.configurations)

    def get_encoder_distances(self):
        return self.get_gripper_state()["distance_m"]

    def get_gripper_state(self):
        self.check_health()
        with self._condition:
            return {
                "distance_m": tuple(self._distances),
                "target_distance_m": tuple(self._targets),
                "encoder_monotonic_ns": tuple(self._encoder_monotonic_ns),
                "encoder_wall_time_ns": tuple(self._encoder_wall_time_ns),
                "encoder_valid": tuple(self._valid),
            }

    def set_idle(self):
        return self._is_connected

    def release(self):
        if self._is_released:
            return
        self._is_released = True
        self._is_connected = False
        self._stop_event.set()
        self._thread.join(timeout=1.0)
        self._executor.remove_node(self._node)
        self._executor.shutdown()
        self._node.destroy_node()
        if self._owns_context and self._rclpy.ok():
            self._rclpy.shutdown()

    def close(self):
        self.release()
