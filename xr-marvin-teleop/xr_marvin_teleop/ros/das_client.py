"""Gripper-adapter client for a DAS source running in another ROS2 process."""

import math
import threading
import time

from xr_marvin_teleop.hardware.interface.das_finger import (
    ARM_NAMES,
    DASFingerConfiguration,
    closedness_to_das_distances,
    das_distances_to_closedness,
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
            from teleop_msgs.msg import DasState, GripperCommand
        except (ImportError, OSError) as error:
            raise RuntimeError(
                "RosDasClient requires sourced ROS2 and teleop_msgs"
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
        self._message_type = GripperCommand
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
            GripperCommand, "/command/das/target", critical_qos
        )
        self._subscriptions = tuple(
            self._node.create_subscription(
                DasState,
                f"/raw/das/{side}/state",
                self._callback,
                critical_qos,
            )
            for side in ARM_NAMES
        )
        self._stop_event = threading.Event()
        self._thread = threading.Thread(
            target=self._spin, name="das-ros-client", daemon=True
        )
        self._is_connected = False
        self._is_released = False
        self._thread.start()

    def _callback(self, message):
        try:
            arm_index = ARM_NAMES.index(message.side)
        except ValueError:
            return
        with self._condition:
            sequence_id = int(message.sequence_id)
            previous_sequence = self._sequence_ids[arm_index]
            if sequence_id <= previous_sequence:
                if sequence_id != 1 or previous_sequence <= 1:
                    return
            self._sequence_ids[arm_index] = sequence_id
            self._distances[arm_index] = float(message.distance_m)
            self._targets[arm_index] = float(message.target_distance_m)
            self._encoder_monotonic_ns[arm_index] = int(
                message.receive_steady_ns
            )
            self._encoder_wall_time_ns[arm_index] = (
                int(message.header.stamp.sec) * 1_000_000_000
                + int(message.header.stamp.nanosec)
            )
            self._status_flags[arm_index] = int(message.status_flags)
            self._valid[arm_index] = bool(message.valid) and math.isfinite(
                self._distances[arm_index]
            )
            self._update_ids[arm_index] += 1
            self._condition.notify_all()

    def _spin(self):
        while not self._stop_event.is_set():
            self._executor.spin_once(timeout_sec=0.02)

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
        message = self._message_type()
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(
            wall_time_ns, 1_000_000_000
        )
        message.header.frame_id = "finger_pair"
        self._command_sequence += 1
        message.sequence_id = self._command_sequence
        message.issue_steady_ns = steady_ns
        message.closedness = list(closedness)
        self._publisher.publish(message)
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
