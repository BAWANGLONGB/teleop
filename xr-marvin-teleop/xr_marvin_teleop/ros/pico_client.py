"""XrClient-compatible subscriber for an independently published PICO stream."""

import threading
import time

from xr_marvin_teleop.common.xr_client import XrSnapshot


class RosPicoClient:
    is_ros_source = True

    def __init__(
        self,
        topic="/raw/pico/frame",
        max_age_seconds=0.2,
        disconnect_timeout_seconds=2.0,
    ):
        if not 0.0 < max_age_seconds < disconnect_timeout_seconds:
            raise ValueError("PICO ROS age must be positive and below timeout")
        try:
            import rclpy
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node
            from rclpy.qos import qos_profile_sensor_data
            from teleop_msgs.msg import PicoFrame
        except (ImportError, OSError) as error:
            raise RuntimeError(
                "RosPicoClient requires sourced ROS2 and teleop_msgs"
            ) from error
        self._rclpy = rclpy
        self._owns_context = not rclpy.ok()
        if self._owns_context:
            rclpy.init()
        try:
            self._node = Node("marvin_pico_client")
        except Exception:
            if self._owns_context and rclpy.ok():
                rclpy.shutdown()
            raise
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._condition = threading.Condition()
        self._snapshot = None
        self._valid = False
        self._sequence_id = 0
        self._update_id = 0
        self._receive_steady_ns = 0
        self._max_age_ns = int(max_age_seconds * 1e9)
        self._disconnect_timeout_ns = int(disconnect_timeout_seconds * 1e9)
        self._subscription = self._node.create_subscription(
            PicoFrame,
            topic,
            self._callback,
            qos_profile_sensor_data,
        )
        self._stop_event = threading.Event()
        self._thread = threading.Thread(
            target=self._spin, name="pico-ros-client", daemon=True
        )
        self._thread.start()

    def _callback(self, message):
        snapshot = None
        if message.valid:
            snapshot = XrSnapshot(
                message.source_timestamp_ns,
                message.left_controller_pose,
                message.right_controller_pose,
                tuple(message.grip_values),
                message.button_a,
                message.button_b,
                tuple(message.trigger_values),
                tuple(message.thumbstick_y_values),
            )
        with self._condition:
            sequence_id = int(message.sequence_id)
            if sequence_id <= self._sequence_id:
                if sequence_id != 1 or self._sequence_id <= 1:
                    return
            self._snapshot = snapshot
            self._valid = bool(message.valid)
            self._sequence_id = sequence_id
            self._update_id += 1
            self._receive_steady_ns = time.monotonic_ns()
            self._condition.notify_all()

    def _spin(self):
        while not self._stop_event.is_set():
            self._executor.spin_once(timeout_sec=0.02)

    def read_snapshot(self):
        with self._condition:
            if self._receive_steady_ns == 0:
                return None
            age_ns = time.monotonic_ns() - self._receive_steady_ns
            if age_ns > self._disconnect_timeout_ns:
                raise TimeoutError("ROS2 PICO stream disconnected")
            if not self._valid or age_ns > self._max_age_ns:
                return None
            return self._snapshot

    def wait_for_fresh_snapshot(self, timeout_seconds=2.0):
        deadline = time.monotonic() + timeout_seconds
        with self._condition:
            initial_update = self._update_id
            while time.monotonic() < deadline:
                if self._valid and self._update_id > initial_update:
                    return self._snapshot
                self._condition.wait(timeout=min(0.05, deadline - time.monotonic()))
        raise TimeoutError("ROS2 PICO stream produced no fresh valid frame")

    def close(self):
        self._stop_event.set()
        self._thread.join(timeout=1.0)
        self._executor.remove_node(self._node)
        self._executor.shutdown()
        self._node.destroy_node()
        if self._owns_context and self._rclpy.ok():
            self._rclpy.shutdown()
