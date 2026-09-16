"""XrClient-compatible subscriber for an independently published PICO stream."""

import threading
import time

from xr_marvin_teleop.common.xr_client import XrSnapshot
from xr_marvin_teleop.common.pico_timing import PicoTimingLog
from . import spin_until_stopped
from .protocol import PICO_TOPIC_PREFIX, SampleJoiner, pose_values, stamp_ns


class RosPicoClient:
    is_ros_source = True

    def __init__(
        self,
        topic=PICO_TOPIC_PREFIX,
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
            from geometry_msgs.msg import PoseArray
            from sensor_msgs.msg import Joy
            from diagnostic_msgs.msg import DiagnosticArray
        except (ImportError, OSError) as error:
            raise RuntimeError(
                "RosPicoClient requires sourced ROS2"
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
        self._topics = (topic.rstrip("/") + "/poses", topic.rstrip("/") + "/joy")
        self._joiner = SampleJoiner(self._topics, self._max_age_ns)
        self._sample_identity = {}
        self._join_completed_ns = 0
        self._last_read_state = None
        self._timing = PicoTimingLog("receiver")
        self._subscriptions = tuple(self._node.create_subscription(
            message_type, name, lambda message, key=key: self._callback(key, message),
            qos_profile_sensor_data) for name, message_type, key in (
                (self._topics[0], PoseArray, self._topics[0]),
                (self._topics[1], Joy, self._topics[1]),
                (topic.rstrip("/") + "/status", DiagnosticArray, "status")))
        self._stop_event = threading.Event()
        self._thread = threading.Thread(
            target=self._spin, name="pico-ros-client", daemon=True
        )
        self._thread.start()

    def _callback(self, topic, message):
        started = time.monotonic_ns()
        try:
            self._accept_message(topic, message)
        finally:
            if self._timing is not None:
                outcome = self._joiner.last_outcome
                try:
                    message_stamp = stamp_ns(message)
                except (ValueError, TypeError, AttributeError):
                    message_stamp = None
                self._timing.record("callback_" + topic.rsplit("/", 1)[-1],
                    identity=dict(stamp_ns=message_stamp, callback_start_ns=started,
                                  expired_pending=self._joiner.expired_pending,
                                  evicted_pending=self._joiner.evicted_pending),
                    event=outcome if outcome in ("malformed", "stale_stamp", "old_sequence_or_session", "source_invalid") else None,
                    duration_ns=time.monotonic_ns() - started)

    def _accept_message(self, topic, message):
        try:
            joined = self._joiner.push(topic, message)
            if joined is None:
                return
            parts, values, received = joined
            snapshot = None
            if values["valid"]:
                poses, joy = parts[self._topics[0]], parts[self._topics[1]]
                if (len(poses.poses) != 2 or len(joy.axes) != 6 or len(joy.buttons) != 4
                        or any(value not in (0, 1) for value in joy.buttons)
                        or poses.header.frame_id != "openxr_local" or joy.header.frame_id != "openxr_local"):
                    raise ValueError("invalid PICO v2 layout/frame")
                snapshot = XrSnapshot(
                    values["source_timestamp_ns"], *(pose_values(pose) for pose in poses.poses),
                    tuple(joy.axes[:2]), bool(joy.buttons[0]), bool(joy.buttons[1]),
                    tuple(joy.axes[2:4]), tuple(joy.axes[4:6]),
                    bool(joy.buttons[2]), bool(joy.buttons[3]))
        except (ValueError, TypeError, KeyError, AttributeError):
            self._joiner.last_outcome = "malformed"
            with self._condition:
                self._valid = False
                self._condition.notify_all()
            return
        with self._condition:
            self._snapshot = snapshot
            self._valid = values["valid"]
            self._sequence_id = values["sequence_id"]
            self._update_id += 1
            self._receive_steady_ns = received
            self._join_completed_ns = time.monotonic_ns()
            self._sample_identity = dict(publisher_session_id=values["publisher_session_id"],
                sequence_id=values["sequence_id"], source_timestamp_ns=values["source_timestamp_ns"],
                stamp_ns=stamp_ns(parts["status"]), publisher_enqueue_ns=values.get("receive_steady_ns"),
                first_callback_ns=received, joined_ns=self._join_completed_ns)
            if self._timing is not None:
                self._timing.record("join", identity=self._sample_identity,
                                    duration_ns=self._join_completed_ns - received)
            self._condition.notify_all()

    def _spin(self):
        spin_until_stopped(self._executor, self._node.context, self._stop_event)

    def read_frame(self):
        """Atomically read the sample and identity without refreshing its age.

        Invalid frames retain their arrival time for disconnect diagnostics.
        Consumers must treat the returned snapshot as immutable.
        """
        with self._condition:
            if self._receive_steady_ns == 0:
                return None
            age = time.monotonic_ns() - self._receive_steady_ns
            return {
                "snapshot": self._snapshot,
                "valid": bool(self._valid and 0 <= age <= self._max_age_ns),
                "publisher_session_id": self._sample_identity["publisher_session_id"],
                "sequence_id": self._sequence_id,
                "receive_monotonic_ns": self._receive_steady_ns,
            }

    def read_snapshot(self):
        with self._condition:
            if self._receive_steady_ns == 0:
                return None
            now = time.monotonic_ns()
            age_ns = now - self._receive_steady_ns
            state = "valid" if self._valid and age_ns <= self._max_age_ns else "hold"
            if self._timing is not None:
                self._timing.record("consume", identity=self._sample_identity,
                    event=state if state != self._last_read_state else None,
                    arrival_age_ns=age_ns, join_to_read_ns=now - self._join_completed_ns)
            self._last_read_state = state
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
                if (self._valid and self._update_id > initial_update
                        and time.monotonic_ns() - self._receive_steady_ns <= self._max_age_ns):
                    return self._snapshot
                self._condition.wait(timeout=min(0.05, deadline - time.monotonic()))
        raise TimeoutError("ROS2 PICO stream produced no fresh valid frame")

    def close(self):
        self._stop_event.set()
        self._thread.join(timeout=1.0)
        self._executor.remove_node(self._node)
        self._executor.shutdown()
        self._node.destroy_node()
        self._timing.close()
        if self._owns_context and self._rclpy.ok():
            self._rclpy.shutdown()
