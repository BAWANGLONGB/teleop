"""Bounded ROS2 publishers for native-rate collection streams."""

import queue
import threading
import time
import uuid

import numpy as np
from .protocol import (
    ARM_NAMES,
    DAS_COMMAND_TOPIC,
    DAS_IMAGE_TOPICS,
    DAS_STATE_TOPICS,
    DAS_TACTILE_TOPICS,
    DIAGNOSTICS_TOPIC,
    GRIPPER_NAMES,
    JOINT_NAMES,
    MARVIN_JOINT_COMMAND_TOPIC,
    MARVIN_JOINT_STATE_TOPIC,
    PICO_TOPICS,
    joint_state,
    pico_messages,
    sample_status,
    stamp,
    status_topic,
    tactile_grid,
    trajectory,
)


class Ros2DataBridge:
    """Publish raw streams without allowing recorder backpressure into control."""

    def __init__(
        self,
        node_name="marvin_data_bridge",
        publish_gripper_commands=True,
        gripper_configurations=None,
        pico_timing=None,
    ):
        try:
            import rclpy
            from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
            from rclpy.node import Node
            from rclpy.qos import (
                DurabilityPolicy,
                HistoryPolicy,
                QoSProfile,
                ReliabilityPolicy,
            )
            from geometry_msgs.msg import PoseArray
            from sensor_msgs.msg import JointState, Joy, Image
            from trajectory_msgs.msg import JointTrajectory
            from foxglove_msgs.msg import Grid
        except (ImportError, OSError) as error:
            raise RuntimeError(
                "ROS2 collection requires sourced ROS2 and foxglove_msgs; "
                "do not preload the system libstdc++ into Conda"
            ) from error

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        critical_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._rclpy = rclpy
        self._pico_timing = pico_timing
        self._node_name = str(node_name)
        self._session = uuid.uuid4().hex
        self.gripper_configurations = gripper_configurations
        self._types = {
            "DiagnosticArray": DiagnosticArray,
            "DiagnosticStatus": DiagnosticStatus,
            "KeyValue": KeyValue,
            "Image": Image,
        }
        self._owns_context = not rclpy.ok()
        if self._owns_context:
            rclpy.init()
        try:
            self._node = Node(self._node_name)
        except Exception:
            if self._owns_context and rclpy.ok():
                rclpy.shutdown()
            raise

        self._publishers = {
            "pico": self._node.create_publisher(
                PoseArray, PICO_TOPICS[0], sensor_qos
            ),
            "joy": self._node.create_publisher(Joy, PICO_TOPICS[1], sensor_qos),
            "marvin": self._node.create_publisher(
                JointState, MARVIN_JOINT_STATE_TOPIC, critical_qos
            ),
            "joint_command": self._node.create_publisher(
                JointTrajectory, MARVIN_JOINT_COMMAND_TOPIC, critical_qos
            ),
        }
        if publish_gripper_commands:
            self._publishers["gripper_command"] = self._node.create_publisher(
                JointTrajectory, DAS_COMMAND_TOPIC, critical_qos
            )
        self._das_publishers = tuple(
            self._node.create_publisher(JointState, topic, critical_qos)
            for topic in DAS_STATE_TOPICS
        )
        self._tactile_publishers = tuple(
            self._node.create_publisher(Grid, topic, sensor_qos)
            for topic in DAS_TACTILE_TOPICS
        )
        self._camera_publishers = tuple(
            self._node.create_publisher(Image, topic, sensor_qos)
            for topic in DAS_IMAGE_TOPICS
        )
        self._diagnostic_publisher = self._node.create_publisher(
            DiagnosticArray, DIAGNOSTICS_TOPIC, critical_qos
        )
        topics = [
            PICO_TOPICS[0],
            MARVIN_JOINT_STATE_TOPIC,
            MARVIN_JOINT_COMMAND_TOPIC,
            DAS_COMMAND_TOPIC,
            *DAS_STATE_TOPICS,
            *DAS_TACTILE_TOPICS,
            *DAS_IMAGE_TOPICS,
        ]
        self._status_publishers = {topic: self._node.create_publisher(
            DiagnosticArray, status_topic(topic), sensor_qos if topic == PICO_TOPICS[0]
            or topic.endswith(("tactile", "image")) else critical_qos) for topic in topics}
        self._critical_queue = queue.Queue(maxsize=512)
        self._tactile_queues = tuple(queue.Queue(maxsize=64) for _ in ARM_NAMES)
        self._camera_queues = tuple(queue.Queue(maxsize=2) for _ in ARM_NAMES)
        self._sequence_lock = threading.Lock()
        self._sequences = {}
        self._drop_counts = {
            "critical": 0,
            "tactile": 0,
            "camera": 0,
            "invalid_camera": 0,
            "publish_error": 0,
        }
        self._last_error = ""
        self._stop_event = threading.Event()
        self._image_available = threading.Event()
        self._critical_thread = threading.Thread(
            target=self._run_critical,
            name="ros2-critical-bridge",
            daemon=True,
        )
        self._image_thread = threading.Thread(
            target=self._run_images,
            name="ros2-image-bridge",
            daemon=True,
        )
        self._critical_thread.start()
        self._image_thread.start()

    def _next_sequence(self, stream):
        with self._sequence_lock:
            value = self._sequences.get(stream, 0) + 1
            self._sequences[stream] = value
            return value

    def _enqueue(self, target_queue, item, stream):
        try:
            target_queue.put_nowait(item)
        except queue.Full:
            self._drop_counts[stream] += 1

    def publish_pico(self, snapshot, wall_time_ns=None, steady_ns=None):
        wall_time_ns = time.time_ns() if wall_time_ns is None else int(wall_time_ns)
        steady_ns = time.monotonic_ns() if steady_ns is None else int(steady_ns)
        self._enqueue(
            self._critical_queue,
            ("pico", self._next_sequence("pico"), snapshot, wall_time_ns, steady_ns),
            "critical",
        )

    def publish_marvin_state(self, feedback, wall_time_ns=None, steady_ns=None):
        wall_time_ns = time.time_ns() if wall_time_ns is None else int(wall_time_ns)
        steady_ns = time.monotonic_ns() if steady_ns is None else int(steady_ns)
        self._enqueue(
            self._critical_queue,
            (
                "marvin",
                self._next_sequence("marvin"),
                feedback,
                wall_time_ns,
                steady_ns,
            ),
            "critical",
        )

    def publish_joint_command(self, q_rad, wall_time_ns=None, steady_ns=None):
        wall_time_ns = time.time_ns() if wall_time_ns is None else int(wall_time_ns)
        steady_ns = time.monotonic_ns() if steady_ns is None else int(steady_ns)
        self._enqueue(
            self._critical_queue,
            (
                "joint_command",
                self._next_sequence("joint_command"),
                np.asarray(q_rad, dtype=float).copy(),
                wall_time_ns,
                steady_ns,
            ),
            "critical",
        )

    def publish_gripper_command(self, closedness, wall_time_ns=None, steady_ns=None):
        if "gripper_command" not in self._publishers or self.gripper_configurations is None:
            return
        wall_time_ns = time.time_ns() if wall_time_ns is None else int(wall_time_ns)
        steady_ns = time.monotonic_ns() if steady_ns is None else int(steady_ns)
        self._enqueue(
            self._critical_queue,
            (
                "gripper_command",
                self._next_sequence("gripper_command"),
                [1.0 - value for value in closedness],
                wall_time_ns,
                steady_ns,
            ),
            "critical",
        )

    def publish_das_state(self, arm_index, state):
        self._enqueue(
            self._critical_queue,
            (
                "das",
                self._next_sequence(f"das_{arm_index}"),
                int(arm_index),
                dict(state),
            ),
            "critical",
        )

    def publish_tactile(self, arm_index, raw_data, wall_time_ns, steady_ns):
        self._enqueue(
            self._tactile_queues[arm_index],
            (
                self._next_sequence(f"tactile_{arm_index}"),
                bytes(raw_data),
                int(wall_time_ns),
                int(steady_ns),
            ),
            "tactile",
        )

    def publish_camera(self, arm_index, frame, wall_time_ns, steady_ns):
        frame = np.asarray(frame)
        if frame.dtype != np.uint8 or frame.ndim not in (2, 3):
            self._drop_counts["invalid_camera"] += 1
            return
        if frame.ndim == 3 and frame.shape[2] not in (1, 3, 4):
            self._drop_counts["invalid_camera"] += 1
            return
        self._enqueue(
            self._camera_queues[arm_index],
            (
                self._next_sequence(f"camera_{arm_index}"),
                np.ascontiguousarray(frame).copy(),
                int(wall_time_ns),
                int(steady_ns),
            ),
            "camera",
        )
        self._image_available.set()

    _stamp = staticmethod(stamp)

    def _sample_status(self, topics, sequence, wall, steady, **values):
        self._status_publishers[topics[0]].publish(
            sample_status(topics, wall, self._session, sequence, steady, **values))

    def _publish_critical(self, item):
        kind = item[0]
        if kind == "das":
            _, sequence_id, arm_index, state = item
            message = joint_state([state["distance_m"]], [GRIPPER_NAMES[arm_index]], state["wall_time_ns"])
            self._das_publishers[arm_index].publish(message)
            self._sample_status([DAS_STATE_TOPICS[arm_index]], sequence_id,
                                state["wall_time_ns"], state["steady_ns"],
                                valid=bool(state.get("valid", True)),
                                target_distance_m=float(state["target_distance_m"]),
                                status_flags=int(state.get("status_flags", 0)))
            return

        _, sequence_id, payload, wall_time_ns, steady_ns = item
        publish_start_ns = time.monotonic_ns()
        metadata = {}
        if kind == "pico":
            message, joy = pico_messages(payload, wall_time_ns)
            self._publishers["joy"].publish(joy)
            topics = PICO_TOPICS
            metadata = dict(valid=payload is not None, source_clock="openxr",
                            source_timestamp_ns=0 if payload is None else payload.timestamp_ns)
        elif kind == "marvin":
            message = joint_state(payload.q_rad, JOINT_NAMES, wall_time_ns, payload.dq_rad_s)
            topics = [MARVIN_JOINT_STATE_TOPIC]
            metadata = {key: list(getattr(payload, key)) for key in
                        ("frame_serial", "arm_state", "error_code", "low_speed")}
        else:
            names = JOINT_NAMES if kind == "joint_command" else GRIPPER_NAMES
            message = trajectory(payload, names, wall_time_ns)
            topics = [
                MARVIN_JOINT_COMMAND_TOPIC
                if kind == "joint_command"
                else DAS_COMMAND_TOPIC
            ]
            metadata = dict(command=True)
        self._publishers[kind].publish(message)
        self._sample_status(topics, sequence_id, wall_time_ns, steady_ns, **metadata)
        if kind == "pico" and self._pico_timing is not None:
            self._pico_timing.record("publish", identity=dict(
                publisher_session_id=self._session, sequence_id=sequence_id, stamp_ns=wall_time_ns,
                source_timestamp_ns=metadata["source_timestamp_ns"], enqueue_ns=steady_ns,
                publish_start_ns=publish_start_ns, publish_end_ns=time.monotonic_ns()),
                queue_ns=publish_start_ns - steady_ns, duration_ns=time.monotonic_ns() - publish_start_ns)

    def _publish_tactile(self, arm_index, item):
        sequence_id, raw_data, wall_time_ns, steady_ns = item
        message = tactile_grid(raw_data, ARM_NAMES[arm_index], wall_time_ns)
        self._tactile_publishers[arm_index].publish(message)
        self._sample_status(
            [DAS_TACTILE_TOPICS[arm_index]], sequence_id, wall_time_ns, steady_ns
        )

    def _publish_camera(self, arm_index, item):
        sequence_id, frame, wall_time_ns, steady_ns = item
        image = self._types["Image"]()
        self._stamp(image.header, wall_time_ns, f"das_{ARM_NAMES[arm_index]}_camera_optical_frame")
        image.height, image.width = frame.shape[:2]
        channels = 1 if frame.ndim == 2 else frame.shape[2]
        image.encoding = {1: "mono8", 3: "bgr8", 4: "bgra8"}[channels]
        image.is_bigendian = 0
        image.step = image.width * channels
        image.data = frame.tobytes()
        self._camera_publishers[arm_index].publish(image)
        self._sample_status(
            [DAS_IMAGE_TOPICS[arm_index]], sequence_id, wall_time_ns, steady_ns
        )

    def _publish_diagnostics(self):
        message = self._types["DiagnosticArray"]()
        self._stamp(message.header, time.time_ns(), "")
        status = self._types["DiagnosticStatus"]()
        status.name = self._node_name
        status.hardware_id = "pico-marvin-das"
        status.level = status.ERROR if self._last_error else status.OK
        status.message = self._last_error or "running"
        status.values = [
            self._types["KeyValue"](key=f"dropped_{name}", value=str(count))
            for name, count in self._drop_counts.items()
        ]
        message.status = [status]
        self._diagnostic_publisher.publish(message)

    def _publish_safely(self, publisher, *args):
        try:
            publisher(*args)
        except Exception as error:
            self._drop_counts["publish_error"] += 1
            self._last_error = str(error)

    def _run_critical(self):
        next_diagnostic_time = time.monotonic()
        queues = (
            self._critical_queue,
            *self._tactile_queues,
        )
        while not self._stop_event.is_set() or any(
            not target_queue.empty() for target_queue in queues
        ):
            try:
                self._publish_safely(
                    self._publish_critical, self._critical_queue.get_nowait()
                )
            except queue.Empty:
                pass
            for arm_index, target_queue in enumerate(self._tactile_queues):
                try:
                    self._publish_safely(
                        self._publish_tactile, arm_index, target_queue.get_nowait()
                    )
                except queue.Empty:
                    pass
            if time.monotonic() >= next_diagnostic_time:
                self._publish_safely(self._publish_diagnostics)
                next_diagnostic_time += 1.0
            time.sleep(0.002)

    def _run_images(self):
        while True:
            self._image_available.clear()
            for arm_index, target_queue in enumerate(self._camera_queues):
                try:
                    self._publish_safely(
                        self._publish_camera, arm_index, target_queue.get_nowait()
                    )
                except queue.Empty:
                    pass
            queues_empty = all(
                target_queue.empty() for target_queue in self._camera_queues
            )
            if self._stop_event.is_set() and queues_empty:
                return
            if queues_empty:
                self._image_available.wait()

    def close(self):
        self._stop_event.set()
        self._image_available.set()
        self._critical_thread.join(timeout=5.0)
        self._image_thread.join(timeout=5.0)
        self._node.destroy_node()
        if self._owns_context and self._rclpy.ok():
            self._rclpy.shutdown()
