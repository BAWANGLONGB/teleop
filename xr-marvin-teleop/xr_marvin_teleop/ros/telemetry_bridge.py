"""Bounded ROS2 publishers for native-rate collection streams."""

import queue
import threading
import time

import numpy as np


ARM_NAMES = ("left", "right")


class Ros2DataBridge:
    """Publish raw streams without allowing recorder backpressure into control."""

    def __init__(
        self,
        node_name="marvin_data_bridge",
        gripper_command_callback=None,
        publish_gripper_commands=True,
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
            from teleop_msgs.msg import (
                DasState,
                GripperCommand,
                ImageFrame,
                JointCommand,
                MarvinState,
                PicoFrame,
                TactileFrame,
            )
        except (ImportError, OSError) as error:
            raise RuntimeError(
                "ROS2 collection requires a sourced teleop_msgs workspace; "
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
        self._node_name = str(node_name)
        self._types = {
            "DiagnosticArray": DiagnosticArray,
            "DiagnosticStatus": DiagnosticStatus,
            "KeyValue": KeyValue,
            "PicoFrame": PicoFrame,
            "MarvinState": MarvinState,
            "JointCommand": JointCommand,
            "GripperCommand": GripperCommand,
            "DasState": DasState,
            "TactileFrame": TactileFrame,
            "ImageFrame": ImageFrame,
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
                PicoFrame, "/raw/pico/frame", sensor_qos
            ),
            "marvin": self._node.create_publisher(
                MarvinState, "/raw/marvin/joint_state", critical_qos
            ),
            "joint_command": self._node.create_publisher(
                JointCommand, "/command/marvin/joint_target", critical_qos
            ),
        }
        if publish_gripper_commands:
            self._publishers["gripper_command"] = self._node.create_publisher(
                GripperCommand, "/command/das/target", critical_qos
            )
        self._gripper_command_callback = gripper_command_callback
        self._gripper_command_subscription = (
            None
            if gripper_command_callback is None
            else self._node.create_subscription(
                GripperCommand,
                "/command/das/target",
                self._handle_gripper_command,
                critical_qos,
            )
        )
        self._das_publishers = tuple(
            self._node.create_publisher(
                DasState, f"/raw/das/{side}/state", critical_qos
            )
            for side in ARM_NAMES
        )
        self._tactile_publishers = tuple(
            self._node.create_publisher(
                TactileFrame, f"/raw/das/{side}/tactile", sensor_qos
            )
            for side in ARM_NAMES
        )
        self._camera_publishers = tuple(
            self._node.create_publisher(
                ImageFrame, f"/raw/das/{side}/image", sensor_qos
            )
            for side in ARM_NAMES
        )
        self._diagnostic_publisher = self._node.create_publisher(
            DiagnosticArray, "/diagnostics", critical_qos
        )
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
        if "gripper_command" not in self._publishers:
            return
        wall_time_ns = time.time_ns() if wall_time_ns is None else int(wall_time_ns)
        steady_ns = time.monotonic_ns() if steady_ns is None else int(steady_ns)
        self._enqueue(
            self._critical_queue,
            (
                "gripper_command",
                self._next_sequence("gripper_command"),
                tuple(float(value) for value in closedness),
                wall_time_ns,
                steady_ns,
            ),
            "critical",
        )

    def _handle_gripper_command(self, message):
        try:
            self._gripper_command_callback(tuple(message.closedness))
        except Exception as error:
            self._drop_counts["publish_error"] += 1
            self._last_error = str(error)

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

    @staticmethod
    def _stamp(header, wall_time_ns, frame_id):
        header.stamp.sec, header.stamp.nanosec = divmod(
            int(wall_time_ns), 1_000_000_000
        )
        header.frame_id = frame_id

    def _publish_critical(self, item):
        kind = item[0]
        message = self._types[
            {
                "pico": "PicoFrame",
                "marvin": "MarvinState",
                "joint_command": "JointCommand",
                "gripper_command": "GripperCommand",
                "das": "DasState",
            }[kind]
        ]()
        if kind == "das":
            _, sequence_id, arm_index, state = item
            self._stamp(
                message.header,
                state["wall_time_ns"],
                f"finger_{ARM_NAMES[arm_index]}",
            )
            message.sequence_id = sequence_id
            message.source_timestamp_ns = int(state.get("source_timestamp_ns", 0))
            message.receive_steady_ns = int(state["steady_ns"])
            message.valid = bool(state.get("valid", True))
            message.side = ARM_NAMES[arm_index]
            message.distance_m = float(state["distance_m"])
            message.target_distance_m = float(state["target_distance_m"])
            message.status_flags = int(state.get("status_flags", 0))
            self._das_publishers[arm_index].publish(message)
            return

        _, sequence_id, payload, wall_time_ns, steady_ns = item
        frame_id = "openxr_local" if kind == "pico" else "marvin_base"
        self._stamp(message.header, wall_time_ns, frame_id)
        message.sequence_id = sequence_id
        if kind == "pico":
            message.source_timestamp_ns = 0 if payload is None else payload.timestamp_ns
            message.receive_steady_ns = steady_ns
            message.valid = payload is not None
            message.left_controller_pose = [float("nan")] * 7 if payload is None else payload.left_controller_pose.tolist()
            message.right_controller_pose = [float("nan")] * 7 if payload is None else payload.right_controller_pose.tolist()
            message.grip_values = [float("nan")] * 2 if payload is None else list(payload.grip_values)
            message.trigger_values = [float("nan")] * 2 if payload is None else list(payload.trigger_values)
            message.thumbstick_y_values = [float("nan")] * 2 if payload is None else list(payload.thumbstick_y_values)
            message.button_a = False if payload is None else payload.button_a
            message.button_b = False if payload is None else payload.button_b
        elif kind == "marvin":
            message.source_timestamp_ns = 0
            message.receive_steady_ns = steady_ns
            message.valid = True
            message.frame_serial = list(payload.frame_serial)
            message.arm_state = list(payload.arm_state)
            message.error_code = list(payload.error_code)
            message.low_speed = list(payload.low_speed)
            message.q_rad = payload.q_rad.tolist()
            message.dq_rad_s = payload.dq_rad_s.tolist()
        elif kind == "joint_command":
            message.issue_steady_ns = steady_ns
            message.q_rad = payload.tolist()
        else:
            message.issue_steady_ns = steady_ns
            message.closedness = list(payload)
        self._publishers[kind].publish(message)

    def _publish_tactile(self, arm_index, item):
        sequence_id, raw_data, wall_time_ns, steady_ns = item
        message = self._types["TactileFrame"]()
        self._stamp(message.header, wall_time_ns, f"finger_{ARM_NAMES[arm_index]}")
        message.sequence_id = sequence_id
        message.source_timestamp_ns = 0
        message.receive_steady_ns = steady_ns
        message.valid = True
        message.side = ARM_NAMES[arm_index]
        message.data = raw_data
        self._tactile_publishers[arm_index].publish(message)

    def _publish_camera(self, arm_index, item):
        sequence_id, frame, wall_time_ns, steady_ns = item
        message = self._types["ImageFrame"]()
        image = message.image
        self._stamp(image.header, wall_time_ns, f"finger_{ARM_NAMES[arm_index]}")
        image.height, image.width = frame.shape[:2]
        channels = 1 if frame.ndim == 2 else frame.shape[2]
        image.encoding = {1: "mono8", 3: "bgr8", 4: "bgra8"}[channels]
        image.is_bigendian = 0
        image.step = image.width * channels
        image.data = frame.tobytes()
        message.sequence_id = sequence_id
        message.source_timestamp_ns = 0
        message.receive_steady_ns = steady_ns
        self._camera_publishers[arm_index].publish(message)

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
            if self._gripper_command_subscription is not None:
                self._rclpy.spin_once(self._node, timeout_sec=0.0)
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


Ros2TelemetryBridge = Ros2DataBridge
