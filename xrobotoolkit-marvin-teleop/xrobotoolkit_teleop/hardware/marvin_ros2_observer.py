"""Optional ROS 2 observation publisher kept outside the Marvin safety loop."""

from __future__ import annotations

import json
import threading
import time

import meshcat.transformations as tf

from xrobotoolkit_teleop.common.marvin_types import LatestValue


class MarvinRos2Observer:
    """Publish latest hardware/control snapshots using standard ROS 2 messages."""

    def __init__(self, joint_names, namespace="/marvin_teleop", publish_hz=100.0):
        if len(joint_names) != 14:
            raise ValueError("joint_names must contain 14 names")
        if publish_hz <= 0.0:
            raise ValueError("publish_hz must be positive")
        self.joint_names = list(joint_names)
        self.namespace = "/" + namespace.strip("/")
        self.publish_hz = float(publish_hz)
        self._robot_state = LatestValue()
        self._control = LatestValue()
        self._safety = LatestValue()
        self._diagnostics = LatestValue()
        self._stop = threading.Event()
        self._thread = None
        self._error = None
        self._rclpy = None
        self._node = None
        self._owns_context = False

    @property
    def error(self):
        return self._error

    def start(self):
        if self._thread is not None:
            return
        try:
            import rclpy
            from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
            from geometry_msgs.msg import PoseStamped
            from rclpy.qos import (
                DurabilityPolicy,
                HistoryPolicy,
                QoSProfile,
                ReliabilityPolicy,
            )
            from sensor_msgs.msg import JointState
            from std_msgs.msg import String
        except ImportError as error:
            raise RuntimeError(
                "ROS 2 observation requested but rclpy or standard ROS messages are unavailable; "
                "source the ROS 2 environment before starting hardware"
            ) from error

        self._types = {
            "DiagnosticArray": DiagnosticArray,
            "DiagnosticStatus": DiagnosticStatus,
            "KeyValue": KeyValue,
            "PoseStamped": PoseStamped,
            "JointState": JointState,
            "String": String,
        }
        self._rclpy = rclpy
        self._owns_context = not rclpy.ok()
        if self._owns_context:
            rclpy.init(args=None)
        self._node = rclpy.create_node("marvin_observer", namespace=self.namespace)
        streaming_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._joint_state_publisher = self._node.create_publisher(
            JointState, "joint_states", streaming_qos
        )
        self._joint_command_publisher = self._node.create_publisher(
            JointState, "joint_command", streaming_qos
        )
        self._pose_publishers = {}
        for arm in ("left", "right"):
            for kind in ("tcp_actual", "tcp_target_raw", "tcp_target_limited"):
                self._pose_publishers[(arm, kind)] = self._node.create_publisher(
                    PoseStamped,
                    f"{arm}/{kind}",
                    streaming_qos,
                )
        self._safety_publisher = self._node.create_publisher(
            String, "safety_state", state_qos
        )
        self._diagnostics_publisher = self._node.create_publisher(
            DiagnosticArray, "diagnostics", streaming_qos
        )
        self._thread = threading.Thread(
            target=self._run,
            name="marvin-ros2-observer",
            daemon=True,
        )
        self._thread.start()

    def update_robot_state(self, state):
        self._robot_state.set(state)

    def update_control(self, control):
        self._control.set(control)

    def update_safety(self, state, reason):
        self._safety.set((str(state), str(reason)))

    def update_diagnostics(self, diagnostics):
        self._diagnostics.set(dict(diagnostics))

    def _stamp(self, message):
        message.header.stamp = self._node.get_clock().now().to_msg()
        message.header.frame_id = "dual_origin"

    def _joint_message(self, positions, velocities=None, efforts=None):
        message = self._types["JointState"]()
        self._stamp(message)
        message.name = self.joint_names
        message.position = [float(value) for value in positions]
        if velocities is not None:
            message.velocity = [float(value) for value in velocities]
        if efforts is not None:
            message.effort = [float(value) for value in efforts]
        return message

    def _pose_message(self, transform):
        message = self._types["PoseStamped"]()
        self._stamp(message)
        message.pose.position.x = float(transform[0, 3])
        message.pose.position.y = float(transform[1, 3])
        message.pose.position.z = float(transform[2, 3])
        quaternion_wxyz = tf.quaternion_from_matrix(transform)
        message.pose.orientation.w = float(quaternion_wxyz[0])
        message.pose.orientation.x = float(quaternion_wxyz[1])
        message.pose.orientation.y = float(quaternion_wxyz[2])
        message.pose.orientation.z = float(quaternion_wxyz[3])
        return message

    def _publish_robot_state(self, state):
        self._joint_state_publisher.publish(
            self._joint_message(state.q_rad, state.dq_rad_s, state.torque_nm)
        )

    def _publish_control(self, control):
        self._joint_command_publisher.publish(self._joint_message(control.q_command_rad))
        transforms_by_kind = {
            "tcp_actual": control.actual_tcp_transforms,
            "tcp_target_raw": control.raw_tcp_transforms,
            "tcp_target_limited": control.limited_tcp_transforms,
        }
        for arm_index, arm in enumerate(("left", "right")):
            for kind, transforms in transforms_by_kind.items():
                self._pose_publishers[(arm, kind)].publish(
                    self._pose_message(transforms[arm_index])
                )

    def _publish_safety(self, safety):
        message = self._types["String"]()
        message.data = json.dumps(
            {"state": safety[0], "reason": safety[1]},
            ensure_ascii=False,
        )
        self._safety_publisher.publish(message)

    def _publish_diagnostics(self, diagnostics):
        array = self._types["DiagnosticArray"]()
        array.header.stamp = self._node.get_clock().now().to_msg()
        status = self._types["DiagnosticStatus"]()
        status.name = f"{self.namespace}/communication"
        status.hardware_id = "marvin"
        safety_state = str(diagnostics.get("safety_state", "unknown"))
        status.level = (
            self._types["DiagnosticStatus"].OK
            if safety_state not in ("hold", "fault")
            else (
                self._types["DiagnosticStatus"].WARN
                if safety_state == "hold"
                else self._types["DiagnosticStatus"].ERROR
            )
        )
        status.message = str(diagnostics.get("safety_reason", ""))
        status.values = [
            self._types["KeyValue"](key=str(key), value=str(value))
            for key, value in diagnostics.items()
        ]
        array.status = [status]
        self._diagnostics_publisher.publish(array)

    def _run(self):
        period = 1.0 / self.publish_hz
        last_robot_serial = None
        last_control_sequence = None
        last_safety = None
        next_deadline = time.monotonic()
        try:
            while not self._stop.is_set():
                robot_state = self._robot_state.get()
                if robot_state is not None and robot_state.frame_serial != last_robot_serial:
                    self._publish_robot_state(robot_state)
                    last_robot_serial = robot_state.frame_serial
                control = self._control.get()
                if control is not None and control.sequence != last_control_sequence:
                    self._publish_control(control)
                    last_control_sequence = control.sequence
                safety = self._safety.get()
                if safety is not None and safety != last_safety:
                    self._publish_safety(safety)
                    last_safety = safety
                diagnostics = self._diagnostics.get()
                if diagnostics is not None:
                    self._publish_diagnostics(diagnostics)
                self._rclpy.spin_once(self._node, timeout_sec=0.0)
                next_deadline += period
                delay = next_deadline - time.monotonic()
                if delay > 0.0:
                    self._stop.wait(delay)
                else:
                    next_deadline = time.monotonic()
        except Exception as error:
            self._error = error

    def close(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        if self._node is not None:
            self._node.destroy_node()
            self._node = None
        if self._owns_context and self._rclpy is not None and self._rclpy.ok():
            self._rclpy.shutdown()
        self._thread = None
