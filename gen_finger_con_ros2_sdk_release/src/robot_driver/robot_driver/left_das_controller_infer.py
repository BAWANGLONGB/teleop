#!/usr/bin/env python3
# ROS2 port of left_das_controller_infer.py (Gen Finger)
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32


class FingerDataConverter(Node):
    def __init__(self):
        super().__init__('das_controller_converter')

        self.publish_rate = 100
        self.latest_left_data = None
        self.latest_left_cmd = None

        self.create_subscription(
            Float32, '/left_finger/encoder', self.left_finger_data_callback, 10)
        self.left_finger_feedback_pub = self.create_publisher(
            PoseStamped, '/finger/left/current_distance', 10)

        self.create_subscription(
            PoseStamped, '/target_finger/left_finger', self.left_cmd_callback, 10)
        self.left_finger_cmd_pub = self.create_publisher(
            Float32, '/left_finger/target_distance', 10)

        self.get_logger().debug("Finger Data Converter Node Started")
        self.get_logger().debug(f"Publish rate: {self.publish_rate}Hz")

        period = 1.0 / self.publish_rate
        self.create_timer(period, self.publish_all_data)

    def left_finger_data_callback(self, msg):
        self.latest_left_data = msg
        self.get_logger().info(f'left finger distance: {msg.data:.4f} m')

    def process_finger_feedback(self, finger_msg, publisher, finger_name):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = f"{finger_name}_finger_frame"

        pose_msg.pose.position.x = float(finger_msg.data)
        pose_msg.pose.position.y = 0.0
        pose_msg.pose.position.z = 0.0

        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = 0.0
        pose_msg.pose.orientation.w = 1.0

        publisher.publish(pose_msg)

    def left_cmd_callback(self, msg):
        self.latest_left_cmd = msg

    def process_finger_cmd(self, pos_msg, publisher, finger_name):
        finger_cmd_msg = Float32()
        finger_cmd_msg.data = float(pos_msg.pose.position.x)
        publisher.publish(finger_cmd_msg)

    def publish_all_data(self):
        if self.latest_left_data is not None:
            self.process_finger_feedback(
                self.latest_left_data, self.left_finger_feedback_pub, "left")
        if self.latest_left_cmd is not None:
            self.process_finger_cmd(
                self.latest_left_cmd, self.left_finger_cmd_pub, "left")
        # self.left_finger_cmd_pub.publish(Float32(data=0.05))  # range 0.0–0.2 m


def main(args=None):
    rclpy.init(args=args)
    node = FingerDataConverter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
