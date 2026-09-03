#!/usr/bin/env python3
# ROS2 port of tactile_dual_print.py
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8MultiArray

COLS = 10
ROWS = 50
GAP = " " * 10


def _normalize_ns(ns):
    ns = (ns or "").strip()
    return ns.strip("/")


def _topic(ns, tactile_name):
    n = _normalize_ns(ns)
    return "/{}/tactile/{}".format(n, tactile_name)


class TactileDualPrinter(Node):
    def __init__(self):
        super().__init__('tactile_dual_print')

        self.declare_parameter('finger_ns', 'left_finger')
        self.declare_parameter('print_hz', 30.0)

        finger_ns = self.get_parameter('finger_ns').value
        hz = float(self.get_parameter('print_hz').value)
        self._topic_left = _topic(finger_ns, "left")
        self._topic_right = _topic(finger_ns, "right")

        self._lock = threading.Lock()
        self._data_left = None
        self._data_right = None

        self.get_logger().info(f"finger_ns={finger_ns}")
        self.get_logger().info(f"subscribe tactile/left:  {self._topic_left}")
        self.get_logger().info(f"subscribe tactile/right: {self._topic_right}")

        self.create_subscription(Int8MultiArray, self._topic_left, self._cb_left, 1)
        self.create_subscription(Int8MultiArray, self._topic_right, self._cb_right, 1)

        period = max(0.02, 1.0 / hz) if hz > 0 else 0.05
        self.create_timer(period, self._on_timer)

    def _cb_left(self, msg):
        with self._lock:
            self._data_left = list(msg.data)

    def _cb_right(self, msg):
        with self._lock:
            self._data_right = list(msg.data)

    def _on_timer(self):
        with self._lock:
            if self._data_left is None or self._data_right is None:
                return
            L, R = list(self._data_left), list(self._data_right)
        n = min(len(L), len(R), ROWS * COLS)
        if n < ROWS * COLS:
            self.get_logger().warn(
                f"tactile length is less than 500: left={len(L)} right={len(R)}, will only print the first {n} numbers",
                throttle_duration_sec=5.0,
            )
        for row in range(ROWS):
            i0 = row * COLS
            if i0 + COLS > n:
                break
            left_seg = L[i0: i0 + COLS]
            right_seg = R[i0: i0 + COLS]
            left_s = " ".join("{:3d}".format(int(x)) for x in left_seg)
            right_s = " ".join("{:3d}".format(int(x)) for x in right_seg)
            print(left_s + GAP + right_s)
        print("", flush=True)


def main(args=None):
    rclpy.init(args=args)
    node = TactileDualPrinter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
