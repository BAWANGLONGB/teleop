"""Run explicitly with ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=191; no hardware."""
import os
import json
import threading
import time
from contextlib import ExitStack, closing
from unittest.mock import patch

from xr_marvin_teleop.common.xr_client import XrSnapshot
from xr_marvin_teleop.common.pico_timing import PicoTimingLog
from xr_marvin_teleop.hardware.interface.das_finger import DASFingerConfiguration
from xr_marvin_teleop.ros.telemetry_bridge import Ros2DataBridge
from xr_marvin_teleop.ros.pico_client import RosPicoClient
from xr_marvin_teleop.ros.das_client import RosDasClient
from xr_marvin_teleop.ros.protocol import SampleJoiner, gripper_targets


def main():
    if os.environ.get("ROS_LOCALHOST_ONLY") != "1" or os.environ.get("ROS_DOMAIN_ID") != "191":
        raise RuntimeError("run in the isolated localhost test domain 191")
    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray
    from trajectory_msgs.msg import JointTrajectory
    configs = tuple(DASFingerConfiguration(f"/dev/{side}", f"/dev/camera-{side}", 0., .15)
                    for side in ("left", "right"))
    thread_errors = []
    with patch("threading.excepthook", lambda args: thread_errors.append(args.exc_value)), ExitStack() as stack:
        source_timing = stack.enter_context(closing(PicoTimingLog("test_source")))
        bridge = stack.enter_context(closing(Ros2DataBridge("v2_test_source", publish_gripper_commands=False,
                                                           pico_timing=source_timing)))
        pico = stack.enter_context(closing(RosPicoClient()))
        das = stack.enter_context(closing(RosDasClient(configs)))
        node = rclpy.create_node("v2_test_command_sink")
        stack.callback(node.destroy_node)
        joined, commands = SampleJoiner(["/command/das/target"], 500_000_000), []

        def receive(key, message):
            result = joined.push(key, message)
            if result:
                commands.append(gripper_targets(result[0]["/command/das/target"], configs))

        node.create_subscription(JointTrajectory, "/command/das/target", lambda m: receive("/command/das/target", m), 10)
        node.create_subscription(DiagnosticArray, "/command/das/target/status", lambda m: receive("status", m), 10)
        stop = threading.Event()

        def produce():
            while not stop.is_set():
                now, steady = time.time_ns(), time.monotonic_ns()
                bridge.publish_pico(XrSnapshot(steady, [0, 0, 0, 0, 0, 0, 1], [0, 0, 0, 0, 0, 0, 1],
                                              (0., 0.), False, False, button_x=True), now, steady)
                for arm in (0, 1):
                    bridge.publish_das_state(arm, dict(wall_time_ns=now, steady_ns=steady, valid=True,
                                                      distance_m=.05, target_distance_m=.05, status_flags=0))
                stop.wait(.02)

        producer = threading.Thread(target=produce)
        producer.start()
        try:
            assert pico.wait_for_fresh_snapshot(5).button_x
            das.connect(5)
            das.send_gripper_command((.2, .8))
            deadline = time.monotonic() + 3
            while not commands and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.02)
            assert commands and all(abs(a - b) < 1e-12 for a, b in zip(commands[0], (.12, .03)))
        finally:
            stop.set()
            producer.join()
        # A source gap must still hold control after 200 ms, never extend freshness.
        time.sleep(.3)
        assert pico.read_snapshot() is None
        # Reproduce SIGINT's context shutdown while both client wait sets are active.
        rclpy.shutdown()
        pico._thread.join(timeout=1)
        das._thread.join(timeout=1)
        assert not pico._thread.is_alive() and not das._thread.is_alive()
    assert not thread_errors, thread_errors
    assert source_timing.error is None and pico._timing.error is None
    source_records = [json.loads(line) for line in source_timing.path.read_text().splitlines()]
    assert any(r.get("counts", {}).get("publish", 0) for r in source_records)
    print("ROS v2 live PICO/DAS/command roundtrip passed (no hardware)")


if __name__ == "__main__":
    main()
