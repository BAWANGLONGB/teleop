#!/usr/bin/env python3
"""Own both DAS SDK instances and publish their data through ROS2."""

import argparse
import threading
import time
from pathlib import Path

from xr_marvin_teleop.hardware.interface.das_finger import (
    DASFingerAdapter,
    load_das_finger_configurations,
)
from xr_marvin_teleop.ros.telemetry_bridge import Ros2DataBridge


def main(arguments=None):
    parser = argparse.ArgumentParser(description="Publish DAS streams to ROS2")
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--sdk-root", required=True, type=Path)
    parser.add_argument("--command-hz", type=float, default=50.0)
    parser.add_argument("--ready-timeout", type=float, default=10.0)
    parsed = parser.parse_args(arguments)

    bridge = None
    ready = threading.Event()
    adapter = DASFingerAdapter(
        load_das_finger_configurations(parsed.config),
        sdk_root_path=parsed.sdk_root,
        command_hz=parsed.command_hz,
        ready_timeout_seconds=parsed.ready_timeout,
        state_callback=lambda arm, state: (
            bridge.publish_das_state(arm, state) if ready.is_set() else None
        ),
        tactile_callback=lambda arm, data, wall, steady: bridge.publish_tactile(
            arm, data, wall, steady
        ),
        frame_callback=lambda arm, frame, wall, steady: bridge.publish_camera(
            arm, frame, wall, steady
        ),
    )
    bridge = Ros2DataBridge(
        "das_data_source",
        gripper_command_callback=adapter.send_gripper_command,
        publish_gripper_commands=False,
    )
    try:
        adapter.connect()
        ready.set()
        print("DAS source ready", flush=True)
        while True:
            adapter.check_health()
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        adapter.set_idle()
        adapter.release()
        bridge.close()


if __name__ == "__main__":
    main()
