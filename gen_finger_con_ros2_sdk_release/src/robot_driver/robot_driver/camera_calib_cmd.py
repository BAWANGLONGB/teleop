#!/usr/bin/env python3
"""ROS2 camera calibration one-shot command sender for Gen Finger."""
import sys
import time

import rclpy

from .databus_single import DataBusNode, find_finger_serial_by_side, find_serial_port
from .pack import CmdPack

VALID_COMMANDS = ("1234", "camerarc", "MCUID")


def main(args=None):
    import argparse
    parser = argparse.ArgumentParser(description="Send a calibration command over DAS serial (Gen Finger)")
    parser.add_argument("record_value", type=str,
                        help="One of 1234/camerarc/MCUID")
    parser.add_argument("--side", type=str, default="", choices=["", "left", "right"])
    parser.add_argument("--serial-port", type=str, default="")
    cli_args, _unknown = parser.parse_known_args()

    if cli_args.record_value not in VALID_COMMANDS:
        print(f"Error: record value must be one of {'/'.join(VALID_COMMANDS)}")
        sys.exit(1)

    rclpy.init(args=args)

    serial_port = cli_args.serial_port or None
    if not serial_port:
        if cli_args.side in ("left", "right"):
            serial_port = find_finger_serial_by_side(cli_args.side)
        else:
            serial_port = find_serial_port()

    if not serial_port:
        print("No configured serial port found")
        rclpy.shutdown()
        sys.exit(1)

    print(f"Using serial: {serial_port}")
    print(f"Sending camera calib command: {cli_args.record_value}")

    record_bytes = cli_args.record_value.encode("utf-8")

    bus = DataBusNode(
        tty_port=serial_port,
        baudrate=921600,
        is_calib_cmd=True,
        calib_cmd_name=cli_args.record_value,
    )
    time.sleep(1.0)
    bus.add_cmd(CmdPack.pack_calib(record=record_bytes))

    if cli_args.record_value == "MCUID":
        bus.wait_for_calib_response(timeout=3.0)
    elif cli_args.record_value.startswith("camera"):
        bus.wait_for_calib_response(timeout=2.0)
    else:
        time.sleep(0.5)

    bus.stop()
    bus.destroy_node()
    rclpy.shutdown()

    if cli_args.record_value == "1234":
        print("Calibration OK !")
    elif cli_args.record_value == "MCUID":
        print("MCUID query executed")
    else:
        print(f"Finished sending command: {cli_args.record_value}")


if __name__ == "__main__":
    main()
