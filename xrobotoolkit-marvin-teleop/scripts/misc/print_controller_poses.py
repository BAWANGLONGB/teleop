#!/usr/bin/env python3
"""Continuously print PICO controller poses received by XRoboToolkit."""

import argparse
import math
import time
from typing import Any


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Print PICO controller poses as position [m] and quaternion [qx, qy, qz, qw]."
    )
    parser.add_argument(
        "--controller",
        choices=("left", "right", "both"),
        default="both",
        help="controller to print (default: both)",
    )
    parser.add_argument(
        "--rate-hz",
        type=float,
        default=5.0,
        help="printing frequency in Hz (default: 5)",
    )
    parser.add_argument(
        "--count",
        type=int,
        default=0,
        help="number of samples to print; 0 means run until Ctrl+C (default: 0)",
    )
    parser.add_argument(
        "--startup-delay",
        type=float,
        default=1.0,
        help="seconds to wait for the asynchronous XR stream (default: 1)",
    )
    parser.add_argument(
        "--precision",
        type=int,
        default=5,
        help="number of digits after the decimal point (default: 5)",
    )
    args = parser.parse_args()

    if args.rate_hz <= 0:
        parser.error("--rate-hz must be positive")
    if args.count < 0:
        parser.error("--count cannot be negative")
    if args.startup_delay < 0:
        parser.error("--startup-delay cannot be negative")
    if args.precision < 0:
        parser.error("--precision cannot be negative")

    return args


def format_pose(side: str, pose: Any, precision: int) -> str:
    try:
        values_float = [float(value) for value in pose]
    except (TypeError, ValueError):
        return f"{side}: invalid pose={pose!r}"

    if len(values_float) != 7:
        return f"{side}: invalid pose length={len(values_float)}, expected 7 values"
    if not all(math.isfinite(value) for value in values_float):
        return f"{side}: invalid non-finite pose={values_float}"

    values = ", ".join(f"{value:.{precision}f}" for value in values_float)
    return f"{side}: [{values}]"


def main() -> None:
    args = parse_args()

    # Import after argument parsing so --help remains available even when the
    # XRoboToolkit SDK environment has not been activated yet.
    from xrobotoolkit_teleop.common.xr_client import XrClient

    sides = ("left", "right") if args.controller == "both" else (args.controller,)
    period = 1.0 / args.rate_hz
    client = XrClient()

    try:
        if args.startup_delay:
            time.sleep(args.startup_delay)

        print("pose format: [x, y, z, qx, qy, qz, qw], position unit: m")
        sample_index = 0
        next_sample_time = time.monotonic()

        while args.count == 0 or sample_index < args.count:
            timestamp_ns = client.get_timestamp_ns()
            poses = [
                format_pose(side, client.get_pose_by_name(f"{side}_controller"), args.precision)
                for side in sides
            ]
            print(f"sample={sample_index:06d} timestamp_ns={timestamp_ns} | " + " | ".join(poses), flush=True)

            sample_index += 1
            next_sample_time += period
            time.sleep(max(0.0, next_sample_time - time.monotonic()))
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        client.close()


if __name__ == "__main__":
    main()
