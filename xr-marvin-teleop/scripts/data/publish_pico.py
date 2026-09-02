#!/usr/bin/env python3
"""Own the PICO SDK and publish advancing raw frames at source rate."""

import argparse
import time
from contextlib import closing

from xr_marvin_teleop.common.xr_client import XrClient
from xr_marvin_teleop.ros.telemetry_bridge import Ros2DataBridge


def main():
    parser = argparse.ArgumentParser(description="Publish raw PICO frames to ROS2")
    parser.add_argument("--poll-hz", type=float, default=120.0)
    arguments = parser.parse_args()
    if not 30.0 <= arguments.poll_hz <= 240.0:
        parser.error("--poll-hz must be within [30, 240]")

    publisher = Ros2DataBridge("pico_data_source")
    period = 1.0 / arguments.poll_hz
    last_timestamp_ns = None
    invalid_published = False
    try:
        with closing(XrClient()) as xr_client:
            xr_client.wait_for_fresh_snapshot()
            next_poll = time.monotonic()
            while True:
                try:
                    snapshot = xr_client.read_snapshot()
                except TimeoutError:
                    snapshot = None
                if snapshot is None:
                    if not invalid_published:
                        publisher.publish_pico(None)
                        invalid_published = True
                elif snapshot.timestamp_ns != last_timestamp_ns:
                    publisher.publish_pico(snapshot)
                    last_timestamp_ns = snapshot.timestamp_ns
                    invalid_published = False
                next_poll += period
                remaining = next_poll - time.monotonic()
                if remaining > 0.0:
                    time.sleep(remaining)
                else:
                    next_poll = time.monotonic()
    except KeyboardInterrupt:
        pass
    finally:
        publisher.close()


if __name__ == "__main__":
    main()
