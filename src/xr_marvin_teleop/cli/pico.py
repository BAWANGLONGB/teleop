#!/usr/bin/env python3
"""Own the PICO SDK and publish advancing raw frames at source rate."""

import argparse
import time
from contextlib import closing

from xr_marvin_teleop.adapters.xr import XrClient
from xr_marvin_teleop.collection.hotkeys import CollectionHotkeys
from xr_marvin_teleop.collection.pico_timing import PicoTimingLog
from xr_marvin_teleop.ros.telemetry_bridge import Ros2DataBridge


def main():
    parser = argparse.ArgumentParser(description="Publish raw PICO frames to ROS2")
    parser.add_argument("--poll-hz", type=float, default=120.0)
    arguments = parser.parse_args()
    if not 30.0 <= arguments.poll_hz <= 240.0:
        parser.error("--poll-hz must be within [30, 240]")

    timing = PicoTimingLog("source")
    publisher = Ros2DataBridge("pico_data_source", pico_timing=timing)
    period = 1.0 / arguments.poll_hz
    last_timestamp_ns = None
    last_new_frame_ns = None
    last_callback_sequence = None
    missing_native_timing_reported = False
    invalid_published = False
    hotkeys = CollectionHotkeys()
    try:
        with closing(XrClient()) as xr_client:
            xr_client.wait_for_fresh_snapshot()
            next_poll = time.monotonic()
            while True:
                poll_ns = time.monotonic_ns()
                try:
                    snapshot = xr_client.read_snapshot()
                except TimeoutError:
                    snapshot = None
                read_done_ns = time.monotonic_ns()
                timing.record("poll", read_ns=read_done_ns - poll_ns,
                              late_ns=max(0, poll_ns - int(next_poll * 1e9)))
                if snapshot is not None and snapshot.timing:
                    sdk = snapshot.timing
                    timing.record("sdk_cache", age_ns=read_done_ns - sdk["sdk_ready_steady_ns"])
                    if sdk["sdk_callback_sequence"] != last_callback_sequence:
                        timing.record("sdk", identity=dict(source_timestamp_ns=snapshot.timestamp_ns, **sdk),
                                      callback_gap_ns=sdk["sdk_callback_gap_ns"], parse_ns=sdk["sdk_parse_ns"],
                                      lock_wait_ns=sdk["sdk_ready_steady_ns"] - sdk["sdk_receive_steady_ns"] - sdk["sdk_parse_ns"])
                        last_callback_sequence = sdk["sdk_callback_sequence"]
                elif snapshot is not None and not missing_native_timing_reported:
                    timing.record("sdk", event="native_timing_unavailable_rebuild_extension")
                    missing_native_timing_reported = True
                if snapshot is None:
                    hotkeys.update(None)
                    if not invalid_published:
                        publisher.publish_pico(None)
                        invalid_published = True
                elif snapshot.timestamp_ns != last_timestamp_ns:
                    timing.record("source", identity=dict(source_timestamp_ns=snapshot.timestamp_ns, **snapshot.timing),
                                  new_frame_gap_ns=0 if last_new_frame_ns is None else read_done_ns - last_new_frame_ns)
                    last_new_frame_ns = read_done_ns
                    publisher.publish_pico(snapshot)
                    hotkeys.update(snapshot)
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
        hotkeys.close()
        publisher.close()
        timing.close()


if __name__ == "__main__":
    main()
