"""Timing and shutdown checks use synthetic input only; never initialize an SDK."""
import json
from pathlib import Path
import queue
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from xr_marvin_teleop.common.pico_timing import PicoTimingLog
from xr_marvin_teleop.common.xr_client import XrClient


class TestPicoTiming(unittest.TestCase):
    def test_bounded_queue_summary_and_anomaly(self):
        with tempfile.TemporaryDirectory() as directory:
            log = PicoTimingLog("test", directory)
            for value in (1, 2, 3, 350):
                log.record("source", identity={"sequence_id": value}, gap_ns=value * 1_000_000)
            log.close()
            self.assertIsNone(log.error)
            rows = [json.loads(line) for line in log.path.read_text().splitlines()]
            summary = next(r for r in rows if r["event"] == "summary")
            self.assertEqual(summary["milliseconds"]["source.gap_ns"],
                             dict(count=4, p50=2, p95=350, p99=350, max=350))
            self.assertEqual(summary["latest_identity"]["source"], {"sequence_id": 350})
            anomaly = next(r for r in rows if r["event"] == "anomaly")
            self.assertEqual(anomaly["durations_ns"]["gap_ns"], 350_000_000)
        # A stalled writer cannot make producers wait or grow the queue.
        log = PicoTimingLog.__new__(PicoTimingLog)
        log._queue = queue.Queue(maxsize=1)
        log._stop = threading.Event()
        log.error, log.dropped = None, 0
        for _ in range(100):
            log.record("poll", read_ns=10)
        self.assertEqual(log._queue.qsize(), 1)
        self.assertEqual(log.dropped, 99)

    def test_log_failure_does_not_propagate_to_producer(self):
        with tempfile.TemporaryDirectory() as directory:
            with patch.object(Path, "open", side_effect=OSError("disk unavailable")):
                log = PicoTimingLog("test", directory)
                log.record("poll", read_ns=1)
                log.close()
            self.assertEqual(log.error, "disk unavailable")
            log.record("poll", read_ns=1)

    def test_sdk_timing_survives_python_snapshot_without_changing_source_clock(self):
        pose = [0, 0, 0, 0, 0, 0, 1]
        timing = dict(sdk_receive_steady_ns=100, sdk_ready_steady_ns=120,
                      sdk_parse_ns=15, sdk_callback_gap_ns=10, sdk_callback_sequence=7)
        sdk = SimpleNamespace(init=lambda: None, close=lambda: None,
            get_snapshot=lambda: dict(timestamp_ns=9000000000, left_controller_pose=pose,
                                      right_controller_pose=pose, grip_values=(0, 0),
                                      button_a=False, button_b=False, timing=timing))
        client = XrClient(sdk)
        try:
            snapshot = client.read_snapshot()
            self.assertEqual(snapshot.timing, timing)
            self.assertEqual(snapshot.timestamp_ns, 9000000000)
        finally:
            client.close()

    def test_shutdown_race_and_real_errors(self):
        try:
            from rclpy.impl.implementation_singleton import rclpy_implementation
            from rclpy.executors import ExternalShutdownException
        except ImportError as error:
            self.skipTest(str(error))
        from xr_marvin_teleop.ros import spin_until_stopped
        executor = Mock()
        executor.spin_once.side_effect = rclpy_implementation.RCLError("context stopped")
        spin_until_stopped(executor, SimpleNamespace(ok=Mock(side_effect=[True, False])), threading.Event())
        with self.assertRaises(rclpy_implementation.RCLError):
            spin_until_stopped(executor, SimpleNamespace(ok=lambda: True), threading.Event())
        executor.spin_once.side_effect = ExternalShutdownException()
        spin_until_stopped(executor, SimpleNamespace(ok=lambda: True), threading.Event())

    def test_receiver_timing_preserves_200ms_hold(self):
        try:
            from xr_marvin_teleop.ros.protocol import PICO_TOPICS, SampleJoiner, pico_messages, sample_status
            from geometry_msgs.msg import PoseArray
        except ImportError as error:
            self.skipTest(str(error))
        from xr_marvin_teleop.common.xr_client import XrSnapshot
        from xr_marvin_teleop.ros.pico_client import RosPicoClient
        client = RosPicoClient.__new__(RosPicoClient)
        client._timing = Mock()
        client._condition = threading.Condition()
        client._topics = PICO_TOPICS
        client._joiner = SampleJoiner(PICO_TOPICS, 200_000_000)
        client._max_age_ns, client._disconnect_timeout_ns = 200_000_000, 2_000_000_000
        client._update_id, client._receive_steady_ns = 0, 0
        client._last_read_state = None
        pose = [0, 0, 0, 0, 0, 0, 1]
        snapshot = XrSnapshot(123, pose, pose, (0, 0), False, False)
        wall = time.time_ns()
        poses, joy = pico_messages(snapshot, wall)
        status = sample_status(PICO_TOPICS, wall, "source", 1, time.monotonic_ns(), source_timestamp_ns=123)
        for topic, message in zip((*PICO_TOPICS, "status"), (poses, joy, status)):
            client._callback(topic, message)
        self.assertEqual(client.read_snapshot().timestamp_ns, 123)
        self.assertEqual(client._sample_identity["publisher_session_id"], "source")
        with patch("xr_marvin_teleop.ros.pico_client.time.monotonic_ns", return_value=client._receive_steady_ns + 201_000_000):
            self.assertIsNone(client.read_snapshot())
        self.assertEqual(client._timing.record.call_args.kwargs["event"], "hold")
        client._callback("status", SimpleNamespace())
        self.assertFalse(client._valid)


if __name__ == "__main__":
    unittest.main()
