"""V2 wire checks: sourced ROS2 + foxglove_msgs; never opens hardware."""
import math
import time
import unittest
import tempfile
from unittest.mock import patch

import numpy as np

from xr_marvin_teleop.ros.protocol import (
    JOINT_NAMES, PICO_TOPICS, SampleJoiner, joint_state, joint_positions, trajectory,
    pico_messages, sample_status, status_values, tactile_grid, gripper_targets, GRIPPER_NAMES,
)
from xr_marvin_teleop.common.xr_client import XrSnapshot
from xr_marvin_teleop.common.episode_postprocessor import _matrix_quaternion, _rpy_matrix
from xr_marvin_teleop.common.xr_target_mapper import _rotation_matrix_from_openxr_pose


class TestProtocol(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        try:
            from rclpy.serialization import serialize_message, deserialize_message
            from foxglove_msgs.msg import Grid
        except ImportError as error:
            raise unittest.SkipTest(str(error))
        cls.roundtrip = staticmethod(lambda message: deserialize_message(serialize_message(message), type(message)))

    def test_named_joints_quaternion_and_tactile_roundtrip(self):
        values = np.linspace(-1, 1, 14).tolist()
        message = joint_state(values[::-1], JOINT_NAMES[::-1], time.time_ns())
        np.testing.assert_allclose(joint_positions(self.roundtrip(message)), values)
        command = trajectory(values, JOINT_NAMES, time.time_ns())
        np.testing.assert_allclose(joint_positions(self.roundtrip(command)), values)
        command.points[0].time_from_start.sec = 1
        with self.assertRaises(ValueError):
            joint_positions(command)
        message.name[0] = message.name[1]
        with self.assertRaises(ValueError):
            joint_positions(message)
        for rpy in ((0, 0, 0), (math.pi, 0, 0), (0, math.pi / 2, 0), (1.1, -0.5, 3.14)):
            rotation = _rpy_matrix(rpy)
            q = _matrix_quaternion(rotation)
            self.assertAlmostEqual(np.linalg.norm(q), 1.)
            np.testing.assert_allclose(_rotation_matrix_from_openxr_pose([0, 0, 0, *q]), rotation, atol=1e-14)
        raw = bytes(range(256)) + bytes(range(192))
        grid = self.roundtrip(tactile_grid(raw, "left", time.time_ns()))
        self.assertEqual(bytes(grid.data), raw)
        self.assertEqual((grid.column_count, grid.row_stride, grid.cell_stride), (224, 224, 1))
        with self.assertRaises(ValueError):
            tactile_grid(raw[:-1], "left", time.time_ns())
        from xr_marvin_teleop.hardware.interface.das_finger import DASFingerConfiguration
        configs = (DASFingerConfiguration("/dev/l", "/dev/cl", .01, .07),) * 2
        self.assertEqual(gripper_targets(trajectory([0., 1.], GRIPPER_NAMES, time.time_ns()), configs), (.01, .07))
        with self.assertRaisesRegex(ValueError, "openness"):
            gripper_targets(trajectory([1.1, 0.], GRIPPER_NAMES, time.time_ns()), configs)

    def test_exact_join_loss_reorder_restart_and_freshness(self):
        now = time.time_ns()
        joiner = SampleJoiner(PICO_TOPICS, 200_000_000)
        snapshot = XrSnapshot(123, [0, 0, 0, 0, 0, 0, 1], [1, 0, 0, 0, 0, 0, 1],
                              (0.2, 0.3), True, False, (0.4, 0.5), (-1, 1), True, False)
        def records(stamp, session="a", sequence=1, valid=True):
            poses, joy = pico_messages(snapshot if valid else None, stamp)
            metadata = sample_status(PICO_TOPICS, stamp, session, sequence, time.monotonic_ns(),
                                     valid=valid, source_timestamp_ns=123)
            return self.roundtrip(poses), self.roundtrip(joy), self.roundtrip(metadata)

        poses, joy, status = records(now)
        self.assertIsNone(joiner.push(PICO_TOPICS[0], poses))
        self.assertIsNone(joiner.push("status", status))
        # A neighboring frame must never supply the missing Joy.
        self.assertIsNone(joiner.push(PICO_TOPICS[1], records(now + 1, sequence=2)[1]))
        joined = joiner.push(PICO_TOPICS[1], joy)
        self.assertEqual(list(joined[0][PICO_TOPICS[1]].buttons), [1, 0, 1, 0])
        self.assertEqual(joined[1]["sequence_id"], 1)
        self.assertIsNone(joiner.push("status", status))
        # Invalid metadata invalidates immediately, even if both payloads were lost.
        result = joiner.push("status", records(now + 2, sequence=2, valid=False)[2])
        self.assertFalse(result[1]["valid"])
        for key, msg in zip((*PICO_TOPICS, "status"), records(now + 3, session="b")):
            result = joiner.push(key, msg)
        self.assertEqual(result[1]["publisher_session_id"], "b")
        self.assertIsNone(joiner.push("status", records(now + 4, sequence=3, valid=False)[2]))
        with patch("xr_marvin_teleop.ros.protocol.time.time_ns", return_value=now + 1_000_000_000):
            self.assertIsNone(joiner.push("status", status))
        for i in range(80):
            joiner.push(PICO_TOPICS[0], records(now + 100 + i, session="b", sequence=i + 2)[0])
        self.assertLessEqual(len(joiner.pending), 32)

    def test_metadata_rejects_missing_identity(self):
        message = sample_status(PICO_TOPICS, time.time_ns(), "test", 1, 123)
        self.assertEqual(status_values(self.roundtrip(message))["receive_steady_ns"], 123)
        message.status[0].values = [v for v in message.status[0].values if v.key != "valid"]
        with self.assertRaises(ValueError):
            status_values(message)

    def test_validator_reports_missing_metadata_and_source_gaps(self):
        import rosbag2_py
        from rclpy.serialization import serialize_message
        from xr_marvin_teleop.common.episode_validator import inspect_bag
        topic = "/raw/marvin/joint_state"
        with tempfile.TemporaryDirectory() as directory:
            path = directory + "/state"
            writer = rosbag2_py.SequentialWriter()
            writer.open(rosbag2_py.StorageOptions(uri=path, storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
            for name, kind in ((topic, "sensor_msgs/msg/JointState"), (topic + "/status", "diagnostic_msgs/msg/DiagnosticArray")):
                writer.create_topic(rosbag2_py.TopicMetadata(name=name, type=kind, serialization_format="cdr"))
            now = time.time_ns()
            for i in (1, 2, 3):
                writer.write(topic, serialize_message(joint_state([0.] * 14, JOINT_NAMES, now + i)), now + i)
                if i != 2:
                    status = sample_status([topic], now + i, "test", i, 100 + i, source_timestamp_ns=10 - i)
                    writer.write(topic + "/status", serialize_message(status), now + i)
            writer.close()
            result = inspect_bag(path)[topic]
            self.assertEqual(result["sequence_check"], "available")
            self.assertEqual((result["metadata_missing"], result["sequence_gaps"], result["source_time_regressions"]), (1, 1, 1))


if __name__ == "__main__":
    unittest.main()
