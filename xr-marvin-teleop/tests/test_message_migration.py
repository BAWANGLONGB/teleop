"""Legacy conversion is tested separately; only this test needs teleop_msgs."""
import hashlib
import json
from pathlib import Path
import runpy
import tempfile
import unittest


class TestMessageMigration(unittest.TestCase):
    def test_raw_bag_copy_converts_names_units_and_retains_source(self):
        try:
            import rosbag2_py
            from rclpy.serialization import serialize_message, deserialize_message
            from rosidl_runtime_py.utilities import get_message
            from teleop_msgs.msg import PicoFrame, MarvinState, JointCommand, GripperCommand, TactileFrame
            from foxglove_msgs.msg import Grid
        except ImportError as error:
            self.skipTest(f"legacy migration dependencies unavailable: {error}")
        from xr_marvin_teleop.hardware.interface.das_finger import DASFingerConfiguration
        from xr_marvin_teleop.ros.protocol import GRIPPER_NAMES, joint_positions
        from xr_marvin_teleop.common.episode_postprocessor import postprocess_episode
        migrate = runpy.run_path(str(Path(__file__).resolve().parents[1] / "scripts/data/migrate_messages_v2.py"))["migrate"]
        configs = (DASFingerConfiguration("/dev/l", "/dev/cl", 0.01, 0.07),
                   DASFingerConfiguration("/dev/r", "/dev/cr", 0.01, 0.07, invert=True))
        with tempfile.TemporaryDirectory() as directory:
            source, target = Path(directory) / "old", Path(directory) / "new"
            source.mkdir()
            (source / "metadata.json").write_text(json.dumps({"episode_id": "episode_migrated", "bags": ["state"]}))
            writer = rosbag2_py.SequentialWriter()
            writer.open(rosbag2_py.StorageOptions(uri=str(source / "state"), storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
            records = [("/raw/pico/frame", PicoFrame()), ("/raw/marvin/joint_state", MarvinState()),
                       ("/command/marvin/joint_target", JointCommand()), ("/command/das/target", GripperCommand()),
                       ("/raw/das/left/tactile", TactileFrame())]
            raw = bytes(range(256)) + bytes(range(192))
            for topic, message in records:
                message.header.stamp.sec = 1700000000
                message.sequence_id = 1
                if hasattr(message, "valid"):
                    message.valid = True
                if isinstance(message, PicoFrame):
                    message.source_timestamp_ns = 123
                    message.left_controller_pose = message.right_controller_pose = [0., 0., 0., 0., 0., 0., 1.]
                if isinstance(message, GripperCommand):
                    message.closedness = [0.2, 0.3]
                if isinstance(message, TactileFrame):
                    message.side, message.data = "left", raw
                writer.create_topic(rosbag2_py.TopicMetadata(name=topic, type="teleop_msgs/msg/" + type(message).__name__, serialization_format="cdr"))
                writer.write(topic, serialize_message(message), 1700000000001000000)
            writer.close()
            hashes = {p: hashlib.sha256(p.read_bytes()).digest() for p in source.rglob("*") if p.is_file()}
            with self.assertRaisesRegex(ValueError, "das-config"):
                migrate(source, target)
            self.assertFalse(target.exists())
            migrate(source, target, configs)
            self.assertEqual(json.loads((target / "manifest.json").read_text())["status"], "validated")
            reader = rosbag2_py.SequentialReader()
            reader.open(rosbag2_py.StorageOptions(uri=str(target / "state"), storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
            types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
            self.assertTrue(all(not t.type.startswith("teleop_msgs/") for t in reader.get_all_topics_and_types()))
            while reader.has_next():
                topic, data, stamp = reader.read_next()
                self.assertEqual(stamp, 1700000000001000000)
                message = deserialize_message(data, types[topic])
                if topic == "/command/das/target":
                    actual = joint_positions(message, GRIPPER_NAMES)
                    self.assertAlmostEqual(actual[0], 0.8)
                    self.assertAlmostEqual(actual[1], 0.7)
                if isinstance(message, Grid):
                    self.assertEqual(bytes(message.data), raw)
            postprocess_episode(target)
            with self.assertRaises(ValueError):
                migrate(source, target, configs)
            self.assertEqual(hashes, {p: hashlib.sha256(p.read_bytes()).digest() for p in hashes})


if __name__ == "__main__":
    unittest.main()
