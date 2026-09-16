"""Run with sourced ROS2 and pip install -e '.[video]'; no hardware required."""

import argparse
import hashlib
from collections import Counter
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

from xr_marvin_teleop.common.episode_video import (
    activity_lock,
    add_video_arguments,
    av1_obu_types,
    export_episode,
    h264_nal_types,
    video_options,
)
from xr_marvin_teleop.common.collection_config import load_config, snapshot_config, read_json


class TestEpisodeVideo(unittest.TestCase):
    def test_av1_low_overhead_obu_parser(self):
        payload = b"\x12\x00\x0a\x01\x00\x32\x01\x00"
        self.assertEqual(av1_obu_types(payload), {1, 2, 6})
        for invalid in (b"", b"\x08", b"\x0a\x02\x00"):
            with self.assertRaises(ValueError):
                av1_obu_types(invalid)

    def test_switches_parameters_and_idle_lock(self):
        parser = argparse.ArgumentParser()
        add_video_arguments(parser, inherit=True)
        options = video_options(
            parser.parse_args(
                ["--no-mjpeg", "--h264", "--no-av1", "--h264-crf", "28"]
            ),
            {"h264_keyint": 45},
        )
        self.assertEqual((options["mjpeg"], options["h264"], options["h264_crf"],
                          options["h264_keyint"]), (False, True, 28, 45))
        self.assertFalse(video_options(saved={"mjpeg": True, "h264": False})["av1"])
        for invalid in ({"mjpeg": False, "h264": False, "av1": False}, {"h264_threads": 0},
                        {"h264_crf": -1}, {"h264_keyint": 0}, {"h264_preset": "invalid"}):
            with self.assertRaises(ValueError):
                video_options(saved=invalid)
        with tempfile.TemporaryDirectory() as directory:
            with activity_lock(directory), activity_lock(directory):
                with self.assertRaises(RuntimeError):
                    activity_lock(directory, exclusive=True)
            with activity_lock(directory, exclusive=True):
                with self.assertRaises(RuntimeError):
                    activity_lock(directory)

    def test_real_ros_to_video_mcaps_and_decode(self):
        try:
            import av
            import numpy as np
            import rosbag2_py
            from mcap.reader import make_reader
            from rclpy.serialization import serialize_message
            from sensor_msgs.msg import CompressedImage as RosImage, JointState, Joy
            from geometry_msgs.msg import PoseArray
            from trajectory_msgs.msg import JointTrajectory
            from diagnostic_msgs.msg import DiagnosticArray
            from foxglove_msgs.msg import Grid
            from xr_marvin_teleop.common.xr_client import XrSnapshot
            from xr_marvin_teleop.ros.protocol import (
                JOINT_NAMES, PICO_TOPICS, joint_state, trajectory, sample_status, status_topic,
                pico_messages, tactile_grid, stamp_ns, status_values)
            from foxglove_schemas_protobuf.CompressedVideo_pb2 import CompressedVideo
        except ImportError as error:
            self.skipTest(f"integration dependencies unavailable: {error}")
        from xr_marvin_teleop.common.episode_postprocessor import postprocess_episode
        from xr_marvin_teleop.common.episode_validator import validate_episode

        with tempfile.TemporaryDirectory() as directory:
            episode = Path(directory) / "episode_test"
            episode.mkdir()
            metadata = {"episode_id": episode.name, "status": "completed", "message_protocol_version": 2,
                        "bags": ["state", "vision_left", "vision_right"],
                        "camera_profiles": {"left": {"latency_correction_ns": 25000000}}}
            config = load_config()
            config["export"]["h264_crf"] = 27
            snapshot = snapshot_config(config, episode / "config", require_devices=False)
            metadata.update(config_snapshot="config/collection.json", capture_config=read_json(snapshot),
                            video_outputs=config["export"], config_files=[
                                {"snapshot": str(p.relative_to(episode)), "sha256": hashlib.sha256(p.read_bytes()).hexdigest()}
                                for p in snapshot.parent.iterdir()])
            (episode / "metadata.json").write_text(json.dumps(metadata))
            jpeg_encoder = av.CodecContext.create("mjpeg", "w")
            jpeg_encoder.width, jpeg_encoder.height = 64, 64
            jpeg_encoder.pix_fmt = "yuvj420p"
            from fractions import Fraction
            jpeg_encoder.time_base = Fraction(1, 30)
            jpeg = bytes(jpeg_encoder.encode(av.VideoFrame.from_ndarray(
                np.full((64, 64, 3), 120, dtype=np.uint8), format="rgb24"))[0])
            stamps = [1_700_000_000_123_456_789 + i * 33_333_337 for i in range(7)]
            originals = {}
            for bag in metadata["bags"]:
                writer = rosbag2_py.SequentialWriter()
                writer.open(rosbag2_py.StorageOptions(uri=str(episode / bag), storage_id="mcap"),
                            rosbag2_py.ConverterOptions("", ""))
                specs = (("/raw/marvin/joint_state", JointState),
                         ("/command/marvin/joint_target", JointTrajectory),
                         (PICO_TOPICS[0], PoseArray), (PICO_TOPICS[1], Joy),
                         ("/raw/das/left/tactile", Grid), ("/raw/das/right/tactile", Grid)) if bag == "state" else (
                    (f"/raw/das/{bag.removeprefix('vision_')}/image/compressed", RosImage),)
                for topic, message_type in specs:
                    writer.create_topic(rosbag2_py.TopicMetadata(
                        name=topic, type=message_type.__module__.split('.')[0] + '/msg/' + message_type.__name__, serialization_format="cdr"))
                for topic in {status_topic(topic) for topic, _ in specs}:
                    writer.create_topic(rosbag2_py.TopicMetadata(name=topic, type="diagnostic_msgs/msg/DiagnosticArray", serialization_format="cdr"))
                for i, stamp in enumerate(stamps):
                    for topic, message_type in specs:
                        message = message_type()
                        if message_type is JointState:
                            message = joint_state([0.] * 14, JOINT_NAMES, stamp, [0.] * 14)
                        elif message_type is JointTrajectory:
                            message = trajectory([0.] * 14, JOINT_NAMES, stamp)
                        elif message_type is RosImage:
                            message.data, message.format = jpeg, "jpeg"
                            message.header.frame_id = bag
                        elif message_type is Grid:
                            message = tactile_grid(bytes(range(256)) + bytes(range(192)), topic.split('/')[3], stamp)
                        elif message_type in (PoseArray, Joy):
                            xr_snapshot = XrSnapshot(i + 1, [0, 0, 0, 0, 0, 0, 1], [0, 0, 0, 0, 0, 0, 1], (0., 0.), False, False)
                            message = pico_messages(xr_snapshot, stamp)[0 if message_type is PoseArray else 1]
                        if hasattr(message, "header"):
                            message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp, 10**9)
                        # Deliberately unrelated monotonic clock: it must not affect final time.
                        if hasattr(message, "receive_steady_ns"):
                            message.receive_steady_ns = 999 + i
                        data = serialize_message(message)
                        originals[topic, stamp] = data
                        writer.write(topic, data, stamp + 1_000_000)
                        if topic != PICO_TOPICS[1]:
                            targets = PICO_TOPICS if topic == PICO_TOPICS[0] else [topic]
                            status = sample_status(targets, stamp, "test", i + 1, 999 + i)
                            status_data = serialize_message(status)
                            originals[status_topic(topic), stamp] = status_data
                            writer.write(status_topic(topic), status_data, stamp + 1_000_000)
                writer.close()
            summary = postprocess_episode(episode)
            self.assertEqual(summary["alignment"]["clock"], "CLOCK_REALTIME")
            self.assertEqual(summary["alignment"]["topic_time_offsets_ns"], {})
            self.assertEqual(validate_episode(episode)["status"], "validated")
            options = video_options(saved={
                "mjpeg": True,
                "h264": True,
                "av1": True,
                "h264_keyint": 3,
                "av1_keyint": 3,
                "av1_threads": 1,
            })
            # A failed transcode cannot publish partial outputs or delete the originals.
            with patch("xr_marvin_teleop.common.episode_video.Av1Encoder.encode", side_effect=RuntimeError("disk/codec failure")):
                with self.assertRaisesRegex(RuntimeError, "disk/codec failure"):
                    export_episode(episode, options)
            self.assertFalse((episode / "final").exists())
            self.assertTrue((episode / "vision_left/metadata.yaml").exists())
            outputs = export_episode(episode, options)
            self.assertEqual(len(outputs), 3)
            for output in outputs:
                from scripts.data.migrate_sessions import verify
                self.assertEqual(len(verify(output)["counts"]), 19)
                variant = output.suffixes[-2]
                counts, keys = Counter(), {"left": [], "right": []}
                codec = "libdav1d" if variant == ".av1" else "h264"
                decoders = {side: av.CodecContext.create(codec, "r") for side in keys}
                for decoder in decoders.values():
                    decoder.thread_count = 1
                decoded = Counter()
                with output.open("rb") as stream:
                    reader = make_reader(stream, validate_crcs=True)
                    for schema, channel, message in reader.iter_messages():
                        topic = channel.topic
                        self.assertIn(message.log_time, stamps)
                        self.assertEqual(message.log_time, message.publish_time)
                        index = counts[topic]
                        counts[topic] += 1
                        if schema.name.startswith("foxglove."):
                            value = CompressedVideo.FromString(message.data)
                            self.assertEqual(value.timestamp.ToNanoseconds(), stamps[index])
                            side = topic.split("/")[3]
                            self.assertEqual(value.format, variant.removeprefix("."))
                            if variant == ".h264":
                                nals = h264_nal_types(value.data)
                                if 5 in nals:
                                    keys[side].append(index)
                                    self.assertTrue({7, 8}.issubset(nals))
                                    # Each IDR is independently decodable for seeking.
                                    self.assertEqual(len(av.CodecContext.create("h264", "r").decode(av.Packet(value.data))), 1)
                                frames = decoders[side].decode(av.Packet(value.data))
                                self.assertEqual(len(frames), 1)
                                self.assertNotEqual(frames[0].pict_type, av.video.frame.PictureType.B)
                                self.assertEqual((frames[0].width, frames[0].height), (64, 64))
                            elif variant == ".av1":
                                obu_types = av1_obu_types(value.data)
                                if 1 in obu_types:
                                    keys[side].append(index)
                                    independent = av.CodecContext.create("libdav1d", "r")
                                    independent.thread_count = 1
                                    frames = independent.decode(av.Packet(value.data))
                                    frames += independent.decode(None)
                                    self.assertEqual(len(frames), 1)
                                decoded[side] += len(
                                    decoders[side].decode(av.Packet(value.data))
                                )
                        elif schema.name == "sensor_msgs/msg/CompressedImage":
                            from rclpy.serialization import deserialize_message
                            frame = deserialize_message(message.data, RosImage)
                            self.assertEqual(bytes(frame.data), jpeg)
                            self.assertEqual(stamp_ns(frame), message.log_time)
                        elif "/video/compressed/status" in topic:
                            from rclpy.serialization import deserialize_message
                            status = status_values(deserialize_message(message.data, DiagnosticArray))
                            self.assertEqual(status["topics"], [topic.removesuffix("/status")])
                        elif (topic, message.log_time) in originals:
                            self.assertEqual(message.data, originals[topic, message.log_time])
                        self.assertFalse(schema.name.startswith("teleop_msgs/"))
                    self.assertEqual(len(counts), 19)  # 6 state + 4 FK + 2 cameras + 7 metadata
                    self.assertEqual(set(counts.values()), {7})
                    attachments = {a.name: a.data for a in reader.iter_attachments()}
                    self.assertEqual(len(attachments), 1 + len(metadata["config_files"]))
                    self.assertEqual(attachments["config/collection.json"], snapshot.read_bytes())
                    self.assertEqual(json.loads(attachments["meta/meta.json"])["capture_config"]["export"]["h264_crf"], 27)
                if variant == ".av1":
                    for side, decoder in decoders.items():
                        decoded[side] += len(decoder.decode(None))
                    self.assertEqual(decoded, {"left": 7, "right": 7})
                if variant in (".h264", ".av1"):
                    self.assertEqual(keys, {"left": [0, 3, 6], "right": [0, 3, 6]})
            with self.assertRaises(FileExistsError):
                export_episode(episode, options)
            # Retain checked exports while exercising independently selected variants.
            (episode / "final").rename(episode / "checked_dual")
            mjpeg_only = video_options(saved={"mjpeg": True, "h264": False, "av1": False})
            with patch("xr_marvin_teleop.common.episode_video.H264Encoder", side_effect=AssertionError("H264 disabled")), \
                 patch("xr_marvin_teleop.common.episode_video.Av1Encoder", side_effect=AssertionError("AV1 disabled")):
                self.assertEqual(len(export_episode(episode, mjpeg_only)), 1)
            mjpeg = episode / "final/episode_test.mjpeg.mcap"
            original_hash = hashlib.sha256(mjpeg.read_bytes()).hexdigest()
            h264_only = video_options(saved={"mjpeg": False, "h264": True, "av1": False})
            self.assertEqual(len(export_episode(episode, h264_only, add_missing=True)), 1)
            self.assertTrue((episode / "final/episode_test.h264.mcap").is_file())
            self.assertEqual(hashlib.sha256(mjpeg.read_bytes()).hexdigest(), original_hash)
            with patch("xr_marvin_teleop.common.episode_video.H264Encoder", side_effect=AssertionError("cached export reencoded")):
                self.assertEqual(len(export_episode(episode, h264_only, add_missing=True)), 1)
            (episode / "final").rename(episode / "checked_mjpeg")
            av1_only = video_options(saved={"mjpeg": False, "h264": False, "av1": True, "av1_threads": 1})
            self.assertEqual(len(export_episode(episode, av1_only)), 1)
            (episode / "final").rename(episode / "checked_h264")
            completed = subprocess.run(
                [sys.executable, str(Path(__file__).resolve().parents[1] / "scripts/data/postprocess_episode.py"),
                 str(episode), "--output-root", directory, "--mjpeg", "--no-h264", "--no-av1", "--h264-crf", "28"],
                capture_output=True, text=True, timeout=30,
            )
            self.assertEqual(completed.returncode, 0, completed.stdout + completed.stderr)
            saved = json.loads((episode / "metadata.json").read_text())
            self.assertEqual(saved["export_status"], "completed")
            self.assertEqual(saved["export_options"]["h264_crf"], 28)
            self.assertEqual(saved["video_outputs"]["h264_crf"], 27)
            self.assertEqual(saved["capture_config"]["export"]["h264_crf"], 27)
            self.assertEqual(saved["final_outputs"], ["final/episode_test.mjpeg.mcap"])


if __name__ == "__main__":
    unittest.main()
