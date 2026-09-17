import json
import runpy
import struct
import tempfile
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from xr_marvin_teleop.collection.episode_postprocessor import (
    UrdfForwardKinematics,
    matrix_rpy,
    rpy_matrix,
    _topic_time_ns,
)
from xr_marvin_teleop.collection.episode_package import (
    attachment_entries,
    extract_episode_mcap,
    read_attachment,
    validate_episode_mcap,
    write_episode_mcap,
)
from xr_marvin_teleop.collection import episode_validator
from xr_marvin_teleop.adapters.das_finger import (
    DASFingerConfiguration,
)


class TestEpisodePostprocessor(unittest.TestCase):
    def test_legacy_attachments_round_trip_and_detect_corruption(self):
        with tempfile.TemporaryDirectory() as directory:
            session = Path(directory) / "session_2026-09-07"
            episode = session / "episode_120000_deadbeef"
            episode.mkdir(parents=True)
            output = session / "data/chunk-000/episode_000000.mcap"
            meta = Path(directory) / "meta.json"
            parquet = Path(directory) / "data.parquet"
            meta.write_text(json.dumps({
                "dataset_format": "lerobot",
                "episode_id": episode.name,
                "episode_index": 0,
                "video_paths": {},
            }), encoding="utf-8")
            parquet.write_bytes(b"PAR1testPAR1")
            write_episode_mcap(output, (
                ("meta/meta.json", "application/json", meta),
                ("data/data.parquet", "application/vnd.apache.parquet", parquet),
            ))
            self.assertEqual(
                [item["name"] for item in attachment_entries(output)],
                ["meta/meta.json", "data/data.parquet"],
            )
            self.assertEqual(validate_episode_mcap(output)["episode_id"], episode.name)
            self.assertEqual(read_attachment(output, "data/data.parquet"), b"PAR1testPAR1")
            extracted = extract_episode_mcap(output, Path(directory) / "extracted")
            self.assertEqual((extracted / "data/data.parquet").read_bytes(), b"PAR1testPAR1")
            entry = attachment_entries(output)[1]
            with output.open("r+b") as file:
                file.seek(entry["data_offset"])
                file.write(b"X")
            with self.assertRaisesRegex(ValueError, "CRC mismatch"):
                validate_episode_mcap(output)

    def test_diagnostics_use_bag_time_for_multi_publisher_order(self):
        first, source = _topic_time_ns(
            "/diagnostics", object(), 1_000
        )
        second, _source = _topic_time_ns(
            "/diagnostics", object(), 1_001
        )
        self.assertEqual((first, second, source), (1_000, 1_001, "bag_time_ns"))

    def test_camera_alignment_uses_system_time_without_latency_correction(self):
        message = SimpleNamespace(
            receive_steady_ns=1_000,
            image=SimpleNamespace(
                header=SimpleNamespace(stamp=SimpleNamespace(sec=10, nanosec=123))
            ),
        )
        timestamp, source = _topic_time_ns(
            "/raw/das/left/image/compressed", message, 9_999
        )
        self.assertEqual((timestamp, source), (10_000_000_123, "header.stamp"))

    def test_episode_time_uses_payload_or_falls_back_to_bag(self):
        for payload, expected in (
            ('{"wall_time_ns": 123}', (123, "payload.wall_time_ns")),
            ('{"wall_time_ns": 0}', (999, "bag_time_ns")),
            ("invalid JSON", (999, "bag_time_ns")),
        ):
            with self.subTest(payload=payload):
                self.assertEqual(
                    _topic_time_ns("/episode/event", SimpleNamespace(data=payload), 999),
                    expected,
                )

    def test_camera_resolution_latency_defaults(self):
        low = DASFingerConfiguration("/dev/l", "/dev/cl", 0.0, 0.15)
        high = DASFingerConfiguration(
            "/dev/l", "/dev/cl", 0.0, 0.15, camera_resolution="1600x1296"
        )
        self.assertEqual((low.camera_latency_ms, high.camera_latency_ms), (25.0, None))

    def test_review_uses_nearest_feedback_and_previous_command(self):
        namespace = runpy.run_path(
            str(
                Path(__file__).resolve().parents[1]
                / "scripts"
                / "data"
                / "review_episode.py"
            )
        )
        series = ([100_000_000, 200_000_000], ["first", "second"])
        self.assertEqual(
            namespace["_nearest"](series, 160_000_000), ("second", 40.0)
        )
        self.assertEqual(
            namespace["_previous"](series, 160_000_000), ("first", 60.0)
        )

    def test_single_side_das_process_holds_feedback_then_applies_commands(self):
        namespace = runpy.run_path(
            str(
                Path(__file__).resolve().parents[1]
                / "scripts"
                / "data"
                / "publish_das.py"
            )
        )
        publisher = namespace["DasSidePublisher"].__new__(
            namespace["DasSidePublisher"]
        )
        publisher.side = "left"
        publisher.arm_index = 0
        publisher.configurations = (
            DASFingerConfiguration("/dev/l", "/dev/cl", 0.0, 0.15),
            DASFingerConfiguration("/dev/r", "/dev/cr", 0.0, 0.15),
        )
        publisher.configuration = publisher.configurations[0]
        publisher._target_lock = threading.Lock()
        publisher._target_m = 0.05
        publisher._last_encoder_steady_ns = 0
        publisher._encoder_ready = threading.Event()
        publisher._calibration_required = False
        publisher._error = None
        publisher._state_sequence = 0
        publisher._last_command_sequence = 0
        publisher._drops = {"state": 0, "tactile": 0}
        import queue

        publisher._state_queue = queue.Queue(maxsize=2)

        class Bus:
            def __init__(self):
                self.targets = []

            def set_target_distance(self, value):
                self.targets.append(value)

        publisher._bus = Bus()
        publisher._handle_encoder(struct.pack(">f", 0.04))
        self.assertTrue(publisher._encoder_ready.is_set())
        self.assertAlmostEqual(publisher._bus.targets[0], 0.04)
        try:
            from xr_marvin_teleop.ros.protocol import SampleJoiner, GRIPPER_NAMES, trajectory, sample_status
            import time
            now = time.time_ns()
            command = trajectory([0., 1.], GRIPPER_NAMES, now)
        except ImportError as error:
            self.skipTest(f"ROS2 unavailable: {error}")
        publisher._command_joiner = SampleJoiner(["/command/das/target"], 500_000_000)
        publisher._handle_command(command)
        self.assertAlmostEqual(publisher._bus.targets[-1], 0.04)
        publisher._handle_command(sample_status(["/command/das/target"], now, "test", 1, time.monotonic_ns(), command=True), "status")
        self.assertEqual(publisher._bus.targets[-1], 0.0)

    def test_native_mjpeg_pipeline_never_decodes_or_reencodes(self):
        namespace = runpy.run_path(
            str(
                Path(__file__).resolve().parents[1]
                / "scripts"
                / "data"
                / "capture_das_mjpeg.py"
            )
        )
        description = namespace["pipeline_description"](
            "/dev/finger_camera_left", 640, 480, 60
        )
        self.assertIn("v4l2src", description)
        self.assertIn("image/jpeg", description)
        self.assertNotIn("jpegdec", description)
        self.assertNotIn("jpegenc", description)
        self.assertTrue(namespace["is_jpeg"](b"\xff\xd8data\xff\xd9"))
        self.assertFalse(namespace["is_jpeg"](b"raw-bgr"))
        with tempfile.TemporaryDirectory() as directory:
            preview = Path(directory) / "left.jpg"
            writer = namespace["NativeMjpegWriter"].__new__(
                namespace["NativeMjpegWriter"]
            )
            writer.preview_file = preview
            writer.preview_fps = namespace["PREVIEW_FPS"]
            writer._next_preview_ns = 0
            writer._preview_disabled = False
            writer.side = "left"
            writer._write_preview(b"\xff\xd8first\xff\xd9", 1)
            writer._write_preview(b"\xff\xd8skipped\xff\xd9", 2)
            self.assertEqual(preview.read_bytes(), b"\xff\xd8first\xff\xd9")
            writer._write_preview(
                b"\xff\xd8second\xff\xd9",
                1_000_000_000 // namespace["PREVIEW_FPS"] + 1,
            )
            self.assertEqual(preview.read_bytes(), b"\xff\xd8second\xff\xd9")
            self.assertFalse(list(preview.parent.glob("*.tmp")))

        recorder = runpy.run_path(
            str(
                Path(__file__).resolve().parents[1]
                / "scripts"
                / "data"
                / "record_episode.py"
            )
        )
        command = recorder["_camera_command"](
            Path("/project"),
            "left",
            SimpleNamespace(
                camera_device="/dev/finger_camera_left",
                camera_resolution="640x480",
                camera_fps=60,
            ),
            Path("/output"),
            Path("/storage.yaml"),
            Path("/ready"),
            Path("/dev/shm/preview/left.jpg"),
        )
        self.assertEqual(
            command[command.index("--preview-file") + 1],
            "/dev/shm/preview/left.jpg",
        )

    def test_urdf_fk_and_rpy(self):
        project_root = Path(__file__).resolve().parents[1]
        kinematics = UrdfForwardKinematics(
            project_root / "assets" / "marvin" / "marvin_dual.urdf"
        )
        left = kinematics.forward(
            0, np.deg2rad((90.0, -90.0, -90.0, -20.0, 90.0, 0.0, 0.0))
        )
        right = kinematics.forward(
            1, np.deg2rad((-90.0, -90.0, 90.0, -20.0, -90.0, 0.0, 0.0))
        )

        np.testing.assert_allclose(
            left[:3, 3], (-0.121134603, -0.218600006, -0.764988472), atol=3e-5
        )
        np.testing.assert_allclose(
            right[:3, 3], (-0.121134603, 0.218600006, -0.764988473), atol=3e-5
        )
        rotation = rpy_matrix((0.4, -0.3, 1.2))
        np.testing.assert_allclose(
            rpy_matrix(matrix_rpy(rotation)), rotation, atol=1e-12
        )

        message = SimpleNamespace(receive_steady_ns=123, issue_steady_ns=0)
        self.assertEqual(
            _topic_time_ns("/raw/marvin/joint_state", message, 999),
            (999, "bag_time_ns"),
        )

    def test_validator_accepts_two_native_mjpeg_fragments(self):
        def topic():
            return {
                "count": 2,
                "bag_time_regressions": 0,
                "source_time_regressions": 0,
                "sequence_gaps": 0,
                "steady_alignment_errors": 0,
            }

        state = {
            "/raw/pico/poses": topic(),
            "/raw/pico/joy": topic(),
            "/raw/pico/status": topic(),
            "/raw/marvin/joint_state": topic(),
            "/command/marvin/joint_target": topic(),
        }
        with tempfile.TemporaryDirectory() as directory:
            episode = Path(directory)
            for name in ("state", "vision_left", "vision_right"):
                (episode / name).mkdir()
            (episode / "metadata.json").write_text(
                '{"bags":["state","vision_left","vision_right"]}',
                encoding="utf-8",
            )
            with patch.object(
                episode_validator,
                "inspect_bag",
                side_effect=(
                    state,
                    {"/raw/das/left/image/compressed": topic()},
                    {"/raw/das/right/image/compressed": topic()},
                ),
            ):
                manifest = episode_validator.validate_episode(episode)
        self.assertEqual(manifest["status"], "validated")


if __name__ == "__main__":
    unittest.main()
