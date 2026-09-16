"""ROS clients, telemetry, and raw episode validation tests."""

import unittest

from tests.marvin_hardware_fakes import (
    DASFingerConfiguration,
    episode_validator,
    FakeXrSdk,
    json,
    make_openxr_pose,
    np,
    patch,
    Path,
    queue,
    Ros2DataBridge,
    RosDasClient,
    RosPicoClient,
    SimpleNamespace,
    tempfile,
    threading,
    time,
    XrClient,
)


class TestMarvinRosData(unittest.TestCase):
    def test_pico_client_waits_for_advancing_data_and_rejects_stale_data(self):
        xr_sdk = FakeXrSdk([1, 1, 2, 2, 2, 2])
        with patch("builtins.print"):
            xr_client = XrClient(xr_sdk=xr_sdk)
            snapshot = xr_client.wait_for_fresh_snapshot(timeout_seconds=0.1)
        self.assertTrue(xr_sdk.initialized)
        self.assertEqual(snapshot.timestamp_ns, 2)
        np.testing.assert_allclose(snapshot.left_controller_pose[0], -0.1)
        xr_client.close()
        self.assertTrue(xr_sdk.closed)

        stale_sdk = FakeXrSdk([7])
        with patch("builtins.print"), patch(
            "xr_marvin_teleop.common.xr_client.time.monotonic_ns",
            side_effect=(100, 102, 103),
        ):
            stale_client = XrClient(
                xr_sdk=stale_sdk,
                max_source_age_seconds=1e-9,
                source_disconnect_timeout_seconds=2e-9,
            )
            stale_client.read_snapshot()
            self.assertIsNone(stale_client.read_snapshot())
            with self.assertRaises(TimeoutError):
                stale_client.read_snapshot()
        stale_client.close()

        class NonAtomicSdk:
            def init(self):
                pass

        with self.assertRaisesRegex(TypeError, "atomic get_snapshot"):
            XrClient(xr_sdk=NonAtomicSdk())


    def test_episode_validator_writes_manifest_for_required_raw_topics(self):
        topic = lambda count=2: {
            "count": count,
            "bag_time_regressions": 0,
            "source_time_regressions": 0,
            "sequence_gaps": 0,
        }
        statistics = {
            "/raw/pico/poses": topic(),
            "/raw/pico/joy": topic(),
            "/raw/pico/status": topic(),
            "/raw/marvin/joint_state": topic(),
            "/command/marvin/joint_target": topic(),
        }
        with tempfile.TemporaryDirectory() as directory:
            episode = Path(directory)
            (episode / "state").mkdir()
            (episode / "state" / "data.mcap").write_bytes(b"mcap")
            with patch.object(
                episode_validator, "inspect_bag", return_value=statistics
            ):
                manifest = episode_validator.validate_episode(episode)
            saved_manifest = json.loads(
                (episode / "manifest.json").read_text(encoding="utf-8")
            )
            (episode / "vision").mkdir()
            with patch.object(
                episode_validator,
                "inspect_bag",
                side_effect=(statistics, {}),
            ):
                missing_vision_manifest = episode_validator.validate_episode(
                    episode
                )

        self.assertEqual(manifest["status"], "validated")
        self.assertEqual(saved_manifest["status"], "validated")
        self.assertIn("state/data.mcap", manifest["files"])
        self.assertEqual(missing_vision_manifest["status"], "rejected")
        self.assertEqual(len(missing_vision_manifest["errors"]), 2)


    def test_ros_pico_client_accepts_source_sequence_restart(self):
        try:
            from geometry_msgs.msg import PoseArray
        except ImportError as error:
            self.skipTest(str(error))
        from xr_marvin_teleop.ros.protocol import PICO_TOPICS, SampleJoiner, pico_messages, sample_status
        client = RosPicoClient.__new__(RosPicoClient)
        client._timing = None
        client._condition = threading.Condition()
        client._snapshot = None
        client._valid = False
        client._sequence_id = 0
        client._update_id = 0
        client._receive_steady_ns = 0
        client._topics = PICO_TOPICS
        client._joiner = SampleJoiner(PICO_TOPICS, 200_000_000)

        def message(sequence_id, timestamp_ns):
            snapshot = SimpleNamespace(
                sequence_id=sequence_id,
                source_timestamp_ns=timestamp_ns,
                valid=True,
                left_controller_pose=make_openxr_pose(),
                right_controller_pose=make_openxr_pose(),
                grip_values=(0.0, 0.0),
                trigger_values=(0.0, 0.0),
                thumbstick_y_values=(0.0, 0.0),
                button_a=False,
                button_b=False,
                button_x=True,
                button_y=False,
                timestamp_ns=timestamp_ns,
            )
            wall = time.time_ns()
            poses, joy = pico_messages(snapshot, wall)
            status = sample_status(PICO_TOPICS, wall, "old" if sequence_id == 10 else "new",
                                   sequence_id, time.monotonic_ns(), source_timestamp_ns=timestamp_ns)
            client._callback(PICO_TOPICS[0], poses)
            client._callback("status", status)
            client._callback(PICO_TOPICS[1], joy)

        message(10, 100)
        message(10, 100)
        message(1, 200)

        self.assertEqual(client._sequence_id, 1)
        self.assertEqual(client._update_id, 2)
        self.assertEqual(client._snapshot.timestamp_ns, 200)
        self.assertTrue(client._snapshot.button_x)


    def test_ros_das_client_maps_feedback_and_publishes_commands(self):
        try:
            from sensor_msgs.msg import JointState
        except ImportError as error:
            self.skipTest(str(error))
        from xr_marvin_teleop.ros.protocol import GRIPPER_NAMES, SampleJoiner, joint_state, sample_status
        configurations = (
            DASFingerConfiguration("/dev/left", "/dev/video-left", 0.01, 0.07),
            DASFingerConfiguration(
                "/dev/right", "/dev/video-right", 0.01, 0.07, invert=True
            ),
        )
        client = RosDasClient.__new__(RosDasClient)
        client.configurations = configurations
        client.encoder_stale_timeout_seconds = 0.5
        client._condition = threading.Condition()
        client._distances = [float("nan"), float("nan")]
        client._targets = [0.05, 0.05]
        client._encoder_monotonic_ns = [0, 0]
        client._encoder_wall_time_ns = [0, 0]
        client._valid = [False, False]
        client._status_flags = [0, 0]
        client._sequence_ids = [0, 0]
        client._update_ids = [0, 0]
        client._command_sequence = 0
        client._is_connected = True
        client._session = "test"
        client._joiners = [SampleJoiner([f"/raw/das/{side}/state"], 500_000_000) for side in ("left", "right")]

        now_ns = time.monotonic_ns()

        for index, side in enumerate(("left", "right")):
            wall = time.time_ns()
            client._callback(index, "", joint_state([0.055], [GRIPPER_NAMES[index]], wall))
            client._callback(index, "/status", sample_status([f"/raw/das/{side}/state"], wall, "test", 1, now_ns,
                                                          target_distance_m=0.05, status_flags=0))

        class Command:
            def __init__(self):
                self.header = SimpleNamespace(
                    stamp=SimpleNamespace(sec=0, nanosec=0), frame_id=""
                )

        published = []
        client._message_type = Command
        client._publisher = SimpleNamespace(publish=published.append)
        client._status_publisher = SimpleNamespace(publish=lambda message: None)

        np.testing.assert_allclose(
            client.get_initial_gripper_closedness(), (0.25, 0.75)
        )
        np.testing.assert_allclose(
            client.send_gripper_command((0.2, 0.3)), (0.058, 0.028)
        )
        np.testing.assert_allclose(published[0].points[0].positions, [0.8, 0.7])


    def test_ros_image_publish_cannot_block_critical_publish_thread(self):
        bridge = Ros2DataBridge.__new__(Ros2DataBridge)
        bridge._critical_queue = queue.Queue()
        bridge._critical_queue.put(object())
        bridge._tactile_queues = (queue.Queue(), queue.Queue())
        bridge._camera_queues = (queue.Queue(), queue.Queue())
        bridge._camera_queues[0].put(object())
        bridge._gripper_command_subscription = None
        bridge._stop_event = threading.Event()
        bridge._stop_event.set()
        bridge._image_available = threading.Event()
        bridge._drop_counts = {"publish_error": 0}
        bridge._last_error = ""
        critical_published = threading.Event()
        image_started = threading.Event()
        release_image = threading.Event()
        bridge._publish_critical = lambda _item: critical_published.set()
        bridge._publish_tactile = lambda _arm, _item: None
        bridge._publish_diagnostics = lambda: None

        def block_image(_arm, _item):
            image_started.set()
            release_image.wait(timeout=1.0)

        bridge._publish_camera = block_image
        image_thread = threading.Thread(target=bridge._run_images)
        critical_thread = threading.Thread(target=bridge._run_critical)
        image_thread.start()
        try:
            self.assertTrue(image_started.wait(timeout=0.2))
            critical_thread.start()
            self.assertTrue(critical_published.wait(timeout=0.2))
        finally:
            release_image.set()
            if critical_thread.ident is not None:
                critical_thread.join(timeout=1.0)
            image_thread.join(timeout=1.0)


if __name__ == "__main__":
    unittest.main()
