"""Marvin and DAS SDK boundary tests."""

import unittest

from tests.marvin_hardware_fakes import (
    _decode_encoder_value,
    _modbus_write_single_register_frame,
    closedness_to_das_distances,
    DASFingerAdapter,
    DASFingerConfiguration,
    FakeDasFingerSystem,
    FakeMarvinRobot,
    FakeMarvinSdkAdapter,
    FakeMarvinVendorKinematics,
    FakeXRClient,
    json,
    load_das_finger_configurations,
    make_openxr_pose,
    MarvinHardwareTeleopController,
    MarvinModbusGripperConfiguration,
    MarvinSdkAdapter,
    MarvinToolConfiguration,
    np,
    Path,
    RecordingTelemetry,
    struct,
    tempfile,
    time,
    XrSnapshot,
)


class TestMarvinInterfaces(unittest.TestCase):
    def test_control_sdk_converts_radians_to_vendor_degrees(self):
        fake_marvin_robot = FakeMarvinRobot()
        adapter = MarvinSdkAdapter(
            marvin_robot=fake_marvin_robot, dcss_structure=object()
        )
        adapter.connect()
        robot_feedback = adapter.read_state()
        np.testing.assert_allclose(robot_feedback.q_rad, 0.0)

        q_deg = np.arange(14, dtype=float)
        tools = (
            MarvinToolConfiguration([0.0] * 6, [1.0] + [0.0] * 9),
            MarvinToolConfiguration([0.0] * 6, [2.0] + [0.0] * 9),
        )
        adapter.configure_control_parameters(
            [5.0] * 7,
            [0.9] * 7,
            [4.0] * 7,
            [0.8] * 7,
            tools,
            joint_velocity_ratio=80,
            joint_acceleration_ratio=70,
        )
        self.assertEqual(fake_marvin_robot.joint_impedance["A"], ([5.0] * 7, [0.9] * 7))
        self.assertEqual(fake_marvin_robot.joint_impedance["B"], ([4.0] * 7, [0.8] * 7))
        self.assertEqual(fake_marvin_robot.joint_motion_limits["A"], (80, 70))
        self.assertEqual(fake_marvin_robot.joint_motion_limits["B"], (80, 70))
        self.assertEqual(fake_marvin_robot.tools["A"][1], [1.0] + [0.0] * 9)
        self.assertEqual(fake_marvin_robot.tools["B"][1], [2.0] + [0.0] * 9)
        self.assertEqual(fake_marvin_robot.wait_response_calls, 0)
        adapter.send_joint_command(np.deg2rad(q_deg))
        np.testing.assert_allclose(
            fake_marvin_robot.q_commands_deg["A"], q_deg[:7]
        )
        np.testing.assert_allclose(
            fake_marvin_robot.q_commands_deg["B"], q_deg[7:]
        )
        adapter.release()
        self.assertTrue(fake_marvin_robot.released)


    def test_incremental_gripper_controls_and_modbus_frame(self):
        self.assertEqual(
            _modbus_write_single_register_frame(1, 0, 1),
            bytes.fromhex("01 06 00 00 00 01 48 0A"),
        )
        fake_marvin_robot = FakeMarvinRobot()
        gripper_configs = (
            MarvinModbusGripperConfiguration(1, 10, 1000, 0, 0.5),
            MarvinModbusGripperConfiguration(1, 10, 0, 1000, 0.5),
        )
        hardware_adapter = MarvinSdkAdapter(
            marvin_robot=fake_marvin_robot,
            dcss_structure=object(),
            gripper_configurations=gripper_configs,
        )
        hardware_adapter.connect()
        self.assertEqual(
            hardware_adapter.send_gripper_command((0.25, 0.75)),
            (750, 750),
        )
        self.assertEqual(fake_marvin_robot.cleared_channels, ["A", "B"])
        self.assertEqual(
            [frame[:6] for _, frame, _ in fake_marvin_robot.channel_frames],
            [bytes.fromhex("01 06 00 0A 02 EE")] * 2,
        )
        hardware_adapter.release()

        def snapshot(timestamp, trigger=0.0, stick_y=0.0, button_b=False):
            return XrSnapshot(
                timestamp,
                make_openxr_pose(),
                make_openxr_pose(),
                (0.0, 0.0),
                False,
                button_b,
                (trigger, 0.0),
                (stick_y, 0.0),
            )

        adapter = FakeMarvinSdkAdapter()
        telemetry = RecordingTelemetry()
        controller = MarvinHardwareTeleopController(
            xr_client=FakeXRClient(
                [
                    snapshot(1),
                    snapshot(2),
                    snapshot(3, trigger=1.0),
                    snapshot(4),
                    snapshot(5, stick_y=-1.0),
                    snapshot(6, stick_y=1.0),
                    snapshot(7, trigger=1.0, stick_y=1.0),
                    snapshot(8, trigger=1.0, button_b=True),
                ]
            ),
            adapter=adapter,
            kinematics=FakeMarvinVendorKinematics(),
            scale_calibration_path=Path("unused.json"),
            requested_scale_factor=1.0,
            expected_sdk_version=1,
            control_parameter_settle_seconds=0.0,
            mode_settle_seconds=0.0,
            pd_settle_seconds=0.0,
            telemetry_publisher=telemetry,
            gripper_control_enabled=True,
            initial_gripper_closedness=(0.5, 0.5),
            gripper_mode="continuous",
            gripper_rate=1.0,
            gripper_command_hz=20.0,
        )
        controller.prepare_hardware()
        for cycle_time in (0.0, 0.1, 0.2, 0.3, 0.4, 0.5):
            controller.execute_control_cycle(cycle_time)
        self.assertAlmostEqual(controller.gripper_closedness[0], 0.58)
        self.assertAlmostEqual(controller.gripper_closedness[1], 0.5)
        self.assertEqual(
            adapter.gripper_commands[-1], controller.gripper_closedness
        )
        controller.execute_control_cycle(0.6)
        self.assertEqual(controller.gripper_closedness, (1.0, 1.0))
        self.assertEqual(adapter.gripper_commands[-1], (1.0, 1.0))
        controller.shutdown_hardware()
        event_names = [name for name, _payload in telemetry.events]
        self.assertEqual(event_names.count("pico"), 7)
        self.assertEqual(event_names.count("marvin"), 7)
        self.assertEqual(event_names.count("joint_command"), 8)
        self.assertIn("gripper_command", event_names)
        self.assertTrue(telemetry.closed)

        binary_adapter = FakeMarvinSdkAdapter()
        binary_controller = MarvinHardwareTeleopController(
            xr_client=FakeXRClient([]),
            adapter=binary_adapter,
            kinematics=FakeMarvinVendorKinematics(),
            scale_calibration_path=Path("unused.json"),
            requested_scale_factor=1.0,
            gripper_control_enabled=True,
            initial_gripper_closedness=(0.5, 0.5),
            gripper_rate=1.0,
        )
        binary_controller._update_gripper_command(snapshot(9), 0.0)
        binary_controller._update_gripper_command(snapshot(10, trigger=0.1), 0.1)
        self.assertAlmostEqual(binary_controller.gripper_closedness[0], 0.54)
        binary_controller._update_gripper_command(snapshot(11, stick_y=0.3), 0.2)
        self.assertAlmostEqual(binary_controller.gripper_closedness[0], 0.5)
        # Neutral keeps the binary endpoint selected; the output still ramps.
        for index in range(1, 14):
            binary_controller._update_gripper_command(
                snapshot(12), 0.2 + index * 0.1
            )
        self.assertEqual(binary_adapter.gripper_commands[-1], (0.0, 0.5))
        binary_controller._update_gripper_command(
            snapshot(13, stick_y=1.0), 1.6, reset_requested=True
        )
        np.testing.assert_allclose(binary_controller.gripper_closedness, (0.04, 0.54))
        for index in range(1, 25):
            binary_controller._update_gripper_command(
                snapshot(14), 1.6 + index * 0.1
            )
        self.assertEqual(binary_adapter.gripper_commands[-1], (1.0, 1.0))
        # Slow ramps and their final sub-deadband step must reach the adapter.
        binary_controller.gripper_rate = 0.1
        binary_controller._gripper_closedness[:] = (0.005, 1.0)
        binary_controller._last_sent_gripper_closedness[:] = (0.005, 1.0)
        binary_controller._update_gripper_command(snapshot(15, stick_y=1.0), 4.1)
        self.assertAlmostEqual(binary_adapter.gripper_commands[-1][0], 0.001)
        binary_controller._update_gripper_command(snapshot(16), 4.2)
        self.assertEqual(binary_adapter.gripper_commands[-1], (0.0, 1.0))


    def test_das_adapter_maps_closedness_and_initializes_from_encoder(self):
        self.assertAlmostEqual(
            _decode_encoder_value(struct.pack(">f", 0.05)), 0.05
        )
        self.assertEqual(_decode_encoder_value(struct.pack(">f", -0.0005)), 0.0)
        with self.assertRaisesRegex(ValueError, "invalid DAS encoder"):
            _decode_encoder_value(struct.pack(">f", -0.01))
        zero_closed = (
            DASFingerConfiguration(
                "/dev/left", "/dev/video-left", 0.0, 0.15
            ),
            DASFingerConfiguration(
                "/dev/right", "/dev/video-right", 0.0, 0.15
            ),
        )
        np.testing.assert_allclose(
            closedness_to_das_distances((0.0, 1.0), zero_closed),
            (0.15, 0.0),
        )
        configurations = (
            DASFingerConfiguration(
                "/dev/left",
                "/dev/video-left",
                0.01,
                0.07,
                startup_distance_m=0.045,
            ),
            DASFingerConfiguration(
                "/dev/right",
                "/dev/video-right",
                0.02,
                0.08,
                startup_distance_m=0.055,
                invert=True,
            ),
        )
        systems = []

        factory_arguments = []

        def factory(**kwargs):
            factory_arguments.append(kwargs)
            serial_port = kwargs["serial_port"]
            encoder_distance = 0.04 if serial_port == "/dev/left" else 0.05
            system = FakeDasFingerSystem(
                kwargs["encoder_callback"], encoder_distance
            )
            systems.append(system)
            return system

        published_states = []
        adapter = DASFingerAdapter(
            configurations,
            finger_system_factory=factory,
            command_hz=100.0,
            ready_timeout_seconds=0.5,
            state_callback=lambda arm, state: published_states.append((arm, state)),
        )
        adapter.connect()
        self.assertEqual(
            [item["initial_distance_m"] for item in factory_arguments],
            [0.045, 0.055],
        )
        np.testing.assert_allclose(
            adapter.get_initial_gripper_closedness(), (0.5, 0.5)
        )
        np.testing.assert_allclose(
            adapter.send_gripper_command((1.0, 0.0)), (0.01, 0.02)
        )
        deadline = time.monotonic() + 0.5
        while not all(
            system.targets
            and np.isclose(system.targets[-1], target)
            for system, target in zip(systems, (0.01, 0.02))
        ):
            if time.monotonic() >= deadline:
                self.fail("DAS command worker did not send a target")
            time.sleep(0.005)
        self.assertAlmostEqual(systems[0].targets[-1], 0.01)
        self.assertAlmostEqual(systems[1].targets[-1], 0.02)
        state = adapter.get_gripper_state()
        self.assertEqual(state["encoder_valid"], (True, True))
        self.assertTrue(all(state["encoder_monotonic_ns"]))
        self.assertEqual({arm for arm, _state in published_states}, {0, 1})
        self.assertTrue(all(item["valid"] for _arm, item in published_states))
        self.assertTrue(adapter.set_idle())
        adapter.release()


    def test_das_calibration_sentinel_recovers_or_names_stuck_side(self):
        configurations = tuple(
            DASFingerConfiguration(
                f"/dev/{side}",
                f"/dev/video-{side}",
                0.01,
                0.07,
                startup_distance_m=0.05,
            )
            for side in ("left", "right")
        )

        class CalibrationSystem(FakeDasFingerSystem):
            def __init__(self, callback, recover):
                super().__init__(callback, -66.66)
                self.recover = recover

            def set_finger_distance(self, distance):
                super().set_finger_distance(distance)
                if self.recover:
                    self.recover = False
                    self._encoder_callback(struct.pack(">f", 0.05))

        recovering = DASFingerAdapter(
            configurations,
            finger_system_factory=lambda **kwargs: CalibrationSystem(
                kwargs["encoder_callback"], True
            ),
            ready_timeout_seconds=0.2,
        )
        recovering.connect()
        recovering.release()

        def stuck_factory(**kwargs):
            recover = kwargs["serial_port"].endswith("right")
            return CalibrationSystem(kwargs["encoder_callback"], recover)

        stuck = DASFingerAdapter(
            configurations,
            finger_system_factory=stuck_factory,
            ready_timeout_seconds=0.05,
        )
        with self.assertRaisesRegex(TimeoutError, "left.*-66.66"):
            stuck.connect()


    def test_das_config_loader_requires_both_arms(self):
        with tempfile.TemporaryDirectory() as directory:
            config_path = Path(directory) / "das.json"
            config_path.write_text(
                json.dumps(
                    {
                        "left": {
                            "serial_port": "/dev/left",
                            "camera_device": "/dev/video-left",
                            "closed_distance_m": 0.01,
                            "open_distance_m": 0.07,
                        },
                        "right": {
                            "serial_port": "/dev/right",
                            "camera_device": "/dev/video-right",
                            "closed_distance_m": 0.01,
                            "open_distance_m": 0.07,
                        },
                    }
                ),
                encoding="utf-8",
            )
            configurations = load_das_finger_configurations(config_path)
            self.assertEqual(configurations[0].serial_port, "/dev/left")
            self.assertEqual(configurations[1].open_distance_m, 0.07)


    def test_marvin_adapter_delegates_gripper_lifecycle_to_das(self):
        class RecordingGripper:
            def __init__(self):
                self.events = []

            def connect(self):
                self.events.append("connect")

            def send_gripper_command(self, closedness):
                self.events.append(("command", tuple(closedness)))
                return "das-targets"

            def get_initial_gripper_closedness(self):
                return (0.25, 0.75)

            def get_gripper_state(self):
                return {"encoder_valid": (True, True)}

            def set_idle(self):
                self.events.append("idle")
                return True

            def release(self):
                self.events.append("release")

        gripper = RecordingGripper()
        robot = FakeMarvinRobot()
        adapter = MarvinSdkAdapter(
            marvin_robot=robot,
            dcss_structure=object(),
            gripper_adapter=gripper,
        )
        adapter.connect()
        self.assertEqual(adapter.get_initial_gripper_closedness(), (0.25, 0.75))
        self.assertEqual(adapter.get_gripper_state(), {"encoder_valid": (True, True)})
        self.assertEqual(adapter.send_gripper_command((0.2, 0.3)), "das-targets")
        adapter.set_idle()
        adapter.release()
        self.assertEqual(
            gripper.events,
            ["connect", ("command", (0.2, 0.3)), "idle", "release"],
        )


    def test_das_preflight_failure_does_not_connect_marvin(self):
        class FailingGripper:
            def send_gripper_command(self, _closedness):
                pass

            def connect(self):
                raise RuntimeError("encoder unavailable")

            def release(self):
                self.released = True

        gripper = FailingGripper()
        gripper.released = False
        robot = FakeMarvinRobot()
        adapter = MarvinSdkAdapter(
            marvin_robot=robot,
            dcss_structure=object(),
            gripper_adapter=gripper,
        )
        with self.assertRaisesRegex(RuntimeError, "encoder unavailable"):
            adapter.connect()
        self.assertEqual(robot.connect_calls, 0)
        self.assertTrue(gripper.released)


    def test_control_sdk_retries_feedback_during_connection_warmup(self):
        fake_marvin_robot = FakeMarvinRobot()
        fake_marvin_robot.invalid_feedback_reads = 2
        adapter = MarvinSdkAdapter(
            marvin_robot=fake_marvin_robot, dcss_structure=object()
        )
        adapter.connect()

        feedback = adapter.wait_for_fresh_feedback(
            timeout_seconds=0.2, required_updates=3
        )

        self.assertEqual(feedback.frame_serial, (3, 3))
        adapter.release()


    def test_control_sdk_does_not_release_before_connection(self):
        fake_marvin_robot = FakeMarvinRobot()
        adapter = MarvinSdkAdapter(
            marvin_robot=fake_marvin_robot, dcss_structure=object()
        )

        adapter.release()

        self.assertFalse(fake_marvin_robot.released)


if __name__ == "__main__":
    unittest.main()
