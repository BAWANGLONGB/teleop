"""Shared controller behavior, calibration, NSP, and dropout regression tests."""

import unittest

from tests.marvin_hardware_fakes import (
    ArmLengthScaleCalibrator,
    FakeMarvinRobot,
    FakeMarvinSdkAdapter,
    FakeMarvinVendorKinematics,
    FakeXRClient,
    make_openxr_pose,
    MARVIN_INITIAL_POSE_Q_RAD,
    MarvinHardwareTeleopController,
    MarvinRobotState,
    MarvinSdkAdapter,
    MarvinSessionLogger,
    MarvinVendorKinematics,
    np,
    Path,
    read_marvin_session,
    resolve_scale_factor,
    save_scale_calibration,
    tempfile,
    transform_controller_poses_to_marvin_frame,
    VendorIkResult,
    XrSnapshot,
    XrTargetMapper,
)


class TestMarvinController(unittest.TestCase):
    def test_impedance_mode_requires_joint_impedance_feedback(self):
        feedback = MarvinRobotState(
            frame_serial=(1, 1),
            q_rad=np.zeros(14),
            dq_rad_s=np.zeros(14),
            arm_state=(3, 3),
            impedance_type=(1, 2),
            error_code=(0, 0),
            low_speed=(True, True),
        )

        with self.assertRaisesRegex(RuntimeError, "imp_type=\\(1, 2\\)"):
            MarvinHardwareTeleopController._require_healthy_feedback(
                feedback, True
            )

    def test_joint_command_interpolation_and_hold_transitions(self):
        released = XrSnapshot(1, make_openxr_pose(), make_openxr_pose(),
                              (0.0, 0.0), False, False)
        active = XrSnapshot(2, make_openxr_pose(), make_openxr_pose(),
                            (1.0, 1.0), False, False)
        adapter = FakeMarvinSdkAdapter()
        kinematics = FakeMarvinVendorKinematics()
        target = np.zeros(14)
        fail = False

        def ik(arm, *_args):
            return VendorIkResult(not fail, None if fail else target[arm*7:arm*7+7].copy(), None)

        kinematics.ik_world = ik
        with tempfile.TemporaryDirectory() as directory:
            logger = MarvinSessionLogger(directory, "interpolation")
            controller = MarvinHardwareTeleopController(
                xr_client=FakeXRClient([released] + [active]*8 + [None, released, active, active]),
                adapter=adapter, kinematics=kinematics,
                scale_calibration_path=Path(directory)/"scale.json",
                requested_scale_factor=1.0, session_logger=logger,
                control_parameter_settle_seconds=0, mode_settle_seconds=0,
                pd_settle_seconds=0,
            )
            controller.prepare_hardware()
            try:
                controller.execute_control_cycle(0.0)
                target[:2] = np.deg2rad([6, 3])
                target[7] = np.deg2rad(1)
                for i in range(1, 4):
                    sent = controller.execute_control_cycle(i * 0.02)
                    np.testing.assert_allclose(np.rad2deg(sent[:2]), [2*i, i])
                    self.assertAlmostEqual(np.rad2deg(sent[7]), 1)
                target[0] = np.deg2rad(12)
                self.assertAlmostEqual(np.rad2deg(controller.execute_control_cycle(0.08)[0]), 8)
                # A new target replaces the old one immediately, without a queue.
                target[0] = np.deg2rad(7)
                self.assertAlmostEqual(np.rad2deg(controller.execute_control_cycle(0.10)[0]), 7)
                target[0] = np.deg2rad(20)
                self.assertAlmostEqual(np.rad2deg(controller.execute_control_cycle(0.11)[0]), 8)
                fail = True
                self.assertAlmostEqual(np.rad2deg(controller.execute_control_cycle(0.13)[0]), 8)
                fail = False
                # Stale input discards pending movement; release holds measured joints.
                self.assertAlmostEqual(np.rad2deg(controller.execute_control_cycle(0.15)[0]), 8)
                adapter.q_rad[0] = np.deg2rad(5)
                # Simulate a Grip release edge independently of stale recovery.
                controller._previous_grip_states = (True, True)
                self.assertAlmostEqual(np.rad2deg(controller.execute_control_cycle(0.17)[0]), 5)
                # A stalled loop can advance at most two nominal periods.
                self.assertAlmostEqual(np.rad2deg(controller.execute_control_cycle(1.17)[0]), 9)
                self.assertAlmostEqual(np.rad2deg(controller.execute_control_cycle(1.17)[0]), 9)
                with self.assertRaises(ValueError):
                    controller.execute_control_cycle(1.16)
            finally:
                controller.shutdown_hardware()
            records = read_marvin_session(logger.path)
            np.testing.assert_allclose(np.rad2deg(records[1]["q_desired_rad"][:2]), [6, 3])
            np.testing.assert_allclose(records[1]["joint_interpolation_alpha"], [1/3, 1])
        for speed in (0, -1, float("nan"), float("inf"), True):
            with self.assertRaises(ValueError):
                MarvinHardwareTeleopController(None, None, None, "unused.json",
                                               joint_command_max_speed_deg_s=speed)


    def test_scale_calibration_and_mapping(self):
        # Lock down all three physical axes independently of the implementation matrix.
        for pose, expected in (
            (make_openxr_pose(x_meters=1), [0, 1, 0]),
            (make_openxr_pose(y_meters=1), [0, 0, 1]),
            (make_openxr_pose(z_meters=-1), [-1, 0, 0]),
        ):
            with self.subTest(expected=expected):
                snapshot = XrSnapshot(1, pose, pose, (0.0, 0.0), False, False)
                for position, _rotation in transform_controller_poses_to_marvin_frame(snapshot):
                    np.testing.assert_allclose(position, expected)
        calibrator = ArmLengthScaleCalibrator()
        down = {"left": np.zeros(3), "right": np.zeros(3)}
        delta = np.array([0.0, 0.558866, 0.664989])
        self.assertEqual(
            calibrator.capture(down).status, "down_captured"
        )
        result = calibrator.capture({"left": delta, "right": delta})
        self.assertEqual(result.status, "completed")

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "scale.json"
            save_scale_calibration(path, result)
            self.assertAlmostEqual(
                resolve_scale_factor(None, path), result.scale_factor
            )

        xr_snapshot = XrSnapshot(
            1,
            make_openxr_pose(),
            make_openxr_pose(),
            (1.0, 0.0),
            False,
            False,
        )
        controller_poses = transform_controller_poses_to_marvin_frame(
            xr_snapshot
        )
        rotated_pose = make_openxr_pose()
        rotated_pose[[3, 6]] = np.sqrt(0.5)
        rotated_snapshot = XrSnapshot(
            1,
            rotated_pose,
            make_openxr_pose(),
            (0.0, 0.0),
            False,
            False,
        )
        np.testing.assert_allclose(
            transform_controller_poses_to_marvin_frame(rotated_snapshot)[0][1],
            [[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [-1.0, 0.0, 0.0]],
            atol=1e-12,
        )
        pose_mapper = XrTargetMapper(0.5)
        current_tcp_transform = np.eye(4)
        np.testing.assert_allclose(
            pose_mapper.map_arm(
                0, controller_poses[0], current_tcp_transform, True
            ),
            current_tcp_transform,
        )

        moved_snapshot = XrSnapshot(
            2,
            make_openxr_pose(x_meters=0.1),
            make_openxr_pose(),
            (1.0, 0.0),
            False,
            False,
        )
        controller_poses = transform_controller_poses_to_marvin_frame(
            moved_snapshot
        )
        target_tcp_transform = pose_mapper.map_arm(
            0, controller_poses[0], current_tcp_transform, True
        )
        np.testing.assert_allclose(
            target_tcp_transform[:3, 3], [0.0, 0.05, 0.0], atol=1e-12
        )

        pose_mapper.map_arm(
            0, controller_poses[0], current_tcp_transform, False
        )
        new_tcp_transform = np.eye(4)
        new_tcp_transform[:3, 3] = [1.0, 2.0, 3.0]
        regrip_snapshot = XrSnapshot(
            4,
            make_openxr_pose(x_meters=0.4),
            make_openxr_pose(),
            (1.0, 0.0),
            False,
            False,
        )
        regrip_poses = transform_controller_poses_to_marvin_frame(
            regrip_snapshot
        )
        np.testing.assert_allclose(
            pose_mapper.map_arm(
                0, regrip_poses[0], new_tcp_transform, True
            ),
            new_tcp_transform,
        )
        after_regrip_snapshot = XrSnapshot(
            5,
            make_openxr_pose(x_meters=0.5),
            make_openxr_pose(z_meters=-0.2),
            (1.0, 1.0),
            False,
            False,
        )
        after_regrip_poses = transform_controller_poses_to_marvin_frame(
            after_regrip_snapshot
        )
        after_regrip_target = pose_mapper.map_arm(
            0, after_regrip_poses[0], new_tcp_transform, True
        )
        np.testing.assert_allclose(
            after_regrip_target[:3, 3], [1.0, 2.05, 3.0], atol=1e-12
        )

        right_tcp_transform = np.eye(4)
        right_tcp_transform[:3, 3] = [4.0, 5.0, 6.0]
        np.testing.assert_allclose(
            pose_mapper.map_arm(
                1, controller_poses[1], right_tcp_transform, True
            ),
            right_tcp_transform,
        )
        right_target = pose_mapper.map_arm(
            1, after_regrip_poses[1], right_tcp_transform, True
        )
        np.testing.assert_allclose(
            right_target[:3, 3], [3.9, 5.0, 6.0], atol=1e-12
        )
        np.testing.assert_allclose(
            pose_mapper.map_arm(
                0, after_regrip_poses[0], new_tcp_transform, True
            )[:3, 3],
            after_regrip_target[:3, 3],
        )


    def test_optional_ik_nsp_is_initialized_and_angle_is_ramped(self):
        def snapshot(timestamp, grip_values):
            return XrSnapshot(
                timestamp,
                make_openxr_pose(),
                make_openxr_pose(),
                grip_values,
                False,
                False,
            )

        kinematics = FakeMarvinVendorKinematics()
        adapter = FakeMarvinSdkAdapter()
        controller = MarvinHardwareTeleopController(
            xr_client=FakeXRClient(
                [
                    snapshot(1, (0.0, 0.0)),
                    snapshot(2, (1.0, 1.0)),
                    snapshot(3, (1.0, 1.0)),
                    snapshot(4, (1.0, 1.0)),
                ]
            ),
            adapter=adapter,
            kinematics=kinematics,
            scale_calibration_path=Path("unused.json"),
            expected_sdk_version=1,
            control_parameter_settle_seconds=0.0,
            mode_settle_seconds=0.0,
            pd_settle_seconds=0.0,
            nsp_enabled=True,
            nsp_angles_deg=(5.0, -5.0),
            nsp_angle_rate_deg_s=20.0,
        )
        controller.prepare_hardware()
        self.assertEqual([arm for arm, _ in kinematics.nsp_reference_calls], [0, 1])
        controller.execute_control_cycle(0.0)
        controller.execute_control_cycle(0.1)
        controller.execute_control_cycle(0.2)
        self.assertEqual(kinematics.nsp_angles_deg[0], 0.0)
        self.assertAlmostEqual(kinematics.nsp_angles_deg[2], 0.8)
        self.assertAlmostEqual(kinematics.nsp_angles_deg[4], 1.6)
        controller.shutdown_hardware()


    def test_lateral_nsp_uses_marvin_x_from_openxr_z(self):
        def snapshot(timestamp, left_z, right_z, grip_values):
            return XrSnapshot(
                timestamp,
                make_openxr_pose(z_meters=left_z),
                make_openxr_pose(z_meters=right_z),
                grip_values,
                False,
                False,
                (0.0, 0.0),
                (0.0, 0.0),
            )

        kinematics = FakeMarvinVendorKinematics()
        controller = MarvinHardwareTeleopController(
            xr_client=FakeXRClient(
                [
                    snapshot(1, 0.0, 0.0, (0.0, 0.0)),
                    snapshot(2, 0.0, 0.0, (1.0, 1.0)),
                    snapshot(3, -0.12, 0.12, (1.0, 1.0)),
                ]
            ),
            adapter=FakeMarvinSdkAdapter(),
            kinematics=kinematics,
            scale_calibration_path=Path("unused.json"),
            expected_sdk_version=1,
            control_parameter_settle_seconds=0.0,
            mode_settle_seconds=0.0,
            pd_settle_seconds=0.0,
            nsp_lateral_enabled=True,
            nsp_angle_rate_deg_s=20.0,
        )
        controller.prepare_hardware()
        controller.execute_control_cycle(0.0)
        controller.execute_control_cycle(0.02)
        self.assertAlmostEqual(kinematics.nsp_angles_deg[-2], -0.4)
        self.assertAlmostEqual(kinematics.nsp_angles_deg[-1], 0.4)
        controller.shutdown_hardware()


    def test_offline_xr_to_marvin_ik_targets_match_input_without_jitter(self):
        sdk_root = (
            Path(__file__).resolve().parents[2]
            / "TJArm"
            / "tj_fx_robot-master"
        )
        if not (sdk_root / "SDK_PYTHON" / "libKine.so").is_file():
            self.skipTest("Marvin kinematics SDK is not installed")

        kinematics = MarvinVendorKinematics(sdk_root)
        for arm_index in (0, 1):
            kinematics.set_tool(arm_index, [0.0] * 6)
        invalid_reference_result = kinematics.ik_world(
            0, np.eye(4), np.zeros(7)
        )
        self.assertFalse(invalid_reference_result.success)
        self.assertEqual(
            invalid_reference_result.failure_reason,
            "reference joints must not all be zero",
        )
        unsafe_j4_q_rad = MARVIN_INITIAL_POSE_Q_RAD[:7].copy()
        unsafe_j4_q_rad[3] = np.deg2rad(-4.0)
        unsafe_j4_result = kinematics.ik_world(
            0,
            kinematics.fk_world(0, unsafe_j4_q_rad),
            unsafe_j4_q_rad,
        )
        self.assertFalse(unsafe_j4_result.success)
        self.assertIn("joint 4 exceeds -5 degree", unsafe_j4_result.failure_reason)
        safe_reference_q_rad = MARVIN_INITIAL_POSE_Q_RAD[:7].copy()
        kinematics.set_nsp_reference(0, safe_reference_q_rad)
        nsp_result = kinematics.ik_world(
            0,
            kinematics.fk_world(0, safe_reference_q_rad),
            safe_reference_q_rad,
            nsp_angle_deg=3.0,
        )
        self.assertTrue(nsp_result.success)
        self.assertAlmostEqual(
            nsp_result.q_rad[3], safe_reference_q_rad[3], places=8
        )

        released_snapshot = XrSnapshot(
            1,
            make_openxr_pose(x_meters=-0.2),
            make_openxr_pose(x_meters=0.2),
            (0.0, 0.0),
            False,
            False,
        )
        active_anchor_snapshot = XrSnapshot(
            2,
            make_openxr_pose(x_meters=-0.2),
            make_openxr_pose(x_meters=0.2),
            (1.0, 1.0),
            False,
            False,
        )
        active_moved_snapshot = XrSnapshot(
            3,
            make_openxr_pose(x_meters=-0.16),
            make_openxr_pose(x_meters=0.24),
            (1.0, 1.0),
            False,
            False,
        )
        repeated_frame_count = 10
        xr_client = FakeXRClient(
            [released_snapshot, active_anchor_snapshot]
            + [active_moved_snapshot] * repeated_frame_count
        )
        capture_adapter = FakeMarvinSdkAdapter()
        capture_adapter.q_rad = MARVIN_INITIAL_POSE_Q_RAD.copy()

        with tempfile.TemporaryDirectory() as directory:
            session_logger = MarvinSessionLogger(directory, "offline_test")
            controller = MarvinHardwareTeleopController(
                xr_client=xr_client,
                adapter=capture_adapter,
                kinematics=kinematics,
                scale_calibration_path=Path(directory) / "scale.json",
                requested_scale_factor=0.5,
                expected_sdk_version=1,
                control_parameter_settle_seconds=0.0,
                mode_settle_seconds=0.0,
                pd_settle_seconds=0.0,
                session_logger=session_logger,
            )
            controller.prepare_hardware()
            controller.execute_control_cycle(0.0)
            repeated_targets_rad = np.asarray(
                [
                    controller.execute_control_cycle((index + 1) * 0.02)
                    for index in range(repeated_frame_count)
                ]
            )
            controller.shutdown_hardware()
            session_records = read_marvin_session(session_logger.path)

        self.assertEqual(len(session_records), repeated_frame_count + 1)
        np.testing.assert_allclose(
            session_records[-1]["q_command_rad"], repeated_targets_rad[-1]
        )

        expected_tcp_delta_m = np.array([0.0, 0.02, 0.0])
        maximum_position_error_mm = 0.0
        maximum_rotation_error_deg = 0.0
        for arm_index in (0, 1):
            arm_slice = slice(arm_index * 7, (arm_index + 1) * 7)
            initial_tcp = kinematics.fk_world(
                arm_index, MARVIN_INITIAL_POSE_Q_RAD[arm_slice]
            )
            commanded_tcp = kinematics.fk_world(
                arm_index, repeated_targets_rad[-1, arm_slice]
            )
            maximum_position_error_mm = max(
                maximum_position_error_mm,
                np.linalg.norm(
                    commanded_tcp[:3, 3]
                    - initial_tcp[:3, 3]
                    - expected_tcp_delta_m
                )
                * 1e3,
            )
            rotation_error = commanded_tcp[:3, :3] @ initial_tcp[:3, :3].T
            maximum_rotation_error_deg = max(
                maximum_rotation_error_deg,
                np.rad2deg(
                    np.arccos(
                        np.clip((np.trace(rotation_error) - 1.0) / 2.0, -1.0, 1.0)
                    )
                ),
            )

        repeated_targets_deg = np.rad2deg(repeated_targets_rad)
        maximum_joint_peak_to_peak_deg = np.ptp(
            repeated_targets_deg[-5:], axis=0
        ).max()
        # The initial step now ramps; a stationary target must settle without jitter.
        self.assertLessEqual(np.max(np.abs(np.diff(repeated_targets_deg, axis=0))), 2.0 + 1e-9)
        self.assertTrue(np.all(np.isfinite(repeated_targets_deg)))
        self.assertLessEqual(maximum_position_error_mm, 0.1)
        self.assertLessEqual(maximum_rotation_error_deg, 0.01)
        self.assertLessEqual(maximum_joint_peak_to_peak_deg, 0.001)

        fake_marvin_robot = FakeMarvinRobot()
        send_adapter = MarvinSdkAdapter(
            marvin_robot=fake_marvin_robot, dcss_structure=object()
        )
        send_adapter.connect()
        send_adapter.send_joint_command(repeated_targets_rad[-1])
        sent_targets_deg = np.concatenate(
            (
                fake_marvin_robot.q_commands_deg["A"],
                fake_marvin_robot.q_commands_deg["B"],
            )
        )
        np.testing.assert_allclose(sent_targets_deg, repeated_targets_deg[-1])
        send_adapter.release()


    def test_release_holds_and_b_resets_robot_without_resetting_calibration(self):
        released_snapshot = XrSnapshot(
            1,
            make_openxr_pose(),
            make_openxr_pose(),
            (0.0, 0.0),
            False,
            False,
        )
        calibration_down_snapshot = XrSnapshot(
            2,
            make_openxr_pose(),
            make_openxr_pose(),
            (0.0, 0.0),
            True,
            False,
        )
        active_anchor_snapshot = XrSnapshot(
            3,
            make_openxr_pose(),
            make_openxr_pose(),
            (1.0, 0.0),
            False,
            False,
        )
        active_moved_snapshot = XrSnapshot(
            4,
            make_openxr_pose(z_meters=0.1),
            make_openxr_pose(),
            (1.0, 0.0),
            False,
            False,
        )
        active_unreachable_snapshot = XrSnapshot(
            5,
            make_openxr_pose(z_meters=0.2),
            make_openxr_pose(),
            (1.0, 0.0),
            False,
            False,
        )
        regrip_anchor_snapshot = XrSnapshot(
            6,
            make_openxr_pose(z_meters=0.4),
            make_openxr_pose(),
            (1.0, 0.0),
            False,
            False,
        )
        regrip_moved_snapshot = XrSnapshot(
            7,
            make_openxr_pose(z_meters=0.5),
            make_openxr_pose(),
            (1.0, 0.0),
            False,
            False,
        )
        reset_snapshot = XrSnapshot(
            8,
            make_openxr_pose(),
            make_openxr_pose(),
            (0.0, 0.0),
            False,
            True,
        )
        calibration_forward_snapshot = XrSnapshot(
            9,
            make_openxr_pose(
                y_meters=0.664989,
                x_meters=0.558866,
            ),
            make_openxr_pose(
                y_meters=0.664989,
                x_meters=0.558866,
            ),
            (0.0, 0.0),
            True,
            False,
        )
        xr_client = FakeXRClient(
            [
                released_snapshot,
                calibration_down_snapshot,
                released_snapshot,
                active_anchor_snapshot,
                active_moved_snapshot,
                active_unreachable_snapshot,
                released_snapshot,
                regrip_anchor_snapshot,
                regrip_moved_snapshot,
                released_snapshot,
                reset_snapshot,
                reset_snapshot,
                released_snapshot,
                calibration_forward_snapshot,
            ]
        )
        adapter = FakeMarvinSdkAdapter()
        kinematics = FakeMarvinVendorKinematics()
        with tempfile.TemporaryDirectory() as directory:
            controller = MarvinHardwareTeleopController(
                xr_client=xr_client,
                adapter=adapter,
                kinematics=kinematics,
                scale_calibration_path=Path(directory) / "scale.json",
                requested_scale_factor=0.5,
                return_duration=3.0,
                expected_sdk_version=1,
                control_parameter_settle_seconds=0.0,
                mode_settle_seconds=0.0,
                pd_settle_seconds=0.0,
            )
            controller.prepare_hardware()
            self.assertIsNotNone(adapter.configured_parameters)
            self.assertEqual(
                adapter.configured_named_parameters,
                {
                    "joint_velocity_ratio": 10,
                    "joint_acceleration_ratio": 10,
                },
            )
            self.assertEqual(
                adapter.events[:4],
                [
                    "configure_control_parameters",
                    "enter_joint_impedance",
                    "enable_pd_feedforward",
                    "send_joint_command",
                ],
            )
            self.assertEqual(adapter.pd_period_milliseconds, 20)
            self.assertEqual(adapter.joint_command_wait_responses, [False])
            startup_hold_q_rad = controller.execute_control_cycle(0.0)
            np.testing.assert_allclose(startup_hold_q_rad, 0.0)
            controller.execute_control_cycle(0.1)
            controller.execute_control_cycle(1.0)
            moved_q_rad = controller.execute_control_cycle(1.5)
            self.assertAlmostEqual(moved_q_rad[0], 0.05)
            kinematics.fail_inverse_kinematics = True
            failed_ik_q_rad = controller.execute_control_cycle(1.75)
            self.assertAlmostEqual(failed_ik_q_rad[0], 0.05)
            kinematics.fail_inverse_kinematics = False
            adapter.q_rad[0] = 0.04
            released_q_rad = controller.execute_control_cycle(2.0)
            self.assertAlmostEqual(released_q_rad[0], 0.04)
            regripped_q_rad = controller.execute_control_cycle(2.5)
            self.assertAlmostEqual(regripped_q_rad[0], 0.04)
            regrip_moved_q_rad = controller.execute_control_cycle(3.0)
            self.assertAlmostEqual(regrip_moved_q_rad[0], 0.09)
            held_q_rad = controller.execute_control_cycle(3.5)
            self.assertAlmostEqual(held_q_rad[0], 0.09)

            reset_start_q_rad = controller.execute_control_cycle(4.0)
            np.testing.assert_allclose(reset_start_q_rad, held_q_rad)
            reset_mid_q_rad = controller.execute_control_cycle(5.5)
            np.testing.assert_allclose(
                reset_mid_q_rad,
                held_q_rad + 0.5 * (MARVIN_INITIAL_POSE_Q_RAD - held_q_rad),
            )
            returned_q_rad = controller.execute_control_cycle(7.0)
            np.testing.assert_allclose(returned_q_rad, MARVIN_INITIAL_POSE_Q_RAD)
            controller.execute_control_cycle(7.1)
            self.assertAlmostEqual(controller.scale_factor, 0.95)
            controller.shutdown_hardware()

        self.assertTrue(adapter.idle)
        self.assertTrue(adapter.released)
        self.assertTrue(xr_client.closed)


    def test_xr_dropout_holds_target_and_reanchors_grip(self):
        released_snapshot = XrSnapshot(
            1,
            make_openxr_pose(),
            make_openxr_pose(),
            (0.0, 0.0),
            False,
            False,
        )
        active_anchor_snapshot = XrSnapshot(
            2,
            make_openxr_pose(),
            make_openxr_pose(),
            (1.0, 0.0),
            False,
            False,
        )
        active_moved_snapshot = XrSnapshot(
            3,
            make_openxr_pose(z_meters=0.1),
            make_openxr_pose(),
            (1.0, 0.0),
            False,
            False,
        )
        recovered_anchor_snapshot = XrSnapshot(
            4,
            make_openxr_pose(z_meters=0.5),
            make_openxr_pose(),
            (1.0, 0.0),
            False,
            False,
        )
        recovered_moved_snapshot = XrSnapshot(
            5,
            make_openxr_pose(z_meters=0.6),
            make_openxr_pose(),
            (1.0, 0.0),
            False,
            False,
        )
        controller = MarvinHardwareTeleopController(
            xr_client=FakeXRClient(
                [
                    released_snapshot,
                    active_anchor_snapshot,
                    active_moved_snapshot,
                    None,
                    recovered_anchor_snapshot,
                    recovered_moved_snapshot,
                ]
            ),
            adapter=FakeMarvinSdkAdapter(),
            kinematics=FakeMarvinVendorKinematics(),
            scale_calibration_path=Path("unused.json"),
            requested_scale_factor=0.5,
            expected_sdk_version=1,
            control_parameter_settle_seconds=0.0,
            mode_settle_seconds=0.0,
            pd_settle_seconds=0.0,
        )
        controller.prepare_hardware()

        controller.execute_control_cycle(0.0)
        moved_q_rad = controller.execute_control_cycle(0.1)
        held_q_rad = controller.execute_control_cycle(0.2)
        recovered_q_rad = controller.execute_control_cycle(0.3)
        resumed_q_rad = controller.execute_control_cycle(0.4)

        self.assertAlmostEqual(moved_q_rad[0], 0.05)
        np.testing.assert_allclose(held_q_rad, moved_q_rad)
        np.testing.assert_allclose(recovered_q_rad, moved_q_rad)
        self.assertAlmostEqual(resumed_q_rad[0], 0.1)
        controller.shutdown_hardware()


    def test_xr_dropout_cancels_return_until_new_button_edge(self):
        pose = make_openxr_pose()
        released = XrSnapshot(1, pose, pose, (0.0, 0.0), False, False)
        reset = XrSnapshot(2, pose, pose, (0.0, 0.0), False, True)
        controller = MarvinHardwareTeleopController(
            FakeXRClient([released, reset, released, None, reset, released, reset, released]),
            FakeMarvinSdkAdapter(), FakeMarvinVendorKinematics(), Path("unused.json"),
            requested_scale_factor=1.0, control_parameter_settle_seconds=0,
            mode_settle_seconds=0, pd_settle_seconds=0,
        )
        controller.prepare_hardware()
        try:
            controller.execute_control_cycle(0.0)
            before = controller.execute_control_cycle(0.1)
            self.assertGreater(np.max(np.abs(before)), 0)
            for timestamp in (1.08, 1.10, 1.12):
                np.testing.assert_allclose(controller.execute_control_cycle(timestamp), before)
            np.testing.assert_allclose(controller.execute_control_cycle(1.14), before)
            self.assertGreater(np.max(np.abs(controller.execute_control_cycle(1.24) - before)), 0)
        finally:
            controller.shutdown_hardware()


    def test_session_log_marks_dropped_xr_frame(self):
        adapter = FakeMarvinSdkAdapter()
        with tempfile.TemporaryDirectory() as directory:
            logger = MarvinSessionLogger(directory, "dropped_xr")
            logger.record_control_cycle(
                None,
                adapter.read_state(),
                np.zeros(14),
                1.0,
                gripper_state={
                    "distance_m": (0.04, 0.05),
                    "target_distance_m": (0.045, 0.055),
                    "encoder_monotonic_ns": (101, 102),
                    "encoder_wall_time_ns": (201, 202),
                    "encoder_valid": (True, True),
                },
                sample_id=7,
                sample_monotonic_ns=100,
                wall_time_ns=200,
            )
            logger.close()
            record = read_marvin_session(logger.path)[0]

        self.assertFalse(record["xr_frame_valid"])
        self.assertIsNone(record["xr_timestamp_ns"])
        self.assertIsNone(record["left_controller_pose"])
        self.assertEqual(record["schema_version"], 2)
        self.assertEqual(record["sample_id"], 7)
        self.assertEqual(record["monotonic_time_ns"], 100)
        self.assertEqual(record["wall_time_ns"], 200)
        self.assertEqual(record["gripper_feedback_distance_m"], [0.04, 0.05])


    def test_teleoperation_rejects_stale_robot_feedback(self):
        snapshot = XrSnapshot(
            1,
            make_openxr_pose(),
            make_openxr_pose(),
            (0.0, 0.0),
            False,
            False,
        )
        xr_client = FakeXRClient([snapshot] * 4)
        adapter = FakeMarvinSdkAdapter()
        controller = MarvinHardwareTeleopController(
            xr_client=xr_client,
            adapter=adapter,
            kinematics=FakeMarvinVendorKinematics(),
            scale_calibration_path=Path("unused.json"),
            requested_scale_factor=1.0,
            expected_sdk_version=1,
            control_parameter_settle_seconds=0.0,
            mode_settle_seconds=0.0,
            pd_settle_seconds=0.0,
        )
        controller.prepare_hardware()
        adapter._feedback = lambda: MarvinRobotState(
            frame_serial=(adapter.frame_serial, adapter.frame_serial),
            q_rad=adapter.q_rad,
            dq_rad_s=np.zeros(14),
            arm_state=adapter.arm_state,
            impedance_type=adapter.impedance_type,
            error_code=(0, 0),
            low_speed=(True, True),
        )

        controller.execute_control_cycle()
        controller.execute_control_cycle()
        with self.assertRaises(TimeoutError):
            controller.execute_control_cycle()
        controller.shutdown_hardware()


if __name__ == "__main__":
    unittest.main()
