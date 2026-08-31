import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np

from xr_marvin_teleop.common.marvin_scale_calibration import (
    ArmLengthScaleCalibrator,
    resolve_scale_factor,
    save_scale_calibration,
)
from xr_marvin_teleop.common.marvin_postures import MARVIN_INITIAL_POSE_Q_RAD
from xr_marvin_teleop.common.marvin_session_logger import (
    MarvinSessionLogger,
    read_marvin_session,
)
from xr_marvin_teleop.common.xr_client import XrClient, XrSnapshot
from xr_marvin_teleop.common.xr_target_mapper import (
    XrTargetMapper,
    transform_controller_poses_to_marvin_frame,
)
from xr_marvin_teleop.hardware.interface.marvin import (
    MarvinRobotState,
    MarvinSdkAdapter,
    MarvinToolConfiguration,
)
from xr_marvin_teleop.hardware.interface.marvin_kinematics import (
    MarvinVendorKinematics,
    VendorIkResult,
)
from xr_marvin_teleop.hardware.marvin_teleop_controller import (
    MarvinHardwareTeleopController,
)
from xr_marvin_teleop.simulation.marvin_mujoco_adapter import (
    MarvinMujocoAdapter,
)


def make_openxr_pose(x_meters=0.0, y_meters=0.0, z_meters=0.0):
    return np.array(
        [x_meters, y_meters, z_meters, 0.0, 0.0, 0.0, 1.0]
    )


class FakeXrSdk:
    def __init__(self, timestamps):
        self.timestamps = list(timestamps)
        self.initialized = False
        self.closed = False

    def init(self):
        self.initialized = True

    def get_snapshot(self):
        if len(self.timestamps) > 1:
            timestamp_ns = self.timestamps.pop(0)
        else:
            timestamp_ns = self.timestamps[0]
        return {
            "timestamp_ns": timestamp_ns,
            "left_controller_pose": make_openxr_pose(-0.1),
            "right_controller_pose": make_openxr_pose(0.1),
            "grip_values": (0.0, 0.0),
            "button_a": False,
            "button_b": False,
        }

    def close(self):
        self.closed = True


class FakeMarvinRobot:
    def __init__(self):
        self.frame_serial = 0
        self.invalid_feedback_reads = 0
        self.q_commands_deg = {}
        self.joint_impedance = {}
        self.joint_motion_limits = {}
        self.tools = {}
        self.wait_response_calls = 0
        self.released = False

    def connect(self, _robot_ip_address):
        return True

    def subscribe(self, _dcss_structure):
        if self.invalid_feedback_reads > 0:
            self.invalid_feedback_reads -= 1
            return None
        self.frame_serial += 1
        return {
            "outputs": [
                {
                    "fb_joint_pos": [0.0] * 7,
                    "fb_joint_vel": [0.0] * 7,
                    "frame_serial": self.frame_serial,
                    "low_speed_flag": b"\x01",
                },
                {
                    "fb_joint_pos": [0.0] * 7,
                    "fb_joint_vel": [0.0] * 7,
                    "frame_serial": self.frame_serial,
                    "low_speed_flag": b"\x01",
                },
            ],
            "states": [
                {"cur_state": 3, "err_code": 0},
                {"cur_state": 3, "err_code": 0},
            ],
        }

    def clear_set(self):
        return True

    def set_joint_cmd_pose(self, arm, joints):
        self.q_commands_deg[arm] = joints
        return True

    def set_joint_kd_params(self, arm, K, D):
        self.joint_impedance[arm] = (K, D)
        return True

    def set_vel_acc(self, arm, velRatio, AccRatio):
        self.joint_motion_limits[arm] = (velRatio, AccRatio)
        return True

    def set_tool(self, arm, kineParams, dynamicParams):
        self.tools[arm] = (kineParams, dynamicParams)
        return True

    def send_cmd(self):
        return True

    def send_cmd_wait_response(self, _timeout_milliseconds):
        self.wait_response_calls += 1
        return 1

    def release_robot(self):
        self.released = True


class FakeXRClient:
    def __init__(self, snapshots):
        self._snapshots = iter(snapshots)
        self.closed = False

    def read_snapshot(self):
        return next(self._snapshots)

    def close(self):
        self.closed = True


class FakeMarvinSdkAdapter:
    def __init__(self):
        self.q_rad = np.zeros(14)
        self.arm_state = (0, 0)
        self.frame_serial = 0
        self.sent_commands_rad = []
        self.events = []
        self.configured_parameters = None
        self.configured_named_parameters = None
        self.pd_period_milliseconds = None
        self.released = False
        self.idle = False

    def connect(self):
        pass

    def sdk_version(self):
        return 1

    def _feedback(self):
        self.frame_serial += 1
        return MarvinRobotState(
            frame_serial=(self.frame_serial, self.frame_serial),
            q_rad=self.q_rad,
            dq_rad_s=np.zeros(14),
            arm_state=self.arm_state,
            error_code=(0, 0),
            low_speed=(True, True),
        )

    def wait_for_fresh_feedback(self, **_kwargs):
        return self._feedback()

    def read_state(self):
        return self._feedback()

    def send_joint_command(self, q_rad, wait_response=False):
        del wait_response
        self.events.append("send_joint_command")
        self.q_rad = np.asarray(q_rad).copy()
        self.sent_commands_rad.append(self.q_rad.copy())

    def configure_control_parameters(self, *parameters, **named_parameters):
        self.events.append("configure_control_parameters")
        self.configured_parameters = parameters
        self.configured_named_parameters = named_parameters

    def enter_joint_impedance(self):
        self.events.append("enter_joint_impedance")
        self.arm_state = (3, 3)

    def enable_pd_feedforward(self, _period_milliseconds):
        self.events.append("enable_pd_feedforward")
        self.pd_period_milliseconds = _period_milliseconds

    def set_idle(self):
        self.idle = True
        self.arm_state = (0, 0)
        return True

    def release(self):
        self.released = True


class FakeMarvinVendorKinematics:
    def __init__(self):
        self.fail_inverse_kinematics = False

    def fk_world(self, _arm, q_rad):
        tcp_transform = np.eye(4)
        tcp_transform[1, 3] = q_rad[0]
        return tcp_transform

    def ik_world(
        self,
        _arm,
        T_world_tcp_m,
        q_ref_rad,
    ):
        if self.fail_inverse_kinematics:
            return VendorIkResult(False, None, "singular or out of range")
        q_rad = np.asarray(q_ref_rad).copy()
        q_rad[0] = T_world_tcp_m[1, 3]
        return VendorIkResult(True, q_rad, None)


class TestMarvinHardware(unittest.TestCase):
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

    def test_scale_calibration_and_mapping(self):
        calibrator = ArmLengthScaleCalibrator()
        down = {"left": np.zeros(3), "right": np.zeros(3)}
        delta = np.array([-0.558866, 0.0, 0.664989])
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
        self.assertEqual(fake_marvin_robot.wait_response_calls, 1)
        adapter.send_joint_command(np.deg2rad(q_deg))
        np.testing.assert_allclose(
            fake_marvin_robot.q_commands_deg["A"], q_deg[:7]
        )
        np.testing.assert_allclose(
            fake_marvin_robot.q_commands_deg["B"], q_deg[7:]
        )
        adapter.release()
        self.assertTrue(fake_marvin_robot.released)

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
            repeated_targets_deg, axis=0
        ).max()
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

        print(
            "Offline XR->IK test: "
            f"position_error={maximum_position_error_mm:.6f} mm, "
            f"rotation_error={maximum_rotation_error_deg:.6f} deg, "
            f"joint_peak_to_peak={maximum_joint_peak_to_peak_deg:.9f} deg, "
            f"targets_A_deg={np.round(sent_targets_deg[:7], 6).tolist()}, "
            f"targets_B_deg={np.round(sent_targets_deg[7:], 6).tolist()}"
        )

    def test_headless_mujoco_adapter_accepts_marvin_joint_targets(self):
        xml_path = (
            Path(__file__).resolve().parents[1]
            / "assets"
            / "marvin"
            / "marvin_dual.mujoco.xml"
        )
        adapter = MarvinMujocoAdapter(
            xml_path,
            MARVIN_INITIAL_POSE_Q_RAD,
            launch_viewer=False,
        )
        adapter.connect()
        np.testing.assert_allclose(
            adapter.read_state().q_rad,
            MARVIN_INITIAL_POSE_Q_RAD,
            atol=1e-12,
        )

        target_q_rad = MARVIN_INITIAL_POSE_Q_RAD.copy()
        target_q_rad[0] += 0.01
        adapter.send_joint_command(target_q_rad)
        self.assertTrue(np.all(np.isfinite(adapter.read_state().q_rad)))
        adapter.set_joint_state(target_q_rad)
        np.testing.assert_allclose(
            adapter.read_state().q_rad, target_q_rad, atol=1e-12
        )
        adapter.release()

    def test_headless_mujoco_runs_xr_to_vendor_ik_control_cycle(self):
        project_root = Path(__file__).resolve().parents[1]
        sdk_root = project_root.parent / "TJArm" / "tj_fx_robot-master"
        kinematics = MarvinVendorKinematics(sdk_root)
        for arm_index in (0, 1):
            kinematics.set_tool(arm_index, [0.0] * 6)
        xr_client = FakeXRClient(
            [
                XrSnapshot(
                    1,
                    make_openxr_pose(x_meters=-0.2),
                    make_openxr_pose(x_meters=0.2),
                    (0.0, 0.0),
                    False,
                    False,
                ),
                XrSnapshot(
                    2,
                    make_openxr_pose(x_meters=-0.2),
                    make_openxr_pose(x_meters=0.2),
                    (1.0, 1.0),
                    False,
                    False,
                ),
                XrSnapshot(
                    3,
                    make_openxr_pose(x_meters=-0.16),
                    make_openxr_pose(x_meters=0.24),
                    (1.0, 1.0),
                    False,
                    False,
                ),
            ]
        )
        adapter = MarvinMujocoAdapter(
            project_root / "assets" / "marvin" / "marvin_dual.mujoco.xml",
            MARVIN_INITIAL_POSE_Q_RAD,
            launch_viewer=False,
        )
        with tempfile.TemporaryDirectory() as directory:
            controller = MarvinHardwareTeleopController(
                xr_client=xr_client,
                adapter=adapter,
                kinematics=kinematics,
                scale_calibration_path=Path(directory) / "scale.json",
                requested_scale_factor=0.5,
                control_parameter_settle_seconds=0.0,
                mode_settle_seconds=0.0,
                pd_settle_seconds=0.0,
            )
            controller.prepare_hardware()
            controller.execute_control_cycle(0.0)
            q_command_rad = controller.execute_control_cycle(0.02)
            np.testing.assert_allclose(adapter.data.ctrl, q_command_rad)
            controller.shutdown_hardware()

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
            make_openxr_pose(x_meters=0.1),
            make_openxr_pose(),
            (1.0, 0.0),
            False,
            False,
        )
        active_unreachable_snapshot = XrSnapshot(
            5,
            make_openxr_pose(x_meters=0.2),
            make_openxr_pose(),
            (1.0, 0.0),
            False,
            False,
        )
        regrip_anchor_snapshot = XrSnapshot(
            6,
            make_openxr_pose(x_meters=0.4),
            make_openxr_pose(),
            (1.0, 0.0),
            False,
            False,
        )
        regrip_moved_snapshot = XrSnapshot(
            7,
            make_openxr_pose(x_meters=0.5),
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
                z_meters=-0.558866,
            ),
            make_openxr_pose(
                y_meters=0.664989,
                z_meters=-0.558866,
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
            self.assertEqual(adapter.pd_period_milliseconds, 5)
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
            make_openxr_pose(x_meters=0.1),
            make_openxr_pose(),
            (1.0, 0.0),
            False,
            False,
        )
        recovered_anchor_snapshot = XrSnapshot(
            4,
            make_openxr_pose(x_meters=0.5),
            make_openxr_pose(),
            (1.0, 0.0),
            False,
            False,
        )
        recovered_moved_snapshot = XrSnapshot(
            5,
            make_openxr_pose(x_meters=0.6),
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

    def test_session_log_marks_dropped_xr_frame(self):
        adapter = FakeMarvinSdkAdapter()
        with tempfile.TemporaryDirectory() as directory:
            logger = MarvinSessionLogger(directory, "dropped_xr")
            logger.record_control_cycle(
                None,
                adapter.read_state(),
                np.zeros(14),
                1.0,
            )
            logger.close()
            record = read_marvin_session(logger.path)[0]

        self.assertFalse(record["xr_frame_valid"])
        self.assertIsNone(record["xr_timestamp_ns"])
        self.assertIsNone(record["left_controller_pose"])

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
