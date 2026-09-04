import json
import queue
import runpy
import struct
import threading
import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from xr_marvin_teleop.common import episode_validator
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
    MarvinModbusGripperConfiguration,
    MarvinRobotState,
    MarvinSdkAdapter,
    MarvinToolConfiguration,
    _modbus_write_single_register_frame,
)
from xr_marvin_teleop.hardware.interface.das_finger import (
    DASFingerAdapter,
    DASFingerConfiguration,
    _decode_encoder_value,
    closedness_to_das_distances,
    load_das_finger_configurations,
)
from xr_marvin_teleop.hardware.interface.marvin_kinematics import (
    MarvinVendorKinematics,
    VendorIkResult,
)
from xr_marvin_teleop.hardware.marvin_teleop_controller import (
    MarvinHardwareTeleopController,
)
from xr_marvin_teleop.ros.das_client import RosDasClient
from xr_marvin_teleop.ros.pico_client import RosPicoClient
from xr_marvin_teleop.ros.telemetry_bridge import Ros2DataBridge
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
            "trigger_values": (0.0, 0.0),
            "thumbstick_y_values": (0.0, 0.0),
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
        self.channel_frames = []
        self.cleared_channels = []
        self.released = False
        self.connect_calls = 0

    def connect(self, _robot_ip_address):
        self.connect_calls += 1
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

    def clear_ch_data(self, arm):
        self.cleared_channels.append(arm)
        return True

    def set_ch_data(self, arm, data, size_int, channel):
        self.channel_frames.append((arm, bytes(data), channel))
        return size_int

    def set_state(self, arm, state):
        return True

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


class RecordingTelemetry:
    def __init__(self):
        self.events = []
        self.closed = False

    def publish_pico(self, snapshot, **_timestamps):
        self.events.append(("pico", snapshot.timestamp_ns))

    def publish_marvin_state(self, state, **_timestamps):
        self.events.append(("marvin", state.frame_serial))

    def publish_joint_command(self, q_rad, **_timestamps):
        self.events.append(("joint_command", tuple(q_rad)))

    def publish_gripper_command(self, closedness, **_timestamps):
        self.events.append(("gripper_command", tuple(closedness)))

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
        self.gripper_commands = []
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

    def send_gripper_command(self, closedness):
        self.gripper_commands.append(tuple(closedness))

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


class FakeDasFingerSystem:
    def __init__(self, encoder_callback, encoder_distance):
        self.databus = None
        self._encoder_callback = encoder_callback
        self._encoder_distance = encoder_distance
        self._stopped = threading.Event()
        self.targets = []

    def start(self):
        self.databus = self
        self._encoder_callback(struct.pack(">f", self._encoder_distance))
        self._stopped.wait()

    def set_finger_distance(self, distance):
        self.targets.append(float(distance))

    def stop(self):
        self._stopped.set()


class FakeMarvinVendorKinematics:
    def __init__(self):
        self.fail_inverse_kinematics = False
        self.nsp_reference_calls = []
        self.nsp_angles_deg = []

    def set_nsp_reference(self, arm, q_rad):
        self.nsp_reference_calls.append((arm, np.asarray(q_rad).copy()))

    def fk_world(self, _arm, q_rad):
        tcp_transform = np.eye(4)
        tcp_transform[0, 3] = q_rad[0]
        return tcp_transform

    def ik_world(
        self,
        _arm,
        T_world_tcp_m,
        q_ref_rad,
        nsp_angle_deg=None,
    ):
        self.nsp_angles_deg.append(nsp_angle_deg)
        if self.fail_inverse_kinematics:
            return VendorIkResult(False, None, "singular or out of range")
        q_rad = np.asarray(q_ref_rad).copy()
        q_rad[0] = T_world_tcp_m[0, 3]
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
            [[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]],
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
            target_tcp_transform[:3, 3], [0.05, 0.0, 0.0], atol=1e-12
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
            after_regrip_target[:3, 3], [1.05, 2.0, 3.0], atol=1e-12
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
            right_target[:3, 3], [4.0, 5.1, 6.0], atol=1e-12
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

    def test_episode_validator_writes_manifest_for_required_raw_topics(self):
        topic = lambda count=2: {
            "count": count,
            "bag_time_regressions": 0,
            "source_time_regressions": 0,
            "sequence_gaps": 0,
        }
        statistics = {
            "/raw/pico/frame": topic(),
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
        client = RosPicoClient.__new__(RosPicoClient)
        client._condition = threading.Condition()
        client._snapshot = None
        client._valid = False
        client._sequence_id = 0
        client._update_id = 0
        client._receive_steady_ns = 0

        def message(sequence_id, timestamp_ns):
            return SimpleNamespace(
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
            )

        client._callback(message(10, 100))
        client._callback(message(10, 100))
        client._callback(message(1, 200))

        self.assertEqual(client._sequence_id, 1)
        self.assertEqual(client._update_id, 2)
        self.assertEqual(client._snapshot.timestamp_ns, 200)

    def test_ros_das_client_maps_feedback_and_publishes_commands(self):
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

        now_ns = time.monotonic_ns()

        def state(side, distance):
            return SimpleNamespace(
                side=side,
                sequence_id=1,
                distance_m=distance,
                target_distance_m=0.05,
                receive_steady_ns=now_ns,
                status_flags=0,
                valid=True,
                header=SimpleNamespace(
                    stamp=SimpleNamespace(sec=123, nanosec=456)
                ),
            )

        client._callback(state("left", 0.055))
        client._callback(state("right", 0.055))

        class Command:
            def __init__(self):
                self.header = SimpleNamespace(
                    stamp=SimpleNamespace(sec=0, nanosec=0), frame_id=""
                )

        published = []
        client._message_type = Command
        client._publisher = SimpleNamespace(publish=published.append)

        np.testing.assert_allclose(
            client.get_initial_gripper_closedness(), (0.25, 0.75)
        )
        np.testing.assert_allclose(
            client.send_gripper_command((0.2, 0.3)), (0.058, 0.028)
        )
        self.assertEqual(published[0].closedness, [0.2, 0.3])

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

    def test_hardware_cli_does_not_enable_das_by_default(self):
        entry_path = (
            Path(__file__).resolve().parents[1]
            / "scripts"
            / "hardware"
            / "teleop_marvin_hardware.py"
        )
        arguments, _parser = runpy.run_path(str(entry_path))[
            "parse_command_line_arguments"
        ]([])
        self.assertIsNone(arguments.das_gripper_config)
        self.assertIsNone(arguments.das_sdk_root)
        self.assertFalse(arguments.das_from_ros2)

    def test_standalone_reset_reuses_safe_cosine_return(self):
        entry_path = (
            Path(__file__).resolve().parents[1]
            / "scripts"
            / "hardware"
            / "reset_marvin_hardware.py"
        )
        reset_robot = runpy.run_path(str(entry_path))["reset_robot"]
        adapter = FakeMarvinSdkAdapter()
        clock = [0.0]

        def sleep(seconds):
            clock[0] += seconds

        reset_robot(
            adapter,
            None,
            duration=0.04,
            expected_sdk_version=1,
            monotonic=lambda: clock[0],
            sleep=sleep,
        )

        np.testing.assert_allclose(
            adapter.sent_commands_rad[-1], MARVIN_INITIAL_POSE_Q_RAD
        )
        self.assertEqual(
            adapter.events[:3],
            [
                "configure_control_parameters",
                "enter_joint_impedance",
                "enable_pd_feedforward",
            ],
        )
        self.assertTrue(adapter.idle)
        self.assertTrue(adapter.released)

    def test_collection_supervisor_builds_safe_job_and_shutdown_order(self):
        entry_path = (
            Path(__file__).resolve().parents[1]
            / "scripts"
            / "data"
            / "run_collection.py"
        )
        namespace = runpy.run_path(str(entry_path))
        command_line = [
            "--task",
            "pick",
            "--robot-model",
            "M6S",
            "--enable-hardware",
            "--confirmed-estop",
            "--confirmed-joint-mapping",
            "--das-config",
            "das.json",
            "--das-sdk-root",
            "das-sdk",
            "--preview-root",
            "/dev/shm/fieldnote-preview-test",
        ]
        arguments = namespace["parse_command_line_arguments"](command_line)
        commands = namespace["_build_commands"](arguments)
        self.assertEqual(arguments.part, "all")
        self.assertEqual(
            namespace["parse_command_line_arguments"](
                ["--part", "devices", *command_line]
            ).part,
            "devices",
        )
        self.assertIn("--pico-from-ros2", commands["hardware"])
        self.assertIn("--ros2", commands["hardware"])
        self.assertIn("--das-from-ros2", commands["hardware"])
        self.assertNotIn("--das-sdk-root", commands["hardware"])
        self.assertIn("--sdk-root", commands["das_left"])
        self.assertIn("--side", commands["das_right"])
        self.assertIn("--das-config", commands["recorder"])
        self.assertIn("--ready-file", commands["recorder"])
        self.assertIn("--calibration", commands["recorder"])
        self.assertIn("--preview-root", commands["recorder"])
        self.assertEqual(
            namespace["PROCESS_CPUS"]["hardware"], (2, 3, 18, 19)
        )

        calls = []
        results = namespace["_shutdown_processes"](
            {
                "pico": object(),
                "das_left": object(),
                "das_right": object(),
                "recorder": object(),
                "hardware": object(),
            },
            stop_process=lambda process, name, timeout: (
                calls.append((name, timeout)) or 0
            ),
        )
        self.assertEqual(
            calls,
            [
                ("hardware", 15.0),
                ("das_left", 10.0),
                ("das_right", 10.0),
                ("recorder", None),
                ("pico", 10.0),
            ],
        )
        self.assertEqual(
            results,
            {
                "hardware": 0,
                "das_left": 0,
                "das_right": 0,
                "recorder": 0,
                "pico": 0,
            },
        )

        calls.clear()
        runtime = namespace["main"].__globals__
        runtime["_preflight"] = lambda _arguments: None
        runtime["_build_commands"] = lambda _arguments: {}
        runtime["_validated_cpu_sets"] = lambda: {}
        runtime["_start_devices"] = lambda *_arguments: calls.append("devices")
        runtime["_start_recording"] = lambda *_arguments: calls.append("recording")
        runtime["_finish_starting_devices"] = lambda *_arguments: calls.append(
            "hardware"
        )
        runtime["_monitor"] = lambda *_arguments: ("signal", 0)
        runtime["_shutdown_processes"] = lambda *_arguments: {}
        for part, expected in (
            ("all", ["devices", "recording", "hardware"]),
            ("devices", ["devices", "hardware"]),
            ("recording", ["recording"]),
        ):
            calls.clear()
            self.assertEqual(namespace["main"](["--part", part, *command_line]), 0)
            self.assertEqual(calls, expected)

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

    def test_lateral_nsp_maps_grip_held_controller_motion_to_angle(self):
        def snapshot(timestamp, left_x, right_x, grip_values):
            return XrSnapshot(
                timestamp,
                make_openxr_pose(x_meters=left_x),
                make_openxr_pose(x_meters=right_x),
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

        expected_tcp_delta_m = np.array([0.02, 0.0, 0.0])
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
            self.assertEqual(adapter.pd_period_milliseconds, 20)
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
