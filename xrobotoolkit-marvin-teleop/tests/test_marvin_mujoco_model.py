import csv
import hashlib
import json
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import mujoco
from meshcat import transformations as tf
import numpy as np

import xrobotoolkit_teleop.common.base_teleop_controller as base_controller_module
from xrobotoolkit_teleop.common.arm_length_calibration import ArmLengthScaleCalibrator
from xrobotoolkit_teleop.common.buffered_xr_client import BufferedXrClient
from xrobotoolkit_teleop.simulation.mujoco_teleop_controller import MujocoTeleopController
import placo
from scripts.simulation.teleop_marvin_mujoco import (
    MARVIN_REST_TO_FORWARD_TCP_DELTA,
    MARVIN_REST_TO_FORWARD_TCP_TRAVEL,
    MARVIN_JOINT_ACCELERATION_LIMITS,
    MARVIN_JOINT_SPEED_LIMITS,
    R_PICO_TO_MARVIN_WORLD,
)
from xrobotoolkit_teleop.utils.geometry import pose_in_head_yaw_frame
from xrobotoolkit_teleop.utils.mujoco_utils import (
    calc_mujoco_qpos_from_placo_q,
    calc_placo_q_from_mujoco_qpos,
)


REPO_ROOT = Path(__file__).resolve().parents[1]
MODEL_PATH = REPO_ROOT / "assets" / "marvin" / "marvin_dual.mujoco.xml"
URDF_PATH = REPO_ROOT / "assets" / "marvin" / "marvin_dual.urdf"
EXPECTED_JOINTS = [
    *(f"Joint{index}_L" for index in range(1, 8)),
    *(f"Joint{index}_R" for index in range(1, 8)),
]
HUMAN_REST_QPOS = np.array(
    [
        np.pi / 2,
        -np.pi / 2,
        -np.pi / 2,
        -np.deg2rad(20.0),
        -np.pi / 2,
        0.0,
        0.0,
        -np.pi / 2,
        -np.pi / 2,
        np.pi / 2,
        -np.deg2rad(20.0),
        np.pi / 2,
        0.0,
        0.0,
    ]
)


def test_buffered_xr_client_exposes_fresh_atomic_snapshots():
    class FakeXrClient:
        def __init__(self):
            self.timestamp = 0
            self.closed = False

        def get_timestamp_ns(self):
            self.timestamp += 1
            return self.timestamp

        def get_pose_by_name(self, _name):
            return np.arange(7, dtype=float) + self.timestamp

        def get_key_value_by_name(self, _name):
            return 0.75

        def get_button_state_by_name(self, _name):
            return False

        def close(self):
            self.closed = True

    source = FakeXrClient()
    client = BufferedXrClient(
        source,
        pose_names=["headset", "left_controller"],
        key_names=["left_grip"],
        button_names=["A"],
        poll_hz=500.0,
    )
    client.start()
    assert client.wait_until_ready(timeout=0.5)
    time.sleep(0.015)

    diagnostics = client.get_diagnostics()
    assert diagnostics["sequence"] >= 2
    assert diagnostics["poll_age_ms"] < 20.0
    assert diagnostics["source_age_ms"] < 20.0
    client.begin_cycle()
    pinned_timestamp = client.get_timestamp_ns()
    time.sleep(0.01)  # The polling thread advances, but this cycle stays atomic.
    np.testing.assert_allclose(client.get_pose_by_name("headset")[0], pinned_timestamp)
    assert client.get_key_value_by_name("left_grip") == 0.75
    assert client.get_timestamp_ns() == pinned_timestamp
    client.end_cycle()
    client.close()
    assert source.closed


def test_two_point_arm_length_calibration_updates_scale_factor():
    calibrator = ArmLengthScaleCalibrator(
        robot_motion_range=MARVIN_REST_TO_FORWARD_TCP_TRAVEL,
        expected_motion_direction=MARVIN_REST_TO_FORWARD_TCP_DELTA,
        workspace_margin=0.95,
    )
    down_positions = {
        "left_hand": np.array([0.0, 0.2, -0.70]),
        "right_hand": np.array([0.0, -0.2, -0.70]),
    }
    forward_positions = {
        "left_hand": np.array([-0.70, 0.2, 0.0]),
        "right_hand": np.array([-0.70, -0.2, 0.0]),
    }

    first_result = calibrator.capture(down_positions)
    assert first_result.status == "down_pose_captured"
    assert calibrator.awaiting_extended_sample

    completed_result = calibrator.capture(forward_positions)
    assert completed_result.status == "completed"
    np.testing.assert_allclose(completed_result.mean_arm_length, 0.70, atol=1e-12)
    expected_human_travel = np.sqrt(2.0) * 0.70
    np.testing.assert_allclose(
        completed_result.scale_factor,
        0.95 * MARVIN_REST_TO_FORWARD_TCP_TRAVEL / expected_human_travel,
        atol=1e-12,
    )
    assert not calibrator.awaiting_extended_sample


def test_arm_length_calibration_rejects_asymmetric_sample_and_keeps_start():
    calibrator = ArmLengthScaleCalibrator(robot_motion_range=MARVIN_REST_TO_FORWARD_TCP_TRAVEL)
    start = {"left_hand": np.zeros(3), "right_hand": np.zeros(3)}
    calibrator.capture(start)

    result = calibrator.capture(
        {
            "left_hand": np.array([0.70, 0.0, 0.0]),
            "right_hand": np.array([0.40, 0.0, 0.0]),
        }
    )

    assert result.status == "rejected"
    assert calibrator.awaiting_extended_sample


def test_marvin_pose_pair_travel_matches_fk():
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    data.qpos[:] = HUMAN_REST_QPOS
    mujoco.mj_forward(model, data)
    rest_tcp = data.site_xpos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "left_tcp")].copy()

    forward_qpos = np.zeros(14)
    forward_qpos[[1, 8]] = -np.pi / 2
    data.qpos[:] = forward_qpos
    mujoco.mj_forward(model, data)
    forward_tcp = data.site_xpos[
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "left_tcp")
    ].copy()

    np.testing.assert_allclose(
        forward_tcp - rest_tcp,
        MARVIN_REST_TO_FORWARD_TCP_DELTA,
        atol=5e-6,
    )


def test_marvin_mujoco_contract():
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))

    assert (model.nq, model.nv, model.nu) == (14, 14, 14)
    assert [
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, index) for index in range(model.njnt)
    ] == EXPECTED_JOINTS
    assert np.all(HUMAN_REST_QPOS >= model.jnt_range[:, 0])
    assert np.all(HUMAN_REST_QPOS <= model.jnt_range[:, 1])

    for name in ("left_tcp", "right_tcp"):
        assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, name) >= 0
    for name in (
        "left_target",
        "right_target",
        "left_commanded_target",
        "right_commanded_target",
    ):
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        assert body_id >= 0
        assert model.body_mocapid[body_id] >= 0

    expected_kp = np.tile([200.0, 180.0, 160.0, 140.0, 120.0, 110.0, 110.0], 2)
    expected_kv = np.tile([15.0, 14.0, 13.0, 12.0, 11.0, 10.0, 10.0], 2)
    np.testing.assert_allclose(model.actuator_gainprm[:, 0], expected_kp, atol=1e-12)
    np.testing.assert_allclose(-model.actuator_biasprm[:, 2], expected_kv, atol=1e-12)

    urdf_root = ET.parse(URDF_PATH).getroot()
    world_joint = urdf_root.find("./joint[@name='world_to_base']")
    assert world_joint is not None
    assert world_joint.find("origin").get("rpy") == "0 0 3.141592653589793"


def test_marvin_demo_mount_spacing_and_fixed_gripper_geometry():
    """Keep the runtime snapshot aligned with showcase_pln... sim geometry."""
    mjcf_root = ET.parse(MODEL_PATH).getroot()
    worldbody = mjcf_root.find("worldbody")
    assert worldbody is not None

    expected_world_positions = {
        "left_base_visual": [0.0, -0.06, -0.1],
        "right_base_visual": [0.0, 0.06, -0.1],
    }
    for name, expected in expected_world_positions.items():
        geom = worldbody.find(f"./geom[@name='{name}']")
        assert geom is not None
        np.testing.assert_allclose(np.fromstring(geom.get("pos"), sep=" "), expected, atol=1e-12)

    link1_positions = {}
    for name, expected in {
        "Link1_L": [0.0, -0.2186, -0.1],
        "Link1_R": [0.0, 0.2186, -0.1],
    }.items():
        body = worldbody.find(f"./body[@name='{name}']")
        assert body is not None
        position = np.fromstring(body.get("pos"), sep=" ")
        link1_positions[name] = position
        np.testing.assert_allclose(position, expected, atol=1e-12)

    assert expected_world_positions["right_base_visual"][1] - expected_world_positions[
        "left_base_visual"
    ][1] == 0.12
    np.testing.assert_allclose(
        link1_positions["Link1_R"][1] - link1_positions["Link1_L"][1],
        0.4372,
        atol=1e-12,
    )

    urdf_root = ET.parse(URDF_PATH).getroot()
    for name, expected_xyz, expected_rpy in (
        (
            "base_to_left_arm",
            [0.0, 0.06, -0.1],
            [-np.pi / 2, 0.0, 0.0],
        ),
        (
            "base_to_right_arm",
            [0.0, -0.06, -0.1],
            [np.pi / 2, 0.0, 0.0],
        ),
    ):
        origin = urdf_root.find(f"./joint[@name='{name}']/origin")
        assert origin is not None
        np.testing.assert_allclose(np.fromstring(origin.get("xyz"), sep=" "), expected_xyz, atol=1e-12)
        np.testing.assert_allclose(np.fromstring(origin.get("rpy"), sep=" "), expected_rpy, atol=1e-12)

    for name in ("JointTCP_L", "JointTCP_R"):
        origin = urdf_root.find(f"./joint[@name='{name}']/origin")
        assert origin is not None
        np.testing.assert_allclose(
            np.fromstring(origin.get("xyz"), sep=" "),
            [0.0, -0.087, 0.0],
            atol=1e-12,
        )

    expected_mesh_hashes = {
        "gripper_base_link.STL": "42ef1aee5ffd1eef1e838dc7973f8de582ace3cf4ea6ef9f4bf1b7112c7699f7",
        "gripper_fflan_Link.STL": "8fe6f68a9dfdedb6e2a6ab444f4c539745d4ba4b88a98807d34116d040edfc7c",
        "gripper_fleft_Link.STL": "9908703b50d60dfdb410202ed68e0dc87c1e9997b6d116e7b96ea0f46f3e9538",
        "gripper_fright_Link.STL": "b1936675cf49188542cfd5a1dd1d83c903ce490205420cc2691d5f2164347398",
    }
    for filename, expected_hash in expected_mesh_hashes.items():
        mesh_path = MODEL_PATH.parent / "meshes" / filename
        assert mesh_path.is_file()
        assert hashlib.sha256(mesh_path.read_bytes()).hexdigest() == expected_hash

    expected_gripper_poses = {
        "left_gripper_flange_visual": (
            [0.0, -0.087, 0.0],
            [-0.707108, -1.29867e-06, 1.29868e-06, 0.707105],
        ),
        "left_gripper_base_visual": (
            [0.0812152, -0.157861, -4.79498e-05],
            [-0.500002, 0.5, -0.499998, 0.5],
        ),
        "left_gripper_left_finger_visual": (
            [0.0467975, -0.1658, -0.0293383],
            [-0.707108, -1.29867e-06, 1.29868e-06, 0.707105],
        ),
        "left_gripper_right_finger_visual": (
            [0.0467972, -0.1658, 0.0293387],
            [-0.707108, -1.29867e-06, 1.29868e-06, 0.707105],
        ),
        "right_gripper_flange_visual": (
            [0.0, -0.087, 0.0],
            [-1.29867e-06, 0.707108, -0.707105, 1.29868e-06],
        ),
        "right_gripper_base_visual": (
            [-0.0812147, -0.157862, 4.79498e-05],
            [0.5, 0.500002, -0.5, -0.499998],
        ),
        "right_gripper_left_finger_visual": (
            [-0.0467969, -0.165801, 0.0293383],
            [-1.29867e-06, 0.707108, -0.707105, 1.29868e-06],
        ),
        "right_gripper_right_finger_visual": (
            [-0.0467967, -0.165801, -0.0293387],
            [-1.29867e-06, 0.707108, -0.707105, 1.29868e-06],
        ),
    }
    for name, (expected_position, expected_quaternion) in expected_gripper_poses.items():
        geom = mjcf_root.find(f".//geom[@name='{name}']")
        assert geom is not None
        np.testing.assert_allclose(
            np.fromstring(geom.get("pos"), sep=" "), expected_position, atol=1e-12
        )
        np.testing.assert_allclose(
            np.fromstring(geom.get("quat"), sep=" "), expected_quaternion, atol=1e-12
        )

    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    for side in ("left", "right"):
        for part in ("flange", "base", "left_finger", "right_finger"):
            geom_id = mujoco.mj_name2id(
                model,
                mujoco.mjtObj.mjOBJ_GEOM,
                f"{side}_gripper_{part}_visual",
            )
            assert geom_id >= 0
            assert model.geom_contype[geom_id] == 0
            assert model.geom_conaffinity[geom_id] == 0


def test_marvin_snapshot_output_checksums():
    manifest = json.loads((MODEL_PATH.parent / "marvin_dual.manifest.json").read_text())
    output = manifest["output"]
    for filename_key, hash_key in (
        ("file", "sha256"),
        ("mujoco_file", "mujoco_sha256"),
    ):
        path = MODEL_PATH.parent / output[filename_key]
        assert hashlib.sha256(path.read_bytes()).hexdigest() == output[hash_key]


def test_marvin_rest_targets_match_tcp_after_demo_mount_update():
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    data.qpos[:] = HUMAN_REST_QPOS
    mujoco.mj_forward(model, data)

    for side in ("left", "right"):
        site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, f"{side}_tcp")
        target_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"{side}_target")
        np.testing.assert_allclose(data.xpos[target_id], data.site_xpos[site_id], atol=5e-9)
        np.testing.assert_allclose(
            data.xmat[target_id].reshape(3, 3),
            data.site_xmat[site_id].reshape(3, 3),
            atol=5e-8,
        )


def test_marvin_identified_tool_payload_dynamics():
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    expected = {
        "left_tool_payload": {
            "mass": 0.481,
            "com": [0.004691, -0.034036, 0.084135],
            "principal_inertia": [0.005333333333, 0.011666666665, 0.006333333333],
        },
        "right_tool_payload": {
            "mass": 0.459,
            "com": [-0.000776, 0.029685, 0.10105],
            "principal_inertia": [0.006999999999, 0.006, 0.001],
        },
    }

    for body_name, payload in expected.items():
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        assert body_id >= 0
        np.testing.assert_allclose(model.body_mass[body_id], payload["mass"], atol=1e-12)
        np.testing.assert_allclose(model.body_ipos[body_id], payload["com"], atol=1e-12)
        # MuJoCo stores principal moments in descending order and rotates the
        # inertial frame as needed, so compare the unordered eigenvalues.
        np.testing.assert_allclose(
            np.sort(model.body_inertia[body_id]),
            np.sort(payload["principal_inertia"]),
            atol=1e-12,
        )
        np.testing.assert_allclose(model.body_gravcomp[body_id], 1.0, atol=1e-12)


def test_pico_motion_matches_operator_view_in_marvin_world():
    np.testing.assert_allclose(R_PICO_TO_MARVIN_WORLD.T @ R_PICO_TO_MARVIN_WORLD, np.eye(3), atol=1e-12)
    np.testing.assert_allclose(np.linalg.det(R_PICO_TO_MARVIN_WORLD), 1.0, atol=1e-12)

    pico_right = np.array([1.0, 0.0, 0.0])
    pico_up = np.array([0.0, 1.0, 0.0])
    pico_forward = np.array([0.0, 0.0, -1.0])
    np.testing.assert_allclose(R_PICO_TO_MARVIN_WORLD @ pico_right, [0.0, 1.0, 0.0])
    np.testing.assert_allclose(R_PICO_TO_MARVIN_WORLD @ pico_up, [0.0, 0.0, 1.0])
    np.testing.assert_allclose(R_PICO_TO_MARVIN_WORLD @ pico_forward, [-1.0, 0.0, 0.0])


def test_head_yaw_frame_follows_headset_translation_and_heading():
    relative_controller_xyz = np.array([0.25, -0.15, -0.4])
    relative_controller_rotation = np.eye(3)

    headset_xyz = np.array([1.2, 0.8, -0.5])
    headset_rotation = tf.rotation_matrix(np.deg2rad(60.0), [0.0, 1.0, 0.0])[:3, :3]
    controller_xyz = headset_xyz + headset_rotation @ relative_controller_xyz
    controller_rotation = headset_rotation @ relative_controller_rotation

    headset_transform = np.eye(4)
    headset_transform[:3, :3] = headset_rotation
    controller_transform = np.eye(4)
    controller_transform[:3, :3] = controller_rotation

    result_xyz, result_quat, yaw_rotation = pose_in_head_yaw_frame(
        controller_xyz,
        tf.quaternion_from_matrix(controller_transform),
        headset_xyz,
        tf.quaternion_from_matrix(headset_transform),
    )

    np.testing.assert_allclose(result_xyz, relative_controller_xyz, atol=1e-12)
    np.testing.assert_allclose(tf.quaternion_matrix(result_quat)[:3, :3], relative_controller_rotation, atol=1e-12)
    np.testing.assert_allclose(yaw_rotation, headset_rotation, atol=1e-12)


def test_marvin_joint_mapping_round_trip():
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    robot = placo.RobotWrapper(str(URDF_PATH))

    placo_q = calc_placo_q_from_mujoco_qpos(model, robot, HUMAN_REST_QPOS)
    round_trip = calc_mujoco_qpos_from_placo_q(model, robot, placo_q)

    np.testing.assert_allclose(round_trip, HUMAN_REST_QPOS, atol=1e-9)

    data = mujoco.MjData(model)
    data.qpos[:] = HUMAN_REST_QPOS
    mujoco.mj_forward(model, data)
    robot.state.q = placo_q
    robot.update_kinematics()
    for side, suffix in (("left", "L"), ("right", "R")):
        site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, f"{side}_tcp")
        np.testing.assert_allclose(
            data.site_xpos[site_id],
            robot.get_T_world_frame(f"TCP_Link_{suffix}")[:3, 3],
            atol=5e-7,
        )


def test_marvin_human_rest_hold_is_finite():
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    data.qpos[:] = HUMAN_REST_QPOS
    data.ctrl[:] = HUMAN_REST_QPOS
    mujoco.mj_forward(model, data)

    left_tcp = data.site_xpos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "left_tcp")]
    right_tcp = data.site_xpos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "right_tcp")]
    left_elbow = data.xpos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "Link4_L")]
    right_elbow = data.xpos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "Link4_R")]
    np.testing.assert_allclose(left_tcp[[0, 2]], right_tcp[[0, 2]], atol=1e-5)
    np.testing.assert_allclose(left_tcp[1], -right_tcp[1], atol=1e-5)
    np.testing.assert_allclose(left_elbow[1], -right_elbow[1], atol=1e-5)
    assert left_tcp[0] < left_elbow[0] - 0.05  # Forearm bends forward.
    assert left_tcp[2] < left_elbow[2] - 0.25  # Hand remains below elbow.

    for _ in range(500):
        mujoco.mj_step(model, data)

    assert np.all(np.isfinite(data.qpos))
    assert np.all(np.isfinite(data.qvel))
    assert np.all(data.qpos >= model.jnt_range[:, 0] - 1e-8)
    assert np.all(data.qpos <= model.jnt_range[:, 1] + 1e-8)
    np.testing.assert_allclose(data.qpos, HUMAN_REST_QPOS, atol=5e-3)


def test_marvin_controller_starts_from_tcp_sites_without_jump(monkeypatch, tmp_path):
    class FakeXrClient:
        def get_key_value_by_name(self, _name):
            return 0.0

        def get_pose_by_name(self, name):
            assert name in ("headset", "left_controller", "right_controller")
            return np.array([0.0, 1.6, 0.0, 0.0, 0.0, 0.0, 1.0])

        def get_button_state_by_name(self, _name):
            return False

        def get_motion_tracker_data(self):
            return {}

        def get_timestamp_ns(self):
            return 123456789

        def close(self):
            pass

    monkeypatch.setattr(base_controller_module, "XrClient", FakeXrClient)
    config = {
        "left_hand": {
            "link_name": "TCP_Link_L",
            "mujoco_site_name": "left_tcp",
            "pose_source": "left_controller",
            "control_trigger": "left_grip",
            "vis_target": "left_target",
            "manipulability_weight": 0.0,
        },
        "right_hand": {
            "link_name": "TCP_Link_R",
            "mujoco_site_name": "right_tcp",
            "pose_source": "right_controller",
            "control_trigger": "right_grip",
            "vis_target": "right_target",
            "manipulability_weight": 0.0,
        },
    }
    controller = MujocoTeleopController(
        xml_path=str(MODEL_PATH),
        robot_urdf_path=str(URDF_PATH),
        manipulator_config=config,
        R_headset_world=R_PICO_TO_MARVIN_WORLD,
        reference_mode="head_yaw",
        mj_qpos_init=HUMAN_REST_QPOS,
        visualize_placo=False,
        viewer_camera="overview",
        xr_poll_hz=200.0,
        telemetry_report_interval=0.0,
        telemetry_output_dir=tmp_path,
        telemetry_session_name="test_latency",
    )

    controller._start_telemetry_logging()
    controller.xr_client.start()
    assert controller.xr_client.wait_until_ready(timeout=0.5)

    np.testing.assert_allclose(controller.mj_data.qpos, HUMAN_REST_QPOS, atol=1e-12)
    for hand, site_name in (("left_hand", "left_tcp"), ("right_hand", "right_tcp")):
        site_id = mujoco.mj_name2id(controller.mj_model, mujoco.mjtObj.mjOBJ_SITE, site_name)
        np.testing.assert_allclose(
            controller.effector_task[hand].T_world_frame[:3, 3],
            controller.mj_data.site_xpos[site_id],
            atol=1e-12,
        )

    controller._update_ik()
    controller._send_command()
    assert np.all(np.isfinite(controller.mj_data.ctrl))
    np.testing.assert_allclose(controller.mj_data.ctrl, HUMAN_REST_QPOS, atol=2e-3)

    sample = controller._control_cycle(deadline_late_ms=0.25)
    assert sample["cycle_ms"] > 0.0
    assert sample["physics_ms"] > 0.0
    assert sample["xr_source_timestamp_ns"] == 123456789
    assert np.isfinite(sample["xr_poll_age_ms"])
    assert np.isfinite(sample["xr_source_age_ms"])
    assert sample["deadline_late_ms"] == 0.25
    assert sample["max_raw_to_command_m"] >= 0.0
    assert sample["max_command_to_actual_m"] >= 0.0
    diagnostics = controller.get_latency_diagnostics()
    assert diagnostics["latest"]["physics_ms"] == sample["physics_ms"]
    controller.xr_client.close()
    controller._stop_telemetry_logging()

    csv_paths = list(tmp_path.glob("test_latency_*.csv"))
    summary_paths = list(tmp_path.glob("test_latency_*.summary.json"))
    assert len(csv_paths) == 1
    assert len(summary_paths) == 1
    with csv_paths[0].open(newline="", encoding="utf-8") as csv_file:
        rows = list(csv.DictReader(csv_file))
    assert len(rows) == 1
    assert int(rows[0]["sample_index"]) == 1
    assert float(rows[0]["deadline_late_ms"]) == 0.25
    with summary_paths[0].open(encoding="utf-8") as summary_file:
        summary = json.load(summary_file)
    assert summary["session"]["sample_count"] == 1
    assert summary["session"]["control_hz"] == 100.0
    assert summary["session"]["physics_hz"] == 500.0
    assert summary["metrics"]["p95"]["deadline_late_ms"] == 0.25


def test_release_returns_one_arm_with_cosine_interpolation(monkeypatch):
    class FakeXrClient:
        def close(self):
            pass

    monkeypatch.setattr(base_controller_module, "XrClient", FakeXrClient)
    config = {
        "left_hand": {
            "link_name": "TCP_Link_L",
            "mujoco_site_name": "left_tcp",
            "pose_source": "left_controller",
            "control_trigger": "left_grip",
            "vis_target": "left_target",
            "manipulability_weight": 0.0,
        },
        "right_hand": {
            "link_name": "TCP_Link_R",
            "mujoco_site_name": "right_tcp",
            "pose_source": "right_controller",
            "control_trigger": "right_grip",
            "vis_target": "right_target",
            "manipulability_weight": 0.0,
        },
    }
    left_targets = dict(zip(EXPECTED_JOINTS[:7], HUMAN_REST_QPOS[:7]))
    controller = MujocoTeleopController(
        xml_path=str(MODEL_PATH),
        robot_urdf_path=str(URDF_PATH),
        manipulator_config=config,
        mj_qpos_init=HUMAN_REST_QPOS,
        return_joint_positions={"left_hand": left_targets},
        return_duration=2.0,
    )

    released_qpos = HUMAN_REST_QPOS.copy()
    released_qpos[0] += 0.4
    controller.mj_data.qpos[:] = released_qpos
    mujoco.mj_forward(controller.mj_model, controller.mj_data)
    controller._update_robot_state()
    controller._previous_active["left_hand"] = True
    controller.active = {"left_hand": False, "right_hand": False}

    start_time = float(controller.mj_data.time)
    controller._apply_release_return()

    controller.mj_data.time = start_time + 1.0
    controller._apply_release_return()
    halfway_qpos = calc_mujoco_qpos_from_placo_q(
        controller.mj_model,
        controller.placo_robot,
        controller.placo_robot.state.q,
    )
    np.testing.assert_allclose(halfway_qpos[0], HUMAN_REST_QPOS[0] + 0.2, atol=1e-9)

    controller.mj_data.time = start_time + 2.0
    controller._apply_release_return()
    final_qpos = calc_mujoco_qpos_from_placo_q(
        controller.mj_model,
        controller.placo_robot,
        controller.placo_robot.state.q,
    )
    np.testing.assert_allclose(final_qpos[:7], HUMAN_REST_QPOS[:7], atol=1e-9)


def test_target_ball_moves_immediately_while_arm_command_is_rate_limited(monkeypatch):
    class FakeXrClient:
        def close(self):
            pass

    monkeypatch.setattr(base_controller_module, "XrClient", FakeXrClient)
    config = {
        "left_hand": {
            "link_name": "TCP_Link_L",
            "mujoco_site_name": "left_tcp",
            "pose_source": "left_controller",
            "control_trigger": "left_grip",
            "vis_target": "left_target",
            "vis_commanded_target": "left_commanded_target",
            "manipulability_weight": 0.0,
        },
        "right_hand": {
            "link_name": "TCP_Link_R",
            "mujoco_site_name": "right_tcp",
            "pose_source": "right_controller",
            "control_trigger": "right_grip",
            "vis_target": "right_target",
            "vis_commanded_target": "right_commanded_target",
            "manipulability_weight": 0.0,
        },
    }
    max_joint_speed = 0.35
    dt = 0.01
    controller = MujocoTeleopController(
        xml_path=str(MODEL_PATH),
        robot_urdf_path=str(URDF_PATH),
        manipulator_config=config,
        mj_qpos_init=HUMAN_REST_QPOS,
        max_joint_speed=max_joint_speed,
        dt=dt,
    )
    assert controller.physics_substeps == 5
    np.testing.assert_allclose(controller.mj_model.opt.timestep, 0.002, atol=1e-12)

    target_pose = controller.effector_task["left_hand"].T_world_frame.copy()
    target_pose[:3, 3] += np.array([0.12, -0.04, 0.08])
    controller.effector_task["left_hand"].T_world_frame = target_pose
    controller._update_mocap_target()

    left_mocap = controller.target_mocap_idx["left_hand"]
    np.testing.assert_allclose(controller.mj_data.mocap_pos[left_mocap], target_pose[:3, 3], atol=1e-12)

    desired_qpos = HUMAN_REST_QPOS.copy()
    desired_qpos[0] += 0.4
    controller.placo_robot.state.q = calc_placo_q_from_mujoco_qpos(
        controller.mj_model,
        controller.placo_robot,
        desired_qpos,
    )
    controller.placo_robot.update_kinematics()
    controller._send_command()
    controller._update_commanded_mocap_target()

    expected_step = max_joint_speed * dt
    np.testing.assert_allclose(
        controller.mj_data.ctrl[0],
        HUMAN_REST_QPOS[0] + expected_step,
        atol=1e-12,
    )
    assert controller.mj_data.ctrl[0] < desired_qpos[0]
    np.testing.assert_allclose(controller.mj_data.mocap_pos[left_mocap], target_pose[:3, 3], atol=1e-12)
    commanded_mocap = controller.commanded_mocap_idx["left_hand"]
    left_site = mujoco.mj_name2id(
        controller.mj_model, mujoco.mjtObj.mjOBJ_SITE, "left_tcp"
    )
    np.testing.assert_allclose(
        controller.mj_data.mocap_pos[commanded_mocap],
        controller._command_fk_data.site_xpos[left_site],
        atol=1e-12,
    )
    assert np.linalg.norm(
        controller.mj_data.mocap_pos[left_mocap]
        - controller.mj_data.mocap_pos[commanded_mocap]
    ) > 0.01

    # One hundred control updates advance exactly one second of simulation
    # through 5 x 2 ms physics substeps each. The command therefore advances
    # by the configured 0.35 rad/s, rather than five times that rate.
    for _ in range(100):
        controller._step_physics()
        if controller.mj_data.time < 1.0 - 1e-12:
            controller._send_command()

    np.testing.assert_allclose(controller.mj_data.time, 1.0, atol=1e-12)
    np.testing.assert_allclose(
        controller.mj_data.ctrl[0],
        HUMAN_REST_QPOS[0] + max_joint_speed,
        atol=1e-12,
    )
    np.testing.assert_allclose(controller.mj_data.mocap_pos[left_mocap], target_pose[:3, 3], atol=1e-12)


def test_joint_acceleration_soft_limits_and_predictive_braking(monkeypatch):
    class FakeXrClient:
        def close(self):
            pass

    monkeypatch.setattr(base_controller_module, "XrClient", FakeXrClient)
    config = {
        "left_hand": {
            "link_name": "TCP_Link_L",
            "mujoco_site_name": "left_tcp",
            "pose_source": "left_controller",
            "control_trigger": "left_grip",
            "vis_target": "left_target",
            "manipulability_weight": 0.0,
        },
        "right_hand": {
            "link_name": "TCP_Link_R",
            "mujoco_site_name": "right_tcp",
            "pose_source": "right_controller",
            "control_trigger": "right_grip",
            "vis_target": "right_target",
            "manipulability_weight": 0.0,
        },
    }
    dt = 0.01
    max_speed = 0.35
    max_acceleration = 0.7
    margin = np.deg2rad(5.0)
    controller = MujocoTeleopController(
        xml_path=str(MODEL_PATH),
        robot_urdf_path=str(URDF_PATH),
        manipulator_config=config,
        mj_qpos_init=HUMAN_REST_QPOS,
        max_joint_speed=max_speed,
        max_joint_acceleration=max_acceleration,
        joint_limit_margin=margin,
        dt=dt,
    )

    np.testing.assert_allclose(
        controller._ctrl_lower_limits,
        controller.mj_model.jnt_range[:, 0] + margin,
        atol=1e-12,
    )
    np.testing.assert_allclose(
        controller._ctrl_upper_limits,
        controller.mj_model.jnt_range[:, 1] - margin,
        atol=1e-12,
    )

    def apply_limit(target):
        command = controller._limit_joint_command(target)
        controller._last_ctrl_command = command.copy()
        return command

    # A step target starts from rest at exactly a*dt velocity, hence a*dt^2
    # position, rather than jumping immediately to the speed ceiling.
    initial = controller._last_ctrl_command.copy()
    target = initial.copy()
    target[0] += 1.0
    first = apply_limit(target)
    np.testing.assert_allclose(
        first[0] - initial[0], max_acceleration * dt**2, atol=1e-12
    )
    np.testing.assert_allclose(
        controller._last_ctrl_velocity[0], max_acceleration * dt, atol=1e-12
    )

    # The target-aware stopping envelope reaches a stationary setpoint without
    # crossing it and starting a second acceleration-limited chase.
    commands = [first[0]]
    for _ in range(500):
        commands.append(apply_limit(target)[0])
    assert max(commands) <= target[0] + 1e-12
    np.testing.assert_allclose(commands[-1], target[0], atol=1e-8)
    assert abs(controller._last_ctrl_velocity[0]) < 1e-6

    # Start from a valid maximum-speed state. The predicted stopping-distance
    # envelope must reduce velocity before the upper soft limit, preserve the
    # acceleration bound, and never cross the boundary.
    controller._last_ctrl_command = initial.copy()
    controller._last_ctrl_command[0] = controller._ctrl_upper_limits[0] - 0.12
    controller._last_ctrl_velocity = np.zeros_like(initial)
    controller._last_ctrl_velocity[0] = max_speed
    target = controller._last_ctrl_command.copy()
    target[0] = controller._ctrl_upper_limits[0] + 1.0

    velocities = []
    for _ in range(100):
        previous_velocity = controller._last_ctrl_velocity[0]
        command = apply_limit(target)
        velocity = controller._last_ctrl_velocity[0]
        velocities.append(velocity)
        assert abs(velocity - previous_velocity) <= max_acceleration * dt + 1e-12
        assert command[0] <= controller._ctrl_upper_limits[0] + 1e-12

    assert any(velocity < max_speed - 1e-6 for velocity in velocities)
    assert velocities[-1] < 1e-6
    assert controller._last_ctrl_command[0] > controller._ctrl_upper_limits[0] - 1e-5

    # A target reversal accelerates inward from rest without sticking at the
    # limit or bypassing the same acceleration constraint.
    controller._last_ctrl_velocity[:] = 0.0
    inward_target = controller._last_ctrl_command.copy()
    inward_target[0] -= 0.2
    previous = controller._last_ctrl_command.copy()
    inward = apply_limit(inward_target)
    np.testing.assert_allclose(
        inward[0] - previous[0], -max_acceleration * dt**2, atol=1e-12
    )

    resolved_speed = controller._resolve_actuator_parameter(
        MARVIN_JOINT_SPEED_LIMITS, "max_joint_speed"
    )
    resolved_acceleration = controller._resolve_actuator_parameter(
        MARVIN_JOINT_ACCELERATION_LIMITS, "max_joint_acceleration"
    )
    np.testing.assert_allclose(
        resolved_speed, [MARVIN_JOINT_SPEED_LIMITS[name] for name in EXPECTED_JOINTS]
    )
    np.testing.assert_allclose(
        resolved_acceleration,
        [MARVIN_JOINT_ACCELERATION_LIMITS[name] for name in EXPECTED_JOINTS],
    )

    def moving_target_error(feedforward):
        controller._max_joint_speed = resolved_speed
        controller._max_joint_acceleration = resolved_acceleration
        controller.target_velocity_feedforward = feedforward
        controller._last_ctrl_command = initial.copy()
        controller._last_ctrl_target = initial.copy()
        controller._last_ctrl_velocity = np.zeros_like(initial)
        controller._filtered_target_velocity = np.zeros_like(initial)
        moving_target = initial.copy()
        for _ in range(100):
            previous_velocity = controller._last_ctrl_velocity.copy()
            moving_target[0] += 0.003  # 0.3 rad/s continuously moving IK target.
            apply_limit(moving_target)
            assert np.all(
                np.abs(controller._last_ctrl_velocity - previous_velocity)
                <= resolved_acceleration * dt + 1e-12
            )
        return moving_target[0] - controller._last_ctrl_command[0]

    error_without_feedforward = moving_target_error(0.0)
    error_with_feedforward = moving_target_error(0.8)
    assert error_with_feedforward < 0.5 * error_without_feedforward


def test_tuned_marvin_actuator_tracks_joint1_profile_without_saturation():
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    data.qpos[:] = HUMAN_REST_QPOS
    data.ctrl[:] = HUMAN_REST_QPOS
    mujoco.mj_forward(model, data)

    target = HUMAN_REST_QPOS.copy()
    target[0] += 0.2
    command = HUMAN_REST_QPOS.copy()
    velocity = np.zeros(14)
    max_force = 0.0
    max_overshoot = 0.0
    time_to_95 = None
    dt = 0.01
    acceleration = 3.0
    speed = 1.0

    for tick in range(120):
        error = target - command
        velocity_step = acceleration * dt
        braking_speed = np.maximum(
            0.0,
            np.sqrt(velocity_step**2 + 2.0 * acceleration * np.abs(error))
            - velocity_step,
        )
        desired_velocity = np.sign(error) * np.minimum(
            np.minimum(np.abs(error) / dt, speed), braking_speed
        )
        velocity = np.clip(
            desired_velocity, velocity - velocity_step, velocity + velocity_step
        )
        command += velocity * dt
        data.ctrl[:] = command
        for _ in range(5):
            mujoco.mj_step(model, data)

        progress = (data.qpos[0] - HUMAN_REST_QPOS[0]) / 0.2
        max_force = max(max_force, abs(float(data.actuator_force[0])))
        max_overshoot = max(max_overshoot, float(data.qpos[0] - target[0]))
        if time_to_95 is None and progress >= 0.95:
            time_to_95 = (tick + 1) * dt

    assert time_to_95 is not None and time_to_95 <= 0.52
    assert max_overshoot < 0.012
    assert max_force < 10.0
