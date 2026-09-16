"""Headless MuJoCo and vendor kinematics integration tests."""

import unittest

from tests.marvin_hardware_fakes import (
    FakeXRClient,
    make_openxr_pose,
    MARVIN_INITIAL_POSE_Q_RAD,
    MarvinHardwareTeleopController,
    MarvinVendorKinematics,
    np,
    Path,
    tempfile,
    XrSnapshot,
)
try:
    from xr_marvin_teleop.simulation.marvin_mujoco_adapter import (
        MarvinMujocoAdapter,
    )
except (ImportError, OSError) as error:
    MarvinMujocoAdapter = None
    MUJOCO_IMPORT_ERROR = str(error)
else:
    MUJOCO_IMPORT_ERROR = ""


@unittest.skipIf(MarvinMujocoAdapter is None, MUJOCO_IMPORT_ERROR)
class TestMarvinSimulation(unittest.TestCase):
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


if __name__ == "__main__":
    unittest.main()
