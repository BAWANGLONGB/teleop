"""Teleoperate the Marvin MuJoCo model from PICO controllers."""

import argparse
from contextlib import closing
from pathlib import Path

from xr_marvin_teleop.common.marvin_postures import MARVIN_INITIAL_POSE_Q_RAD
from xr_marvin_teleop.common.marvin_session_logger import MarvinSessionLogger
from xr_marvin_teleop.common.xr_client import XrClient
from xr_marvin_teleop.hardware.interface.marvin import load_active_tool_configs
from xr_marvin_teleop.hardware.interface.marvin_kinematics import (
    MarvinVendorKinematics,
)
from xr_marvin_teleop.hardware.marvin_teleop_controller import (
    MarvinHardwareTeleopController,
)
from xr_marvin_teleop.simulation.marvin_mujoco_adapter import (
    MarvinMujocoAdapter,
)


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_SDK_ROOT = PROJECT_ROOT.parent / "TJArm" / "tj_fx_robot-master"
DEFAULT_TOOLS_CONFIGURATION = PROJECT_ROOT.parent / "TJArm" / "tools_cfg.json"
DEFAULT_XML_PATH = PROJECT_ROOT / "assets" / "marvin" / "marvin_dual.mujoco.xml"
DEFAULT_SCALE_CALIBRATION = PROJECT_ROOT / "logs" / "marvin_scale_calibration.json"


def parse_command_line_arguments():
    parser = argparse.ArgumentParser(
        description="PICO-to-Marvin MuJoCo teleoperation"
    )
    parser.add_argument("--xml-path", type=Path, default=DEFAULT_XML_PATH)
    parser.add_argument("--sdk-root", type=Path, default=DEFAULT_SDK_ROOT)
    parser.add_argument("--kinematics-config-path", type=Path)
    parser.add_argument(
        "--tools-config", type=Path, default=DEFAULT_TOOLS_CONFIGURATION
    )
    parser.add_argument("--scale-factor", type=float)
    parser.add_argument(
        "--scale-calibration-path", type=Path, default=DEFAULT_SCALE_CALIBRATION
    )
    parser.add_argument("--control-hz", type=float, default=50.0)
    parser.add_argument("--return-duration", type=float, default=3.0)
    parser.add_argument(
        "--log-directory", type=Path, default=PROJECT_ROOT / "logs"
    )
    parser.add_argument("--headless", action="store_true")
    return parser.parse_args()


def main():
    arguments = parse_command_line_arguments()
    with closing(XrClient()) as xr_client:
        kinematics = MarvinVendorKinematics(
            arguments.sdk_root, arguments.kinematics_config_path
        )
        tool_configurations = load_active_tool_configs(arguments.tools_config)
        for arm_index, tool_configuration in enumerate(tool_configurations):
            kinematics.set_tool(
                arm_index, tool_configuration.kinematics_mm_deg
            )
        adapter = MarvinMujocoAdapter(
            arguments.xml_path,
            MARVIN_INITIAL_POSE_Q_RAD,
            control_hz=arguments.control_hz,
            launch_viewer=not arguments.headless,
        )
        session_logger = MarvinSessionLogger(arguments.log_directory, "mujoco")
        controller = MarvinHardwareTeleopController(
            xr_client=xr_client,
            adapter=adapter,
            kinematics=kinematics,
            scale_calibration_path=arguments.scale_calibration_path,
            initial_pose_q_rad=MARVIN_INITIAL_POSE_Q_RAD,
            requested_scale_factor=arguments.scale_factor,
            control_hz=arguments.control_hz,
            return_duration=arguments.return_duration,
            expected_sdk_version=None,
            control_parameter_settle_seconds=0.0,
            mode_settle_seconds=0.0,
            pd_settle_seconds=0.0,
            session_logger=session_logger,
        )
        print(f"Session log: {session_logger.path.resolve()}")
        controller.run()


if __name__ == "__main__":
    main()
