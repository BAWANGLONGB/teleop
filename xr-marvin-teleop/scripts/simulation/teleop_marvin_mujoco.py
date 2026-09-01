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
    DEFAULT_GRIPPER_COMMAND_HZ,
    DEFAULT_GRIPPER_RATE,
    DEFAULT_NSP_ANGLE_RATE_DEG_S,
    DEFAULT_NSP_LATERAL_DEADZONE_M,
    DEFAULT_NSP_LATERAL_FULL_SCALE_M,
    DEFAULT_NSP_LATERAL_MAX_ANGLE_DEG,
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
        "--gripper-rate", type=float, default=DEFAULT_GRIPPER_RATE
    )
    parser.add_argument(
        "--gripper-command-hz", type=float, default=DEFAULT_GRIPPER_COMMAND_HZ
    )
    parser.add_argument(
        "--thumbstick-y-sign", type=int, choices=(-1, 1), default=1
    )
    parser.add_argument("--nsp-angle-left", type=float, default=0.0)
    parser.add_argument("--nsp-angle-right", type=float, default=0.0)
    parser.add_argument(
        "--nsp-angle-rate",
        type=float,
        default=DEFAULT_NSP_ANGLE_RATE_DEG_S,
        help="IK_NSP angle slew rate in degrees per second",
    )
    parser.add_argument(
        "--nsp-lateral",
        action="store_true",
        help="map Grip-held controller lateral motion to IK_NSP angle",
    )
    parser.add_argument(
        "--nsp-max-angle",
        type=float,
        default=DEFAULT_NSP_LATERAL_MAX_ANGLE_DEG,
        help="maximum lateral IK_NSP angle in degrees (default: 5)",
    )
    parser.add_argument(
        "--nsp-lateral-deadzone",
        type=float,
        default=DEFAULT_NSP_LATERAL_DEADZONE_M,
        help="lateral controller deadzone in metres (default: 0.03)",
    )
    parser.add_argument(
        "--nsp-lateral-range",
        type=float,
        default=DEFAULT_NSP_LATERAL_FULL_SCALE_M,
        help="lateral displacement for full NSP angle in metres (default: 0.12)",
    )
    parser.add_argument(
        "--nsp-lateral-sign-left", type=int, choices=(-1, 1), default=1
    )
    parser.add_argument(
        "--nsp-lateral-sign-right", type=int, choices=(-1, 1), default=1
    )
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
            gripper_control_enabled=True,
            gripper_rate=arguments.gripper_rate,
            gripper_command_hz=arguments.gripper_command_hz,
            thumbstick_y_sign=arguments.thumbstick_y_sign,
            nsp_enabled=any(
                abs(value) > 1e-9
                for value in (
                    arguments.nsp_angle_left,
                    arguments.nsp_angle_right,
                )
            ) or arguments.nsp_lateral,
            nsp_angles_deg=(
                arguments.nsp_angle_left,
                arguments.nsp_angle_right,
            ),
            nsp_angle_rate_deg_s=arguments.nsp_angle_rate,
            nsp_lateral_enabled=arguments.nsp_lateral,
            nsp_lateral_max_angle_deg=arguments.nsp_max_angle,
            nsp_lateral_deadzone_m=arguments.nsp_lateral_deadzone,
            nsp_lateral_full_scale_m=arguments.nsp_lateral_range,
            nsp_lateral_signs=(
                arguments.nsp_lateral_sign_left,
                arguments.nsp_lateral_sign_right,
            ),
        )
        print(f"Session log: {session_logger.path.resolve()}")
        controller.run()


if __name__ == "__main__":
    main()
