import argparse
from contextlib import closing
from pathlib import Path

from xr_marvin_teleop.common.marvin_postures import MARVIN_INITIAL_POSE_Q_RAD
from xr_marvin_teleop.common.marvin_session_logger import MarvinSessionLogger
from xr_marvin_teleop.common.xr_client import XrClient
from xr_marvin_teleop.hardware.interface.marvin import (
    MarvinSdkAdapter,
    load_active_tool_configs,
)
from xr_marvin_teleop.hardware.interface.marvin_kinematics import (
    MarvinVendorKinematics,
)
from xr_marvin_teleop.hardware.marvin_teleop_controller import (
    DEFAULT_CONTROL_HZ,
    DEFAULT_JOINT_ACCELERATION_RATIO,
    DEFAULT_JOINT_D,
    DEFAULT_JOINT_K,
    DEFAULT_JOINT_VELOCITY_RATIO,
    MarvinHardwareTeleopController,
)


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_SDK_ROOT = PROJECT_ROOT.parent / "TJArm" / "tj_fx_robot-master"
DEFAULT_TOOLS_CONFIG = PROJECT_ROOT.parent / "TJArm" / "tools_cfg.json"
DEFAULT_SCALE_CALIBRATION = PROJECT_ROOT / "logs" / "marvin_scale_calibration.json"
DEFAULT_LOG_DIRECTORY = PROJECT_ROOT / "logs"


def parse_command_line_arguments():
    parser = argparse.ArgumentParser(
        description="Minimal PICO-to-Marvin dual-arm teleoperation"
    )
    parser.add_argument("--enable-hardware", action="store_true")
    parser.add_argument("--confirmed-estop", action="store_true")
    parser.add_argument("--confirmed-joint-mapping", action="store_true")
    parser.add_argument("--confirmed-robot-model", default="")
    parser.add_argument("--robot-ip", default="192.168.1.190")
    parser.add_argument("--sdk-root", type=Path, default=DEFAULT_SDK_ROOT)
    parser.add_argument("--kinematics-config-path", type=Path)
    parser.add_argument(
        "--tools-config",
        type=Path,
        default=DEFAULT_TOOLS_CONFIG,
    )
    parser.add_argument("--scale-factor", type=float)
    parser.add_argument(
        "--scale-calibration-path",
        type=Path,
        default=DEFAULT_SCALE_CALIBRATION,
    )
    parser.add_argument(
        "--log-directory", type=Path, default=DEFAULT_LOG_DIRECTORY
    )
    parser.add_argument("--control-hz", type=float, default=DEFAULT_CONTROL_HZ)
    parser.add_argument("--return-duration", type=float, default=3.0)
    parser.add_argument(
        "--joint-velocity-ratio",
        type=int,
        default=DEFAULT_JOINT_VELOCITY_RATIO,
    )
    parser.add_argument(
        "--joint-acceleration-ratio",
        type=int,
        default=DEFAULT_JOINT_ACCELERATION_RATIO,
    )
    parser.add_argument(
        "--left-k",
        type=float,
        nargs=7,
        default=DEFAULT_JOINT_K,
        metavar=("K1", "K2", "K3", "K4", "K5", "K6", "K7"),
    )
    parser.add_argument(
        "--left-d",
        type=float,
        nargs=7,
        default=DEFAULT_JOINT_D,
        metavar=("D1", "D2", "D3", "D4", "D5", "D6", "D7"),
    )
    parser.add_argument(
        "--right-k",
        type=float,
        nargs=7,
        default=DEFAULT_JOINT_K,
        metavar=("K1", "K2", "K3", "K4", "K5", "K6", "K7"),
    )
    parser.add_argument(
        "--right-d",
        type=float,
        nargs=7,
        default=DEFAULT_JOINT_D,
        metavar=("D1", "D2", "D3", "D4", "D5", "D6", "D7"),
    )
    parser.add_argument(
        "--expected-sdk-version", type=int, default=100343014
    )
    return parser.parse_args(), parser


def main():
    arguments, parser = parse_command_line_arguments()
    missing_confirmations = []
    if not arguments.enable_hardware:
        missing_confirmations.append("--enable-hardware")
    if not arguments.confirmed_estop:
        missing_confirmations.append("--confirmed-estop")
    if not arguments.confirmed_joint_mapping:
        missing_confirmations.append("--confirmed-joint-mapping")
    if not arguments.confirmed_robot_model.strip():
        missing_confirmations.append("--confirmed-robot-model <exact model>")
    if missing_confirmations:
        parser.error(
            "required hardware confirmations: " + ", ".join(missing_confirmations)
        )

    with closing(XrClient()) as xr_client:
        marvin_kinematics = MarvinVendorKinematics(
            arguments.sdk_root,
            arguments.kinematics_config_path,
        )
        active_tool_configurations = load_active_tool_configs(
            arguments.tools_config
        )
        for arm_index, tool_configuration in enumerate(active_tool_configurations):
            marvin_kinematics.set_tool(
                arm_index, tool_configuration.kinematics_mm_deg
            )
        marvin_adapter = MarvinSdkAdapter(
            robot_ip_address=arguments.robot_ip,
            sdk_root_path=arguments.sdk_root,
        )
        session_logger = MarvinSessionLogger(
            arguments.log_directory, "hardware"
        )
        teleop_controller = MarvinHardwareTeleopController(
            xr_client=xr_client,
            adapter=marvin_adapter,
            kinematics=marvin_kinematics,
            scale_calibration_path=arguments.scale_calibration_path,
            initial_pose_q_rad=MARVIN_INITIAL_POSE_Q_RAD,
            tool_configurations=active_tool_configurations,
            left_k=arguments.left_k,
            left_d=arguments.left_d,
            right_k=arguments.right_k,
            right_d=arguments.right_d,
            joint_velocity_ratio=arguments.joint_velocity_ratio,
            joint_acceleration_ratio=arguments.joint_acceleration_ratio,
            requested_scale_factor=arguments.scale_factor,
            control_hz=arguments.control_hz,
            return_duration=arguments.return_duration,
            expected_sdk_version=arguments.expected_sdk_version,
            session_logger=session_logger,
        )
        print(
            f"Confirmed robot model: "
            f"{arguments.confirmed_robot_model}"
        )
        print(f"Translation scale factor: {teleop_controller.scale_factor:.6f}")
        print(f"Session log: {session_logger.path.resolve()}")
        teleop_controller.run()


if __name__ == "__main__":
    main()
