import argparse
from contextlib import closing
from pathlib import Path

from xr_marvin_teleop.common.marvin_postures import MARVIN_INITIAL_POSE_Q_RAD
from xr_marvin_teleop.common.marvin_session_logger import MarvinSessionLogger
from xr_marvin_teleop.common.xr_client import XrClient
from xr_marvin_teleop.hardware.interface.marvin import (
    MarvinSdkAdapter,
    load_active_tool_configs,
    load_modbus_gripper_configurations,
)
from xr_marvin_teleop.hardware.interface.das_finger import (
    DASFingerAdapter,
    load_das_finger_configurations,
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
    DEFAULT_GRIPPER_COMMAND_HZ,
    DEFAULT_GRIPPER_RATE,
    DEFAULT_NSP_ANGLE_RATE_DEG_S,
    DEFAULT_NSP_LATERAL_DEADZONE_M,
    DEFAULT_NSP_LATERAL_FULL_SCALE_M,
    DEFAULT_NSP_LATERAL_MAX_ANGLE_DEG,
    MarvinHardwareTeleopController,
)
from xr_marvin_teleop.ros.telemetry_bridge import Ros2TelemetryBridge
from xr_marvin_teleop.ros.das_client import RosDasClient
from xr_marvin_teleop.ros.pico_client import RosPicoClient


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_SDK_ROOT = PROJECT_ROOT.parent / "TJArm" / "tj_fx_robot-master"
DEFAULT_TOOLS_CONFIG = PROJECT_ROOT.parent / "TJArm" / "tools_cfg.json"
DEFAULT_SCALE_CALIBRATION = PROJECT_ROOT / "logs" / "marvin_scale_calibration.json"
DEFAULT_LOG_DIRECTORY = PROJECT_ROOT / "logs"


def parse_command_line_arguments(arguments=None):
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
    parser.add_argument(
        "--gripper-config",
        type=Path,
        help="validated per-arm Marvin Modbus register and position settings",
    )
    parser.add_argument(
        "--das-gripper-config",
        type=Path,
        help="validated per-arm DAS serial, camera, and distance settings",
    )
    parser.add_argument(
        "--das-sdk-root",
        type=Path,
        default=None,
        help="root of the gen_finger_con_python_sdk_release checkout",
    )
    parser.add_argument(
        "--das-from-ros2",
        action="store_true",
        help="subscribe to an independent DAS source instead of opening its SDK",
    )
    parser.add_argument(
        "--das-command-hz", type=float, default=50.0
    )
    parser.add_argument(
        "--das-ready-timeout", type=float, default=10.0
    )
    parser.add_argument(
        "--ros2",
        action="store_true",
        help="publish native PICO, Marvin, DAS, command, tactile, and image streams",
    )
    parser.add_argument(
        "--pico-from-ros2",
        action="store_true",
        help="subscribe to the independent raw PICO publisher instead of opening SDK",
    )
    parser.add_argument("--pico-topic", default="/raw/pico/frame")
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
    return parser.parse_args(arguments), parser


def main():
    arguments, parser = parse_command_line_arguments()
    if (
        arguments.gripper_config is not None
        and arguments.das_gripper_config is not None
    ):
        parser.error(
            "--gripper-config and --das-gripper-config are mutually exclusive"
        )
    if (
        arguments.das_gripper_config is not None
        and not arguments.das_from_ros2
        and arguments.das_sdk_root is None
    ):
        parser.error("--das-sdk-root is required with --das-gripper-config")
    if arguments.das_from_ros2 and arguments.das_gripper_config is None:
        parser.error("--das-from-ros2 requires --das-gripper-config")
    if arguments.ros2 and not arguments.pico_from_ros2:
        parser.error(
            "--ros2 collection requires the independent PICO source: "
            "start scripts/data/publish_pico.py and add --pico-from-ros2"
        )
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

    xr_source = (
        RosPicoClient(arguments.pico_topic)
        if arguments.pico_from_ros2
        else XrClient()
    )
    with closing(xr_source) as xr_client:
        marvin_kinematics = MarvinVendorKinematics(
            arguments.sdk_root,
            arguments.kinematics_config_path,
        )
        active_tool_configurations = load_active_tool_configs(
            arguments.tools_config
        )
        modbus_gripper_configurations = (
            None
            if arguments.gripper_config is None
            else load_modbus_gripper_configurations(arguments.gripper_config)
        )
        telemetry_bridge = (
            Ros2TelemetryBridge(
                publish_gripper_commands=not arguments.das_from_ros2
            )
            if arguments.ros2
            else None
        )
        das_gripper_adapter = None
        if arguments.das_gripper_config is not None:
            das_gripper_configurations = load_das_finger_configurations(
                arguments.das_gripper_config
            )
            if arguments.das_from_ros2:
                das_gripper_adapter = RosDasClient(
                    das_gripper_configurations,
                    ready_timeout_seconds=arguments.das_ready_timeout,
                )
            else:
                das_gripper_adapter = DASFingerAdapter(
                    das_gripper_configurations,
                    sdk_root_path=arguments.das_sdk_root,
                    command_hz=arguments.das_command_hz,
                    ready_timeout_seconds=arguments.das_ready_timeout,
                    state_callback=(
                        None
                        if telemetry_bridge is None
                        else telemetry_bridge.publish_das_state
                    ),
                    tactile_callback=(
                        None
                        if telemetry_bridge is None
                        else telemetry_bridge.publish_tactile
                    ),
                    frame_callback=(
                        None
                        if telemetry_bridge is None
                        else telemetry_bridge.publish_camera
                    ),
                )
        for arm_index, tool_configuration in enumerate(active_tool_configurations):
            marvin_kinematics.set_tool(
                arm_index, tool_configuration.kinematics_mm_deg
            )
        marvin_adapter = MarvinSdkAdapter(
            robot_ip_address=arguments.robot_ip,
            sdk_root_path=arguments.sdk_root,
            gripper_configurations=modbus_gripper_configurations,
            gripper_adapter=das_gripper_adapter,
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
            telemetry_publisher=telemetry_bridge,
            gripper_control_enabled=(
                modbus_gripper_configurations is not None
                or das_gripper_adapter is not None
            ),
            initial_gripper_closedness=(
                (0.0, 0.0)
                if modbus_gripper_configurations is None
                else tuple(
                    config.initial_closedness
                    for config in modbus_gripper_configurations
                )
            ),
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
        print(
            f"Confirmed robot model: "
            f"{arguments.confirmed_robot_model}"
        )
        print(f"Translation scale factor: {teleop_controller.scale_factor:.6f}")
        print(f"Session log: {session_logger.path.resolve()}")
        teleop_controller.run()


if __name__ == "__main__":
    main()
