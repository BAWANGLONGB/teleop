"""Explicitly authorized PICO teleoperation entry point for Marvin hardware."""

import hashlib
import os

import numpy as np
import tyro

from xrobotoolkit_teleop.common.marvin_motion_limits import (
    HUMAN_PEAK_TCP_ANGULAR_SPEED_RAD_S,
    HUMAN_PEAK_TCP_LINEAR_SPEED_M_S,
    MARVIN_PEAK_JOINT_ACCELERATION_RAD_S2,
    MARVIN_PEAK_JOINT_JERK_RAD_S3,
    MARVIN_PEAK_JOINT_VELOCITY_RAD_S,
    MARVIN_PEAK_TARGET_NATURAL_FREQUENCY_RAD_S,
)
from xrobotoolkit_teleop.common.marvin_postures import MARVIN_HUMAN_REST_Q_RAD
from xrobotoolkit_teleop.common.marvin_scale_calibration import (
    R_PICO_TO_MARVIN_WORLD,
    resolve_scale_factor,
)
from xrobotoolkit_teleop.hardware.interface.marvin import (
    MarvinSdkAdapter,
    load_active_tool_configs,
    validate_vendor_driver_dependency,
)
from xrobotoolkit_teleop.hardware.marvin_teleop_controller import (
    MarvinHardwareTeleopController,
)
from xrobotoolkit_teleop.utils.path_utils import ASSET_PATH


PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
DEFAULT_SDK_ROOT = os.path.abspath(
    os.path.join(PROJECT_ROOT, "..", "TJArm", "tj_fx_robot-master")
)
DEFAULT_TOOLS_CONFIG = os.path.abspath(
    os.path.join(PROJECT_ROOT, "..", "TJArm", "tools_cfg.json")
)
DEFAULT_SCALE_CALIBRATION = os.path.join(
    PROJECT_ROOT, "logs", "marvin_scale_calibration.json"
)
MARVIN_ASSET_PATH = os.path.join(ASSET_PATH, "marvin")
MARVIN_JOINT_NAMES = [
    *(f"Joint{index}_L" for index in range(1, 8)),
    *(f"Joint{index}_R" for index in range(1, 8)),
]


def _sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as source_file:
        for chunk in iter(lambda: source_file.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main(
    enable_hardware: bool = False,
    confirmed_estop: bool = False,
    confirmed_joint_mapping: bool = False,
    confirmed_startup_path_clear: bool = False,
    confirmed_robot_model: str = "",
    robot_ip: str = "192.168.1.190",
    expected_sdk_version: int = 100343014,
    sdk_root: str = DEFAULT_SDK_ROOT,
    tools_config: str = DEFAULT_TOOLS_CONFIG,
    robot_urdf_path: str = os.path.join(MARVIN_ASSET_PATH, "marvin_dual.urdf"),
    scale_factor: float | None = None,
    scale_calibration_path: str = DEFAULT_SCALE_CALIBRATION,
    enable_release_return: bool = True,
    return_duration: float = 3.0,
    startup_move_duration_s: float = 10.0,
    enable_arm_length_calibration: bool = True,
    calibration_workspace_margin: float = 0.95,
    control_hz: float = 200.0,
    feedback_hz: float = 200.0,
    command_hz: float = 200.0,
    xr_poll_hz: float = 200.0,
    max_joint_velocity: tuple[float, ...] = MARVIN_PEAK_JOINT_VELOCITY_RAD_S,
    max_joint_acceleration: tuple[float, ...] = MARVIN_PEAK_JOINT_ACCELERATION_RAD_S2,
    max_joint_jerk: tuple[float, ...] = MARVIN_PEAK_JOINT_JERK_RAD_S3,
    joint_target_natural_frequency: float = MARVIN_PEAK_TARGET_NATURAL_FREQUENCY_RAD_S,
    max_tcp_displacement_m: float = 0.25,
    max_tcp_linear_speed_m_s: float = HUMAN_PEAK_TCP_LINEAR_SPEED_M_S,
    max_tcp_angular_speed_rad_s: float = HUMAN_PEAK_TCP_ANGULAR_SPEED_RAD_S,
    max_tcp_frame_jump_m: float = 0.15,
    max_tcp_frame_jump_deg: float = 45.0,
    singularity_fault_sigma: float = 0.003,
    singularity_full_speed_sigma: float = 0.015,
    startup_max_joint_speed_rad_s: float = 0.02,
    joint_limit_margin_deg: float = 5.0,
    velocity_ratio: int = 100,
    acceleration_ratio: int = 100,
    left_k: tuple[float, float, float, float, float, float, float] = (
        2.0,
        2.0,
        2.0,
        1.5,
        0.8,
        0.8,
        0.8,
    ),
    left_d: tuple[float, float, float, float, float, float, float] = (0.3,) * 7,
    right_k: tuple[float, float, float, float, float, float, float] = (
        2.0,
        2.0,
        2.0,
        1.5,
        0.8,
        0.8,
        0.8,
    ),
    right_d: tuple[float, float, float, float, float, float, float] = (0.3,) * 7,
    parameter_settle_s: float = 0.2,
    mode_settle_s: float = 1.0,
    pd_settle_s: float = 1.0,
    configure_tools: bool = False,
    enable_ros2_observation: bool = False,
    ros2_namespace: str = "/marvin_teleop",
    ros2_publish_hz: float = 100.0,
    log_dir: str = os.path.join(PROJECT_ROOT, "logs"),
    visualize_placo: bool = False,
):
    """Run Marvin hardware teleoperation after explicit physical safety confirmations."""
    missing = []
    if not enable_hardware:
        missing.append("--enable-hardware")
    if not confirmed_estop:
        missing.append("--confirmed-estop")
    if not confirmed_joint_mapping:
        missing.append("--confirmed-joint-mapping")
    if not confirmed_startup_path_clear:
        missing.append("--confirmed-startup-path-clear")
    if not confirmed_robot_model.strip():
        missing.append("--confirmed-robot-model <exact model>")
    if missing:
        raise PermissionError(
            "hardware remains disabled; required confirmations: " + ", ".join(missing)
        )
    if not 1 <= velocity_ratio <= 100 or not 1 <= acceleration_ratio <= 100:
        raise ValueError("velocity_ratio and acceleration_ratio must be in [1, 100]")

    dependency = validate_vendor_driver_dependency(sdk_root)
    dependency_metadata = {
        **dependency,
        "sdk_python_sha256": _sha256(dependency["sdk_python_path"]),
        "sdk_library_sha256": _sha256(dependency["sdk_library_path"]),
        "robot_config_sha256": _sha256(dependency["robot_config_path"]),
    }

    resolved_scale_factor, scale_factor_metadata = resolve_scale_factor(
        scale_factor,
        scale_calibration_path,
    )

    left_tool, right_tool = load_active_tool_configs(tools_config)
    adapter = MarvinSdkAdapter(robot_ip=robot_ip, sdk_root=sdk_root)
    config = {
        "left_hand": {
            "link_name": "TCP_Link_L",
            "pose_source": "left_controller",
            "control_trigger": "left_grip",
            "manipulability_weight": 0.0,
        },
        "right_hand": {
            "link_name": "TCP_Link_R",
            "pose_source": "right_controller",
            "control_trigger": "right_grip",
            "manipulability_weight": 0.0,
        },
    }
    controller = MarvinHardwareTeleopController(
        adapter=adapter,
        robot_urdf_path=robot_urdf_path,
        manipulator_config=config,
        joint_names=MARVIN_JOINT_NAMES,
        R_headset_world=R_PICO_TO_MARVIN_WORLD,
        scale_factor=resolved_scale_factor,
        reference_mode="head_yaw",
        enable_release_return=enable_release_return,
        return_duration=return_duration,
        startup_move_duration_s=startup_move_duration_s,
        initial_pose_q_rad=MARVIN_HUMAN_REST_Q_RAD,
        enable_arm_length_calibration=enable_arm_length_calibration,
        scale_calibration_path=scale_calibration_path,
        calibration_workspace_margin=calibration_workspace_margin,
        control_hz=control_hz,
        feedback_hz=feedback_hz,
        command_hz=command_hz,
        xr_poll_hz=xr_poll_hz,
        max_joint_velocity=max_joint_velocity,
        max_joint_acceleration=max_joint_acceleration,
        max_joint_jerk=max_joint_jerk,
        joint_target_natural_frequency=joint_target_natural_frequency,
        max_tcp_displacement_m=max_tcp_displacement_m,
        max_tcp_linear_speed_m_s=max_tcp_linear_speed_m_s,
        max_tcp_angular_speed_rad_s=max_tcp_angular_speed_rad_s,
        max_tcp_frame_jump_m=max_tcp_frame_jump_m,
        max_tcp_frame_jump_deg=max_tcp_frame_jump_deg,
        singularity_fault_sigma=singularity_fault_sigma,
        singularity_full_speed_sigma=singularity_full_speed_sigma,
        startup_max_joint_speed_rad_s=startup_max_joint_speed_rad_s,
        joint_limit_margin=np.deg2rad(joint_limit_margin_deg),
        velocity_ratio=velocity_ratio,
        acceleration_ratio=acceleration_ratio,
        left_k=left_k,
        left_d=left_d,
        right_k=right_k,
        right_d=right_d,
        parameter_settle_s=parameter_settle_s,
        mode_settle_s=mode_settle_s,
        pd_settle_s=pd_settle_s,
        left_tool=left_tool,
        right_tool=right_tool,
        configure_tools=configure_tools,
        enable_hardware=True,
        log_dir=log_dir,
        visualize_placo=visualize_placo,
        expected_sdk_version=expected_sdk_version,
        enable_ros2_observation=enable_ros2_observation,
        ros2_namespace=ros2_namespace,
        ros2_publish_hz=ros2_publish_hz,
        session_metadata={
            "confirmed_robot_model": confirmed_robot_model,
            "robot_urdf_path": os.path.abspath(robot_urdf_path),
            "robot_urdf_sha256": _sha256(robot_urdf_path),
            "tools_config_path": os.path.abspath(tools_config),
            "tools_config_sha256": _sha256(tools_config),
            "vendor_driver_dependency": dependency_metadata,
            "scale_factor_source": scale_factor_metadata,
            "confirmed_identity_joint_mapping": {
                "sdk_A": MARVIN_JOINT_NAMES[:7],
                "sdk_B": MARVIN_JOINT_NAMES[7:],
                "sign": [1] * 14,
                "offset_rad": [0.0] * 14,
            },
            "confirmed_startup_path_clear": confirmed_startup_path_clear,
        },
    )

    # Use the same redundancy reference as the MuJoCo natural-rest posture.
    joints_task = controller.solver.add_joints_task()
    joints_task.set_joints(dict(zip(MARVIN_JOINT_NAMES, MARVIN_HUMAN_REST_Q_RAD)))
    joints_task.configure("joints_regularization", "soft", 1e-4)
    print(f"Confirmed robot model: {confirmed_robot_model}")
    print(
        f"Scale factor: {resolved_scale_factor:.6f} "
        f"(source={scale_factor_metadata['source']})"
    )
    controller.run()


if __name__ == "__main__":
    tyro.cli(main)
