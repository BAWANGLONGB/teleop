"""Teleoperate the simulated Tianji Marvin M6S-Lite dual arm from PICO controllers."""

import os

import numpy as np
import tyro

from xrobotoolkit_teleop.common.marvin_scale_calibration import (
    MARVIN_REST_TO_FORWARD_TCP_DELTA,
    MARVIN_REST_TO_FORWARD_TCP_TRAVEL,
    R_PICO_TO_MARVIN_WORLD,
    make_marvin_scale_calibration_config,
)
from xrobotoolkit_teleop.common.marvin_postures import MARVIN_HUMAN_REST_Q_RAD
from xrobotoolkit_teleop.simulation.mujoco_teleop_controller import MujocoTeleopController
from xrobotoolkit_teleop.utils.path_utils import ASSET_PATH


MARVIN_ASSET_PATH = os.path.join(ASSET_PATH, "marvin")
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
MARVIN_JOINT_NAMES = [
    *(f"Joint{index}_L" for index in range(1, 8)),
    *(f"Joint{index}_R" for index in range(1, 8)),
]
MARVIN_JOINT_SPEED_LIMITS = dict(
    zip(MARVIN_JOINT_NAMES, [1.0, 1.0, 1.0, 1.2, 1.2, 1.0, 1.0] * 2)
)
MARVIN_JOINT_ACCELERATION_LIMITS = dict(
    zip(MARVIN_JOINT_NAMES, [3.0, 3.0, 3.0, 4.0, 4.0, 3.0, 3.0] * 2)
)

# Symmetric human-like resting posture: upper arms hang beside the torso,
# elbows bend forward by 20 degrees, and the wrist-roll joints compensate the
# mirrored elbow planes. Joint order is Joint1..Joint7 left, then right.
MARVIN_HUMAN_REST_QPOS = MARVIN_HUMAN_REST_Q_RAD


def main(
    xml_path: str = os.path.join(MARVIN_ASSET_PATH, "marvin_dual.mujoco.xml"),
    robot_urdf_path: str = os.path.join(MARVIN_ASSET_PATH, "marvin_dual.urdf"),
    scale_factor: float = 0.5,
    max_joint_speed: float = 1.2,
    max_joint_acceleration: float = 4.0,
    joint_limit_margin_deg: float = 5.0,
    target_velocity_feedforward: float = 0.8,
    xr_poll_hz: float = 200.0,
    render_hz: float = 60.0,
    telemetry_report_interval: float = 2.0,
    telemetry_output_dir: str = os.path.join(PROJECT_ROOT, "logs"),
    return_duration: float = 3.0,
    enable_arm_length_calibration: bool = True,
    calibration_workspace_margin: float = 0.95,
    visualize_placo: bool = False,
):
    """Run PICO Grip-gated Cartesian teleoperation for the simulated Marvin dual arm."""
    if scale_factor <= 0:
        raise ValueError("scale_factor must be positive")
    if max_joint_speed <= 0:
        raise ValueError("max_joint_speed must be positive")
    if max_joint_acceleration <= 0:
        raise ValueError("max_joint_acceleration must be positive")
    if joint_limit_margin_deg < 0:
        raise ValueError("joint_limit_margin_deg must be non-negative")
    if not 0.0 <= target_velocity_feedforward <= 1.0:
        raise ValueError("target_velocity_feedforward must be in [0, 1]")
    if xr_poll_hz <= 0.0:
        raise ValueError("xr_poll_hz must be positive")
    if render_hz <= 0.0:
        raise ValueError("render_hz must be positive")
    if telemetry_report_interval < 0.0:
        raise ValueError("telemetry_report_interval must be non-negative")
    if return_duration <= 0:
        raise ValueError("return_duration must be positive")
    if not 0.0 < calibration_workspace_margin <= 1.0:
        raise ValueError("calibration_workspace_margin must be in (0, 1]")

    config = {
        "right_hand": {
            "link_name": "TCP_Link_R",
            "mujoco_site_name": "right_tcp",
            "pose_source": "right_controller",
            "control_trigger": "right_grip",
            "vis_target": "right_target",
            "vis_commanded_target": "right_commanded_target",
            # Preserve the specified human-like rest pose exactly at startup;
            # a manipulability objective would otherwise adjust it immediately.
            "manipulability_weight": 0.0,
        },
        "left_hand": {
            "link_name": "TCP_Link_L",
            "mujoco_site_name": "left_tcp",
            "pose_source": "left_controller",
            "control_trigger": "left_grip",
            "vis_target": "left_target",
            "vis_commanded_target": "left_commanded_target",
            "manipulability_weight": 0.0,
        },
    }

    controller = MujocoTeleopController(
        xml_path=xml_path,
        robot_urdf_path=robot_urdf_path,
        manipulator_config=config,
        R_headset_world=R_PICO_TO_MARVIN_WORLD,
        reference_mode="head_yaw",
        return_joint_positions={
            "left_hand": dict(zip(MARVIN_JOINT_NAMES[:7], MARVIN_HUMAN_REST_QPOS[:7])),
            "right_hand": dict(zip(MARVIN_JOINT_NAMES[7:], MARVIN_HUMAN_REST_QPOS[7:])),
        },
        return_duration=return_duration,
        max_joint_speed={
            name: min(limit, max_joint_speed)
            for name, limit in MARVIN_JOINT_SPEED_LIMITS.items()
        },
        max_joint_acceleration={
            name: min(limit, max_joint_acceleration)
            for name, limit in MARVIN_JOINT_ACCELERATION_LIMITS.items()
        },
        joint_limit_margin=np.deg2rad(joint_limit_margin_deg),
        target_velocity_feedforward=target_velocity_feedforward,
        xr_poll_hz=xr_poll_hz,
        render_hz=render_hz,
        telemetry_report_interval=telemetry_report_interval,
        telemetry_output_dir=telemetry_output_dir,
        telemetry_session_name="marvin_latency",
        scale_factor=scale_factor,
        visualize_placo=visualize_placo,
        mj_qpos_init=MARVIN_HUMAN_REST_QPOS,
        viewer_camera="overview",
        scale_calibration_config=(
            {
                "button": "A",
                "cancel_button": "B",
                **make_marvin_scale_calibration_config(calibration_workspace_margin),
            }
            if enable_arm_length_calibration
            else None
        ),
    )

    # Keep the redundant 7-DoF arms near the human-like resting posture while
    # Cartesian TCP tasks have the dominant weight.
    joints_task = controller.solver.add_joints_task()
    joints_task.set_joints(dict(zip(MARVIN_JOINT_NAMES, MARVIN_HUMAN_REST_QPOS)))
    joints_task.configure("joints_regularization", "soft", 1e-4)

    controller.run()


if __name__ == "__main__":
    tyro.cli(main)
