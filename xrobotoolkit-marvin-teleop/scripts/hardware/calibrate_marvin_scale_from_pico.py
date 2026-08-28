"""Calibrate Marvin translation scale from PICO without connecting to the robot."""

from __future__ import annotations

import os
import time

import numpy as np
import tyro

from xrobotoolkit_teleop.common.arm_length_calibration import ArmLengthScaleCalibrator
from xrobotoolkit_teleop.common.buffered_xr_client import BufferedXrClient
from xrobotoolkit_teleop.common.marvin_scale_calibration import (
    controller_positions_in_marvin_head_yaw_frame,
    make_marvin_scale_calibration_config,
    save_scale_calibration,
)
from xrobotoolkit_teleop.common.xr_client import XrClient


PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
DEFAULT_OUTPUT_PATH = os.path.join(PROJECT_ROOT, "logs", "marvin_scale_calibration.json")


def main(
    output_path: str = DEFAULT_OUTPUT_PATH,
    xr_poll_hz: float = 200.0,
    max_xr_age_ms: float = 100.0,
    workspace_margin: float = 0.95,
):
    """Save a two-point PICO scale; this process never opens the Marvin SDK."""
    if xr_poll_hz <= 0.0:
        raise ValueError("xr_poll_hz must be positive")
    if max_xr_age_ms <= 0.0:
        raise ValueError("max_xr_age_ms must be positive")
    if not 0.0 < workspace_margin <= 1.0:
        raise ValueError("workspace_margin must be in (0, 1]")

    calibrator = ArmLengthScaleCalibrator(
        **make_marvin_scale_calibration_config(workspace_margin)
    )
    client = BufferedXrClient(
        XrClient(),
        pose_names=("headset", "left_controller", "right_controller"),
        key_names=("left_grip", "right_grip"),
        button_names=("A", "B"),
        poll_hz=xr_poll_hz,
    )
    previous_a = False
    previous_b = False
    yaw_rotation = np.eye(3)
    print("PICO-only scale calibration: Marvin SDK will not be opened.")
    print("Release both Grip buttons. Natural-down: press A; forward-straight: press A again.")
    print("Press B to reset the first sample; Ctrl+C exits without changing the saved scale.")
    try:
        client.start()
        if not client.wait_until_ready(timeout=2.0):
            raise RuntimeError("PICO produced no valid XR snapshot within 2 seconds")
        while True:
            client.begin_cycle()
            try:
                a_pressed = bool(client.get_button_state_by_name("A"))
                b_pressed = bool(client.get_button_state_by_name("B"))
                a_edge = a_pressed and not previous_a
                b_edge = b_pressed and not previous_b
                previous_a = a_pressed
                previous_b = b_pressed

                if b_edge:
                    calibrator.reset()
                    print("Calibration reset; capture the natural-down pose again.")

                if a_edge:
                    diagnostics = client.get_diagnostics()
                    if diagnostics["source_age_ms"] > max_xr_age_ms:
                        print(
                            f"Sample rejected: XR source age {diagnostics['source_age_ms']:.1f} ms "
                            f"exceeds {max_xr_age_ms:.1f} ms."
                        )
                        continue
                    grips = (
                        client.get_key_value_by_name("left_grip"),
                        client.get_key_value_by_name("right_grip"),
                    )
                    if max(grips) > 0.1:
                        print("Sample rejected: release both Grip buttons before pressing A.")
                        continue
                    positions, yaw_rotation = controller_positions_in_marvin_head_yaw_frame(
                        client.get_pose_by_name("headset"),
                        client.get_pose_by_name("left_controller"),
                        client.get_pose_by_name("right_controller"),
                        yaw_rotation,
                    )
                    result = calibrator.capture(positions)
                    print(result.message)
                    if result.controller_travels is not None:
                        print(
                            "Controller travel: "
                            + ", ".join(
                                f"{name}={value:.3f} m"
                                for name, value in result.controller_travels.items()
                            )
                        )
                    if result.arm_lengths is not None:
                        print(
                            "Estimated arm length: "
                            + ", ".join(
                                f"{name}={value:.3f} m"
                                for name, value in result.arm_lengths.items()
                            )
                        )
                    if result.status == "completed":
                        current_path, history_path = save_scale_calibration(
                            output_path,
                            result,
                            workspace_margin,
                        )
                        print(f"Saved scale_factor={result.scale_factor:.6f}")
                        print(f"Current calibration: {current_path}")
                        print(f"Calibration history: {history_path}")
                        print(
                            "Calibration complete. Exit now and restart the hardware teleop; "
                            "it will load this scale before connecting to Marvin."
                        )
                        return
            finally:
                client.end_cycle()
            time.sleep(min(0.01, 0.5 / xr_poll_hz))
    except KeyboardInterrupt:
        print("\nCalibration cancelled; saved scale was not changed.")
    finally:
        client.close()


if __name__ == "__main__":
    tyro.cli(main)
