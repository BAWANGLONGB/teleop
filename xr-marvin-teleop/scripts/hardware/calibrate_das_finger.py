#!/usr/bin/env python3
"""Explicit one-finger DAS encoder calibration; never connects Marvin."""

import argparse
import threading
from pathlib import Path

from xr_marvin_teleop.hardware.interface.das_finger import (
    DASFingerCalibrationRequired,
    _decode_encoder_value,
    load_das_finger_configurations,
    load_das_sdk,
)


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_DAS_SDK_ROOT = PROJECT_ROOT.parent / "gen_finger_con_python_sdk_release"


def main():
    parser = argparse.ArgumentParser(description="Calibrate one cleared DAS finger")
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--sdk-root", type=Path, default=DEFAULT_DAS_SDK_ROOT)
    parser.add_argument("--side", choices=("left", "right"), required=True)
    parser.add_argument("--timeout", type=float, default=10.0)
    parser.add_argument("--confirmed-gripper-clear", action="store_true")
    arguments = parser.parse_args()
    if not arguments.confirmed_gripper_clear:
        parser.error("--confirmed-gripper-clear is required")

    configuration = load_das_finger_configurations(arguments.config)[
        0 if arguments.side == "left" else 1
    ]
    valid_feedback = threading.Event()
    calibration_requested = threading.Event()
    result = {"distance_m": None, "error": None}

    def encoder_callback(record_data):
        try:
            result["distance_m"] = _decode_encoder_value(record_data)
            result["error"] = None
            if calibration_requested.is_set():
                valid_feedback.set()
        except DASFingerCalibrationRequired:
            pass
        except Exception as error:
            result["error"] = error
            if calibration_requested.is_set():
                valid_feedback.set()

    data_bus = load_das_sdk(arguments.sdk_root).DataBus(
        tty_port=configuration.serial_port,
        baudrate=921600,
        encoder_freq=30,
        encoder_callback=encoder_callback,
        initial_distance_m=configuration.startup_distance_m,
    )
    try:
        calibration_requested.set()
        data_bus.calib_encoder()
        if not valid_feedback.wait(arguments.timeout):
            raise TimeoutError(
                f"{arguments.side} DAS encoder still returns -66.66 after calibration"
            )
        if result["error"] is not None:
            raise RuntimeError("invalid DAS encoder feedback") from result["error"]
        print(
            f"{arguments.side} DAS encoder calibrated: "
            f"{result['distance_m']:.6f} m"
        )
    finally:
        data_bus.stop()


if __name__ == "__main__":
    main()
