"""Replay a Marvin JSONL session in MuJoCo."""

import argparse
import time
from pathlib import Path

import numpy as np

from xr_marvin_teleop.common.marvin_session_logger import read_marvin_session
from xr_marvin_teleop.simulation.marvin_mujoco_adapter import (
    MarvinMujocoAdapter,
)


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_XML_PATH = PROJECT_ROOT / "assets" / "marvin" / "marvin_dual.mujoco.xml"


def parse_command_line_arguments():
    parser = argparse.ArgumentParser(description="Replay Marvin JSONL in MuJoCo")
    parser.add_argument("log_path", type=Path)
    parser.add_argument("--xml-path", type=Path, default=DEFAULT_XML_PATH)
    parser.add_argument(
        "--source", choices=("command", "feedback"), default="command"
    )
    parser.add_argument("--speed", type=float, default=1.0)
    parser.add_argument("--max-frames", type=int)
    parser.add_argument("--headless", action="store_true")
    return parser.parse_args()


def main():
    arguments = parse_command_line_arguments()
    if arguments.speed <= 0.0:
        raise ValueError("speed must be positive")
    if arguments.max_frames is not None and arguments.max_frames <= 0:
        raise ValueError("max_frames must be positive")
    records = read_marvin_session(arguments.log_path)
    if not records:
        raise RuntimeError(f"no control_cycle records in {arguments.log_path}")
    field_name = (
        "q_command_rad" if arguments.source == "command" else "q_feedback_rad"
    )
    initial_q_rad = np.asarray(records[0][field_name], dtype=float)
    adapter = MarvinMujocoAdapter(
        arguments.xml_path,
        initial_q_rad,
        launch_viewer=not arguments.headless,
    )
    adapter.connect()
    replayed_frames = 0
    previous_time_ns = None
    try:
        for record in records:
            if arguments.max_frames is not None and replayed_frames >= arguments.max_frames:
                break
            if not adapter.is_running():
                break
            record_time_ns = record["monotonic_time_ns"]
            if previous_time_ns is not None:
                time.sleep(
                    max(0.0, record_time_ns - previous_time_ns)
                    / 1e9
                    / arguments.speed
                )
            previous_time_ns = record_time_ns
            q_rad = np.asarray(record[field_name], dtype=float)
            if arguments.source == "feedback":
                adapter.set_joint_state(
                    q_rad, np.asarray(record["dq_feedback_rad_s"], dtype=float)
                )
            else:
                adapter.send_joint_command(q_rad)
            replayed_frames += 1
    finally:
        adapter.release()
    print(f"Replayed {replayed_frames} frames from {arguments.log_path}")


if __name__ == "__main__":
    main()
