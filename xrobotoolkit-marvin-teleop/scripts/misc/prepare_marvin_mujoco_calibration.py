"""Validate a Marvin calibration CSV and export dense NumPy arrays for fitting."""

from __future__ import annotations

import csv
import json
from pathlib import Path

import numpy as np
import tyro


JOINT_NAMES = [
    *(f"Joint{index}_L" for index in range(1, 8)),
    *(f"Joint{index}_R" for index in range(1, 8)),
]


def _columns(prefix):
    return [f"{prefix}_{name}" for name in JOINT_NAMES]


def _transform_columns(kind, arm):
    return [f"{kind}_{arm}_T_{row}{column}" for row in range(4) for column in range(4)]


def _float_matrix(rows, columns):
    return np.asarray(
        [[float(row[column]) if row[column] else np.nan for column in columns] for row in rows],
        dtype=float,
    )


def convert(csv_path, output_path=None):
    csv_path = Path(csv_path).expanduser().resolve()
    if output_path is None:
        output_path = csv_path.with_suffix(".npz")
    else:
        output_path = Path(output_path).expanduser().resolve()
    with csv_path.open(newline="", encoding="utf-8") as csv_file:
        reader = csv.DictReader(csv_file)
        rows = list(reader)
        fieldnames = set(reader.fieldnames or ())
    if len(rows) < 2:
        raise ValueError("calibration CSV must contain at least two feedback samples")

    required = {
        "host_monotonic_ns",
        "frame_serial_A",
        "frame_serial_B",
        "arm_state_A",
        "arm_state_B",
        "error_code_A",
        "error_code_B",
        "safety_state",
        *_columns("q_rad"),
        *_columns("dq_rad_s"),
        *_columns("tau_nm"),
        *_columns("controller_q_command_rad"),
        *_columns("software_q_command_rad"),
        *_columns("q_ik_rad"),
    }
    for kind in ("raw_target", "limited_target", "actual"):
        for arm in ("left", "right"):
            required.update(_transform_columns(kind, arm))
    missing = sorted(required - fieldnames)
    if missing:
        raise ValueError(f"calibration CSV is missing required columns: {missing}")

    monotonic_ns = np.asarray([int(row["host_monotonic_ns"]) for row in rows], dtype=np.int64)
    if np.any(np.diff(monotonic_ns) <= 0):
        raise ValueError("host_monotonic_ns must be strictly increasing")
    time_s = (monotonic_ns - monotonic_ns[0]).astype(float) / 1e9
    q = _float_matrix(rows, _columns("q_rad"))
    dq = _float_matrix(rows, _columns("dq_rad_s"))
    tau = _float_matrix(rows, _columns("tau_nm"))
    controller_q_command = _float_matrix(rows, _columns("controller_q_command_rad"))
    software_q_command = _float_matrix(rows, _columns("software_q_command_rad"))
    q_ik = _float_matrix(rows, _columns("q_ik_rad"))
    ddq = np.gradient(dq, time_s, axis=0, edge_order=1)
    arm_state = np.asarray(
        [[int(row["arm_state_A"]), int(row["arm_state_B"])] for row in rows],
        dtype=np.int16,
    )
    error_code = np.asarray(
        [[int(row["error_code_A"]), int(row["error_code_B"])] for row in rows],
        dtype=np.int64,
    )
    frame_serial = np.asarray(
        [[int(row["frame_serial_A"]), int(row["frame_serial_B"])] for row in rows],
        dtype=np.int64,
    )
    safety_state = np.asarray([row["safety_state"] for row in rows], dtype="U16")
    valid_feedback = (
        np.all(error_code == 0, axis=1)
        & np.all(np.isfinite(q), axis=1)
        & np.all(np.isfinite(dq), axis=1)
        & np.all(np.isfinite(tau), axis=1)
    )
    valid_dynamics = (
        valid_feedback
        & np.all(arm_state == 3, axis=1)
        & np.all(np.isfinite(controller_q_command), axis=1)
        & np.isin(safety_state, ("armed", "teleop"))
    )
    valid_tracking = (
        valid_dynamics
        & np.all(np.isfinite(software_q_command), axis=1)
        & np.all(np.isfinite(q_ik), axis=1)
    )

    transforms = {}
    for kind in ("raw_target", "limited_target", "actual"):
        values = []
        for arm in ("left", "right"):
            values.append(_float_matrix(rows, _transform_columns(kind, arm)).reshape(-1, 4, 4))
        transforms[kind] = np.stack(values, axis=1)

    metadata_path = csv_path.with_suffix(".metadata.json")
    metadata_json = ""
    if metadata_path.is_file():
        metadata_json = json.dumps(
            json.loads(metadata_path.read_text(encoding="utf-8")),
            ensure_ascii=False,
        )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        output_path,
        time_s=time_s,
        host_monotonic_ns=monotonic_ns,
        frame_serial=frame_serial,
        q_rad=q,
        dq_rad_s=dq,
        ddq_rad_s2=ddq,
        tau_nm=tau,
        controller_q_command_rad=controller_q_command,
        software_q_command_rad=software_q_command,
        q_ik_rad=q_ik,
        arm_state=arm_state,
        error_code=error_code,
        safety_state=safety_state,
        valid_feedback_mask=valid_feedback,
        valid_dynamics_mask=valid_dynamics,
        valid_tracking_mask=valid_tracking,
        raw_target_transform=transforms["raw_target"],
        limited_target_transform=transforms["limited_target"],
        actual_tcp_transform=transforms["actual"],
        joint_names=np.asarray(JOINT_NAMES, dtype="U16"),
        source_csv=str(csv_path),
        metadata_json=metadata_json,
    )
    return output_path, int(np.count_nonzero(valid_dynamics)), len(rows)


def main(csv_path: str, output_path: str | None = None):
    """Prepare one recorded hardware session for offline MuJoCo parameter fitting."""
    output, valid_count, total_count = convert(csv_path, output_path)
    print(f"Calibration dataset: {output}")
    print(f"Valid dynamics samples: {valid_count}/{total_count}")


if __name__ == "__main__":
    tyro.cli(main)
