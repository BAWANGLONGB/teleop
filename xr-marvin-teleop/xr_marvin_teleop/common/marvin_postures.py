"""Named Marvin joint postures in SDK arm A/B order."""

import numpy as np
"""
    J2>=-90,
    J4 <= 0,
"""

MARVIN_INITIAL_POSE_Q_DEG = np.array(
    [
        90.0,
        -90.0,
        -90.0,
        -20.0,
        90.0,
        0.0,
        0.0,
        -90.0,
        -90.0,
        90.0,
        -20.0,
        -90.0,
        0.0,
        0.0,
    ],
    dtype=float,
)
MARVIN_INITIAL_POSE_Q_RAD = np.deg2rad(MARVIN_INITIAL_POSE_Q_DEG)
MARVIN_INITIAL_POSE_Q_DEG.setflags(write=False)
MARVIN_INITIAL_POSE_Q_RAD.setflags(write=False)
