"""Named Marvin joint postures shared by simulation and hardware."""

from __future__ import annotations

import numpy as np


MARVIN_HUMAN_REST_Q_DEG = (
    90.0,
    -90.0,
    90.0,
    20.0,
    -90.0,
    0.0,
    0.0,
    -90.0,
    -90.0,
    -90.0,
    20.0,
    90.0,
    0.0,
    0.0,
)

MARVIN_HUMAN_REST_Q_RAD = np.deg2rad(MARVIN_HUMAN_REST_Q_DEG)
MARVIN_HUMAN_REST_Q_RAD.setflags(write=False)
