"""Shared human-peak teleoperation limits for the Marvin dual arm."""

import numpy as np


# A measured daily-living reach peaks at about 0.62 m/s.  The corresponding
# elbow peak is not a valid per-joint robot limit, but it is a useful upper
# bound for commanded TCP orientation rate.
HUMAN_PEAK_TCP_LINEAR_SPEED_M_S = 0.62
HUMAN_PEAK_TCP_ANGULAR_SPEED_RAD_S = float(np.deg2rad(122.0))

# Robot-joint limits remain robot-specific.  They are bounded by the Marvin
# limits used by MuJoCo. Joint6 is deliberately derated: its +/-60 degree range
# would otherwise lose too much reachable motion to the full-speed jerk-limited
# stopping guard. This does not lower the Cartesian human peak itself.
MARVIN_PEAK_JOINT_VELOCITY_RAD_S = tuple(
    [1.0, 1.0, 1.0, 1.2, 1.2, 0.35, 1.0] * 2
)
MARVIN_PEAK_JOINT_ACCELERATION_RAD_S2 = tuple(
    [3.0, 3.0, 3.0, 4.0, 4.0, 3.0, 3.0] * 2
)

# Reach peak acceleration in roughly 150-160 ms while retaining an S-curve.
MARVIN_PEAK_JOINT_JERK_RAD_S3 = tuple(
    [20.0, 20.0, 20.0, 25.0, 25.0, 20.0, 20.0] * 2
)
MARVIN_PEAK_TARGET_NATURAL_FREQUENCY_RAD_S = 15.0

# Startup always moves toward the known natural-rest posture along a validated
# joint-space segment. Joint6 can therefore use the model limit during this
# inward move instead of the lower general-teleoperation limit above.
MARVIN_STARTUP_JOINT_VELOCITY_RAD_S = tuple(
    [1.0, 1.0, 1.0, 1.2, 1.2, 1.0, 1.0] * 2
)
MARVIN_STARTUP_JOINT_ACCELERATION_RAD_S2 = MARVIN_PEAK_JOINT_ACCELERATION_RAD_S2
MARVIN_STARTUP_JOINT_JERK_RAD_S3 = MARVIN_PEAK_JOINT_JERK_RAD_S3
