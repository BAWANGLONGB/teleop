import meshcat.transformations as transformations
import numpy as np


# OpenXR right/up/back -> Marvin +Y/+Z/+X.
OPENXR_TO_MARVIN_ROTATION = np.array(
    [[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]
)


def _rotation_matrix_from_openxr_pose(openxr_pose):
    quaternion_wxyz = np.array(
        [openxr_pose[6], openxr_pose[3], openxr_pose[4], openxr_pose[5]]
    )
    return transformations.quaternion_matrix(quaternion_wxyz)[:3, :3]


def transform_controller_poses_to_marvin_frame(xr_snapshot):
    """Return controller poses in the fixed OpenXR tracking frame."""
    marvin_controller_poses = []
    for controller_pose in (
        xr_snapshot.left_controller_pose,
        xr_snapshot.right_controller_pose,
    ):
        controller_rotation = _rotation_matrix_from_openxr_pose(controller_pose)
        marvin_controller_poses.append(
            (
                OPENXR_TO_MARVIN_ROTATION @ controller_pose[:3],
                OPENXR_TO_MARVIN_ROTATION
                @ controller_rotation
                @ OPENXR_TO_MARVIN_ROTATION.T,
            )
        )
    return tuple(marvin_controller_poses)


class XrTargetMapper:
    """Grip-anchor controller poses to the current Marvin TCP poses."""

    def __init__(self, scale_factor):
        self._controller_pose_anchors = [None, None]
        self._tcp_transform_anchors = [None, None]
        self.scale_factor = scale_factor

    @property
    def scale_factor(self):
        return self._scale_factor

    @scale_factor.setter
    def scale_factor(self, value):
        value = float(value)
        if not np.isfinite(value) or value <= 0.0:
            raise ValueError("scale_factor must be positive and finite")
        self._scale_factor = value

    def reset_arm(self, arm_index=None):
        arm_indices = range(2) if arm_index is None else (arm_index,)
        for index in arm_indices:
            self._controller_pose_anchors[index] = None
            self._tcp_transform_anchors[index] = None

    def map_arm(
        self,
        arm_index,
        controller_pose_marvin,
        current_tcp_transform,
        is_active,
    ):
        if arm_index not in (0, 1):
            raise ValueError("arm_index must be 0 or 1")
        if not is_active:
            self.reset_arm(arm_index)
            return None

        controller_position, controller_rotation = controller_pose_marvin
        current_tcp_transform = np.asarray(current_tcp_transform, dtype=float)
        if current_tcp_transform.shape != (4, 4) or not np.all(
            np.isfinite(current_tcp_transform)
        ):
            raise ValueError("current_tcp_transform must be a finite 4x4 transform")
        if self._controller_pose_anchors[arm_index] is None:
            self._controller_pose_anchors[arm_index] = (
                controller_position.copy(),
                controller_rotation.copy(),
            )
            self._tcp_transform_anchors[arm_index] = current_tcp_transform.copy()
            return current_tcp_transform.copy()

        anchor_position, anchor_rotation = self._controller_pose_anchors[arm_index]
        target_tcp_transform = self._tcp_transform_anchors[arm_index].copy()
        target_tcp_transform[:3, 3] += self.scale_factor * (
            controller_position - anchor_position
        )
        target_tcp_transform[:3, :3] = (
            controller_rotation
            @ anchor_rotation.T
            @ target_tcp_transform[:3, :3]
        )
        return target_tcp_transform
