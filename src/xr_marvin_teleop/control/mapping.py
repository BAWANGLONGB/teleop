import numpy as np


# OpenXR right/up/forward (+X/+Y/-Z) -> Marvin +Y/+Z/-X.
OPENXR_TO_MARVIN_ROTATION = np.array(
    [[0.0, 0.0, 1.0], 
     [1.0, 0.0, 0.0], 
     [0.0, 1.0, 0.0]]
)


def rotation_matrix_from_openxr_pose(openxr_pose):
    quaternion = np.asarray(openxr_pose[3:], dtype=float)
    norm_squared = np.dot(quaternion, quaternion)
    if norm_squared < 4.0 * np.finfo(float).eps:
        return np.eye(3)
    x, y, z, w = quaternion * np.sqrt(2.0 / norm_squared)
    return np.array(
        [
            [1.0 - y * y - z * z, x * y - z * w, x * z + y * w],
            [x * y + z * w, 1.0 - x * x - z * z, y * z - x * w],
            [x * z - y * w, y * z + x * w, 1.0 - x * x - y * y],
        ]
    )


def yaw_rotation_from_openxr_pose(openxr_pose):
    """Return the gravity-aligned heading captured from an OpenXR pose."""
    rotation = rotation_matrix_from_openxr_pose(openxr_pose)
    forward = rotation @ np.array([0.0, 0.0, -1.0])
    forward[1] = 0.0
    norm = np.linalg.norm(forward)
    if norm < 1e-6:
        right = rotation @ np.array([1.0, 0.0, 0.0])
        right[1] = 0.0
        right /= np.linalg.norm(right)
        forward = np.cross(right, np.array([0.0, 1.0, 0.0]))
    else:
        forward /= norm
        right = np.cross(forward, np.array([0.0, 1.0, 0.0]))
    return np.column_stack((right, (0.0, 1.0, 0.0), -forward))


def transform_controller_poses_to_marvin_frame(
    xr_snapshot, reference_rotation=None
):
    """Express controller poses in Marvin axes and an optional captured heading."""
    basis = OPENXR_TO_MARVIN_ROTATION
    if reference_rotation is not None:
        reference_rotation = np.asarray(reference_rotation, dtype=float)
        if reference_rotation.shape != (3, 3) or not np.all(
            np.isfinite(reference_rotation)
        ):
            raise ValueError("reference_rotation must be a finite 3x3 matrix")
        basis = basis @ reference_rotation.T
    marvin_controller_poses = []
    for controller_pose in (
        xr_snapshot.left_controller_pose,
        xr_snapshot.right_controller_pose,
    ):
        controller_rotation = rotation_matrix_from_openxr_pose(controller_pose)
        marvin_controller_poses.append(
            (
                basis @ controller_pose[:3],
                # Change both rotation bases; left multiplication alone is insufficient.
                basis @ controller_rotation @ basis.T,
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
