"""SI-unit boundary for the Marvin vendor FK/IK SDK."""

import importlib
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np


MARVIN_CCS_ROBOT_TYPE = 1017
DEFAULT_KINEMATICS_CONFIGURATION = Path(
    "CommonConfig/config/ccs_680.MvKDCfg"
)
MARVIN_ARM_BASE_TO_WORLD_TRANSFORMS = np.array(
    [
        [
            [-1.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, -1.0, -0.06],
            [0.0, -1.0, 0.0, -0.10],
            [0.0, 0.0, 0.0, 1.0],
        ],
        [
            [-1.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.06],
            [0.0, 1.0, 0.0, -0.10],
            [0.0, 0.0, 0.0, 1.0],
        ],
    ]
)


def _validate_homogeneous_transform(transform, field_name):
    transform = np.asarray(transform, dtype=float).copy()
    if transform.shape != (4, 4) or not np.all(np.isfinite(transform)):
        raise ValueError(f"{field_name} must be a finite 4x4 transform")
    if not np.allclose(transform[3], [0.0, 0.0, 0.0, 1.0], atol=1e-9):
        raise ValueError(f"{field_name} must have a homogeneous final row")
    rotation = transform[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6):
        raise ValueError(f"{field_name} rotation must be orthonormal")
    if not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-6):
        raise ValueError(f"{field_name} rotation must be right-handed")
    return transform


def _validate_seven_joint_vector(q_rad, field_name):
    q_rad = np.asarray(q_rad, dtype=float).reshape(-1)
    if q_rad.shape != (7,) or not np.all(np.isfinite(q_rad)):
        raise ValueError(f"{field_name} must be a finite 7-joint vector")
    return q_rad


def _load_marvin_kinematics_module(sdk_root_path, configuration_path=None):
    sdk_root_path = Path(sdk_root_path).expanduser().resolve()
    sdk_python_path = sdk_root_path / "SDK_PYTHON" / "fx_kine.py"
    sdk_library_path = sdk_root_path / "SDK_PYTHON" / "libKine.so"
    configuration_path = (
        sdk_root_path / DEFAULT_KINEMATICS_CONFIGURATION
        if configuration_path is None
        else Path(configuration_path).expanduser().resolve()
    )
    for description, required_path in (
        ("Marvin kinematics SDK", sdk_python_path),
        ("Marvin kinematics library", sdk_library_path),
        ("Marvin kinematics configuration", configuration_path),
    ):
        if not required_path.is_file():
            raise FileNotFoundError(f"{description} not found: {required_path}")

    sdk_root = str(sdk_root_path)
    if sdk_root not in sys.path:
        sys.path.insert(0, sdk_root)
    sdk_module = importlib.import_module("SDK_PYTHON.fx_kine")
    return sdk_module, str(configuration_path)


@dataclass(frozen=True)
class VendorIkResult:
    success: bool
    q_rad: np.ndarray | None
    failure_reason: str | None


class MarvinVendorKinematics:
    """Expose Marvin dual-arm FK/IK in metres and radians."""

    def __init__(
        self,
        sdk_root_path,
        configuration_path=None,
        sdk_module=None,
    ):
        if sdk_module is None:
            sdk_module, configuration_path = _load_marvin_kinematics_module(
                sdk_root_path, configuration_path
            )
        elif configuration_path is None:
            raise ValueError(
                "configuration_path is required when sdk_module is injected"
            )
        self._sdk_module = sdk_module
        self._arm_kinematics = []
        for arm_index in range(2):
            arm_kinematics = sdk_module.Marvin_Kine()
            log_switch = getattr(arm_kinematics, "log_switch", None)
            if log_switch is not None:
                log_switch(0)
            arm_configuration = arm_kinematics.load_config(
                arm_type=arm_index, config_path=str(configuration_path)
            )
            if arm_configuration is None:
                raise RuntimeError(
                    f"failed to load kinematics configuration for arm {arm_index}"
                )
            robot_type = int(arm_configuration["TYPE"][arm_index])
            if robot_type != MARVIN_CCS_ROBOT_TYPE:
                raise RuntimeError(
                    f"expected robot type {MARVIN_CCS_ROBOT_TYPE}, got {robot_type}"
                )
            if not arm_kinematics.initial_kine(
                robot_type=robot_type,
                dh=arm_configuration["DH"][arm_index],
                pnva=arm_configuration["PNVA"][arm_index],
                j67=arm_configuration["BD"][arm_index],
            ):
                raise RuntimeError(
                    f"failed to initialize kinematics for arm {arm_index}"
                )
            self._arm_kinematics.append(arm_kinematics)

    @staticmethod
    def _validate_arm_index(arm):
        if arm not in (0, 1):
            raise ValueError("arm must be 0 or 1")
        return arm

    def set_tool(self, arm, tool_xyzabc_mm_deg):
        arm = self._validate_arm_index(arm)
        tool_xyzabc_mm_deg = np.asarray(tool_xyzabc_mm_deg, dtype=float)
        if tool_xyzabc_mm_deg.shape != (6,) or not np.all(
            np.isfinite(tool_xyzabc_mm_deg)
        ):
            raise ValueError("tool kinematics must be finite XYZABC")
        tool_transform = self._arm_kinematics[arm].xyzabc_to_mat4x4(
            tool_xyzabc_mm_deg.tolist()
        )
        if tool_transform is False or tool_transform is None:
            raise RuntimeError("Marvin tool transform conversion failed")
        if not self._arm_kinematics[arm].set_tool_kine(tool_transform):
            raise RuntimeError("Marvin tool kinematics setup failed")

    def fk_world(self, arm, q_rad):
        arm = self._validate_arm_index(arm)
        q_rad = _validate_seven_joint_vector(q_rad, "q_rad")
        T_arm_tcp_mm = self._arm_kinematics[arm].fk(
            np.rad2deg(q_rad).tolist()
        )
        if T_arm_tcp_mm is False or T_arm_tcp_mm is None:
            raise RuntimeError(f"Marvin FK failed for arm {arm}")
        T_arm_tcp_m = _validate_homogeneous_transform(
            T_arm_tcp_mm, "Marvin FK result"
        )
        T_arm_tcp_m[:3, 3] *= 1e-3
        return MARVIN_ARM_BASE_TO_WORLD_TRANSFORMS[arm] @ T_arm_tcp_m

    def ik_world(self, arm, T_world_tcp_m, q_ref_rad):
        arm = self._validate_arm_index(arm)
        T_world_tcp_m = _validate_homogeneous_transform(
            T_world_tcp_m, "T_world_tcp_m"
        )
        q_ref_rad = _validate_seven_joint_vector(q_ref_rad, "q_ref_rad")
        if np.all(q_ref_rad == 0.0):
            return VendorIkResult(
                False,
                None,
                "reference joints must not all be zero",
            )
        if abs(q_ref_rad[3]) <= np.deg2rad(0.1):
            return VendorIkResult(
                False,
                None,
                "reference joint 4 is at the singular boundary",
            )
        T_arm_tcp_mm = (
            np.linalg.inv(MARVIN_ARM_BASE_TO_WORLD_TRANSFORMS[arm])
            @ T_world_tcp_m
        )
        T_arm_tcp_mm[:3, 3] *= 1e3
        inverse_kinematics_parameters = self._sdk_module.FX_InvKineSolvePara()
        inverse_kinematics_parameters.set_input_ik_target_tcp(
            T_arm_tcp_mm.reshape(-1).tolist()
        )
        inverse_kinematics_parameters.set_input_ik_ref_joint(
            np.rad2deg(q_ref_rad).tolist()
        )
        # Ordinary IK: RefJoint selects the nearest solution and preserves branch continuity.
        inverse_kinematics_parameters.set_input_ik_zsp_type(0)
        raw_result = self._arm_kinematics[arm].ik(
            inverse_kinematics_parameters
        )
        result = (
            inverse_kinematics_parameters
            if raw_result is False or raw_result is None
            else raw_result
        )
        failure_reasons = []
        if raw_result is False or raw_result is None:
            failure_reasons.append("vendor IK call failed")
        if bool(result.m_Output_IsOutRange):
            failure_reasons.append("target outside reachable workspace")
        if any(bool(value) for value in result.m_Output_IsDeg):
            failure_reasons.append("singular joint configuration")
        if bool(result.m_Output_IsJntExd):
            failure_reasons.append("joint limit exceeded")
        if int(result.m_OutPut_Result_Num) <= 0:
            failure_reasons.append("no IK solution")
        q_rad = np.deg2rad(
            np.asarray(result.m_Output_RetJoint.to_list(), dtype=float)
        )
        if q_rad.shape != (7,) or not np.all(np.isfinite(q_rad)):
            failure_reasons.append("invalid IK joint result")
        return VendorIkResult(
            not failure_reasons,
            None if failure_reasons else q_rad,
            "; ".join(failure_reasons) if failure_reasons else None,
        )
