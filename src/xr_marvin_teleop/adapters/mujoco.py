"""Headless or viewed MuJoCo adapter for the Marvin teleoperation loop."""

from pathlib import Path

import mujoco
import numpy as np

from xr_marvin_teleop.hardware.interface.marvin import MarvinRobotState


MARVIN_MUJOCO_JOINT_NAMES = tuple(
    [f"Joint{index}_L" for index in range(1, 8)]
    + [f"Joint{index}_R" for index in range(1, 8)]
)


def _joint_vector(values, field_name):
    values = np.asarray(values, dtype=float).reshape(-1)
    if values.shape != (14,) or not np.all(np.isfinite(values)):
        raise ValueError(f"{field_name} must be a finite 14-joint vector")
    return values


class MarvinMujocoAdapter:
    """Implement the Marvin adapter contract with MuJoCo position actuators."""

    def __init__(
        self,
        xml_path,
        initial_q_rad,
        control_hz=50.0,
        launch_viewer=True,
    ):
        self.model = mujoco.MjModel.from_xml_path(
            str(Path(xml_path).expanduser().resolve())
        )
        self.data = mujoco.MjData(self.model)
        self._initial_q_rad = _joint_vector(initial_q_rad, "initial_q_rad").copy()
        physics_steps = (1.0 / float(control_hz)) / float(self.model.opt.timestep)
        if not np.isclose(physics_steps, round(physics_steps)):
            raise ValueError(
                "control period must be an integer multiple of the MuJoCo timestep"
            )
        self._physics_steps = int(round(physics_steps))
        self._qpos_addresses = []
        self._dof_addresses = []
        self._actuator_ids = []
        for joint_name in MARVIN_MUJOCO_JOINT_NAMES:
            joint_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name
            )
            actuator_id = mujoco.mj_name2id(
                self.model,
                mujoco.mjtObj.mjOBJ_ACTUATOR,
                f"{joint_name}_position",
            )
            if joint_id < 0 or actuator_id < 0:
                raise ValueError(f"MuJoCo model is missing {joint_name}")
            self._qpos_addresses.append(int(self.model.jnt_qposadr[joint_id]))
            self._dof_addresses.append(int(self.model.jnt_dofadr[joint_id]))
            self._actuator_ids.append(actuator_id)
        self._launch_viewer = bool(launch_viewer)
        self._viewer = None
        self._is_connected = False
        self._is_released = False
        self._arm_state = (0, 0)
        self._frame_serial = 0
        self._gripper_closedness = np.zeros(2)

    def _sync_viewer(self):
        if self._viewer is not None:
            self._viewer.sync()

    def connect(self):
        if self._is_connected:
            return
        if self._is_released:
            raise RuntimeError("a released MuJoCo adapter cannot reconnect")
        self.set_joint_state(self._initial_q_rad)
        for actuator_id, value in zip(self._actuator_ids, self._initial_q_rad):
            self.data.ctrl[actuator_id] = value
        if self._launch_viewer:
            from mujoco import viewer

            self._viewer = viewer.launch_passive(self.model, self.data)
        self._is_connected = True
        self._sync_viewer()

    def sdk_version(self):
        return None

    def read_state(self):
        self._frame_serial += 1
        q_rad = np.array(
            [self.data.qpos[address] for address in self._qpos_addresses]
        )
        dq_rad_s = np.array(
            [self.data.qvel[address] for address in self._dof_addresses]
        )
        return MarvinRobotState(
            frame_serial=(self._frame_serial, self._frame_serial),
            q_rad=q_rad,
            dq_rad_s=dq_rad_s,
            arm_state=self._arm_state,
            error_code=(0, 0),
            low_speed=tuple(
                bool(np.max(np.abs(dq_rad_s[index * 7:(index + 1) * 7])) < 1e-3)
                for index in range(2)
            ),
        )

    def wait_for_fresh_feedback(self, **_unused):
        return self.read_state()

    def configure_control_parameters(self, *_unused, **_unused_named):
        return None

    def enter_joint_impedance(self):
        self._arm_state = (3, 3)

    def enable_pd_feedforward(self, _period_milliseconds):
        return None

    def send_joint_command(self, q_rad, wait_response=False):
        del wait_response
        if not self._is_connected:
            raise RuntimeError("MuJoCo adapter is not connected")
        q_rad = _joint_vector(q_rad, "q_rad")
        for actuator_id, value in zip(self._actuator_ids, q_rad):
            self.data.ctrl[actuator_id] = value
        for _ in range(self._physics_steps):
            mujoco.mj_step(self.model, self.data)
        self._sync_viewer()

    def send_gripper_command(self, closedness):
        closedness = np.asarray(closedness, dtype=float).reshape(-1)
        if (
            closedness.shape != (2,)
            or not np.all(np.isfinite(closedness))
            or np.any(closedness < 0.0)
            or np.any(closedness > 1.0)
        ):
            raise ValueError(
                "gripper closedness must contain two values within [0, 1]"
            )
        self._gripper_closedness = closedness.copy()

    @property
    def gripper_closedness(self):
        return tuple(self._gripper_closedness)

    def set_joint_state(self, q_rad, dq_rad_s=None):
        q_rad = _joint_vector(q_rad, "q_rad")
        dq_rad_s = (
            np.zeros(14)
            if dq_rad_s is None
            else _joint_vector(dq_rad_s, "dq_rad_s")
        )
        for q_address, dq_address, q_value, dq_value in zip(
            self._qpos_addresses,
            self._dof_addresses,
            q_rad,
            dq_rad_s,
        ):
            self.data.qpos[q_address] = q_value
            self.data.qvel[dq_address] = dq_value
        mujoco.mj_forward(self.model, self.data)
        self._sync_viewer()

    def set_idle(self):
        self._arm_state = (0, 0)
        return True

    def is_running(self):
        return self._viewer is None or self._viewer.is_running()

    def release(self):
        if not self._is_released:
            self._is_released = True
            if self._viewer is not None:
                self._viewer.close()
                self._viewer = None
            self._is_connected = False
