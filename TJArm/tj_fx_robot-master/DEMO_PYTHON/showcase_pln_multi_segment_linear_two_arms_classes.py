#!/usr/bin/env python3
"""统一接口的双臂多段 MOVL 实机控制与 MuJoCo 回放。"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from SDK_PYTHON.fx_kine import Marvin_Kine  # noqa: E402


DEFAULT_CONFIG = ROOT / "CommonConfig/config/ccs_680.MvKDCfg"
DEFAULT_MODEL = ROOT / "DEMO_PYTHON/striding_doc/marvin_m6s_lite_dual_ccs_680.xml"
DEFAULT_REFERENCE = {
    "L": [17.970, -35.197, 11.414, -73.344, -9.154, -17.035, 7.086],
    "R": [-17.970, -35.197, -11.414, -73.344, 9.154, -17.035, -7.086],
}


class _RobotBase:
    """两种后端共用同一套 SDK 运动学规划。"""

    def __init__(
        self,
        *,
        config: Path = DEFAULT_CONFIG,
        model: Path = DEFAULT_MODEL,
        ip: str = "192.168.1.190",
        velocity: float = 100.0,
        acceleration: float = 100.0,
        frequency: int = 50,
    ):
        self.config = Path(config)
        self.model_path = Path(model)
        self.ip = ip
        self.velocity = float(velocity)
        self.acceleration = float(acceleration)
        self.frequency = int(frequency)
        self.allow_range = 5.0
        self.zsp_type = 1
        self.zsp_params = [0.0, 0.0, -1.0, 0.0, 0.0, 0.0]

        self.kines = (self._make_kine(0), self._make_kine(1))
        self.left_points = None
        self.right_points = None
        self.left_joints = None
        self.right_joints = None
        self.point_sets = [None, None]
        self.pose_initialized = False

    def _make_kine(self, arm_type: int) -> Marvin_Kine:
        kine = Marvin_Kine()
        kine.log_switch(0)
        cfg = kine.load_config(arm_type=arm_type, config_path=str(self.config))
        if cfg is None or not kine.initial_kine(
            cfg["TYPE"][arm_type], cfg["DH"][arm_type],
            cfg["PNVA"][arm_type], cfg["BD"][arm_type],
        ):
            raise RuntimeError(f"failed to initialize arm {arm_type}")
        return kine

    def _plan_arm(self, index: int, points: np.ndarray) -> tuple[np.ndarray, object]:
        kine = self.kines[index]
        side = "L" if index == 0 else "R"
        if not kine.multi_movL_set_start(
            DEFAULT_REFERENCE[side], points[0].tolist(), points[1].tolist(),
            self.allow_range, self.zsp_type, self.zsp_params,
            self.velocity, self.acceleration, self.frequency,
        ):
            raise RuntimeError(f"{side} multi_movL_set_start failed")
        for point in points[2:]:
            if not kine.multi_movL_next_point(
                point.tolist(), self.allow_range, self.zsp_type,
                self.zsp_params, self.velocity, self.acceleration,
            ):
                raise RuntimeError(f"{side} multi_movL_next_point failed")
        values, point_set = kine.multi_movL_get_points()
        if point_set is None or not values:
            raise RuntimeError(f"{side} multi_movL_get_points failed")
        return np.asarray(values, dtype=float), point_set

    def plan(self, left_points, right_points) -> tuple[np.ndarray, np.ndarray]:
        """输入双臂 N×6 笛卡尔点，生成两侧关节轨迹。"""
        left = np.asarray(left_points, dtype=float)
        right = np.asarray(right_points, dtype=float)
        for name, points in (("left_points", left), ("right_points", right)):
            if points.ndim != 2 or points.shape[1] != 6 or len(points) < 2:
                raise ValueError(f"{name} must have shape (N, 6), N >= 2")
            if not np.isfinite(points).all():
                raise ValueError(f"{name} contains non-finite values")
        if len(left) != len(right):
            raise ValueError("left_points and right_points must have equal length")

        # 重复调用 plan() 时先释放上一条 SDK 轨迹。
        self._destroy_point_sets()
        self.left_points = left
        self.right_points = right
        self.left_joints = None
        self.right_joints = None
        self.pose_initialized = False
        try:
            self.left_joints, self.point_sets[0] = self._plan_arm(0, left)
            self.right_joints, self.point_sets[1] = self._plan_arm(1, right)
            if len(self.left_joints) != len(self.right_joints):
                raise RuntimeError(
                    f"left/right point counts differ: "
                    f"{len(self.left_joints)} != {len(self.right_joints)}"
                )
        except Exception:
            self._destroy_point_sets()
            self.left_joints = None
            self.right_joints = None
            raise
        return self.left_joints, self.right_joints

    def _first_joints(self) -> tuple[np.ndarray, np.ndarray]:
        if self.left_joints is None or self.right_joints is None:
            raise RuntimeError("call plan(left_points, right_points) first")
        return self.left_joints[0].copy(), self.right_joints[0].copy()

    def _resolve_default_pose(
        self, left_joints=None, right_joints=None,
    ) -> tuple[np.ndarray, np.ndarray]:
        if left_joints is None and right_joints is None:
            return self._first_joints()
        if left_joints is None or right_joints is None:
            raise ValueError("left_joints and right_joints must be set together")

        left = np.asarray(left_joints, dtype=float)
        right = np.asarray(right_joints, dtype=float)
        for name, joints in (("left_joints", left), ("right_joints", right)):
            if joints.shape != (7,):
                raise ValueError(f"{name} must have shape (7,)")
            if not np.isfinite(joints).all():
                raise ValueError(f"{name} contains non-finite values")
        return left.copy(), right.copy()

    def _require_ready(self) -> None:
        self._first_joints()
        if not self.pose_initialized:
            raise RuntimeError("call default_pose() before run()")

    def _destroy_point_sets(self) -> None:
        for index, point_set in enumerate(self.point_sets):
            if point_set is not None:
                self.kines[index].destroy_point_set(point_set)
                self.point_sets[index] = None


class RobotControl(_RobotBase):
    """实机后端。"""

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.robot = None
        self.dcss = None

    def _connect(self) -> None:
        if self.robot is not None:
            return
        from SDK_PYTHON.fx_robot import Marvin_Robot, DCSS

        self.robot = Marvin_Robot()
        self.dcss = DCSS()
        if self.robot.connect(self.ip) == 0:
            self.robot = None
            raise RuntimeError(f"failed to connect robot: {self.ip}")
        self.robot.check_error_and_clear(self.dcss)
        self.robot.clear_set()
        self.robot.set_state(arm="A", state=1)
        self.robot.set_state(arm="B", state=1)
        if self.robot.send_cmd_wait_response(100) < 0:
            raise RuntimeError("failed to enter position mode")

    def default_pose(
        self, left_joints=None, right_joints=None,
    ) -> tuple[np.ndarray, np.ndarray]:
        left, right = self._resolve_default_pose(left_joints, right_joints)
        self._connect()
        self.robot.clear_set()
        time.sleep(0.2)

        #### 去初始位置降低速度
        self.robot.set_vel_acc(arm='A', velRatio=10, AccRatio=10)
        self.robot.set_vel_acc(arm='B', velRatio=10, AccRatio=10)

        self.robot.set_joint_cmd_pose(arm="A", joints=left.tolist())
        self.robot.set_joint_cmd_pose(arm="B", joints=right.tolist())
        time.sleep(0.2)

        self.robot.send_cmd()
        time.sleep(0.2)

        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline:
            data = self.robot.subscribe(self.dcss)
            error = max(
                np.max(np.abs(np.asarray(data["outputs"][0]["fb_joint_pos"]) - left)),
                np.max(np.abs(np.asarray(data["outputs"][1]["fb_joint_pos"]) - right)),
            )
            if error < 0.1:
                self.pose_initialized = True
                return left, right
            time.sleep(0.05)
        time.sleep(2)
        raise TimeoutError("robot did not reach the first trajectory point")

    def run(self) -> None:
        self._require_ready()
        if not self.robot.setPln_Cart_AB(self.point_sets[0], self.point_sets[1]):
            raise RuntimeError("setPln_Cart_AB failed")
        try:
            while True:
                data = self.robot.subscribe(self.dcss)
                if all(output["traj_state"] == b"\x00" for output in data["outputs"][:2]):
                    return
                print(data['outputs'][0]['est_cart_fn'], data['outputs'][0]['est_joint_force'])
                
                time.sleep(0.05)
        except KeyboardInterrupt:
            self.stop()

    def stop(self) -> None:
        if self.robot is not None:
            self.robot.stopPln_AB()

    def close(self) -> None:
        if self.robot is not None:
            self.stop()
            time.sleep(0.5)

            self.robot.clear_set()
            self.robot.set_state(arm='A', state=0)
            self.robot.set_state(arm='B', state=0)
            self.robot.send_cmd()
            time.sleep(0.5)
            print("clear_set reset done")
            
            self.robot.release_robot()
            self.robot = None
        self._destroy_point_sets()


class RobotSim(_RobotBase):
    """MuJoCo 关节轨迹回放后端。"""

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.model = None
        self.data = None
        self.addresses = None
        self.running = False

    def _load_model(self) -> None:
        if self.model is not None:
            return
        import mujoco

        self.model = mujoco.MjModel.from_xml_path(str(self.model_path))
        self.data = mujoco.MjData(self.model)
        self.addresses = {}
        for side in ("L", "R"):
            ids = [mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_JOINT, f"Joint{i}_{side}"
            ) for i in range(1, 8)]
            if min(ids) < 0:
                raise ValueError(f"MuJoCo {side} arm joints not found")
            self.addresses[side] = [int(self.model.jnt_qposadr[i]) for i in ids]
        if self.model.nmocap:
            self.data.mocap_pos[:] = (0.0, 0.0, -100.0)

    def _apply_joints(self, left: np.ndarray, right: np.ndarray) -> None:
        import mujoco

        self.data.qpos[self.addresses["L"]] = np.deg2rad(left)
        self.data.qpos[self.addresses["R"]] = np.deg2rad(right)
        mujoco.mj_forward(self.model, self.data)

    def _apply(self, index: int) -> None:
        self._apply_joints(self.left_joints[index], self.right_joints[index])

    def default_pose(
        self, left_joints=None, right_joints=None,
    ) -> tuple[np.ndarray, np.ndarray]:
        left, right = self._resolve_default_pose(left_joints, right_joints)
        self._load_model()
        self._apply_joints(left, right)
        self.pose_initialized = True
        return left, right

    def run(self) -> None:
        self._require_ready()
        from mujoco import viewer

        self.running = True
        with viewer.launch_passive(self.model, self.data) as window:
            window.cam.lookat[:] = (0.25, 0.0, -0.25)
            window.cam.distance = 1.9
            window.cam.azimuth = 135.0
            window.cam.elevation = -25.0
            start = time.monotonic()
            while self.running and window.is_running():
                index = int((time.monotonic() - start) * self.frequency)
                if index >= len(self.left_joints):
                    return
                self._apply(index)
                window.sync()
                time.sleep(1.0 / 60.0)

    def stop(self) -> None:
        self.running = False

    def close(self) -> None:
        self.stop()
        self._destroy_point_sets()
        self.model = None
        self.data = None


LEFT_POINTS = [
    [509.731, 233.614, 265.949, -169.144, 55.011, -146.752],
    [509.731, 233.614, 205.949, -169.144, 55.011, -146.752],
    [509.731, 133.614, 205.949, -169.144, 55.011, -146.752],
    [509.731, 133.614, 265.949, -169.144, 55.011, -146.752],
]
RIGHT_POINTS = [
    [509.731, -233.614, 265.949, 169.144, 55.011, 146.752],
    [509.731, -233.614, 205.949, 169.144, 55.011, 146.752],
    [509.731, -133.614, 205.949, 169.144, 55.011, 146.752],
    [509.731, -133.614, 265.949, 169.144, 55.011, 146.752],
]


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="双臂多段 MOVL")
    parser.add_argument("mode", choices=("sim", "real"), help="仿真或真机")
    return parser.parse_args(argv)


def create_robot(mode: str):
    if mode == "sim":
        robot_type = RobotSim
    elif mode == "real":
        robot_type = RobotControl
    else:
        raise ValueError("mode must be 'sim' or 'real'")
    return robot_type(
        ip="192.168.1.190",
        velocity=100,
        acceleration=100,
        frequency=50,
    )


def main(argv=None) -> None:
    args = parse_args(argv)
    robot = create_robot(args.mode)

    # 规划轨迹
    robot.plan(LEFT_POINTS, RIGHT_POINTS)

    # 走到初始位置: 
    robot.default_pose()  

    # 运行轨迹
    try:
        robot.run()
    finally:
        robot.close()


if __name__ == "__main__":
    main()
