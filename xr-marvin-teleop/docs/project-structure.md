# 项目结构

## 1. 目录

```text
xr-marvin-teleop/
├── README.md
├── pyproject.toml
├── setup.py
├── native/
│   └── xrobotoolkit_sdk.cpp
├── docs/
│   ├── 首次部署.md
│   ├── 操作指南.md
│   └── project-structure.md
├── assets/marvin/
│   ├── marvin_dual.mujoco.xml
│   ├── marvin_dual.urdf
│   ├── marvin_dual.manifest.json
│   └── meshes/
├── scripts/
│   ├── hardware/
│   │   └── teleop_marvin_hardware.py
│   └── simulation/
│       ├── teleop_marvin_mujoco.py
│       └── replay_marvin_log.py
├── tests/
│   └── test_marvin_hardware.py
└── xr_marvin_teleop/
    ├── common/
    │   ├── marvin_postures.py
    │   ├── marvin_scale_calibration.py
    │   ├── marvin_session_logger.py
    │   ├── xr_client.py
    │   └── xr_target_mapper.py
    ├── hardware/
    │   ├── marvin_teleop_controller.py
    │   └── interface/
    │       ├── marvin.py
    │       └── marvin_kinematics.py
    └── simulation/
        └── marvin_mujoco_adapter.py
```

`__pycache__/`、`*.egg-info/` 和 `logs/` 是运行生成内容，不属于核心源码结构。

## 2. 模块职责

| 模块 | 职责 |
| --- | --- |
| `native/xrobotoolkit_sdk.cpp` | 在 SDK JSON 回调中组装整帧，并通过一个 mutex 原子发布 |
| `common/xr_client.py` | 读取原子快照，检查时间戳新鲜度和断流状态 |
| `common/xr_target_mapper.py` | OpenXR → Marvin 坐标转换、Grip 锚点和 scale 位姿映射 |
| `common/marvin_scale_calibration.py` | A/A 两点在线臂长 scale 标定、保存与读取 |
| `common/marvin_postures.py` | B 键回位的 A/B 初始关节姿态 |
| `common/marvin_session_logger.py` | 非阻塞 JSONL 控制周期日志与回放记录读取 |
| `hardware/interface/marvin_kinematics.py` | 厂家 FK/IK 的米/弧度边界和 IK 异常解释 |
| `hardware/interface/marvin.py` | 控制 SDK 连接、反馈预热、速度/加速度、K/D、Tool 和 `set_joint_cmd_pose(A/B)` |
| `hardware/marvin_teleop_controller.py` | 共享遥操状态、IK、B 键回位和最终关节目标 |
| `simulation/marvin_mujoco_adapter.py` | 用 MuJoCo 实现与硬件适配器相同的最小控制接口 |
| `scripts/hardware/...` | 实机确认参数、依赖组装和启动入口 |
| `scripts/simulation/teleop_...` | PICO → MuJoCo 组装和启动入口 |
| `scripts/simulation/replay_...` | JSONL command/feedback 回放入口 |
| `tests/test_marvin_hardware.py` | 合成 XR、真实厂家 IK、SDK mock 和 headless MuJoCo 回归 |

## 3. 最小闭环

```text
PXREADeviceStateJson → native get_snapshot()
  → XrClient.read_snapshot()
  → transform_controller_poses_to_marvin_frame()
  → XrTargetMapper.map_arm()
  → MarvinVendorKinematics.ik_world()
  → MarvinHardwareTeleopController._compute_q_command()
  ├→ MarvinSdkAdapter.send_joint_command()
  │    → set_joint_cmd_pose(A/B)
  └→ MarvinMujocoAdapter.send_joint_command()
       → MuJoCo position actuators
```

Grip 松开时锁存当前反馈关节姿态并清除遥操锚点；再次按下会从新的手柄位置和
机器人 TCP 继续。双 Grip 松开后按 B，控制器才生成返回初始姿态的 3 秒余弦
关节轨迹。应用层没有 limiter。

手柄位姿使用固定 OpenXR tracking space，不使用头显位置或朝向：OpenXR
`-Z/+X/+Y`（前/右/上）分别映射到 Marvin `-X/+Y/+Z`。每只手第一次按下
Grip 时记录当前手柄位姿和当前机器人 TCP；按住期间只映射相对位姿增量，松开
后清除锚点，因此再次按下 Grip 会从新的手柄位置和机器人 TCP 重新开始。

## 4. 依赖边界

| 边界 | 加载内容 | 是否连接机械臂 |
| --- | --- | --- |
| XR | 项目内 `_xrobotoolkit_sdk` + `/opt/apps/roboticsservice/SDK` | 否 |
| 运动学 | `fx_kine.py`、`libKine.so`、`ccs_680.MvKDCfg` | 否 |
| MuJoCo | `mujoco`、Marvin MJCF/meshes | 否 |
| 实机控制 | `fx_robot.py`、`libMarvinSDK.so`、`robot.ini` | 是 |

仿真入口只使用前三项。控制 SDK 只能由硬件入口通过 `MarvinSdkAdapter` 加载。

外部配置默认位置：

| 配置 | 路径 |
| --- | --- |
| Marvin SDK | `../TJArm/tj_fx_robot-master` |
| 运动学参数 | `CommonConfig/config/ccs_680.MvKDCfg` |
| Tool 运动学/动力学 | `../TJArm/tools_cfg.json` |
| 控制器/急停配置 | `../TJArm/tj_fx_robot-master/robot.ini` |
| scale | `logs/marvin_scale_calibration.json` |

## 5. 命名与单位

| 名称 | 约定 |
| --- | --- |
| `arm_index=0/1` | SDK A/B，即左臂/右臂 |
| `q_rad` | 14 轴或单臂 7 轴关节角，弧度 |
| `q_deg` | 厂家 `set_joint_cmd_pose` 边界，角度 |
| `dq_rad_s` | 关节速度，弧度每秒 |
| `T_world_tcp_m` | 4×4 齐次矩阵，平移单位米 |
| `*_mm_deg` | 厂家运动学或 Tool 边界，毫米/角度 |
| `left_k/right_k` | 厂家关节 K，`N·m/deg`，每轴 `[0,22]` |
| `left_d/right_d` | 厂家关节 D，无量纲，每轴 `[0,1]` |
| `joint_velocity_ratio` | 厂家关节速度百分比，首次实机默认 `10` |
| `joint_acceleration_ratio` | 厂家关节加速度百分比，首次实机默认 `10` |
| `frame_serial` | SDK A/B 反馈帧号 |

厂商 Python API 的关键字保持原样：`set_joint_kd_params(arm, K, D)`；项目内部
统一使用小写 snake_case：`left_k/left_d/right_k/right_d`。

## 6. 初始位姿与关节顺序

```text
SDK A / left:  [ 90, -90, -90, -20,  90, 0, 0]°
SDK B / right: [-90, -90,  90, -20, -90, 0, 0]°
```

所有 14 轴数组均为 `[A1..A7, B1..B7]`。MuJoCo 对应
`[Joint1_L..Joint7_L, Joint1_R..Joint7_R]`，不做符号或偏置转换。

## 7. 日志格式

文件名：

```text
marvin_hardware_<timestamp>.jsonl
marvin_mujoco_<timestamp>.jsonl
```

每行一个 `schema_version=1, event=control_cycle` JSON 对象。核心字段为：

```text
monotonic_time_ns, xr_timestamp_ns
left_controller_pose, right_controller_pose
grip_values, button_a, button_b, scale_factor
frame_serial, arm_state, error_code
q_feedback_rad, dq_feedback_rad_s, q_command_rad
```

日志写线程与控制循环分离；程序正常退出时 `close()` 等待队列落盘。回放只接受
包含 14 个有限 `q_feedback_rad` 和 `q_command_rad` 的 `control_cycle` 记录。

## 8. MuJoCo 资产

`assets/marvin/` 来自参考仓库中已与 Marvin 厂家模型对齐的快照。MJCF 使用
本目录相对路径加载 meshes；复制或移动 XML 时必须同时保留 `meshes/`。
详细来源、哈希和模型约定见 `assets/marvin/README.md` 与 manifest。
