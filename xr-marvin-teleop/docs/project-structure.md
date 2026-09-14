# 项目结构

## 1. 目录

```text
xr-marvin-teleop/
├── README.md
├── pyproject.toml
├── setup.py
├── config/
│   ├── collection.json
│   ├── das_gripper.example.json
│   └── data_collection/
├── native/
│   └── xrobotoolkit_sdk.cpp
├── ros2_ws/src/teleop_msgs/
│   └── msg/
├── docs/
│   ├── 首次部署.md
│   ├── 操作指南.md
│   ├── 常见报错.md
│   ├── architecture-and-interface-spec.md
│   └── project-structure.md
├── assets/marvin/
│   ├── marvin_dual.mujoco.xml
│   ├── marvin_dual.urdf
│   ├── marvin_dual.manifest.json
│   └── meshes/
├── scripts/
│   ├── data/                    # 采集、发布、迁移、后处理、校验与审阅入口
│   ├── hardware/                # 实机遥操、复位和 DAS 标定入口
│   └── simulation/              # MuJoCo 遥操与日志回放入口
├── tests/
│   ├── test_marvin_controller.py
│   ├── test_marvin_interfaces.py
│   ├── test_marvin_ros_data.py
│   ├── test_marvin_entrypoints.py
│   ├── test_marvin_simulation.py
│   └── test_*.py                # 配置、Episode、协议、迁移和 UI 回归
└── xr_marvin_teleop/
    ├── common/                   # 配置、XR 映射、日志和 Episode 离线处理
    ├── hardware/
    │   ├── marvin_teleop_controller.py
    │   └── interface/            # Marvin、运动学和 DAS 厂商边界
    ├── ros/                      # 统一消息契约、客户端和遥测发布
    └── simulation/
        └── marvin_mujoco_adapter.py
```

生成内容和有效标定的保留规则见第 9 节；不要整目录删除 `logs/`。

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
| `hardware/interface/das_finger.py` | DAS 左右夹爪 Python SDK 生命周期、编码器初始化和闭合度→开口距离转换 |
| `hardware/marvin_teleop_controller.py` | 共享遥操状态、IK、B 键回位和最终关节目标 |
| `ros/das_client.py` | 订阅独立 DAS 状态，并通过 ROS2 下发闭合度命令 |
| `ros/pico_client.py` | 订阅独立 PICO 原始流并提供与 `XrClient` 相同的控制快照接口 |
| `ros/telemetry_bridge.py` | 以独立 critical/image 线程有界发布状态、命令、触觉、原始图像和诊断 |
| `common/episode_postprocessor.py` | 合并在线 MCAP 分片，保留采集系统时间并由关节流生成 TCP 6D pose |
| `common/collection_config.py` | 统一数采配置、CLI 覆盖、路径/类型校验、文件快照和活动设备配置核对 |
| `config/collection.json` | 数采默认参数唯一来源，引用既有 DAS JSON 和 ROS YAML |
| `common/episode_video.py` | 离线 Foxglove MJPEG/H.264 双 MCAP 导出、编码参数和采集互斥锁 |
| `common/episode_validator.py` | 离线检查 MCAP 话题、频率、时间回退、序号缺口和文件哈希 |
| `common/episode_package.py` | 旧 LeRobot MCAP 的附件读写、CRC 校验和安全解包；供历史文件与 UI 使用 |
| `ros2_ws/src/teleop_msgs` | 仅为旧 Episode 离线迁移保留；新采集使用标准 ROS2 / Foxglove 类型 |
| `ros/protocol.py` | v2 消息构造、采样诊断、关节名映射与有界精确配对 |
| `scripts/data/migrate_messages_v2.py` | 将旧 teleop_msgs 原始 Episode 复制迁移到新目录 |
| `simulation/marvin_mujoco_adapter.py` | 用 MuJoCo 实现与硬件适配器相同的最小控制接口 |
| `scripts/hardware/...` | 实机确认参数、DAS 独立标定、依赖组装和启动入口 |
| `scripts/data/...` | supervisor、PICO/DAS 发布、原生 MJPEG 写盘、完整 MCAP 后处理与校验入口 |
| `scripts/simulation/teleop_...` | PICO → MuJoCo 组装和启动入口 |
| `scripts/simulation/replay_...` | JSONL command/feedback 回放入口 |
| `tests/test_marvin_controller.py` | 合成 XR、控制状态、IK/NSP、回位和断流回归 |
| `tests/test_marvin_interfaces.py` | Marvin/DAS SDK 边界、配置和夹爪回归 |
| `tests/test_marvin_ros_data.py` | ROS2 客户端、遥测线程和原始 Episode 校验回归 |
| `tests/test_marvin_entrypoints.py` | 实机 CLI、复位和采集 supervisor 回归 |
| `tests/test_marvin_simulation.py` | headless MuJoCo 与真实厂家 IK 集成回归 |
| `tests/test_collection_config.py` | 配置优先级、路径解析、快照和设备配置一致性 |
| `tests/test_episode_postprocessor.py` | 时间戳选择、URDF FK、原生 MJPEG、旧附件兼容与校验 |
| `tests/test_episode_video.py` | 导出互斥、双 MCAP 内容、H.264 解码和失败恢复 |

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
关节轨迹；启用夹爪时 B 同时把闭合度设为 `1`。Grip 遥操的 IK 关节目标经过
可配置的每周期速度步长限制；B 回位仍按固定时长余弦轨迹执行。

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
SDK A / left:  [ 122, -60, -87, -115,  88, -10,  15.313]°
SDK B / right: [-122, -60,  87, -115, -88, -10, -15.313]°
```

所有 14 轴数组均为 `[A1..A7, B1..B7]`。MuJoCo 对应
`[Joint1_L..Joint7_L, Joint1_R..Joint7_R]`，不做符号或偏置转换。

## 7. 日志格式

文件名：

```text
marvin_hardware_<timestamp>.jsonl
marvin_mujoco_<timestamp>.jsonl
```

每行一个 `schema_version=2, event=control_cycle` JSON 对象。核心字段为：

```text
sample_id, monotonic_time_ns, wall_time_ns, xr_frame_valid, xr_timestamp_ns
left_controller_pose, right_controller_pose
grip_values, button_a, button_b, scale_factor
frame_serial, arm_state, error_code
q_feedback_rad, dq_feedback_rad_s, q_command_rad
```

日志写线程与控制循环分离；程序正常退出时 `close()` 等待队列落盘。回放只接受
包含 14 个有限 `q_feedback_rad` 和 `q_command_rad` 的 `control_cycle` 记录。
XR 暂时失效的周期仍记录保持目标，XR 输入字段为 `null`。完整字段见[接口规范](architecture-and-interface-spec.md#72-控制周期-jsonl)。

## 8. MuJoCo 资产

`assets/marvin/` 来自参考仓库中已与 Marvin 厂家模型对齐的快照。MJCF 使用
本目录相对路径加载 meshes；复制或移动 XML 时必须同时保留 `meshes/`。
详细来源、哈希和模型约定见 `assets/marvin/README.md` 与 manifest。

## 9. 维护与生成文件

| 路径 | 用途与保留规则 |
| --- | --- |
| `config/collection.json` | 数采默认值；本地覆盖建议放在忽略的 `config/*.local.json` |
| `logs/marvin_scale_calibration.json` | A/A 标定配置，启动与快照会读取，必须保留 |
| `logs/*.jsonl` | 运行调试/回放日志，Git 忽略；确认不再排障或回放后可归档 |
| `dataset/` | 采集原件、配置快照与最终 MCAP，Git 忽略；不属于代码清理对象 |
| `dataset/**/final/` | 离线导出结果，不覆盖已存在目录；重新导出前先移走旧结果 |
| `ros2_ws/build/`、`install/`、`log/` | 旧消息包 colcon 生成内容，Git 忽略；仅旧数据迁移需 source |

新采集的唯一最终导出入口是 `scripts/data/postprocess_episode.py`，输出 Foxglove MJPEG/H.264 MCAP。
旧 `package_episode()` 自动转 Parquet/MP4 并删除原始目录的流程已移除；附件工具继续服务历史文件及 UI。
`review_episode.py` 读取合并后的 ROS bag 目录，`extract_episode_mcap.py` 只处理旧 LeRobot 附件格式。
`migrate_sessions.py` 是单独的历史数据迁移入口，不由采集启动流程调用。

修改后运行 `python -m unittest discover -s tests -v`；视频集成项需要 source ROS2 和消息工作区并安装 `.[h264]`。
只校验数采设置可用 `python scripts/data/run_collection.py --print-effective-config`，不会连接硬件。
