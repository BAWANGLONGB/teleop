# 当前架构边界与接口规范

本文描述当前源码中的实际边界和调用契约。接口采用 Python 结构化约定
（duck typing），项目没有额外定义 `Protocol` 或抽象基类；若本文与实现冲突，
以 [`xr_marvin_teleop/`](../xr_marvin_teleop/) 下的代码为准。

## 1. 总体架构

```text
PICO / XRoboToolkit
        │ callback JSON
        ▼
native/_xrobotoolkit_sdk ──原子快照──> XrClient
                                           │ XrSnapshot | None
                                           ▼
                                   MarvinHardwareTeleopController
                                    │       │        │
                          位姿映射 ──┘       │        └──> SessionLogger
                          scale 标定/读取 ───┘
                                           │ 14 轴 q_rad
                           ┌───────────────┴────────────────┐
                           ▼                                ▼
                 MarvinSdkAdapter                 MarvinMujocoAdapter
                           │                                │
                      Marvin 实机                      MuJoCo 模型

控制器 ──7 轴反馈/TCP 目标──> MarvinVendorKinematics ──7 轴 IK 结果──> 控制器
```

硬件和 MuJoCo 共用同一个控制器、位姿映射、标定、运动学和日志逻辑，只替换
机器人适配器。`scripts/` 是组装入口，不承载控制策略。

## 2. 模块边界

| 边界 | 拥有的职责 | 不拥有的职责 |
| --- | --- | --- |
| `native/xrobotoolkit_sdk.cpp` | 解析一次 XR SDK 回调中的完整 JSON；mutex 下发布和复制最新原子帧 | 新鲜度判断、坐标转换、机器人控制 |
| `common/xr_client.py` | 初始化/关闭 XR SDK；构造并校验 `XrSnapshot`；判断时间戳停滞和断流 | Grip/B/A 策略、机器人状态 |
| `common/xr_target_mapper.py` | OpenXR 到 Marvin 坐标变换；每臂 Grip 锚定；相对 TCP 目标计算 | IK、限位、回位轨迹 |
| `common/marvin_scale_calibration.py` | scale 来源优先级、A/A 两点标定、校验和原子持久化 | B 键回位、机器人运动 |
| `hardware/interface/marvin_kinematics.py` | 厂商 FK/IK 的 SI 单位边界、基座坐标变换、IK 失败归一化 | 网络连接、发送关节命令 |
| `hardware/interface/marvin.py` | 实机连接、反馈、控制参数、模式切换、双臂命令事务和厂商单位换算 | 遥操状态机、XR 解释 |
| `hardware/marvin_teleop_controller.py` | 启停顺序、健康检查、Grip/A/B 状态、IK 编排、回位和最终关节目标 | 厂商 API 细节、日志文件格式 |
| `simulation/marvin_mujoco_adapter.py` | 用 MuJoCo 实现机器人适配器契约 | 复制一套遥操策略 |
| `common/marvin_session_logger.py` | 控制周期 JSONL 异步落盘和回放输入校验 | 控制决策 |
| `scripts/hardware`、`scripts/simulation` | 参数解析、依赖实例化、安全确认和运行入口 | 可复用业务逻辑 |

依赖方向必须保持：控制器可以依赖 `common` 数据/算法和注入的接口对象；
`common` 不得依赖实机控制 SDK；仿真不得实例化 `MarvinSdkAdapter`。仿真仍使用
厂商运动学库和 Tool 配置，但不会加载 `libMarvinSDK.so` 或连接机械臂。

## 3. 公共数据模型和单位

### 3.1 `XrSnapshot`

```text
timestamp_ns: int                         # 正整数，XR 源时间戳
headset_pose: ndarray(7)                  # [x,y,z,qx,qy,qz,qw]
left_controller_pose: ndarray(7)          # 同上
right_controller_pose: ndarray(7)         # 同上
grip_values: tuple[float, float]           # [0,1]，左/右
button_a: bool
button_b: bool
```

三个位姿必须有限，四元数范数允许 `1e-3` 误差，进入对象时会归一化。手柄位姿
使用固定 OpenXR tracking space，当前控制映射不使用头显位姿。

### 3.2 `MarvinRobotState`

```text
frame_serial: tuple[int, int]              # SDK A/B 反馈帧号
q_rad: ndarray(14)                         # 关节位置，rad
dq_rad_s: ndarray(14)                      # 关节速度，rad/s
arm_state: tuple[int, int]                 # SDK A/B 状态
error_code: tuple[int, int]                # SDK A/B 错误码
low_speed: tuple[bool, bool]               # SDK A/B 低速标志
```

所有 14 轴向量顺序固定为 `[A1..A7, B1..B7]`，对应左臂、右臂。MuJoCo 顺序
固定为 `[Joint1_L..Joint7_L, Joint1_R..Joint7_R]`，不增加符号或零位偏置。

### 3.3 统一单位

| 名称 | 接口单位/约束 |
| --- | --- |
| `q_rad` / `q_ref_rad` | rad，单臂 7 轴或双臂 14 轴 |
| `dq_rad_s` | rad/s |
| `T_world_tcp_m` | 有限 4×4 右手齐次矩阵；平移为 m |
| OpenXR position | m |
| `*_mm_deg` | 仅在厂商 Tool/FK/IK 边界使用 mm/deg |
| `left_k/right_k` | 7 项，厂家关节 K，每项 `[0,22]` |
| `left_d/right_d` | 7 项，厂家关节 D，每项 `[0,1]` |
| `joint_velocity_ratio` / `joint_acceleration_ratio` | 整数百分比 `[0,100]` |
| `scale_factor` | 正有限值；持久化标定限定 `[0.5,1.5]` |

应用层只传米和弧度。角度/毫米转换只能留在厂商边界模块中。

## 4. 接口契约

以下签名是当前调用方要求的结构化契约，不是可导入的抽象类。

### 4.1 原生 XR binding

```python
init() -> None
get_snapshot() -> dict | None
close() -> None
```

`get_snapshot()` 返回一次完整回调的副本，字典字段必须能直接构造
`XrSnapshot`。尚未收到有效帧时返回 `None`。回调解析失败时丢弃整帧，不允许
把不同回调中的 Head、Controller 或按钮字段拼成一帧。

### 4.2 XR 客户端

```python
wait_for_fresh_snapshot(timeout_seconds=2.0) -> XrSnapshot
read_snapshot() -> XrSnapshot | None
close() -> None
```

- 启动时必须观察到两个递增的正时间戳才返回 fresh snapshot。
- 时间戳回退抛 `RuntimeError`。
- 默认同一时间戳持续不超过 `0.5 s` 仍可用；超过 `0.5 s` 返回 `None`；
  超过 `2.0 s` 抛 `TimeoutError`。
- `close()` 幂等。

### 4.3 机器人适配器

硬件和 MuJoCo 后端都必须满足：

```python
connect() -> None
sdk_version() -> int | None
read_state() -> MarvinRobotState
wait_for_fresh_feedback(timeout_seconds=..., required_updates=...) -> MarvinRobotState
configure_control_parameters(
    left_k, left_d, right_k, right_d,
    tool_configurations=None,
    joint_velocity_ratio=10,
    joint_acceleration_ratio=10,
) -> None
enter_joint_impedance() -> None
enable_pd_feedforward(period_milliseconds) -> None
send_joint_command(q_rad, wait_response=False) -> None
set_idle() -> bool
release() -> None
```

可选接口：

```python
is_running() -> bool
```

适配器约束：

- `send_joint_command` 接收一个有限 14 轴弧度向量，必须覆盖双臂同一控制周期；
- 实机适配器在单个 SDK command buffer 中设置 A/B 目标，再统一发送；
- 实机适配器负责 rad↔deg 转换，控制器不得调用厂商 `set_*` 方法；
- `read_state` 返回的两臂反馈必须属于一次 adapter 采样；
- `set_idle` 返回是否实际发送了 idle 命令；`release` 幂等；
- MuJoCo 的控制参数和 PD 配置允许为 no-op，但方法和状态转换必须存在。

实机依赖加载前还必须通过 `validate_vendor_driver_dependency()`：存在
`fx_robot.py`、`libMarvinSDK.so`、`robot.ini`，并且 `UseEMG=1`、双臂
`JointPIDCtlType=1`。

### 4.4 运动学

```python
set_tool(arm: int, tool_xyzabc_mm_deg: sequence[6]) -> None
fk_world(arm: int, q_rad: sequence[7]) -> ndarray(4, 4)
ik_world(
    arm: int,
    T_world_tcp_m: ndarray(4, 4),
    q_ref_rad: sequence[7],
) -> VendorIkResult
```

`arm=0/1` 对应 SDK A/B。正常的不可达、奇异、超限或无解不抛异常，而返回：

```text
VendorIkResult(success=False, q_rad=None, failure_reason=<原因>)
```

输入损坏、配置/库缺失或 FK 执行失败才抛异常。IK 使用当前分支附近的
`q_ref_rad` 保持解连续；控制器在 Grip 首次按下时使用实时反馈，随后使用上次
成功 IK 结果。

### 4.5 位姿映射

```python
transform_controller_poses_to_marvin_frame(snapshot)
    -> ((position_3, rotation_3x3), (position_3, rotation_3x3))

XrTargetMapper(scale_factor).map_arm(
    arm_index,
    controller_pose_marvin,
    current_tcp_transform,
    is_active,
) -> ndarray(4, 4) | None
```

坐标约定为 OpenXR `right/up/back -> Marvin +Y/+Z/+X`。第一次 active 调用同时
记录手柄位姿和机器人 TCP，并返回当前 TCP；后续平移使用
`tcp_anchor + scale × controller_delta`，旋转使用相对手柄旋转。inactive 调用
清除该臂锚点并返回 `None`。

### 4.6 日志

控制器只依赖两个方法：

```python
record_control_cycle(xr_snapshot, robot_feedback, q_command_rad, scale_factor)
close() -> None
```

记录器在后台线程写 JSONL；`close()` 必须等待队列结束，并把写线程错误传播给
调用方。日志不参与控制决策。

## 5. 控制生命周期

### 5.1 启动

`prepare_hardware()` 的顺序固定：

1. 等待 XR 递增帧，并要求两侧 Grip 都不高于 `0.1`；
2. 连接 adapter，按配置检查 SDK 版本；
3. 等待双臂递增反馈，确认无错误且双臂低速；
4. 配置 K/D、Tool、速度和加速度百分比；
5. 把启动时的实际反馈保存为首个保持目标和 IK 参考；
6. 进入双臂关节阻抗模式，启用对应控制周期的 PD 前馈并复核反馈；
7. 向双臂发送一次启动反馈关节角。

启动不会自动运动到初始姿态。只有操作者在两侧 Grip 松开时按下 B，才开始回位。

### 5.2 每周期优先级

每个周期先读 XR 和机器人反馈，再检查机器人健康/帧号，最后生成并发送一个
14 轴目标。默认实机频率为 `200 Hz`，允许范围为 `[50,200] Hz`，且周期必须
对应整数毫秒的 PD period。

| 输入/状态 | 该臂行为 |
| --- | --- |
| Grip 从松开变为按下（默认 `value > 0.9`） | 取消该臂正在进行的 B 回位；以当前手柄位姿和当前实际 TCP 建立新锚点 |
| Grip 持续按下 | 映射相对 TCP 位姿并求 IK；IK 成功则更新目标，失败则保持上一关节目标 |
| Grip 从按下变为松开 | 清除映射锚点；锁存该周期实际反馈关节角并保持 |
| Grip 持续松开 | 保持锁存目标；操作者可以移动手柄，不影响机器人 |
| 松开后再次按下 | 从新的手柄位置和当时机器人 TCP 重新锚定，不产生手柄绝对位置跳变 |
| XR 暂时 stale，`read_snapshot()` 返回 `None` | 双臂保持上一目标，清除映射锚点，记录 `xr_frame_valid=false`；恢复后在当前手柄/TCP 重新锚定 |

左右 Grip 独立控制两臂；一臂重新抓取只取消该臂回位，另一臂可继续回位。

### 5.3 B 键机器人 reset

B 采用上升沿触发，并且只在两侧 Grip 都松开时接受：

```text
q(t) = q_start + (0.5 - 0.5*cos(pi*t/T)) * (q_initial - q_start)
T = return_duration，默认 3 s
```

- `q_start` 是按 B 当周期的实际反馈，不是上一次命令；
- 目标是 `MARVIN_INITIAL_POSE_Q_RAD`：
  - SDK A/left：`[90,-90,-90,-20,90,0,0]°`
  - SDK B/right：`[-90,-90,90,-20,-90,0,0]°`
- B 持续按住不会重复启动；Grip 未松开时 B 被忽略；
- 回位过程中重新按下某侧 Grip，会立即取消该臂回位并重新锚定；
- 轨迹按时间完成后保持初始关节目标，不额外判断反馈收敛；
- B 不修改当前 `scale_factor`、标定文件或 A/A 标定器状态；若已完成第一次 A
  采样，该采样在 B 回位后仍保留。

### 5.4 A/A scale 标定

A 采用上升沿触发，仅在双 Grip 松开、没有 B 回位且双臂 `low_speed=True` 时接受。
第一次 A 保存自然下垂位置，第二次 A 保存双臂水平前伸位置并计算 scale。完成后
原子替换标定 JSON，并立即更新 mapper 的平移比例。

scale 来源优先级为：命令行显式值 > 有效标定文件 > 默认 `1.2`。scale 只缩放
平移增量，不缩放旋转。

## 6. 故障与关闭语义

| 条件 | 行为 |
| --- | --- |
| 任一 `error_code != 0`、`arm_state == 100` 或运行中不在 `(3,3)` | 抛 `RuntimeError`，进入统一关闭 |
| 任一臂反馈帧连续 3 个控制周期不递增或为 0 | 抛 `TimeoutError`，进入统一关闭 |
| XR 时间戳短时停滞 | 保持上一关节目标，不做自动回位 |
| XR 时间戳断流超过默认 2 s 或回退 | 抛异常，进入统一关闭 |
| 普通 IK 无解 | 只保持对应臂上一目标，控制循环继续 |
| 参数形状、范围、单位非法 | 在边界处抛 `ValueError`，不下发命令 |
| SDK/配置文件缺失 | 抛 `FileNotFoundError` |
| 实机网络连接失败 | 抛 `ConnectionError` |

`run()` 无论正常结束、异常或 `KeyboardInterrupt` 都调用 `shutdown_hardware()`：
先请求双臂 idle 并等待最多 `2 s`，随后在 `finally` 中依次释放 adapter、XR 和
日志资源。
程序关闭不是物理急停，不能替代 `robot.ini` 的急停监控和现场急停按钮。

## 7. 持久化接口

### 7.1 scale 标定 JSON

```json
{
  "schema_version": 1,
  "created_at": "<ISO-8601>",
  "scale_factor": 1.0,
  "controller_travels_m": {"left": 0.0, "right": 0.0},
  "arm_lengths_m": {"left": 0.0, "right": 0.0}
}
```

写入采用同目录临时文件、`fsync` 和 `replace`，读入只接受 schema 1 和合法 scale。

### 7.2 控制周期 JSONL

每行是 `schema_version=1`、`event="control_cycle"`，至少包含：

```text
monotonic_time_ns, xr_frame_valid, xr_timestamp_ns,
headset_pose, left_controller_pose, right_controller_pose,
grip_values, button_a, button_b, scale_factor,
frame_serial, arm_state, error_code,
q_feedback_rad, dq_feedback_rad_s, q_command_rad
```

XR stale 保持周期仍写日志，但 XR 字段为 `null`。回放读取器忽略其他 event，并
要求 `q_feedback_rad`、`q_command_rad` 都是 14 个有限数值。

## 8. 明确不在当前边界内的能力

- 应用层没有碰撞检测、关节/笛卡尔 limiter 或轨迹规划器；
- B 回位是开环时间插值，不是基于反馈误差的完成状态机；
- 控制循环是同步 Python 循环，不提供硬实时保证；
- 头显位姿不参与手柄到机器人映射；
- adapter 契约当前只服务这两个后端，因此保持结构化接口，不增加抽象基类。

新增后端时，应实现第 4.3 节的最小契约，并复用现有控制器；新增控制策略时，
应放在控制器或 `common` 的纯算法模块中，不得泄漏到实机/仿真 adapter。
