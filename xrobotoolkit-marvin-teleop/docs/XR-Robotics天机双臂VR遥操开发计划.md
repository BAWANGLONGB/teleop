# 基于 XR-Robotics 的天机双臂 VR 遥操开发计划

> 更新日期：2026-08-28
>
> 文档状态：实施基线 v9（真机 Grip 松手复位与进程内 A/B scale 标定）
>
> 当前结论：MuJoCo 链路和 MarvinSDK 实机软件后端均已集成；实机侧已有显式授权、带 SDK 版本门禁的只读检查、双臂原子命令、反馈式关节限幅、TCP 目标保护、奇异性降速、watchdog、锁存 FAULT、Grip 松手受限复位、进程内及 PICO-only scale 标定、旁路 ROS2 观测和 200 Hz 标定数据自动保存。当前自动回归为 `40 passed`，但尚未完成 PICO 与真实机械臂的 M1/M4/M6 现场验收，因此只能视为“软件实现完成”，不得视为实机可安全投产。

## 1. 项目目标与首版边界

以 `XR-Robotics/XRoboToolkit-Teleop-Sample-Python` 为主工程，复用其 PICO 数据接入、遥操生命周期、MuJoCo、相机和数据记录能力，接入天机 Marvin 双臂控制器，交付一条可验证、可回放、默认不运动的低速 VR 遥操链路：

```text
PICO 左右 Grip 独立控制双臂 deadman
→ 手柄增量 SE(3) 映射为左右 TCP 目标
→ TCP guard → Placo IK → 反馈式关节 limiter 输出 14 轴目标
→ MuJoCo 与实机复用 XR 映射和 IK 基础能力，分别执行各自安全保护
→ MarvinSDK 以关节位置目标驱动控制器内部关节阻抗
→ 松手、XR 丢帧、反馈停更、网络异常和连续求解失败进入 HOLD/FAULT
```

首版不包含移动底盘、浮动基座 IMU 补偿、恒力接触和高速轨迹。PGC 夹爪、相机与 LeRobot 数据转换在双臂本体通过实机验收后接入。

## 2. 已核实基线与待确认项

### 2.1 参考资料基线

本计划以以下本地资料为准：

| 资料 | 用途 | 基线标识 |
| --- | --- | --- |
| `TJArm/tj_fx_robot-master.zip` | Marvin 控制/运动学 SDK、Python 封装、示例、M6S-Lite 双臂模型 | SHA-256 `9188a0d856740eb64c74a8b1da817f4a71356c97f37007caab1d3b95a75c1761` |
| `TJArm/tj_robot.zip` | 左右臂空载/带载辨识原始数据及结果 | SHA-256 `09d1ab7ab2e0422a53d4d07c01be6136f23ad5c0c5c4763a2f3b298e824c9c2f` |
| `TJArm/tools_cfg.json` | 当前双臂 `tool-1` 动力学参数 | SHA-256 `3b67eb4a39e5f4ac3bd370fe87581c2fadee85c29b5a8591c96eb2d9a499f535` |
| `TJArm/天机TJArm.md` | 现场连接、示例和坐标约定 | 本地文档 |
| `TJArm/天机Marvin系列_Marvin PlatformEN软件使用说明260804.pptx` | 模式切换、工具辨识、错误处理和方向检查 | 2026.06 文档 |
| `TJArm/天机Marvin系列_急停功能启用方法说明251203.pdf` | 物理急停启用方法 | 2025.12 文档 |

XRoboToolkit 主工程已经在 `UPSTREAM.md` 锁定到：

```text
XRoboToolkit-Teleop-Sample-Python
commit 79e5cb8a56e3455515ce1b476e993c764ec58739
```

PC Service、Pybind 和 PICO/Unity 客户端版本已完成补录，并在 `docs/XRoboToolkit环境部署与PICO联调流程.md` 中记录版本、commit、安装包哈希和联调步骤。后续升级仍不得使用浮动 `main` 作为验收基线。供应商 ZIP 没有可验证的 Git commit，因此现阶段使用文件哈希，并在引入 `third_party` 时额外记录 SDK/控制器版本。

| XRoboToolkit 组件 | 已验证基线 |
| --- | --- |
| PC Service | Release v1.0.0；Debian 包版本 `1.0.0.0`；SHA-256 `61961067eb4b41f81ed7cae35f4690dbb0ddfefb329a12b24e0b90ebc46ada91` |
| Python Pybind SDK | `xrobotoolkit_sdk 1.0.2`；commit `c64ccf6acd577a333e03b66fafe8efeeceb511b1` |
| Unity Client | commit `cdc53166b0bf412efae71046c6a225eb5091605f`；bundle version `1.1.1` |
| PICO APK | `XRoboToolkit-PICO-1.1.1.apk`；SHA-256 `6b2bb282405673d24abcb1980e3478b8f1052e90f7207b1f24cc56a59f8d8261` |

### 2.2 从 SDK 与调试资料确认的事实

| 项目 | 已确认约定 |
| --- | --- |
| 控制器默认地址 | `192.168.1.190`；以现场实际地址为准，PC 设置为同网段且不得与控制器同 IP |
| 通信 | MarvinSDK 使用 UDP；`connect()` 返回成功不等于实时数据有效，必须检查 A/B 两臂 `frame_serial` 非零且持续递增 |
| 双臂命名 | SDK `arm='A'` 为左臂，`arm='B'` 为右臂；模型命名后缀分别为 `_L`、`_R` |
| SDK 关节单位 | 关节位置、关节速度和目标关节位置均按供应商 Python 示例使用“度”；本工程内部统一使用 rad/rad·s⁻¹ |
| SDK 笛卡尔单位 | `[X,Y,Z,A,B,C]` 中 XYZ 为 mm、ABC 为 degree；`R = Rz(C) · Ry(B) · Rx(A)` |
| TCP 语义 | `TargetTCP = UserFrame @ FlangeTip(q) @ Tool`；左右轨迹分别位于各自 `Base_L`、`Base_R` 坐标系 |
| 关节阻抗模式 | `state=3`（TORQ）且 `impedance_type=1`；上位机仍下发关节位置，阻抗闭环在机器人控制器内部完成 |
| SDK 状态 | `0=IDLE/下伺服`、`1=POSITION`、`2=PVT`、`3=TORQ`、`4=RELEASE`、`100=错误态` |
| SDK 调用协议 | 一次命令按 `clear_set()` → 设置参数/目标 → `send_cmd()` 或 `send_cmd_wait_response()` 发送 |
| 数据能力 | 控制器反馈缓冲最高按 1 kHz 更新；实时点位文档要求不超过 200 Hz。首版实机从 50 Hz 起步，100/200 Hz 只能在抖动、丢帧和延迟测试通过后启用 |
| 运行互斥 | 连接和释放只调用一次；结束必须 `release_robot()`，避免 MarvinPlatform 或其他进程占用控制器 |
| 平台 | 供应商 Linux SDK 仅明确在 x86_64 上开发测试，其他架构必须重新编译和验证 ABI |

### 2.3 必须在 M1 前关闭的不确定项

参考包同时出现了 `M6S-Lite-CCS-680` 模型、`ccs_680.MvKDCfg`，而 MarvinPlatform 说明又指出当前 M6S-CCS 机型使用 `ccs_m6_40`。因此以下内容不得靠文件名推断：

- 真机准确型号、左右臂构型、控制器版本和 SDK 版本；
- 应使用的 `.MvKDCfg`，以及其中 `TYPE/DH/PNVA/BD` 是否与真机一致；
- 双臂基座相对安装变换是否等于示例模型中的 `y=±0.06 m、z=-0.10 m、roll=∓π/2`；
- URDF 的 `Joint1...Joint7` 与 SDK A/B 通道是否为同序同号；
- `TCP_Link_L/R` 的 87 mm 固定偏置与现场法兰、夹爪、`Tool` 设置是否重复计入；
- 控制器是否已启用硬件急停，以及断网/进程退出时控制器的真实保持或下伺服行为。

任何一项未确认，均不得进入带使能的自动运动测试。

### 2.4 当前工程状态

| 内容 | 当前状态 | 下一动作 |
| --- | --- | --- |
| XRoboToolkit Python Sample | 已克隆并锁定 commit；PC Service、Pybind、PICO 版本已补录；已新增左/右/双手柄位姿持续打印工具 | 完成 PICO 真机长时间位姿、Grip 与头显跟随验收 |
| `assets/marvin/` | 已导入 M6S-Lite CCS-680 双臂开发基线、14 轴 MJCF、供应商 mesh 和哈希清单 | 现场确认准确机型与双臂安装变换；当前模型不得直接作为实机身份依据 |
| `configs/` | `marvin_hardware.json` 保存当前低速参数快照，但实机入口尚未加载该文件，运行真值仍是 CLI 默认值 | 在现场冻结前决定将其接成唯一配置源，或删除重复项并增加 CLI 默认值漂移测试 |
| Marvin 公共/硬件/仿真模块 | MuJoCo 链路可用；已新增 SDK 适配、反馈式关节限幅、TCP guard、安全状态机、后台日志和专用硬件控制器 | 通过现场模型、时序、故障注入和阻抗参数验收，不把 mock 测试等同于真机验证 |
| Marvin 启动脚本 | 仿真入口保持原功能；新增严格只读检查入口和必须四项显式确认的实机入口 | 先完成 30 分钟只读和供应商逐轴映射签字，再从单臂 TCP 5 cm 开始遥操验收 |
| `tests/` | 已覆盖 MuJoCo 合同及 SDK 单位/双臂事务、工具原始单位、A/B 帧停更、关节限幅、TCP guard、状态机、只读 SDK 版本门禁、自动日志和默认只读行为 | 增加录制回放、网络故障与目标机器周期压力测试 |

因此项目当前处于 **M5 软件实现完成、现场验收待执行** 状态。M1 的真机身份/映射冻结、M4 的 PICO 实测与故障注入、M6 的分级实机运动仍是未关闭门槛。特别是供应商 `showcase_pln_multi_segment_linear_two_arms_classes.py real` 默认使用较大笛卡尔轨迹和 `velocity/acceleration=100`，只可先运行 `sim`；不得拿它替代本工程低速分级验收。

### 2.5 截至 2026-08-28 的实施进度与注意事项

已完成：

- 新增 `scripts/misc/print_controller_poses.py`，可选左/右/双手柄、采样频率和采样数，输出 `[x,y,z,qx,qy,qz,qw]`；
- MuJoCo 启动初态改为 14 轴对称类人放松姿态：上臂沿躯干两侧下垂、肘部向前弯曲约 20°、左右腕镜像；
- 确认 Marvin 模型背部朝向操作者，机械臂与操作者的物理前方一致；
- PICO/OpenXR `X右、Y上、Z后` 到 Marvin world 采用 `[[0,0,1],[1,0,0],[0,1,0]]`，保证右/上/前与 TCP 同向；
- 参考模式改为 `head_yaw`：原点持续跟随头显位置，水平轴持续跟随头显 yaw，忽略 pitch/roll；每个 IK 周期左右臂共用同一帧头显位姿；
- Grip 上升沿仍在头显参考系变换之后锁存左右手柄/TCP 起点，松开后重置，保持增量映射无跳变语义。
- Marvin MuJoCo 中某侧 Grip 松开时，该侧从当前反馈关节角出发，默认用 3 秒余弦插值独立回到类人放松姿态；回位时重新按下 Grip 会立即取消回位并锁存新零点；
- Marvin MuJoCo 新增 A/B 两点臂长标定：松开双侧 Grip 后，双臂自然下垂按一次 A，再双臂水平向前伸直按一次 A；这两个人体姿态分别对应 Marvin 自然下垂和向前伸展姿态，按 `伸展比例 × 0.86864 / 左右平均手柄行程` 即时更新运行中的 `scale_factor`，默认伸展比例为 0.95，B 可取消重来；
- Marvin 真机进程采用同样的 A/A、B 操作，但只允许双 Grip 松开、自动复位完成且反馈静止时保存并切换比例；切换时清除旧参考点，下一次 Grip 重新锁存零点后使用新值，无需重启且不会造成 TCP 目标跳变；
- 标定输入限制为 `0.35–1.00 m`，左右测量差异不得超过 15%，输出比例限制为 `0.25–1.50`；异常的第二点不会覆盖肩部起点，可直接重新伸直采样；
- `pyproject.toml` 已将 pytest 收集范围限定为本项目 `tests/`，并改为安装全部 `xrobotoolkit_teleop*` 子包；当前自动回归为 `40 passed`，除原 MuJoCo 合同外，已覆盖 SDK 单位/双臂事务、启动事务忙重试与应答等待、工具原始单位、安全状态机、反馈与命令过期、运行模式离开、单臂帧冻结、Grip 松手单臂复位/取消、待机精确保持、速度/加速度/jerk/软限位与 jerk 速度前瞻、TCP 工作空间/速率/跳变、日志、标定 CSV/NPZ、PICO scale 坐标变换/持久化/优先级/失效关闭、只读 SDK 版本门禁、默认只读行为和 SDK 版本失配释放；
- MuJoCo 控制循环固定为 100 Hz，每次控制计算后执行 5 个 2 ms 物理子步，并用单调时钟按 10 ms 周期节拍运行；XR 由 200 Hz 独立线程写入带源时间戳的 latest-value 快照，渲染使用独立 MuJoCo data 副本以 60 Hz 刷新，`viewer.sync()` 不再阻塞控制/物理线程；
- 仿真窗口同时显示原始 TCP 目标球、绿色受限指令 TCP 和机械臂实际 TCP；每 2 s 输出周期、XR 数据龄期、物理/渲染耗时、raw→command、command→actual 距离与 deadline miss 的 P95 诊断。
- 每次 Marvin MuJoCo 运行自动将全部 100 Hz 延迟样本写入 `logs/marvin_latency_<时间戳>.csv`，正常退出后生成同名 `.summary.json` 全会话汇总；磁盘写入使用独立后台线程，不进入控制周期计时和物理锁临界区。
- 新增 `hardware/interface/marvin.py`：动态加载供应商 SDK、启动 ABI 检查、A/B 各自帧号新鲜度验证、degree↔rad 边界转换、双臂同一事务发送、模式/KD/Tool 配置和单次释放；本机未连接机械臂的 SDK 探测值为 `SDK_version()=100343014`，`check_sdk_type_compat()=(1,0)`；
- 新增 `inspect_marvin_state.py` 和 `teleop_marvin_hardware.py`。前者严格只读；后者在创建连接前强制 `--enable-hardware`、物理急停确认、关节映射确认和准确机型字符串四项授权；
- `inspect_marvin_state.py` 已增加 `--expected-sdk-version` 门禁，默认要求 `100343014`；版本不符会在采样前失败并释放 SDK，不再只打印版本；
- 实机控制使用 200 Hz 反馈线程、50 Hz 命令线程和 100 Hz IK 线程，SDK 只由 I/O 线程访问；启动目标等于反馈，左右 Grip 松开后各自以反馈为起点返回本次启动实测姿态，重新按下可取消；并在模式回读、Tool、错误码、反馈、命令和 XR 任一校验异常时阻止 TELEOP；
- 实机侧增加 0.1 rad/s 关节速度、0.3 rad/s² 加速度、2 rad/s³ jerk、5° 软限位及预测制动；TCP 限制为启动点 0.25 m 球形工作区、0.1 m/s、0.5 rad/s、单帧 0.15 m/45°，并按平移 Jacobian 最小奇异值降速或 FAULT；
- 实机运行自动保存 `logs/marvin_hardware_<时间戳>.jsonl` 和 `.summary.json`，包含配置/文件哈希、反馈、raw/limited/actual TCP、奇异值、IK/下发关节、SDK 耗时、watchdog 决策和安全状态转换。
- 实机反馈另以 200 Hz 自动保存为 `marvin_calibration_<时间戳>.csv` 与 `.metadata.json`，并可离线转换为包含 `q/dq/ddq/tau`、多级命令、TCP 矩阵和有效样本掩码的压缩 NPZ；
- 新增可选 ROS2 旁路观测器，使用标准消息发布关节反馈/命令、左右 TCP、Safety 和通信诊断；不订阅运动命令，不改变 MarvinSDK 单线程所有权和本地 watchdog。
- 项目说明已收敛：`README.md` 只保留 Marvin 快速入口，`teleop_details.md` 改为文档索引，`CLAUDE.md` 删除失效的 UR/ARX/R1 硬件命令；现场只以 `docs/XRoboToolkit环境部署与PICO联调流程.md` 为唯一 SOP；
- 当前 PICO 实机入口只提供 TCP 笛卡尔遥操，不提供单关节点动。`--confirmed-joint-mapping` 必须来自 MarvinPlatform 或供应商批准的低速逐轴点动与签字结论，不能由本入口自证。

当前注意事项：

- 类人放松姿态用 20°肘部弯曲避开了完全伸直姿态；为保证启动时严格保持指定关节角，Marvin 仿真仍暂时将可操作度辅助权重设为 0；首次操作使用 `scale-factor=0.5` 并缓慢移动；
- `head_yaw` 是动态参考系：头显与手柄整体同步移动时 TCP 目标不变；只移动或转动头显时，手柄相对头显的位姿会变化，因而可能驱动 TCP；
- 臂长标定通过“自然下垂→水平前伸”的手柄弦长估计肩到手柄长度 `L≈行程/√2`；仿真支持进程内即时更新。实机既支持 PICO-only 标定，也支持带使能进程内 A/A、B 操作；后者只在 Grip 松开、复位完成且静止时采样，成功后保存、切换比例并清除旧参考点，下一次 Grip 使用新值。独立 PICO-only 标定仍在随后启动时加载；启动时显式 `--scale-factor` 优先于保存值，无保存值时使用 0.5；
- PICO 重新定位、重置安全区或重置跟踪原点前必须松开 Grip，防止头显与手柄异步更新形成短时目标跳变；
- 仿真侧已有分关节速度/加速度、移动目标前馈、5° 软限位和预测制动；实机侧另有 TCP 工作空间/速度、关节 jerk、单帧跳变拒绝、奇异性保护和完整安全状态机。两侧尚未抽象为完全同一套 supervisor，现场验收时必须分别验证；
- 机械臂网格自碰撞当前禁用，仅保留机械臂与地面/支架接触；未生成凸碰撞代理和 allowed-pair matrix 前，不能将当前模型当作自碰撞安全验证器；
- `head_yaw` 已配置到 Marvin MuJoCo 和实机入口，但尚未完成 PICO 实测下的长时间、快速转头、数据停更和重定位故障注入验收。
- 实机松手复位目标是本次启动实测关节姿态，不是 MuJoCo 固定类人放松姿态；名义 3 秒余弦目标继续经过速度/加速度/jerk/软限位和预测制动。当前仍无障碍物/自碰撞在线检查，因此只允许清空工作区低速验收，实际完成时间允许长于 3 秒。

### 2.6 当前 MuJoCo 控制与延迟记录参数

以下参数是 `scripts/simulation/teleop_marvin_mujoco.py` 的当前默认仿真基线，不是实机安全参数：

| 类别 | 参数 | 当前值 |
| --- | --- | --- |
| 物理 | MuJoCo 频率/步长 | 500 Hz / 2 ms，`implicitfast` |
| 控制 | 控制频率/周期 | 100 Hz / 10 ms；每次控制计算后严格执行 5 个物理子步 |
| XR | latest-value 轮询频率 | 200 Hz，控制周期内固定使用同一原子快照 |
| 渲染 | 独立渲染频率 | 60 Hz，使用独立 `MjModel/MjData` 副本 |
| 关节速度 | Joint1～Joint7，左右镜像 | `[1.0, 1.0, 1.0, 1.2, 1.2, 1.0, 1.0] rad/s` |
| 关节加速度 | Joint1～Joint7，左右镜像 | `[3, 3, 3, 4, 4, 3, 3] rad/s²` |
| 软限位 | 机械限位内缩 | 5°；基于当前速度、加速度和 10 ms 控制周期预测制动距离 |
| 移动目标 | 速度前馈/滤波时间常数 | `0.8 / 0.04 s`；目标停止时立即清零滤波速度，避免滤波尾部超调 |
| 松手回位 | 默认时长 | 3 s 余弦插值；仿真回固定放松姿态，实机回本次启动实测姿态且继续经过关节 limiter |
| 执行器 | Joint1～Joint7 `kp`，左右镜像 | `[200, 180, 160, 140, 120, 110, 110]` |
| 执行器 | Joint1～Joint7 `kv`，左右镜像 | `[15, 14, 13, 12, 11, 10, 10]` |
| 左工具 | 质量/法兰坐标系质心 | `0.481 kg / [0.004691, -0.034036, 0.084135] m` |
| 左工具 | 惯量对角项 | `[0.005333333333, 0.011666666665, 0.006333333333] kg·m²`；原始惯量不满足刚体三角不等式，当前值为物理一致性投影 |
| 右工具 | 质量/法兰坐标系质心 | `0.459 kg / [-0.000776, 0.029685, 0.10105] m` |
| 右工具 | 惯量对角项 | `[0.006999999999, 0.006, 0.001] kg·m²` |
| 工具重力 | MuJoCo `gravcomp` | 左右工具均为 `1` |

延迟记录合同如下：

| 项目 | 当前约定 |
| --- | --- |
| 默认目录 | 仓库根目录 `logs/`；当前工作区为 `/home/zxcx/TeleOp/xrobotoolkit-marvin-teleop/logs` |
| 文件名 | 原始数据 `marvin_latency_YYYYMMDD_HHMMSS_ffffff.csv`；汇总 `marvin_latency_YYYYMMDD_HHMMSS_ffffff.summary.json` |
| 原始采样 | 每个 100 Hz 控制周期一行；包含样本序号、墙钟/单调时钟时间戳、MuJoCo 时间、周期与 deadline 延迟 |
| 分段耗时 | `cycle_ms`、`ik_and_xr_ms`、`command_ms`、`physics_ms`、最近一次 `render_ms` |
| XR 指标 | `xr_poll_age_ms`、`xr_source_age_ms`、`xr_source_timestamp_ns` |
| TCP 指标 | 双臂最大及左右臂各自的 raw→command、command→actual 距离，单位 m |
| 终端汇总 | 默认每 2 s 输出最近最多 1000 个样本（100 Hz 下约 10 s 窗口）的 P95 和累计 deadline miss |
| JSON 汇总 | 正常退出时对完整 CSV 数值列计算 latest、P50、P95、P99，并记录样本数、deadline miss、控制/物理频率和子步数 |
| 写入策略 | 无界日志队列连接控制线程与独立 CSV 写线程；每个样本均入队，CSV 每 1 s flush，正常退出时排空队列后生成 JSON |
| 命令行 | `--telemetry-report-interval` 调整终端汇总周期；`--telemetry-output-dir` 修改落盘目录 |

强制结束进程（例如 `SIGKILL` 或断电）时无法生成 JSON，CSV 最多可能丢失最后约 1 s 尚未 flush 的数据。配置、自动测试和无 PICO 的仿真性能结果不能替代实测延迟基线；仍需连接 PICO 后按统一动作脚本采集不少于静止、慢速连续跟随、快速往返和双臂同时运动四组数据，并归档对应 CSV、JSON、启动参数和异常现象。

### 2.7 当前实机软件基线

下表按 `configs/marvin_hardware.json` 与实机入口默认参数交叉核对。JSON 当前只是审查快照，并非运行时输入；实际执行以 CLI `--help` 和代码默认值为准。它是首次现场调试的保守上限，不是已经验收的生产参数：

各参数的控制层级、代码入口、手动覆盖和验证方法统一记录在
[`Marvin机械臂控制与测试调参指南.md`](Marvin机械臂控制与测试调参指南.md)。

| 类别 | 当前软件值 |
| --- | --- |
| 默认使能 | `false`；必须同时提供 hardware、急停、映射和准确机型四项确认 |
| SDK | 预期 `100343014`；本机 Python/C ABI 探测 `(1,0)`，控制器版本仍待实机记录 |
| 平移 scale | 启动优先级为显式 CLI > PICO 保存标定 > 默认 0.5；独立标定在随后启动时加载，进程内 A/A 在安全静止窗口切换并于下一次 Grip 生效 |
| Grip 松手复位 | 默认启用；RETURNING；目标为本次启动实测关节姿态；名义 3 s，受关节 limiter 约束 |
| 调度 | XR 200 Hz、IK 100 Hz、反馈 200 Hz、命令 50 Hz；PD 速度估计周期 20 ms |
| 关节运动 | 0.1 rad/s、0.3 rad/s²、2 rad/s³、机械限位内缩 5°并预测制动 |
| TCP | 启动 TCP 半径 0.25 m；线速度 0.1 m/s；角速度 0.5 rad/s；单帧拒绝 0.15 m/45° |
| 奇异性 | 平移 Jacobian `sigma_min≤0.003` FAULT，`0.003–0.015` 连续降速 |
| 上使能静止门槛 | A/B `low_speed_flag` 均为真，且最大反馈关节速度不超过 0.02 rad/s；模式切换前再次检查 |
| 控制器比例 | 速度/加速度均 10%；`state=3`，`impedance_type=1` |
| K | 左右均 `[2,2,2,1.5,0.8,0.8,0.8]` |
| D | 左右均 `[0.3,0.3,0.3,0.2,0.2,0.2,0.2]` |
| watchdog | XR HOLD/FAULT 100/500 ms；反馈 HOLD/FAULT 30/100 ms；命令有效期 40 ms |
| 跟踪误差 | 最大关节误差 3° HOLD、8° FAULT |
| 工具 | 默认只比对控制器当前 Tool 与 `tools_cfg.json`，不写入；只有显式 `--configure-tools` 才按供应商原始单位写入 |

推荐现场入口顺序：

```bash
# 1. 短时只读门禁
python scripts/hardware/inspect_marvin_state.py --duration-s 10

# 2. 短时通过后，按默认 200 Hz 持续 30 分钟并保存基线
python scripts/hardware/inspect_marvin_state.py --duration-s 1800

# 3. 用供应商批准的低速逐轴点动完成映射确认并签字；本项目不提供该运动入口

# 4. 仅在第 9.2 节全部为 Go 后，由急停观察员在场执行
python scripts/hardware/teleop_marvin_hardware.py \
  --enable-hardware \
  --confirmed-estop \
  --confirmed-joint-mapping \
  --confirmed-robot-model "现场读取的准确型号"
```

硬件日志不是仿真 CSV 的替代品。JSONL 中 `robot_state` 保存 A/B 帧号、`q/dq/tau`、状态/错误/丢帧计数；`control_cycle` 保存 XR 数据龄期、raw/limited/actual TCP、奇异值、IK 和受限关节命令；`hardware_command_decision/sent` 保存命令序号、安全决策与 SDK 发送耗时。summary 保存完整启动配置、模型/工具哈希、事件数和最终状态。发生 `SIGKILL` 或断电时 summary 可能不存在，因此现场记录还必须保留控制器日志和录像。

### 2.8 ROS2 旁路观测与 MuJoCo 标定数据

ROS2 默认关闭，使用 `--enable-ros2-observation` 后只发布标准消息，不存在 ROS2 运动命令订阅者：

| 话题（默认命名空间 `/marvin_teleop`） | 消息 | 内容 |
| --- | --- | --- |
| `joint_states` | `sensor_msgs/JointState` | 14 轴反馈 `q/dq/tau` |
| `joint_command` | `sensor_msgs/JointState` | 软件安全限幅后的 14 轴目标，仅供观测 |
| `left/right/tcp_actual` | `geometry_msgs/PoseStamped` | 反馈关节经当前 URDF FK 得到的 TCP |
| `left/right/tcp_target_raw` | `geometry_msgs/PoseStamped` | PICO 映射后的原始 TCP 目标 |
| `left/right/tcp_target_limited` | `geometry_msgs/PoseStamped` | 工作区、速率和奇异性保护后的 TCP 目标 |
| `safety_state` | `std_msgs/String` | JSON 编码的状态与原因 |
| `diagnostics` | `diagnostic_msgs/DiagnosticArray` | 帧号、数据龄期、SDK 读取耗时与错误码 |

高频观测使用 `BEST_EFFORT + KEEP_LAST(1)`，Safety 使用 `RELIABLE + TRANSIENT_LOCAL`。ROS2 发布线程只读取 latest-value 快照；发布失败会写入本地事件日志并标记观测失效，但不会获得 SDK 控制权或绕过本地 watchdog。

每次实机运行无条件增加以下文件：

- `marvin_calibration_<时间戳>.csv`：按 200 Hz 反馈周期逐样本写入，后台线程每秒 flush；
- 同名 `.metadata.json`：模型和工具哈希、Tool 原始参数、K/D、频率、限幅、样本统计、帧号范围和写入错误；
- 通过 `prepare_marvin_mujoco_calibration.py` 生成的 `.npz`：`q/dq/ddq/tau`、控制器/软件/IK 目标、TCP 矩阵和 `valid_dynamics_mask`。

CSV 使用主机单调时间关联 200 Hz 反馈与最近的 100 Hz 控制快照，不直接相减 PICO 与主机的不同时钟。`tcp_actual` 是当前 URDF 对反馈关节做 FK 的结果，不能独立证明连杆尺寸正确；运动学几何校正仍需光学测量、激光跟踪或已标定外部相机。动力学拟合应分空载/左右已知 Tool、无接触、低中速往返采集，并保留原始数据，不能只使用数值微分后的 `ddq`。

### 2.9 下一步执行优先级

1. **P0，离线关闭**：为 CLI 默认值与 `marvin_hardware.json` 增加单一真值来源或漂移测试；补固定 XR/反馈回放、帧号回绕、线程退出和 ROS2 隔离测试。
2. **P0，PICO 实测**：按统一动作脚本完成静止、慢速连续、快速往返和双臂同时运动，记录延迟基线；完成 `Send` 停更、重定位和快速转头故障注入。
3. **P1，现场身份冻结**：确认准确机型、控制器版本、`.MvKDCfg`、基座安装、TCP/Tool，并通过供应商低速逐轴点动签署 identity mapping；若不是 identity，先回到软件开发。
4. **P1，只读与低速实机**：10 秒只读、30 分钟只读、单臂 TCP ≤5 cm、双臂空载、Grip 松手和急停测试严格按现场 SOP 逐级执行。
5. **P2，数据驱动优化**：使用实机 CSV/NPZ 校正 MuJoCo；只有 50 Hz 实测周期、反馈龄期和跟踪误差通过后才评估 100/200 Hz；只有 limiter 引起的 TCP 误差被数据确认后才迁移约束 QP。

## 3. 目标架构与频率

```text
PICO 4 Ultra / XRoboToolkit Unity Client
                 ↓ Wi-Fi
XRoboToolkit PC Service + xrobotoolkit_sdk
                 ↓ 30–60 Hz，带源时间戳
BufferedXrClient 原子 latest-value 快照（主机轮询 200 Hz）
                 ↓
BaseTeleopController 增量目标映射
  独立 Grip deadman、增量 SE(3)、坐标转换、跳变抑制
                 ↓ 左右 TCP 目标
             Placo IK（100 Hz）
            ┌────────┴─────────┐
            ↓                  ↓
 MuJoCo limiter/执行器      TCP guard + feedback-aware joint limiter
 控制 100 Hz、物理 500 Hz       ↓ MarvinJointCommand[14] latest-value
                               SafetySupervisor / HOLD / FAULT
                                  ↓ 50 Hz 双臂关节目标
                         Marvin_Robot / libMarvinSDK.so
                                  ↑ 200 Hz 主机轮询
                         DCSS 反馈与 A/B 帧号 watchdog

硬件旁路观测：ROS2 标准话题 100 Hz + 标定 CSV 200 Hz + JSONL 事件日志
```

首版仍采用单机进程内架构，不强制引入 ROS2。XR、IK、反馈和命令通过带锁最新值槽交换，不使用无界队列积压旧命令。需要跨进程/跨主机时，再为公共数据合同增加 ROS2 适配器，核心控制逻辑不依赖 ROS2。

## 4. 复用范围与代码组织

### 4.1 复用但必须加固的上游能力

- `XrClient`：复用 PICO 位姿、Grip、A/B 和源时间戳读取；实机不订阅 Trigger、X/Y、菜单、摇杆或追踪器；
- `BaseTeleopController`：复用末端任务和增量遥操思路；Marvin 不直接沿用其“求解异常只打印后继续”的行为；
- `HardwareTeleopController`：经审查其通用行为不满足 Marvin 的 SDK 单线程所有权与锁存故障要求，因此 Marvin 使用专用硬件控制器，只复用 `BaseTeleopController` 的 IK/映射合同；
- `MujocoTeleopController`：复用模型加载、仿真、目标可视化和 Placo IK，并加入 Marvin 模型、关节 limiter、调度和延迟记录；仿真安全逻辑不冒充实机 SafetySupervisor；
- `DataLogger`：复用保存入口，补齐源时间戳、反馈帧号、命令序号和安全状态。

当前继续使用 Placo 求解末端任务，并在解后通过基于反馈和命令历史的独立 limiter 强制关节位置、速度、加速度、jerk 与预测制动约束。它不是带硬约束的完整 QP-WBC；现场若证明解后投影导致明显 TCP 误差或奇异附近抖动，再迁移为 ProxQP/Pinocchio 约束求解，并固定相关依赖。

### 4.2 已实现文件

```text
assets/marvin/
├── marvin_dual.urdf
├── marvin_dual.xml
└── meshes/

configs/
└── marvin_hardware.json

xrobotoolkit_teleop/
├── common/
│   ├── marvin_types.py
│   ├── joint_command_limiter.py
│   ├── cartesian_target_guard.py
│   ├── marvin_safety.py
│   ├── marvin_session_logger.py
│   ├── marvin_observation.py
│   └── marvin_calibration_recorder.py
├── hardware/
│   ├── marvin_teleop_controller.py
│   ├── marvin_ros2_observer.py
│   └── interface/
│       └── marvin.py
└── simulation/（复用并加固现有 MuJoCo 控制器）

scripts/
├── simulation/teleop_marvin_mujoco.py
├── hardware/teleop_marvin_hardware.py
├── hardware/inspect_marvin_state.py
└── misc/prepare_marvin_mujoco_calibration.py

tests/
├── test_marvin_mujoco_model.py
└── test_marvin_hardware.py
```

供应商 SDK 仍由 `--sdk-root` 部署路径动态注入，不复制到业务模块。运行时记录 SDK 根目录和版本；发布前还需补录动态库哈希、控制器兼容版本、LICENSE 和 x86_64 ABI 说明。

## 5. 公共数据与坐标合同

### 5.1 内部单位和时间

- 所有内部位置为 m、关节角为 rad、速度为 rad/s；
- 所有旋转使用正交 SO(3) 矩阵或单位四元数，不在核心接口中使用欧拉角；
- 调度、超时和命令有效期使用 `time.monotonic_ns()`；
- XR 的设备时间戳作为数据字段保留，但不直接与主机单调时钟相减，除非完成时钟域标定；
- SDK 的 degree/mm 只允许在 `hardware/interface/marvin.py` 边界转换，并对转换前后做有限值和范围检查。

当前代码的数据合同如下，早期草案中的 `PoseSE3/RawXRFrame/DualArmTarget` 类并未实现，不再作为接口承诺：

```python
# BufferedXrClient 原子 latest-value 快照
{
    "sequence": int,
    "source_timestamp_ns": int,
    "receipt_monotonic_ns": int,
    "source_age_ms_at_receipt": float,
    "poses": {
        "headset": np.ndarray(7),          # [x,y,z,qx,qy,qz,qw]
        "left_controller": np.ndarray(7),
        "right_controller": np.ndarray(7),
    },
    "keys": {"left_grip": float, "right_grip": float},
    "buttons": {},
    "motion_trackers": {},
}

MarvinRobotState(
    receipt_monotonic_ns, frame_serial, q_rad, dq_rad_s, torque_nm,
    arm_state, command_state, error_code, low_speed,
    input_frame_serial, frame_miss_count, system_cycle_miss_count,
    commanded_q_rad,
)

MarvinJointCommand(
    sequence, created_monotonic_ns, q_rad, active_arms,
)

MarvinControlObservation(
    sequence, monotonic_ns, duration_ms, deadline_lateness_ms, deadline_miss,
    xr_sequence, xr_source_timestamp_ns, xr_poll_age_ms, xr_source_age_ms,
    q_ik_rad, q_command_rad,
    active_arms, raw_tcp_transforms, limited_tcp_transforms,
    actual_tcp_transforms, translational_sigma_min,
)
```

`MarvinJointCommand.sequence/created_monotonic_ns/active_arms/returning_arms` 用于主机内部 watchdog、TELEOP/RETURNING 和 HOLD 逻辑；适配层周期下发给 Marvin 的业务目标只有 A/B 各 7 个关节角，内部 rad 在 SDK 边界转为 degree。

### 5.2 关节合同

模型统一顺序：

```text
Joint1_L ... Joint7_L, Joint1_R ... Joint7_R
```

SDK 映射为：

```text
SDK A[0:7] ↔ model Joint1_L...Joint7_L
SDK B[0:7] ↔ model Joint1_R...Joint7_R

q_model_rad[i]                  = deg2rad(sign[i] * q_sdk_deg[source_index[i]] + offset_deg[i])
q_sdk_deg[source_index[i]]      = sign[i] * (rad2deg(q_model_rad[i]) - offset_deg[i])
```

速度映射应用顺序、比例和符号，不应用位置零偏。当前实机入口只实现 A→左、B→右、同序、同号、零偏的 identity mapping；必须先通过供应商低速逐轴点动确认后才能提供 `--confirmed-joint-mapping`。若现场结果不是 identity mapping，应先实现显式 `source_index/sign/offset` 配置、回读和测试，不得仅修改确认文本后上使能。

### 5.3 基座、TCP 与工具合同

- Pinocchio/Placo 和 MuJoCo 使用共同 `dual_origin`；SDK 左右臂目标仍分别解释在 `Base_L`、`Base_R`；
- `T_dual_BaseL`、`T_dual_BaseR` 必须来自实测安装或已确认 CAD；
- `TCP_Link_L/R` 表示控制 TCP，必须与控制器 `Tool` 一致；不得同时在 URDF 固定关节和 SDK `Tool` 中重复增加夹爪长度；
- 若调用供应商笛卡尔接口，只在适配层进行 m↔mm、rad↔degree 和 `Rz·Ry·Rx` 转换；主遥操链路首版仍下发关节位置。

当前辨识结果仅作为待复核配置输入，不代表已经在控制器生效：

```yaml
tool_identification:
  source: "TJArm/tools_cfg.json"
  left_A:
    kine_sdk: [0, 0, 0, 0, 0, 0]
    dyn_sdk: [0.481, 4.691, -34.036, 84.135, 0.001, 0, 0, 0.016, 0, 0.002]
  right_B:
    kine_sdk: [0, 0, 0, 0, 0, 0]
    dyn_sdk: [0.459, -0.776, 29.685, 101.050, 0.007, 0, 0, 0.006, 0, 0.001]
```

`dyn_sdk` 的 10 项保持供应商原始顺序和单位，由 SDK 适配层原样设置；复核时需确认左右安装物、工具编号和辨识数据没有变化。

## 6. 安全状态机与硬件前提

```text
DISCONNECTED
    ↓ A/B 反馈双帧递增、SDK 版本和错误检查通过
READ_ONLY（只订阅，不发送目标）
    ↓ 四项 CLI 授权、Tool/静止检查、反馈目标同步、模式回读正确
ARMED
    ↓ Grip 按下并锁存 XR/TCP 起点
TELEOP ──Grip 松开──→ RETURNING ──到达启动姿态──→ ARMED
  │                       │
  └──XR/反馈/命令异常─────┴──→ HOLD/FAULT（按阈值，FAULT 锁存）

FAULT --当前进程退出并下伺服；人工排障后重新启动--> READ_ONLY
```

安全约束：

- 实机脚本必须同时提供 `--enable-hardware`、`--confirmed-estop`、`--confirmed-joint-mapping` 和非空 `--confirmed-robot-model`；IP、机型、工作区和物理急停的真实性仍由现场清单和签字保证；
- 启动时不回零、不跳到配置姿态，所有初始目标取当前反馈；
- Grip 从 ARMED 进入单臂 TELEOP，松开进入对应臂 RETURNING；不能自动清除 FAULT 或自动上伺服；
- HOLD 锁存最新反馈安全位置，不继续追赶旧 XR 目标；
- 命令必须带递增序号和绝对有效期，过期、重复、NaN/Inf、越限或跳变命令不得下发；
- SDK/伺服错误不得在后台无限自动 `check_error_and_clear()`；读取并记录错误，人工确认原因后执行一次显式清错；
- 正常退出前先等待 Grip 松手复位完成；随后按已验证流程切 `state=0`，确认状态后 `release_robot()`；
- 软件 `soft_stop('AB')` 只是软件急停手段，不能替代物理急停。

物理急停是实机 Go/No-Go 前提。根据供应商说明，应确认控制器 `/home/FUSION/Config/cfg/robot.ini` 中 `UseEMG=1`，单独断电重启控制器后实际测试急停：触发后应自动下伺服，复位后必须清错并重新显式上伺服。修改控制器参数属于现场维护操作，执行前要备份 `robot.ini`，不得由遥操程序自动修改。

## 7. 分阶段实施计划

### M0：锁定软件与设备基线（2 个工程日，软件基线完成、设备长测待完成）

工作内容：

- 保留已锁定的 Python Sample commit 和已补录的 PC Service、Pybind、PICO 客户端版本及安装包哈希；按联调流程归档后续升级结果；
- 记录 `libMarvinSDK.so` 哈希、SDK `SDK_version()`、控制器 `VERSION`、CPU 架构和系统版本；
- 确认网卡静态地址、`ping 192.168.1.190`、防火墙和单进程占用；
- 建立 Marvin XR/控制数据记录、pytest、格式检查和最小 CI；上游多机器人示例仅作为版本来源，不属于当前现场 SOP。

验收：当前使用的头显、左右手柄位姿和 Grip 连续 30 分钟有效；所有版本写入可追溯清单；Marvin 自动回归通过。Trigger、手部追踪和其他机器人入口不属于首版验收范围。

### M1：真机身份、模型与工具参数冻结（3 个工程日）

工作内容：

- 从铭牌、控制器配置和供应商确认真机是 M6S、M6S-Lite 或其他型号；
- 选择匹配的 `.MvKDCfg`，核对 `TYPE/DH/PNVA/BD`；
- 以供应商 `marvin_m6s_lite_dual_ccs_680.urdf/xml` 为候选，不匹配则重新生成正确模型；
- 固定 `dual_origin` 到双基座的安装变换、`TCP_Link_L/R` 和夹爪模型；
- 复核已导入仿真的 `tools_cfg.json` 左右工具参数、左侧惯量物理一致性修正，并由现场人员确认；
- 使用 MarvinPlatform 或供应商批准的点动方法，以单轴不超过 2°、速度不超过 0.1 rad/s 建立 SDK A/B 到模型 L/R 的顺序、方向、零偏和单位标定表；该步骤不使用 PICO 实机入口。

验收：

- 运动学模型为 `nq=14, nv=14`（不含夹爪关节）；
- URDF 与 MuJoCo 的 14 个臂关节、TCP 和基座变换一致；
- 供应商 FK 与本工程 FK 在不少于 20 个安全姿态上，位置误差 < 1 mm、姿态误差 < 0.2°；
- 逐关节正方向、软限位和反馈单位经人工签字确认；
- 模型安全限位采用“供应商限位再内缩”，且不小于关节阻抗模式自带的 1.5°内缩要求。

### M2：MarvinSDK 只读适配与离线测试（3 个工程日）

软件状态：**已实现并通过 mock 自动测试；30 分钟真机只读验收未执行。**

实现在 `hardware/interface/marvin.py`：

- 单次 `connect(ip)` / `release_robot()` 生命周期；
- `subscribe(DCSS)` 到 `RobotState` 的 A/B 合并、degree→rad、帧号和错误码解析；
- `clear_set()`、目标设置、`send_cmd[_wait_response]()` 的原子化封装；
- SDK 动态库加载、ABI、版本和单进程占用检查；
- mock 后端模拟帧号回绕（0～1,000,000）、停更、错误码、延迟和断网；
- 新增 `inspect_marvin_state.py`，默认只读并打印双臂状态，不切模式、不清错、不运动；默认强制 SDK 版本 `100343014`，只有供应商确认升级后才修改预期值。

连接验收不能只判断 `connect()`：连续采样时 A/B `frame_serial` 均需非零且变化；启动实时反馈所需的 SDK 握手不得改变模式或目标。持续 30 分钟无帧停更、单位异常和资源泄漏。

### M3：XR 增量映射与安全 IK（5 个工程日）

软件状态：**增量映射、TCP guard、奇异性降速和反馈式关节硬限幅已实现。当前仍是 Placo IK 后接 limiter，并非完整约束 QP；是否迁移 QP 由现场跟踪误差和抖动数据决定。**

XR 映射：

```text
p_target = p_robot_start
         + motion_scale · R_xr_to_robot · (p_xr - p_xr_start)

R_rel    = R_xr · R_xr_startᵀ
R_delta  = R_xr_to_robot · R_rel · R_xr_to_robotᵀ
R_target = R_delta · R_robot_start
```

- Grip 上升沿分别锁存手柄和对应 TCP 起点；左右臂独立激活；
- Grip 释放后相应实机手臂从反馈位置开始，以受限余弦目标返回本次启动实测姿态；另一臂不受影响，重新按 Grip 可取消回位；
- 支持 `head_yaw`、`head_locked`、`world`，首版默认 `head_yaw`；
- 对 SO(3) 正交化，拒绝零四元数、NaN/Inf、时间戳回退和单帧大跳变；
- XR 最新值零阶保持，不做无条件外推。

若现场数据证明解后 limiter 造成不可接受的 TCP 误差或抖动，再迁移到以下约束 QP；这不是当前实现：

```text
决策变量：q_dot[14]

min ||W(J q_dot - v_ref)||²
    + q_dotᵀ R q_dot
    + 舒适姿态零空间代价

s.t. -v_max <= q_dot <= v_max
     (q_safe_min-q)/dt <= q_dot <= (q_safe_max-q)/dt

q_next = integrate(q, q_dot · dt_measured)
```

- Jacobian、位姿误差和 `dq` 阻尼项使用同一参考系；
- 位置任务高于姿态任务，肩/肘/腕正则可配置；
- 按最小奇异值连续增加阻尼；
- 使用实测且夹紧到合理范围的 `dt`；
- 单臂未激活时，其任务目标持续同步反馈而不是保留旧目标；
- 连续求解失败达到阈值后进入 FAULT。

验收：Grip 按下/松开 100 次无跳变；单臂释放不影响另一臂；所有输出在安全限位和速度界内；奇异附近无速度爆炸；求解耗时 P99 < 2 ms（目标机器）；无非法旋转、NaN 或 Inf。

### M4：PICO → MuJoCo 闭环（4 个工程日）

软件状态：**闭环、500/100 Hz 调度与自动延迟记录已实现；PICO 真机长时间与故障注入验收未执行。**

工作内容：

- 接入经 M1 确认的双臂 URDF/XML；
- 分离 XR、IK 控制和 MuJoCo 步进频率，使用最新值槽；
- 显示左右 TCP 当前位姿、目标位姿和安全状态；
- 支持无 PICO 的 XR 日志回放；
- 注入 XR 丢帧/时间戳回退/NaN/瞬移、反馈延迟和求解失败；
- 对照供应商 `showcase_pln_multi_segment_linear_two_arms_classes.py sim` 的关节轨迹和 TCP 结果。

验收：双臂完成圆、直线和姿态轨迹；XR 超过 100 ms 未更新进入 HOLD，超过 500 ms 进入 FAULT；无界面 30 分钟无越限和数值发散；故障恢复必须重新锁存起点。

### M5：实机后端与模式切换（5 个工程日）

软件状态：**后端、模式切换、安全状态机、自动日志和显式授权已实现并通过 mock 测试；所有实机步骤均未执行。**

首版 50 Hz 下发链路：

```text
connect(192.168.1.190)（只调用一次）
  ↓
subscribe(DCSS)，验证 A/B frame_serial 递增
  ↓
SDK A/B degree → model L/R rad → RobotState
  ↓
读取最新、递增且未过期的 JointCommand
  ↓
model L/R rad → SDK A/B degree
  ↓
clear_set → set_joint_cmd_pose(A/B) → send_cmd
```

初始化顺序：

1. 完成物理急停、观察员和工作区检查，同时提供 hardware、急停、逐轴映射和准确机型四项 CLI 确认；
2. 确认 MarvinPlatform 和其他 SDK 进程已经释放控制器；
3. 连接 UDP，校验 SDK 版本；控制器版本和准确机型必须已在 M1 外部确认并由 CLI 字符串记录，当前代码不自动证明机型匹配；
4. 只读反馈，验证 A/B 帧号、状态、错误码和 14 轴有限值；
5. 将当前反馈设为内部初始目标，不发送回零或舒适姿态；
6. 默认只比对并回读已确认的 Tool，同时设置/回读速度、加速度百分比和低刚度 K/D；只有独立完成工具复核且显式使用 `--configure-tools` 时才写 Tool；
7. `state=3`、`impedance_type=1`，用 `send_cmd_wait_response(100)` 发送；预留模式切换时间并回读 `cur_state==3`、`imp_type==1`；
8. 连续发送等于反馈的保持目标，保持 ARMED，等待操作者 Grip；
9. 任一校验失败均执行安全停机，不进入 TELEOP。

初始 K/D 不直接照搬旧控制链路或供应商“上限值”。从供应商 M6S-Lite 适中示例 `K=[5,5,5,4,3,3,2]`、`D=[0.3,0.3,0.3,0.2,0.2,0.2,0.2]` 以下的保守值开始，由现场阶跃和外力实验逐臂辨识；左右臂、负载或控制器版本不同必须独立保存参数。

验收：命令频率 50 Hz 下 P99 周期、反馈龄期和 SDK 往返延迟达标；重复/过期命令被拒绝；网络中断、反馈停更和线程退出均停止旧轨迹；退出不回零，完成 `state=0` 回读和 `release_robot()`。只有在这些数据通过后，才能分别尝试 100 Hz、最高 200 Hz。

### M6：实机分级运动验收（3 个工程日，未开始）

必须按顺序执行，每级通过后才能进入下一级：

1. 下伺服只读 14 轴反馈 10 秒，随后连续记录 30 分钟；两次均要求 SDK 版本正确且 A/B 帧独立递增；
2. 通过 MarvinPlatform 或供应商批准的低风险方式核对每轴序号、正方向、零偏、内外编码器和传感器方向；
3. 供应商逐轴点动：单轴幅度 ≤ 2°、速度 ≤ 0.1 rad/s，签字后才允许使用 `--confirmed-joint-mapping`；本项目 PICO 入口不执行此步骤；
4. 启动本项目关节阻抗遥操，先做单臂保持与微动：TCP 位移 ≤ 5 cm，另一侧 Grip 始终松开；
5. 双臂空载小范围运动；在保守参数验收完成前保持软件关节限速 0.1 rad/s，不提前提高到 0.3 rad/s；
6. 左右 Grip 各松开 100 次，确认相应实机手臂进入 RETURNING、受限返回启动姿态且不影响另一臂；
7. 分别关闭 PICO `Send`、停止控制线程、停止硬件线程、断开控制网络，验证 HOLD/FAULT 和下伺服行为；
8. 触发物理急停，验证自动下伺服；复位/清错后不得自动恢复运动；
9. 安全姿态下施加小外力，验证关节阻抗柔顺性并记录 K/D 响应。

所有实机测试必须具备物理急停、观察员、清空的工作区和测试记录。第一次上使能不得安装易损工件。

### M7：夹爪、相机与数据（可选，4 个工程日）

当前状态：**关节/TCP/Safety ROS2 旁路观测、200 Hz 标定 CSV 和 NPZ 准备工具已实现；夹爪、相机、rosbag2/图像同步、回放和 LeRobot 转换未实现。**

- PGC 夹爪放在独立串口线程，Trigger 模拟量映射开度；串口失败不能阻塞双臂；
- 接入头部和左右腕部相机；
- 日志记录原始 XR、映射 TCP、`q/dq/tau`、`q_des`、双臂反馈帧号、IK 误差/奇异值/耗时、Safety 状态及转换原因、SDK 状态/错误和图像；
- 提供日志完整性检查、无界面回放和 LeRobot 转换工具。

## 8. 首版配置建议

以下是启动模板，不是可直接上真机的最终参数。所有 `TBD_CONFIRM` 必须在 M1/M5 关闭。

```yaml
runtime:
  xr_source_hz_expected: 30-60
  xr_poll_hz: 200
  ik_control_hz: 100
  feedback_hz: 200
  hardware_command_hz: 50
  hardware_command_hz_max_vendor: 200
  hardware_command_hz_max_validated: TBD_REAL_ROBOT
  mujoco_control_hz: 100
  mujoco_physics_hz: 500

model:
  robot_model: TBD_CONFIRM
  kine_config: TBD_CONFIRM
  joint_order:
    [Joint1_L, Joint2_L, Joint3_L, Joint4_L, Joint5_L, Joint6_L, Joint7_L,
     Joint1_R, Joint2_R, Joint3_R, Joint4_R, Joint5_R, Joint6_R, Joint7_R]
  left_tcp: TCP_Link_L
  right_tcp: TCP_Link_R

xr:
  left_pose_source: left_controller
  right_pose_source: right_controller
  left_deadman: left_grip
  right_deadman: right_grip
  buttons_used_by_hardware: [A, B]
  motion_trackers_used: false
  motion_scale: 0.5
  reference_mode: head_yaw
  r_pico_to_marvin_world: [0, 0, 1,
                           1, 0, 0,
                           0, 1, 0]

ik_and_limits:
  solver: Placo_IK_then_feedback_aware_limiter
  complete_constrained_qp: false
  velocity_limit_rad_s: [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1,
                         0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
  acceleration_limit_rad_s2: 0.3
  jerk_limit_rad_s3: 2.0
  target_natural_frequency_rad_s: 8.0
  limit_margin_deg: 5.0
  tcp_displacement_m: 0.25
  tcp_linear_speed_m_s: 0.1
  tcp_angular_speed_rad_s: 0.5
  tcp_single_frame_jump_m: 0.15
  tcp_single_frame_jump_deg: 45.0
  singularity_fault_sigma: 0.003
  singularity_full_speed_sigma: 0.015

hardware:
  enable_by_default: false
  ip: 192.168.1.190
  sdk_expected_version: 100343014
  controller_expected_version: TBD_CONFIRM
  robot_model: TBD_CONFIRM
  left_sdk_arm: A
  right_sdk_arm: B
  joint_mapping: identity_only_after_external_signoff
  target_state: 3
  impedance_type: 1
  pd_velocity_estimation_period_ms: 20
  velocity_ratio_percent: 10
  acceleration_ratio_percent: 10
  joint_k_left: [2, 2, 2, 1.5, 0.8, 0.8, 0.8]
  joint_d_left: [0.3, 0.3, 0.3, 0.2, 0.2, 0.2, 0.2]
  joint_k_right: [2, 2, 2, 1.5, 0.8, 0.8, 0.8]
  joint_d_right: [0.3, 0.3, 0.3, 0.2, 0.2, 0.2, 0.2]
  tool_default_action: compare_only
  tool_write_requires_flag: configure_tools

safety:
  xr_hold_ms: 100
  xr_fault_ms: 500
  feedback_hold_ms: 30
  feedback_fault_ms: 100
  command_validity_ms: 40
  tracking_error_hold_deg: 3
  tracking_error_fault_deg: 8
  max_solver_failures: 3
  fault_latched: true

observation:
  ros2_enabled_by_default: false
  ros2_namespace: /marvin_teleop
  ros2_publish_hz: 100
  ros2_control_authority: false
  calibration_csv_enabled: true
  calibration_sample_hz: 200
```

## 9. 测试矩阵与 Go/No-Go

### 9.1 自动测试

当前 `40 passed` 覆盖：

- XR latest-value 原子快照、两点臂长标定、PICO→Marvin 方向和 `head_yaw` 参考系；
- PICO scale 当前值/历史持久化、重启加载、显式 CLI 优先级和无效文件 fail-closed；
- 真机 Grip 松手单臂 RETURNING、重按取消、启动姿态目标及进程内 A/B 标定安全保存、即时切换和下一次 Grip 新零点；
- 14 轴名称/顺序/限位/TCP、URDF↔MuJoCo 合同、工具质量/质心/惯量和关节映射往返；
- MuJoCo 启动无跳变、单臂松手回位、目标球即时更新、速度/加速度/软限位/预测制动和执行器跟踪；
- SDK degree↔rad、A/B 双臂事务、启动保持应答、`clear_set()` 短暂忙恢复/永久忙超时、Tool 供应商原始单位、A/B 帧分别停更、反馈龄期和单次释放；
- 关节指令持续跟踪时的速度/加速度/jerk 上限、jerk 制动速度前瞻，以及 Grip 松开待机目标的精确保持；
- 只读 SDK 版本门禁、默认禁用实机、实机版本失配释放；
- Safety 的 RETURNING/HOLD/FAULT、Grip 释放后重 ARM、反馈/命令过期、运行中离开关节阻抗模式；
- TCP 工作区/速度/跳变保护、JSONL 日志、同步标定 CSV 和 NPZ 转换。

仍需增加：固定 XR/反馈录制回放、帧号回绕、真实网络断开、线程退出、ROS2 关闭隔离、SO(3) 边界和目标机器周期压力测试。未完成项不得在文档中标为已有自动覆盖。

### 9.2 进入实机的硬门槛

| 条件 | Go 标准 |
| --- | --- |
| 版本与机型 | SDK、控制器、模型、`.MvKDCfg` 全部记录且匹配 |
| 模型一致性 | FK 对照和逐关节方向验收通过 |
| 只读基线 | 10 秒与 30 分钟检查均通过；SDK 版本正确，A/B 帧独立递增且无错误码 |
| 关节映射 | 供应商逐轴点动 ≤2°/≤0.1 rad/s 完成签字；PICO 入口不负责该验证 |
| Tool | 控制器当前左右 Tool 与 `tools_cfg.json` 选中项一致，负载未变化 |
| 物理急停 | `UseEMG=1` 已生效，实际触发/复位测试通过 |
| 仿真稳定性 | 30 分钟故障注入无越限、无数值发散 |
| Watchdog | XR、反馈、命令和求解器故障全部进入预期状态 |
| 默认行为 | 启动不运动、退出不回零、FAULT 不自动恢复 |
| 人员与场地 | 观察员、急停操作者和清空工作区就绪 |

任一项为 No-Go 时，只允许只读或仿真调试。

## 10. 关键风险与处理

| 风险 | 处理方式 |
| --- | --- |
| 示例模型与真机型号不一致 | M1 从铭牌和控制器配置冻结型号；不用文件名猜测 |
| SDK ZIP 无 commit | 记录归档、动态库和文档哈希；运行时校验 SDK 版本，控制器版本由 M1 现场记录和供应商兼容表确认 |
| UDP `connect` 假阳性 | A/B 反馈帧号 watchdog，不能只看返回值 |
| SDK 单位污染核心控制 | 仅适配层做 degree/mm↔SI；单元测试覆盖每个字段 |
| 当前入口仅支持 identity 关节映射 | 供应商逐轴点动签字；若结果非 identity，先实现映射配置与测试，禁止仅勾选确认参数 |
| TCP 偏置重复 | 对比 URDF `JointTCP` 与 SDK Tool；统一唯一 TCP 真值来源 |
| Python 线程抖动 | 首版 50 Hz 硬件下发，记录 P50/P95/P99；按数据验证 100/200 Hz，必要时下沉 C++ |
| SDK 被其他程序占用 | 启动前检测，单次连接；所有退出路径 `release_robot()` |
| 过期目标持续执行 | 最新值槽、序号、有效期、feedback/XR watchdog 和锁存 FAULT |
| 自动清错掩盖故障 | 错误只记录并停机，人工确认后显式清错 |
| 工具参数或负载变化 | 工具参数带来源哈希；更换夹爪/负载后重新辨识和验收 |
| 关节阻抗参数过激 | 从低 K/D 单臂调试，不复制其他控制器版本的参数 |
| 物理急停未启用 | `UseEMG=1`、断电重启和实测作为实机硬门槛 |

## 11. 工期与里程碑

首版低速双臂 VR 遥操预计 21～25 个工程日，外加可选外设 4 日：

| 周期 | 里程碑 | 当前状态 | 交付 |
| --- | --- | --- | --- |
| 第 1 周 | M0/M1 | M0 软件基线完成；M1 未完成 | 软件版本、真机身份、双臂模型、工具与关节合同 |
| 第 2 周 | M2/M3 | 软件实现和 mock/模型测试完成；真机与回放压力测试待完成 | SDK 只读适配、XR 映射、安全 IK/limiter 和自动测试 |
| 第 3 周 | M4 | 仿真链路完成；PICO 长测与故障注入待完成 | PICO→MuJoCo 闭环、回放和故障注入 |
| 第 4 周 | M5 | 软件后端完成；实机模式切换未执行 | MarvinSDK 后端、模式回读、50 Hz 低速控制 |
| 第 5 周 | M6 | 未开始 | 分级实机、安全和阻抗验收 |
| 后续可选 | M7 | 观测/标定数据完成；夹爪相机未开始 | 夹爪、图传、数据采集与 LeRobot 转换 |

进入下一阶段必须满足上一阶段验收条件，不能以“机械臂能够运动”替代模型、watchdog、急停和故障恢复验收。

## 12. 首版完成定义

只有同时满足以下条件才视为首版完成：

- PICO 左右 Grip 可独立、无跳变地控制对应手臂；
- MuJoCo 与实机复用 XR 映射、Placo IK 和关节合同；两侧各自的安全保护均完成独立验收；
- MarvinSDK 在已验证频率下稳定发送关节位置，控制器正确处于关节阻抗模式；
- 所有关节命令满足安全限位、速度和有效期，过期命令从未下发；
- XR、反馈、求解器、网络、线程退出和物理急停测试均有记录并通过；
- 启动默认不运动，退出不回零，FAULT 只能显式恢复；
- 30 分钟仿真和 30 分钟低速实机稳定性测试无越限、无失控、无未解释错误。

## 13. 参考资料

- 本地机械臂调试说明：`../../TJArm/天机TJArm.md`
- 本地 SDK 归档：`../../TJArm/tj_fx_robot-master.zip`
- 本地工具辨识结果：`../../TJArm/tools_cfg.json`
- 本地急停说明：`../../TJArm/天机Marvin系列_急停功能启用方法说明251203.pdf`
- 本地 MarvinPlatform 说明：`../../TJArm/天机Marvin系列_Marvin PlatformEN软件使用说明260804.pptx`
- 本地控制链路分析：`../../VR遥操天机机械臂控制链路分析.md`
- XR-Robotics：<https://github.com/XR-Robotics>
- Python Teleop Sample：<https://github.com/XR-Robotics/XRoboToolkit-Teleop-Sample-Python>
- Marvin SDK 上游地址（本地文档所列）：<https://github.com/cynthia-you/TJ_FX_ROBOT_CONTRL_SDK>
