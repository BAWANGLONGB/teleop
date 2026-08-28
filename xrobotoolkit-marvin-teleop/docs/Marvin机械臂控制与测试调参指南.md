# Marvin 机械臂控制实现与测试调参指南

> 适用范围：当前仓库的 PICO → Marvin 双臂实机链路  
> 当前状态：软件实现和 mock 自动测试已完成，真实机械臂分级验收尚未完成  
> 安全边界：本文解释现有代码和低速测试方法，不代替机型、关节映射、Tool、物理急停及现场 Go/No-Go 签字

现场上机顺序仍以
[`XRoboToolkit环境部署与PICO联调流程.md`](XRoboToolkit环境部署与PICO联调流程.md)
为准。本文只回答三个问题：控制命令如何产生、每个参数实际限制什么、测试时怎样修改并确认修改已生效。

## 1. 当前控制方式

当前实机运行在 **关节阻抗模式**：主机向控制器周期下发 14 轴关节位置目标，控制器内部按每轴 K/D 完成阻抗控制。

```text
PICO 手柄位姿和 Grip
  → XR latest-value 200 Hz
  → 头部 yaw 参考系、增量 SE(3) 映射
  → TCP 工作区/速度/跳变/奇异性保护
  → Placo 双臂 IK 100 Hz
  → 关节速度/加速度/jerk/软限位/预测制动
  → SafetySupervisor
  → MarvinSDK 双臂位置目标 50 Hz
  → 控制器 state=3、impedance_type=1、每轴 K/D
```

它不是主机侧力矩闭环，也不是笛卡尔阻抗。主机不直接计算电机力矩；反馈 `q/dq/tau` 用于安全判断、限幅、记录和下一次 IK 状态更新。

## 2. 代码入口和执行流程

| 层 | 代码 | 作用 |
| --- | --- | --- |
| 实机 CLI | [`scripts/hardware/teleop_marvin_hardware.py`](../scripts/hardware/teleop_marvin_hardware.py) | 四项使能确认、CLI 参数、PICO→Marvin 坐标变换、A/B→左右臂合同、创建控制器 |
| XR→IK | [`common/base_teleop_controller.py`](../xrobotoolkit_teleop/common/base_teleop_controller.py) | Grip deadman、按下时锁存手柄/TCP 零点、增量位姿映射、Placo IK |
| 实机调度 | [`hardware/marvin_teleop_controller.py`](../xrobotoolkit_teleop/hardware/marvin_teleop_controller.py) | 200 Hz 反馈、100 Hz 控制、50 Hz 命令、启动门禁、日志与正常退出 |
| TCP 保护 | [`common/cartesian_target_guard.py`](../xrobotoolkit_teleop/common/cartesian_target_guard.py) | 启动点工作区、TCP 线/角速度、单帧跳变拒绝 |
| 关节保护 | [`common/joint_command_limiter.py`](../xrobotoolkit_teleop/common/joint_command_limiter.py) | 速度、加速度、jerk、5° 软限位和基于反馈的预测制动 |
| 安全状态机 | [`common/marvin_safety.py`](../xrobotoolkit_teleop/common/marvin_safety.py) | ARMED/TELEOP/RETURNING/HOLD/FAULT、数据龄期和跟踪误差 watchdog |
| SDK 适配 | [`hardware/interface/marvin.py`](../xrobotoolkit_teleop/hardware/interface/marvin.py) | 双臂原子事务、度/弧度转换、模式/Tool/K/D 配置及状态回读 |

### 2.1 启动阶段

实机脚本在连接前要求同时给出 `--enable-hardware`、`--confirmed-estop`、`--confirmed-joint-mapping` 和非空的 `--confirmed-robot-model`。随后按固定顺序执行：

1. 连接 SDK，检查 ABI、SDK 版本和 A/B 新鲜反馈；
2. 检查错误码、`state=100`、`low_speed_flag` 和最大反馈关节速度；
3. 用实测关节位置初始化 Placo、HOLD 目标、TCP 工作区原点和关节 limiter；
4. 默认只比对控制器 Tool 与 `tools_cfg.json`；
5. 设置控制器速度/加速度比例和左右 K/D；
6. 再次检查静止状态，并先把当前反馈位置写成目标；
7. 切换双臂 `state=3`、`impedance_type=1`，设置 PD 速度估计周期；
8. 回读模式、速度比例、加速度比例和 K/D，全部一致后才进入 ARMED。

因此启动时没有回零动作，也不会突然追赶仿真 home pose。IK 冗余项只以启动实测姿态为很弱的软参考，权重为 `1e-4`。

### 2.2 每个控制周期

100 Hz 控制线程在同一周期固定一份 XR 快照，然后执行：

1. 把最新 14 轴反馈写入 Placo；
2. Grip 按下时计算相对手柄位姿；松开时以反馈位置为起点，对应臂按余弦目标返回本次启动实测关节姿态；
3. 计算平移 Jacobian 最小奇异值，接近奇异位形时降低 TCP 速度，越过故障阈值时 FAULT；
4. 对 raw TCP 目标做跳变拒绝、启动点球形工作区和线/角速度限制；
5. 求解 Placo IK；连续 3 次 IK 失败时触发控制故障；
6. Grip 松开且不在复位的手臂精确保持锁存目标，其指令速度/加速度状态置零；其他 IK/复位目标使用临界阻尼跟踪，再做关节软限位、速度、加速度、jerk 和预测制动；
7. 发布 latest-value 关节命令，50 Hz I/O 线程通过 SafetySupervisor 后一次事务下发 A/B 两臂。

启动时从当前反馈姿态返回自然下垂姿态的默认时间限制为 10 秒。正常松手复位使用 RETURNING，不是故障 HOLD；回位目标仍经过关节速度、加速度、jerk、软限位和预测制动，因而实际时间可以长于名义 3 秒；重新按对应 Grip 会取消该臂回位并锁存新零点。HOLD 才会冻结反馈位置，恢复前必须松开全部 Grip；FAULT 会请求双臂 `state=0` 并终止进程。

### 2.3 PICO 输入与真机行为合同

| PICO 输入 | 真机遥操入口 | PICO-only scale 标定入口 |
| --- | --- | --- |
| 左 Grip | 模拟量大于 `0.9` 时使能左臂；上升沿附近锁存当前左手柄/左 TCP 相对零点；松开后左臂自动返回启动姿态 | 必须保持松开，否则拒绝采样 |
| 右 Grip | 模拟量大于 `0.9` 时使能右臂；上升沿附近锁存当前右手柄/右 TCP 相对零点；松开后右臂自动返回启动姿态 | 必须保持松开，否则拒绝采样 |
| A | 双 Grip 松开、复位完成且机械臂静止时：第一次记录自然下垂点，第二次记录水平前伸点；成功后保存并安全切换，新值在下一次按 Grip 锁存零点后生效 | 同左 |
| B | 清除本轮第一个 scale 采样点，重新开始 | 同左 |
| 左右 Trigger | 无行为；当前首版尚未接入夹爪 | 无行为 |
| X/Y、菜单键 | 无行为 | 无行为 |
| 左右摇杆及摇杆按压 | 无行为；不控制底盘、关节或急停 | 无行为 |
| 左右手柄位姿 | 仅在对应 Grip 按住时生成该臂相对 TCP 目标 | A 按下沿采集左右位置 |
| 头显位姿 | 只提供连续 `head_yaw` 参考系；不控制机械臂基座或独立头部 | 用于把两次手柄采样变换到同一 `head_yaw` 坐标定义 |

日志在带使能运行时自动开始、正常退出时自动结束，不由 PICO 按键切换。PICO 的 `Send`、Grip 或任何软件按键都不是急停；数据停更只由 watchdog 处理，碰撞风险必须使用物理急停。

## 3. 哪些配置真正生效

当前运行真值是 `teleop_marvin_hardware.py` 的 CLI 默认值和本次命令行覆盖值。

```bash
python scripts/hardware/teleop_marvin_hardware.py --help
```

必须特别注意：

- 修改 [`configs/marvin_hardware.json`](../configs/marvin_hardware.json) **目前不会改变运行参数**；它只是审查快照。
- CLI 参数和已有的 PICO scale 标定在进程启动时读取；真机进程内 A/A 标定完成后，允许在双 Grip 松开、复位完成且机械臂静止时安全更新，并强制下一次 Grip 重新锁存零点。
- `scale_factor` 的优先级为：显式 `--scale-factor` > `--scale-calibration-path` 指向的有效标定 > 代码默认 `0.5`。标定文件存在但无效时启动失败，不静默回退。
- 每次带使能运行都会把启动参数、URDF/Tool 文件路径和哈希写入 JSONL summary 与 calibration metadata；运行时 A/A 切换会单独记录事件，JSONL 控制周期和 calibration CSV 每个样本都记录当时的 scale，退出摘要记录最终 scale，可据此确认参数而不是只看终端命令。

## 4. CLI 可调参数

以下参数可以在停机后，通过下一次启动命令手动覆盖。

### 4.1 调度和映射

| CLI 参数 | 默认值 | 含义与影响 |
| --- | ---: | --- |
| `--scale-factor` | 默认未显式设置 | 手柄平移增量到 TCP 平移增量的比例；显式给出时覆盖已保存标定。无标定时实际值为 `0.5`。不缩放姿态旋转 |
| `--scale-calibration-path` | `logs/marvin_scale_calibration.json` | 启动时读取的 PICO 标定，也是进程内 A/A 成功后的保存位置；文件不存在时使用 `0.5` |
| `--enable-release-return` | 默认开启 | Grip 松开后是否让对应臂返回本次启动实测姿态；显式关闭后松手保持反馈位置 |
| `--return-duration` | `3.0` s | 松手复位的名义余弦插值时长；关节 limiter 可以把实际完成时间延长 |
| `--enable-arm-length-calibration` | 默认开启 | 在真机进程中订阅 A/B；只允许双 Grip 松开、复位完成且机械臂静止时采样 |
| `--calibration-workspace-margin` | `0.95` | 两点标定中机器人姿态行程的安全使用比例 |
| `--xr-poll-hz` | `200` Hz | 主机读取 XR latest-value 的频率；不会提高 PICO 原始发送率 |
| `--control-hz` | `100` Hz | TCP guard、IK 和关节 limiter 周期；所有软件关节限速严格使用 `dt=1/control_hz` |
| `--feedback-hz` | `200` Hz | SDK 状态订阅和标定 CSV 采样频率 |
| `--command-hz` | `50` Hz | 向 Marvin 下发双臂关节目标的频率；同时决定 PD 速度估计周期，默认 `20 ms` |

频率不是任意组合：`feedback_hz` 和 `control_hz` 都必须不低于 `command_hz`，且必须是其整数倍；`command_hz≤200`；`1000/command_hz` 必须是整数毫秒且不超过供应商允许的 20 ms。首次现场验收固定使用 `200/100/50 Hz`，没有实测证据前不要把 50 Hz 提高到 100/200 Hz。

### 4.2 TCP 目标保护

| CLI 参数 | 默认值 | 单位 | 实际行为 |
| --- | ---: | --- | --- |
| `--max-tcp-displacement-m` | `0.25` | m | TCP 相对本次启动实测位置的最大球形半径；超出部分被截在球面，不是单周期位移 |
| `--max-tcp-linear-speed-m-s` | `0.1` | m/s | guard 后 TCP 平移目标的每周期变化上限 |
| `--max-tcp-angular-speed-rad-s` | `0.5` | rad/s | guard 后 TCP 姿态目标的每周期旋转上限 |
| `--max-tcp-frame-jump-m` | `0.15` | m/帧 | 相邻两份 raw XR 目标超过该值立即故障，不做静默截断 |
| `--max-tcp-frame-jump-deg` | `45` | °/帧 | 相邻两份 raw XR 姿态超过该值立即故障 |
| `--singularity-fault-sigma` | `0.003` | Jacobian 数值 | 平移 Jacobian `sigma_min` 小于等于该值时故障 |
| `--singularity-full-speed-sigma` | `0.015` | Jacobian 数值 | 高于该值允许全 TCP 速度；两阈值之间线性降速，最低为 10% |

减小 TCP 线/角速度会直接增加“目标在前、机械臂追赶”的视觉延迟。若日志中 raw→limited TCP 误差持续增大，应先判断是不是这些保护正在正常限速，不要直接归因于 IK。

### 4.3 关节目标保护

| CLI 参数 | 默认值 | 单位 | 实际行为 |
| --- | ---: | --- | --- |
| `--max-joint-velocity` | `0.1` | rad/s | 软件下发关节目标速度的逐轴上限 |
| `--max-joint-acceleration` | `0.3` | rad/s² | 软件命令速度每秒变化的逐轴上限 |
| `--max-joint-jerk` | `2.0` | rad/s³ | 软件命令加速度每秒变化的逐轴上限 |
| `--joint-target-natural-frequency` | `8.0` | rad/s | 关节目标临界阻尼跟踪快慢；不改变下方硬限制 |
| `--joint-limit-margin-deg` | `5` | ° | 从 URDF 上下限两侧各内缩的安全余量 |

limiter 还使用反馈 `q/dq` 计算到软限位的保守制动距离，并在接近速度上限前按离散控制周期精确预留把加速度以最大 jerk 降到零所需的速度余量。反馈已经越过软限位，或当前速度/加速度/jerk 无法形成可行制动命令时，程序进入 FAULT，而不是继续向限位外发命令。

指令历史表示软件生成的轨迹，不等于带噪声的实测速度。因此启动时以当前 `q` 初始化静止保持，指令速度为零；实测 `dq` 仍仅用于限位预测制动。这避免反馈速度噪声被误当成指令轨迹并在待机保持中激起往返。

速度、加速度或 jerk 越小通常越平滑、越保守，但追赶延迟越大。目标自然频率越高跟踪越紧，但不应用它绕过速度/加速度/jerk 硬限制；首次真机保持 `8.0`。软限位余量越大，可用行程越小且更早制动。五者彼此耦合，测试中一次只改一项。

### 4.4 控制器侧参数

| CLI 参数 | 默认值 | 含义 |
| --- | ---: | --- |
| `--velocity-ratio` | `10` | 供应商控制器速度百分比，允许范围 1–100 |
| `--acceleration-ratio` | `10` | 供应商控制器加速度百分比，允许范围 1–100 |
| `--startup-max-joint-speed-rad-s` | `0.02` | 进入模式前允许的最大实测关节速度；A/B 还必须都报告 `low_speed_flag` |

控制器百分比是硬件侧的附加上限，不能替代 SI 单位的软件关节限制。任一层先达到限制都会产生追赶误差。首次测试不要同时提高控制器百分比和软件限速，否则无法定位改善来自哪一层。

### 4.5 连接、模型、Tool、日志和观测

| CLI 参数 | 默认值/行为 | 修改条件 |
| --- | --- | --- |
| `--robot-ip` | `192.168.1.190` | 只按现场控制器固定地址修改 |
| `--expected-sdk-version` | `100343014` | 只有供应商确认 SDK/控制器配套版本变化后修改，不能用来绕过未知版本 |
| `--sdk-root` | `../TJArm/tj_fx_robot-master` | 指向经验证的供应商 SDK 根目录 |
| `--robot-urdf-path` | `assets/marvin/marvin_dual.urdf` | 机型、关节轴、限位和 TCP_Link 均已复核后才替换 |
| `--tools-config` | `../TJArm/tools_cfg.json` | 指向经 MarvinPlatform/供应商确认的 Tool 导出配置 |
| `--configure-tools` | 默认关闭 | 关闭时只比较并拒绝不匹配；打开会把所选左右 Tool 写入控制器，属于专项维护操作 |
| `--log-dir` | 仓库 `logs/` | 改到本地 SSD 目录前确认容量和写权限 |
| `--enable-ros2-observation` | 默认关闭 | 仅开启 ROS2 观测旁路，没有控制权 |
| `--ros2-namespace` | `/marvin_teleop` | 多实例观测时避免重名 |
| `--ros2-publish-hz` | `100` Hz | 只改变 ROS2 发布频率，不改变控制频率 |
| `--visualize-placo` | 默认关闭 | 打开 MeshCat/Placo 开发可视化；首次实机验收保持关闭，避免额外调度负载 |

Tool `kine` 是法兰到工具的 `[X,Y,Z,A,B,C]` 六项供应商参数；`dyn` 是质量、质心和六项惯量组成的十项供应商参数。代码保留供应商原始单位，不在主机端猜测或换算。不要在带 Grip 的运行中手填这些值。

## 5. 当前不能通过 CLI 修改的参数

| 参数 | 当前值 | 代码位置 | 修改要求 |
| --- | --- | --- | --- |
| 左右 K | `[2,2,2,1.5,0.8,0.8,0.8]` | `MarvinHardwareTeleopController.__init__` | 停机后评审代码变更、补测试、再启动；供应商定义单位为 N·m/deg |
| 左右 D | `[0.3,0.3,0.3,0.2,0.2,0.2,0.2]` | 同上 | 停机后评审；供应商定义为 0–1 阻尼比例 |
| XR HOLD/FAULT | `100/500 ms` | `MarvinSafetyConfig` | watchdog 改动必须做停更/重定位故障注入 |
| 反馈 HOLD/FAULT | `30/100 ms` | `MarvinSafetyConfig` | 必须结合 30 分钟反馈龄期和 SDK 耗时分布修改 |
| 命令有效期 | `40 ms` | `MarvinSafetyConfig` | 必须大于正常命令龄期并覆盖调度抖动，不能用于掩盖卡顿 |
| 跟踪误差 HOLD/FAULT | `3°/8°` | `MarvinSafetyConfig` | 必须由低速实测跟踪误差和风险分析支持 |
| 连续 IK 失败 | `3 次` | `_control_loop()` | 只允许经回放与故障注入修改 |
| A/B 关节映射 | identity | 实机入口常量和 metadata | 当前不支持 `source_index/sign/offset` 配置；现场不一致时必须停止开发，不能只改确认文本 |
| PICO→Marvin 坐标旋转 | 固定 3×3 矩阵 | 实机入口 `R_PICO_TO_MARVIN_WORLD` | 只在 MuJoCo 和逐轴/逐方向复核后改，禁止运动中改 |

K 越大，关节对位置误差的恢复力越强、感觉越“硬”；过高会增加冲击和振动风险。D 越大，振动衰减更快，但响应可能更粘滞。当前 K/D 是首次低速测试的低刚度基线，不应为了消除追赶误差而直接提高 K；应先用日志分解 XR、guard、IK、limiter、下发和机械跟踪各段延迟。

## 6. 测试时怎样手动修改

### 6.1 修改原则

当前不支持运动中的热修改。每组测试都按以下流程：

1. 松开双 Grip，等待自动复位完成并确认状态进入 ARMED；
2. `Ctrl+C` 正常退出，确认程序已请求 `state=0` 并输出日志路径；
3. 记录上一组日志文件、完整启动命令和观察结果；
4. 只改变一个参数，在下一次命令行中显式覆盖；
5. 重新执行启动门禁，先单臂、TCP 小于 5 cm；
6. 出现振动、异响、跟踪误差快速增大、HOLD/FAULT 或失控趋势时立即松 Grip；有碰撞风险或进程无响应时按物理急停；
7. 不满意时删除该命令行覆盖即可回到代码默认值，不要为回退参数执行 `git reset`。

### 6.2 保守小范围样例

以下样例是在现有默认值上进一步减小动作范围和速度，适合完成全部现场门禁后的单臂初次小范围验证；它不是已经通过真机验收的生产配置：

```bash
python scripts/hardware/teleop_marvin_hardware.py \
  --enable-hardware \
  --confirmed-estop \
  --confirmed-joint-mapping \
  --confirmed-robot-model "现场读取的准确型号" \
  --scale-factor 0.3 \
  --max-tcp-displacement-m 0.05 \
  --max-tcp-linear-speed-m-s 0.03 \
  --max-tcp-angular-speed-rad-s 0.2 \
  --max-joint-velocity 0.05 \
  --max-joint-acceleration 0.15 \
  --max-joint-jerk 1.0 \
  --velocity-ratio 5 \
  --acceleration-ratio 5
```

### 6.3 PICO 比例标定与安全应用

可以在实机启动前使用独立标定进程。它只连接 XRoboToolkit PC Service，不导入 MarvinSDK、不连接控制器、不切换模式，也不发送关节命令：

```bash
python scripts/hardware/calibrate_marvin_scale_from_pico.py
```

操作顺序：

1. 保持 Marvin 下伺服或按现场规定处于安全状态；双 Grip 全部松开；
2. 操作者自然站立、双臂下垂，按右手柄 `A` 记录起点；
3. 躯干和头部朝向保持稳定，双臂对称水平向前伸直，再按 `A`；
4. 若姿态或采样有误，按 `B` 清除起点后重来；
5. 成功后程序立即退出，原子更新 `logs/marvin_scale_calibration.json`，并保留 `marvin_scale_calibration_<时间戳>.json` 历史；
6. 返回现场 SOP，从实机启动门禁重新开始。新进程会在连接 Marvin 前读取该值。

带使能的真机进程中也使用同样的 A/A、B 操作。额外门禁是：双 Grip 松开、两臂自动复位完成、反馈 `low_speed_flag` 有效且最大关节速度不超过 `0.02 rad/s`。成功后原子保存并在这个静止窗口更新当前进程的 scale，同时清除旧的手柄/TCP 参考点；程序保持 ARMED，下一次按对应 Grip 会先锁存新零点，再使用新比例，不需要重启且不会产生 TCP 目标跳变。独立 PICO-only 标定没有真机进程可更新，仍需随后启动真机入口加载新值。

实机启动时终端会打印实际值和来源：`pico_calibration`、`cli` 或 `code_default`。临时覆盖保存值可显式使用：

```bash
python scripts/hardware/teleop_marvin_hardware.py \
  --scale-factor 0.4 \
  --enable-hardware \
  --confirmed-estop \
  --confirmed-joint-mapping \
  --confirmed-robot-model "现场读取的准确型号"
```

使用其他标定文件时，校正命令的 `--output-path` 与实机命令的 `--scale-calibration-path` 必须指向同一文件。标定值只改变平移映射比例，仍受 TCP 工作区、TCP 速度和关节 limiter 限制。

若目的是定位追赶延迟，建议保持其余参数不动，按下列顺序分别采集：

1. 默认基线；
2. 只改 `--scale-factor`，确认是动作幅度问题还是速度饱和；
3. 恢复 scale，只改 TCP 线/角速度之一；
4. 恢复 TCP 参数，只改关节速度；
5. 再分别评估关节加速度、jerk；
6. 只有软件目标与实际关节之间仍有明显误差时，才在供应商参与下评估控制器比例或 K/D。

### 6.4 Tool 专项修改

Tool 变化不属于普通遥操调参。正确流程是：

1. 下伺服并确认左右实际安装工具；
2. 在 MarvinPlatform 或供应商工具中得到对应 `kine/dyn`，更新外部 `tools_cfg.json` 的记录和 `current_tool.arm0/arm1`；
3. 先用默认模式启动，让程序只比较控制器现值；不一致时程序应拒绝进入 TELEOP；
4. 只有专项 Tool 写入任务获批时才增加 `--configure-tools`；
5. 写入后重新启动且不带该开关，验证只读比对和 metadata 哈希均一致。

不得用 `--configure-tools` 作为“忽略 Tool 不匹配”的开关。

### 6.5 K/D 和 watchdog 的代码级修改

K/D 当前是 `MarvinHardwareTeleopController.__init__` 中的四个七元素默认元组：`left_k`、`left_d`、`right_k`、`right_d`；watchdog 默认值在 `MarvinSafetyConfig`。如果专项测试确实需要修改，必须停机后完成代码变更，并至少执行：

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  python -m pytest -q tests/test_marvin_hardware.py
```

随后重新启动，程序会写入 K/D、切换关节阻抗并回读；回读不一致时不进入 TELEOP。更稳妥的后续工程方式是把这几项接入经过范围校验、自动记录和漂移测试的单一运行配置源，而不是现场反复改库文件。当前尚未实现该配置源，所以普通测试人员不要修改 K/D、watchdog、映射矩阵或关节顺序。

## 7. 如何确认参数是否生效

正常退出后检查：

```text
logs/marvin_hardware_<timestamp>.summary.json
logs/marvin_calibration_<timestamp>.metadata.json
```

重点字段包括：

- `control_hz/feedback_hz/command_hz/xr_poll_hz/pd_period_ms`；
- `scale_factor`、`scale_factor_source` 及标定文件 SHA-256；
- `max_joint_velocity_rad_s/max_joint_acceleration_rad_s2/max_joint_jerk_rad_s3`；
- `joint_limit_margin_rad`、`tcp_guard`、`safety`；
- `velocity_ratio/acceleration_ratio/left_k/left_d/right_k/right_d`；
- `robot_urdf_sha256/tools_config_sha256` 和左右 Tool 原始参数；
- 最终安全状态、样本范围和写入错误。

诊断追赶延迟时同时比较：

| 比较量 | 主要说明 |
| --- | --- |
| raw TCP → limited TCP | TCP 工作区、速度或奇异性降速造成的延迟 |
| limited TCP → `FK(q_ik)` | IK 求解后的 TCP 是否达到受限目标；需离线用同一 URDF 对 `q_ik` 做 FK |
| `q_ik` → `q_command` | 关节速度/加速度/jerk/预测制动造成的延迟 |
| `q_command` → `q` | 50 Hz 下发、控制器比例、K/D、负载和机械本体跟踪 |
| `xr_source_age_ms` | PICO→PC Service→主机输入延迟或停更 |
| `control_duration_ms/deadline_lateness_ms` | IK/控制计算是否超过 10 ms 周期 |
| `sdk_read_duration_ms`、命令发送耗时、帧 miss | SDK/控制器通信和调度质量 |

只有 `control_duration_ms` 或 deadline miss 明显异常，才有证据把主要延迟归到 IK/主机计算；若 `FK(q_ik)` 已跟上 limited TCP、但 `q_command` 缓慢变化，主要限制在关节 limiter；若 `q_command` 已变化而反馈 `q` 落后，才继续检查命令频率、控制器比例、K/D、Tool 和机械负载。

## 8. 测试记录最小模板

每次只改一项，并把以下内容与对应日志放在同一试验记录中：

```text
测试编号：
日期/操作者/急停观察员：
机型/控制器/SDK 版本：
左右 Tool 与负载：
完整启动命令：
唯一改动参数：旧值 → 新值：
动作：左/右臂、方向、幅度、往返次数：
结果：最大跟踪误差、HOLD/FAULT、deadline miss、异响/振动：
JSONL/summary/CSV/metadata 路径：
结论：保留 / 回退 / 需供应商确认：
```

在 M6 分级验收完成前，任何“更快、更硬”的参数都只是候选测试值，不得写成生产默认值。
