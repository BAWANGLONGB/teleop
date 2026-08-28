# PICO → Marvin 环境部署与操作流程

> 更新日期：2026-08-28
>
> 适用主机：Ubuntu 22.04 x86_64
>
> 当前状态：PICO→PC Service→Python 位姿链路已联调通过；PICO→MuJoCo 与 Marvin 实机软件链路已实现，真实机械臂分级验收尚未执行

本文是现场操作的唯一 SOP。设计依据、参数推导和后续开发项放在开发计划中，不在现场操作时交叉执行。

## 1. 数据链路

```text
PICO 4 Ultra / XRoboToolkit App
        │ Wi-Fi，TCP 63901
        ▼
XRoboToolkit PC Service
        │ 本机回环，gRPC 60061
        ▼
xrobotoolkit_sdk
        │ 最新值快照：XR 轮询 200 Hz，设备源帧约 30–60 Hz
        ▼
xrobotoolkit_teleop
  坐标映射 → TCP guard → Placo IK 100 Hz
  → 关节速度/加速度/jerk/限位保护 → Safety Supervisor
        │
        ├── MuJoCo：控制 100 Hz，物理 500 Hz
        │
        └── MarvinSDK：反馈 200 Hz，双臂命令 50 Hz，UDP
                         │ 独立有线网段
                         ▼
                  Marvin 控制器 192.168.1.190

旁路观测（无控制权）：
  ROS2 标准话题 100 Hz + 实机标定 CSV 200 Hz + JSONL 事件日志
```

- PICO 只填写主机的局域网 IPv4 地址，不填写 `127.0.0.1`。
- Python SDK 默认连接本机 `127.0.0.1:60061`，不直接连接 PICO。
- 对外只需允许 PICO 访问 `63901/tcp`；不要把 `60061` 暴露到局域网。
- PICO 不直接连接 Marvin。主机应使用 Wi-Fi 接 PICO 网络、独立有线网卡接 Marvin 网络，并保证两个接口不在同一子网。
- ROS2 只发布观测，不订阅运动命令；MarvinSDK、限幅、watchdog、HOLD/FAULT 和退出下伺服始终留在同一硬件进程内。

## 2. 已验证基线

| 项目 | 已验证值 |
| --- | --- |
| 操作系统 | Ubuntu 22.04.5 LTS，x86_64 |
| 主工程 | commit `79e5cb8a56e3455515ce1b476e993c764ec58739` |
| Conda | 25.7.0，安装于 `/home/zxcx/TeleOp/.miniconda-xr` |
| Conda 环境 | `Teleop`，Python 3.10.21 |
| PC Service | Debian 包 `roboticsservice 1.0.0.0`，安装于 `/opt/apps/roboticsservice` |
| Python Pybind SDK | `xrobotoolkit_sdk 1.0.2`，commit `c64ccf6acd577a333e03b66fafe8efeeceb511b1` |
| Pybind 所用 PC Service 源码 | commit `85bac4dbc1fd5cef42c74a160d9c30aa3491f122` |
| Unity Client | commit `cdc53166b0bf412efae71046c6a225eb5091605f` |
| PICO App | `1.1.1`，包名 `com.xrobotoolkit.client` |
| 关键 Python 依赖 | NumPy 1.26.4、MuJoCo 3.12.0、Torch 2.3.0+cpu、Placo 0.5.9、Pinocchio 2.7.0、ProxSuite 0.7.3 |
| Marvin Linux SDK | 本机加载版本 `SDK_version()=100343014`；ABI 探测 `(1,0)`，表示检查通过且为小端 |
| Marvin 开发模型 | M6S-Lite CCS-680 候选 URDF/MJCF；必须由现场铭牌和控制器配置确认，不能据此认定真机型号 |
| Marvin 默认地址 | `192.168.1.190`；以现场控制器配置为准 |
| 自动回归 | `40 passed`；包含 MuJoCo 合同、SDK mock、安全保护、Grip 松手复位、PICO scale 持久化、只读 SDK 版本门禁、启动事务忙重试、关节保持/速度前瞻、标定 CSV/NPZ 和默认只读行为 |

安装包校验值：

```text
61961067eb4b41f81ed7cae35f4690dbb0ddfefb329a12b24e0b90ebc46ada91  XRoboToolkit_PC_Service_1.0.0_ubuntu_22.04_amd64.deb
6b2bb282405673d24abcb1980e3478b8f1052e90f7207b1f24cc56a59f8d8261  XRoboToolkit-PICO-1.1.1.apk
```

## 3. 首次部署

### 3.1 Conda 环境

当前主机已经完成安装。每次打开新终端先执行：

```bash
source /home/zxcx/TeleOp/.miniconda-xr/etc/profile.d/conda.sh
conda activate Teleop
```

验证环境：

```bash
which python
python --version
python -m pip check
python -c "import numpy, mujoco, torch, placo, pinocchio, proxsuite, xrobotoolkit_sdk; print('environment OK')"
```

预期 Python 路径：

```text
/home/zxcx/TeleOp/.miniconda-xr/envs/Teleop/bin/python
```

如果终端提示 `conda：未找到命令`，不要重新安装，重新执行上面的 `source` 即可。

注意：上游 `setup_conda.sh --install` 会删除并重新克隆整个 `dependencies/`，当前工程已经锁定并构建完成，不应在现有工作树中重复运行该参数。

### 3.2 安装 PC Service

安装包位置：

```text
/home/zxcx/下载/XRoboToolkit_PC_Service_1.0.0_ubuntu_22.04_amd64.deb
```

首次安装命令：

```bash
sha256sum /home/zxcx/下载/XRoboToolkit_PC_Service_1.0.0_ubuntu_22.04_amd64.deb
sudo dpkg -i /home/zxcx/下载/XRoboToolkit_PC_Service_1.0.0_ubuntu_22.04_amd64.deb
dpkg-query -W roboticsservice
```

预期包版本为 `1.0.0.0`。

### 3.3 安装 PICO App

APK 位置：

```text
/home/zxcx/TeleOp/XRoboToolkit-PICO-1.1.1.apk
```

在 PICO 中开启开发者模式和 USB 调试，连接 USB 后执行：

```bash
adb devices -l
sha256sum /home/zxcx/TeleOp/XRoboToolkit-PICO-1.1.1.apk
adb install -r /home/zxcx/TeleOp/XRoboToolkit-PICO-1.1.1.apk
```

`adb devices` 没有设备时，检查 USB 数据线、头显中的调试授权弹窗和开发者模式。USB 只用于安装和调试；正式位姿链路仍通过局域网传输。

### 3.4 网络与防火墙

查询主机地址：

```bash
ip -brief -4 addr show scope global
```

本次联调时主机地址为：

```text
有线 enp4s0：192.168.120.172/23
无线 wlo1：  192.168.120.115/23
```

这些地址由局域网分配，重启或更换网络后可能变化。PICO 与主机使用同一 Wi-Fi 时，优先填写无线网卡地址。本次成功连接使用 `192.168.120.115`。

如果 UFW 已启用，只开放 PICO 入口：

```bash
sudo ufw allow 63901/tcp
sudo ufw status
```

### 3.5 Marvin 独立有线网络

推荐拓扑：

```text
PICO ──Wi-Fi── 主机 wlo1（PICO/办公网，例如 192.168.120.0/23）
Marvin ──网线── 主机专用以太网口（机械臂网，例如 192.168.1.0/24）
```

主机机械臂网口可使用类似 `192.168.1.100/24` 的静态地址，但不得使用控制器地址 `192.168.1.190`。网卡名称和地址必须由现场网络管理员确认，本文不提供可直接复制的改网命令，以免覆盖主机现有连接。配置后检查：

```bash
ip -brief -4 addr show scope global
ip route get 192.168.1.190
ping -c 4 192.168.1.190
```

预期 `ip route get` 指向机械臂专用有线接口，而不是 Wi-Fi。不要桥接 PICO 网和机械臂控制网，不要为排查方便关闭全部防火墙；Marvin 的 UDP 端口和握手由供应商 SDK 管理。

## 4. 日常启动顺序

### 4.1 启动 PC Service

使用独立服务脚本：

```bash
bash /opt/apps/roboticsservice/runService.sh
```

检查进程和端口：

```bash
pgrep -af RoboticsServiceProcess
ss -lnt | grep -E ':(63901|60061)\b'
```

预期结果：

- `*:63901` 监听 PICO；
- `127.0.0.1:60061` 或其 IPv4 映射地址监听 Python SDK。

不要把桌面菜单中的快捷方式作为日常启动入口。该快捷方式执行 `run3D.sh`，会同时启动 Unity 3D Demo，关闭 Demo 时容易误以为 PC Service 仍在运行。

### 4.2 在 PICO 中连接

1. 戴上头显并保持屏幕唤醒。
2. 打开 `XRoboToolkit`。
3. 在 `Data & Control → PC Service → Enter` 中输入主机局域网地址。
4. 确认 Network 状态显示 `WORKING`。
5. 只打开当前流程需要的 `Head` 和 `Controller`。
6. 打开 `Data & Control → Send`。
7. 移动头显和手柄，确认 FPS 和状态持续刷新。

保持 `Switch w/ A Button` 关闭。当前 MuJoCo 臂长标定使用 A/B 键，若同时用 A 键切换 `Send`，会在标定时意外中断 XR 数据。

### 4.3 激活 Python 环境

打开另一个终端：

```bash
source /home/zxcx/TeleOp/.miniconda-xr/etc/profile.d/conda.sh
conda activate Teleop
cd /home/zxcx/TeleOp/xrobotoolkit-marvin-teleop
```

### 4.4 连续读取位姿

SDK 数据流异步建立，不应在 `init()` 后立即只读取一次。使用下面的连续测试：

```bash
python -c "import time; from xrobotoolkit_teleop.common.xr_client import XrClient; c=XrClient(); time.sleep(1); [(print(c.get_pose_by_name('headset')), time.sleep(0.5)) for _ in range(10)]; c.close()"
```

成功时应持续输出非零且随运动变化的：

```text
[x, y, z, qx, qy, qz, qw]
```

也可读取手柄：

```bash
python -c "import time; from xrobotoolkit_teleop.common.xr_client import XrClient; c=XrClient(); time.sleep(1); print('left=', c.get_pose_by_name('left_controller')); print('right=', c.get_pose_by_name('right_controller')); c.close()"
```

### 4.5 启动仿真示例

天机 Marvin M6S-Lite CCS-680 双臂仿真入口：

```bash
python scripts/simulation/teleop_marvin_mujoco.py --scale-factor 0.5
```

左右 Grip 分别控制左右臂。按下时锁存手柄/TCP 相对零点；松开后仿真臂用 3 秒轨迹回到放松姿态。首次测试保持 `0.5` 比例。当前模型只是 CCS-680 开发基线，不能代替现场机型和安装变换确认。

## 5. 从 PICO 接入 Marvin

现场强制顺序：`PC Service/PICO位姿 → PICO→MuJoCo → Marvin网络 → 10秒只读 → 30分钟只读 → 供应商低速逐轴映射确认 → Go/No-Go签字 → 单臂TCP小范围 → 双臂空载`。任一步失败都退回仿真或只读，不跨级继续。

### 5.1 实机 Go/No-Go 前提

以下条件必须全部满足，否则只允许运行 MuJoCo 或 Marvin 只读检查：

- 现场铭牌、控制器配置和供应商资料已确认准确机型及匹配的运动学配置；
- 已使用 MarvinPlatform 或供应商批准的低速点动流程逐轴确认：SDK `A` 对应模型 `Joint1_L…Joint7_L`、SDK `B` 对应 `Joint1_R…Joint7_R`，顺序、正方向、零偏和单位均已签字；当前 PICO 入口不是单关节点动工具，只接受该 identity mapping 结论；
- 控制器当前 Tool 与 `TJArm/tools_cfg.json` 中左右选中工具完全一致，夹具、线缆和负载没有变化；默认只比较 Tool，不写入控制器；
- 物理急停已启用并完成实际触发/复位测试，急停操作者和观察员在场，工作区清空；
- MarvinPlatform、供应商 Demo 和其他 SDK 进程已完全退出，不是只点界面中的 Disconnect；控制器只允许一个进程占用；
- PICO 重新定位、安全区重置、手柄失跟踪和 Wi-Fi 断开测试已先在 MuJoCo 完成；
- 上使能前机械臂静止：A/B `low_speed_flag=1`，最大反馈关节速度不超过 `0.02 rad/s`，14 轴均位于内缩 5°后的软限位内。

软件急停、ROS2 服务或关闭 PICO `Send` 都不能替代物理急停。程序不会自动清错、自动回零或自动恢复 FAULT。

### 5.2 先完成 PICO→MuJoCo 门槛

保持真实机械臂断开或下伺服，启动：

```bash
python scripts/simulation/teleop_marvin_mujoco.py --scale-factor 1.2
```

至少验证：

1. 左右 Grip 只控制对应手臂，按下无跳变，松开互不影响；
2. PICO 右/上/前运动与仿真 TCP 同向；
3. 静止、慢速连续跟随、快速往返、双臂同时运动四组数据均自动落盘；
4. 关闭 PICO `Send` 后，XR 数据龄期按预期增长；
5. 重定位或重新戴上头显前已松开 Grip；
6. 仿真无越限、无数值发散，延迟 CSV/summary 中没有无法解释的 deadline miss。

仿真数据默认保存到：

```text
logs/marvin_latency_<timestamp>.csv
logs/marvin_latency_<timestamp>.summary.json
```

### 5.3 Marvin 短时与 30 分钟只读检查

首先确认机械臂网络可达、其他 SDK 进程已退出，然后执行 10 秒检查：

```bash
python scripts/hardware/inspect_marvin_state.py --duration-s 10
```

该工具只调用连接和订阅，不切模式、不清错、不修改 Tool/K/D/速度比例、不发送关节目标。默认要求 SDK 版本 `100343014`；只有供应商确认版本变更后才可通过 `--expected-sdk-version` 修改预期值。判定通过必须同时满足：

- SDK 版本符合预期；
- A/B `frame_serial` 均非零并分别持续递增，不能由一侧更新掩盖另一侧停更；
- 14 轴 `q/dq/tau` 均为有限值；
- 观察期间错误码始终为 0，未出现 `state=100`。

短时通过后执行 30 分钟只读基线：

```bash
df -h .
python scripts/hardware/inspect_marvin_state.py --duration-s 1800
```

默认按 200 Hz 自动保存：

```text
logs/marvin_calibration_<timestamp>.csv
logs/marvin_calibration_<timestamp>.metadata.json
```

这组数据用于检查反馈噪声、静态偏置、帧连续性、SDK 读取耗时和力矩零点。CSV 保存全部样本，30 分钟会占用数百 MB 到数 GB，实测前应在本地 SSD 预留数 GB 空间；只有磁盘空间不足且不做验收时才使用 `--no-save-calibration-data`。

### 5.4 低速实机启动

启动前：PICO 已显示 `WORKING` 且打开 `Head/Controller/Send`；左右 Grip 均松开；机械臂处于已验证的静止安全姿态；观察员手持物理急停。

若要在上使能前按当前操作者臂长更新真机平移比例，可先运行不连接 Marvin 的 PICO-only 标定：

```bash
python scripts/hardware/calibrate_marvin_scale_from_pico.py
```

 左臂：[ 90, -90, 90, 20, -90, 0, 0]°
 右臂：[-90, -90, -90, 20,  90, 0, 0]°


双 Grip 松开，自然下垂按 `A`，水平前伸再按 `A`，`B` 可重置。独立 PICO-only 标定成功后程序退出并保存当前值及时间戳历史；随后执行下面的实机启动加载。带使能的真机进程也支持相同 A/A、B 操作，但只在复位完成且机械臂静止时采样；成功后保存并安全切换比例，进程保持 ARMED，下一次按 Grip 锁存新零点后立即使用新值。无保存值时为 `0.5`；启动时显式 `--scale-factor` 优先于保存值。详细合同见调参指南第 6.3 节。

真机按键合同：左右 Grip 只使能对应手臂，松开后对应臂受限返回本次启动实测姿态，重新按下会取消回位并锁存新零点。A/A 在复位完成且静止时执行两点 scale 标定，B 重置；成功后保存并切换，下一次按 Grip 使用新值，无需重启。Trigger、X/Y、菜单键、摇杆和摇杆按压均无控制行为。日志自动保存，不由 PICO 按键启停；任何 PICO 按键都不能代替物理急停。

不需要 ROS2 时：

```bash
python scripts/hardware/teleop_marvin_hardware.py \
    --enable-hardware \
    --confirmed-estop \
    --confirmed-joint-mapping \
    --confirmed-startup-path-clear \
    --confirmed-robot-model M6S-Lite-CCS-680-B \
    --configure-tools

```

需要 RViz、PlotJuggler、rosbag2 或外部监视时，先加载 ROS2，再增加旁路观测：

```bash
source /opt/ros/humble/setup.bash

python scripts/hardware/teleop_marvin_hardware.py \
    --enable-hardware \
    --confirmed-estop \
    --confirmed-joint-mapping \
    --confirmed-startup-path-clear \
    --confirmed-robot-model M6S-Lite-CCS-680-B \
    --configure-tools
    --enable-ros2-observation
```

四项确认缺少任何一项，程序会在创建 Marvin 连接前拒绝运行。成功启动仍会依次执行：SDK/ABI 和版本检查、A/B 新鲜帧检查、Tool 比对、两次静止检查、当前反馈目标同步、低 K/D 与 10% 速度/加速度比例设置、`state=3`/`impedance_type=1` 回读；任何一步不匹配都不会进入 TELEOP。启动保持目标使用等待应答的 SDK 事务，`clear_set()` 对前一事务的短暂 busy 会在 20 ms 内按 1 ms 重试，超时仍 fail-closed。

当前首次现场参数：

参数的代码位置、精确定义、命令行覆盖方法和日志核验字段见
[`Marvin机械臂控制与测试调参指南.md`](Marvin机械臂控制与测试调参指南.md)。除上述带专用静止门禁的 A/A scale 标定外，现场测试不支持运行时热修改；其他参数必须先松开双 Grip、正常退出，再用下一次启动命令覆盖。

| 参数 | 默认值 |
| --- | --- |
| XR 轮询 / IK / Marvin 反馈 / Marvin 命令 | 200 / 100 / 200 / 50 Hz |
| PICO→TCP 平移比例 | 保存的 PICO 标定；无标定时 0.5；显式 CLI 可覆盖 |
| Grip 松手复位 | 返回本次启动实测姿态；3 s 名义余弦轨迹；受全部关节保护约束 |
| 关节速度 / 加速度 / jerk | 0.1 rad/s / 0.3 rad/s² / 2 rad/s³ |
| 关节目标临界阻尼自然频率 | 8 rad/s；不改变关节硬限制 |
| 关节软限位余量 | 5°，带反馈式预测制动 |
| TCP 最大启动位移 / 线速度 / 角速度 | 0.25 m / 0.1 m/s / 0.5 rad/s |
| 单帧 TCP 跳变拒绝 | 0.15 m / 45° |
| XR HOLD / FAULT | 100 / 500 ms |
| 反馈 HOLD / FAULT | 30 / 100 ms |
| 命令有效期 | 40 ms；重复序号 HOLD、回退序号 FAULT |

`--confirmed-joint-mapping` 只能在供应商低速逐轴点动（单轴不超过 2°、速度不超过 0.1 rad/s）完成并签字后使用。该点动不由本项目的 PICO 入口执行。进入遥操后，先做单臂 TCP 不超过 5 cm，再允许双臂空载；不得直接做大幅 PICO 运动，也不得使用供应商 `real` 大轨迹 Demo 代替验收。

### 5.5 ROS2 旁路观测

默认命名空间为 `/marvin_teleop`：

```bash
ros2 topic list | grep marvin_teleop
ros2 topic hz /marvin_teleop/joint_states
ros2 topic echo --once /marvin_teleop/safety_state
ros2 topic echo --once /marvin_teleop/diagnostics
```

主要话题：

| 话题 | 标准消息 | 说明 |
| --- | --- | --- |
| `/marvin_teleop/joint_states` | `sensor_msgs/JointState` | 14 轴反馈位置、速度、力矩 |
| `/marvin_teleop/joint_command` | `sensor_msgs/JointState` | 软件限幅后的关节目标，仅供观测 |
| `/marvin_teleop/{left,right}/tcp_actual` | `geometry_msgs/PoseStamped` | 左右反馈关节经当前 URDF FK 得到的 TCP |
| `/marvin_teleop/{left,right}/tcp_target_raw` | `geometry_msgs/PoseStamped` | 左右 PICO 原始映射目标 |
| `/marvin_teleop/{left,right}/tcp_target_limited` | `geometry_msgs/PoseStamped` | 左右 TCP guard 后目标 |
| `/marvin_teleop/safety_state` | `std_msgs/String` | JSON 状态和原因 |
| `/marvin_teleop/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 帧号、龄期、SDK 耗时和错误码 |

这些话题没有控制权。ROS2/DDS 异常只会记录 `ros2_observer_exception`，不会把陈旧 ROS 消息发送到机械臂；硬件进程仍是唯一 SDK 和 Safety 所有者。

### 5.6 实机日志与 MuJoCo 校正数据

每次带使能运行都会自动保存两套互补数据：

```text
logs/marvin_hardware_<timestamp>.jsonl
logs/marvin_hardware_<timestamp>.summary.json
logs/marvin_calibration_<timestamp>.csv
logs/marvin_calibration_<timestamp>.metadata.json
```

- JSONL：安全状态转换、异常、每次 SDK 决策和发送耗时；
- CSV：每个 200 Hz 反馈样本及最近的 100 Hz 控制快照，包括 `q/dq/tau`、控制器/软件/IK 目标、raw/limited/actual TCP、XR 龄期、帧号、错误码和安全状态；
- metadata：URDF 与工具配置路径/哈希、Tool 原始参数、K/D、限幅、频率、样本范围和写入错误。

转为不含 pickle 的压缩 NumPy 数据：

```bash
python scripts/misc/prepare_marvin_mujoco_calibration.py \
  --csv-path logs/marvin_calibration_<timestamp>.csv
```

输出同名 `.npz`，包含 `q_rad`、`dq_rad_s`、数值微分 `ddq_rad_s2`、`tau_nm`、多级命令、左右 TCP 变换及 feedback/dynamics/tracking 三类有效样本掩码。`tcp_actual` 是当前 URDF 对反馈关节计算的 FK，不是外部测量；它可用于时序和动力学分析，但连杆长度、基座安装和 TCP 外参仍需外部测量系统校正。

建议按以下数据集分别采集并保留原始文件：静止多姿态、单关节低速往返、单关节中速往返、单臂组合运动、双臂组合运动；空载与已知左右 Tool 不得混在同一拟合批次。训练集拟合完成后必须用独立保留轨迹比较 MuJoCo 与实机的关节响应、TCP 误差、相位延迟和峰值力矩。

## 6. 正常停止与紧急停止

正常停止实机：

1. 松开左右 Grip，确认安全状态经过 RETURNING 后回到 ARMED，双臂已返回本次启动姿态；
2. 在 Python 终端按 `Ctrl+C`；程序停止线程、请求双臂 `state=0`、回读后释放 SDK，不会自动回零；
3. 确认终端打印 JSONL、标定 CSV 和 metadata 路径；
4. PICO 中关闭 `Send` 并退出 App；
5. 需要关闭 PC Service 时执行：

```bash
pkill -f '[/]RoboticsServiceProcess'
```

再次确认端口已释放：

```bash
ss -lnt | grep -E ':(63901|60061)\b' || true
```

出现失控趋势、碰撞风险、程序无响应或硬件线程不能退出时，先按物理急停，再处理软件和网络；不要等待 ROS2、PICO 或 Python 完成优雅关闭。强制 `SIGKILL` 或断电可能没有 summary/metadata，CSV 最多丢失最后约 1 秒未 flush 数据。

## 7. 常见问题

| 现象 | 判断与处理 |
| --- | --- |
| `conda：未找到命令` | 执行 `source /home/zxcx/TeleOp/.miniconda-xr/etc/profile.d/conda.sh`，再 `conda activate Teleop`。 |
| SDK 输出七个 `0.0` | SDK 已加载但没有有效追踪帧；检查 PICO 是否唤醒、是否显示 `WORKING`，并开启 `Head/Controller` 与 `Send`。 |
| 首次读取为零 | SDK 使用异步流；初始化后等待约 1 秒并连续读取，不要只读取一次。 |
| `server stream end`、`uninitialize sdk` | 调用 `c.close()` 时的正常关闭日志，不是连接错误。 |
| `63901`、`60061` 均未监听 | PC Service 未运行；重新执行 `bash /opt/apps/roboticsservice/runService.sh`。 |
| `63901` 在监听但 PICO 不显示 `WORKING` | 核对主机 IP、同网段、UFW 和 Wi-Fi AP 客户端隔离设置。 |
| PICO 已连接但位姿仍全零 | 戴上并唤醒头显；关闭后重开 `Send`；必要时在 App 中断开并重新输入主机 IP。 |
| 桌面 Demo 关闭后链路消失 | 不使用 `run3D.sh`；单独执行 `runService.sh`。 |
| PICO 网和机械臂网位于同一子网 | 不进入实机使能；把 Marvin 放在独立有线子网，并用 `ip route get 192.168.1.190` 确认路由。 |
| `ping 192.168.1.190` 失败 | 检查专用网线、网口静态地址、子网掩码、控制器实际 IP 和路由；不要先运行 SDK 运动入口。 |
| SDK `connect()` 成功但只读检查失败 | UDP 连接成功不代表数据有效；检查 A/B `frame_serial` 是否分别递增、防火墙和其他 SDK 进程占用。 |
| `failed to connect ... controller may be occupied` | 完全关闭 MarvinPlatform 及其他 SDK/Demo 进程，等待其执行 `release_robot()` 后先重做 10 秒只读检查；不要并行连接。 |
| `Marvin clear_set() failed` | 当前版本已对启动事务忙做有界重试并在切模式前等待保持目标应答。若仍出现，停止使能，检查是否运行旧代码、控制器被占用或 SDK/控制器应答异常，不要无限重试。 |
| 启动报告 SDK 版本失配 | 当前预期 `100343014`；停止使能，核对本地 SDK、控制器兼容表和部署路径，不跳过版本检查。 |
| 启动报告 Tool mismatch | 在 MarvinPlatform 复核左右当前 Tool、夹具和 `tools_cfg.json`；默认不要用 `--configure-tools` 强行覆盖。 |
| 启动报告机械臂未静止或进入软限位区 | 不上使能；人工将机械臂置于已确认安全姿态，等待两臂 `low_speed_flag=1`，重新执行只读检查。 |
| Grip 按住时进入 HOLD | 查看日志中的 XR/反馈/命令龄期、重复序号或 3°跟踪误差；必须先松开全部 Grip 才能重新 ARM。 |
| `no command satisfies ... indices [...]` | 先保留本次 JSONL/CSV 并停止使能，不直接放宽限制。当前版本已修复 Grip 松开时固定目标被指令历史激起摆动，并在到达速度上限前为 jerk 制动预留余量。若新版本仍报错，按索引核对末帧 `q/dq/q_command`、URDF 软限位和物理姿态，视为真实不可行状态排查。 |
| 进入 FAULT 后 PICO 恢复但机械臂不动 | 正常的锁存行为；人工查明错误并按现场流程复位/清错，程序不会自动恢复。 |
| ROS2 观测启动失败 | 确认已执行 `source /opt/ros/humble/setup.bash`、ROS_DOMAIN_ID 和 DDS 网卡配置；ROS2不影响本地日志和SDK安全链路。 |
| 没有 `.metadata.json` | 进程可能被 `SIGKILL`、断电或磁盘写入失败；该会话不得作为正式标定数据。 |

服务日志位于：

```text
/home/zxcx/.local/share/PICOBusinessSuitData/log/
```

排查时可查看当天日志：

```bash
tail -n 200 /home/zxcx/.local/share/PICOBusinessSuitData/log/$(date +%Y%m%d).txt
```

## 8. 联调验收清单

- [x] `Teleop` 环境可激活，`pip check` 无依赖冲突。
- [x] `xrobotoolkit_sdk` 可导入。
- [x] PC Service 包安装成功。
- [x] `63901/tcp` 和本机 `60061/tcp` 正常监听。
- [x] PICO App 可连接 PC Service 并显示 `WORKING`。
- [x] 开启 `Head/Controller/Send` 后可读取非零动态位姿。
- [x] Marvin MuJoCo 和实机后端、安全状态机、ROS2旁路观测及标定记录的软件实现和自动测试完成。
- [ ] Marvin MuJoCo + PICO Grip 完成静止、慢速、快速往返和双臂连续运行验收。
- [ ] 真机准确型号、运动学配置、A/B关节顺序/方向/零偏、Tool和TCP完成现场冻结。
- [ ] 物理急停完成实际触发/复位测试，观察员和清空工作区就绪。
- [ ] `inspect_marvin_state.py` 的10秒检查和30分钟200 Hz只读记录均通过，SDK 版本与 A/B 帧均合格。
- [ ] 供应商逐轴点动≤2°/≤0.1 rad/s已签字；再按单臂TCP≤5 cm、双臂空载顺序通过。
- [ ] PICO停更、反馈停更、命令停更、网络断开、IK失败和线程退出故障注入符合HOLD/FAULT预期。
- [ ] ROS2话题与本地CSV时间序列连续，关闭ROS2不会影响硬件控制。
- [ ] MuJoCo参数使用训练数据拟合，并在独立保留轨迹上完成响应和延迟验证。
