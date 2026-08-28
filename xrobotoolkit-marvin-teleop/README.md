# PICO → Marvin 双臂遥操

本仓库基于 XRoboToolkit Python Sample，当前主线仅面向天机 Marvin 双臂的
PICO 遥操、MuJoCo 验证、实机安全接入和标定数据采集。上游来源与锁定版本见
[`UPSTREAM.md`](UPSTREAM.md)。

当前状态：软件链路和自动测试已完成，真实机械臂分级验收尚未完成。未经现场
机型、关节映射、Tool 和物理急停确认，不得运行带使能的实机入口。

日常修改、仿真、只读检查和实机启停的一页式清单见
[`docs/简洁版开发流程与操作指南.md`](docs/简洁版开发流程与操作指南.md)。

## 控制链路

```text
PICO ─TCP 63901→ XRoboToolkit PC Service
     ─本机 gRPC 60061→ XR latest-value 200 Hz
     → TCP guard → Placo IK 100 Hz
     → 关节限速/加速度/jerk/软限位
     → MarvinSDK 双臂关节目标 50 Hz（UDP）
```

- PICO 输入：头显和左右手柄位姿 `[x,y,z,qx,qy,qz,qw]`、左右 Grip。
- 左右 Grip 分别是左右臂 deadman；实机松开后，对应臂受限返回本次启动实测姿态，重新按下可取消回位。
- 实机遥操中 A/A 执行两点 scale 标定、B 重置；成功后在安全静止窗口保存并切换，下一次按 Grip 使用新值。Trigger、X/Y、菜单键或摇杆当前无真机行为。
- 实机使用关节阻抗：`state=3`、`impedance_type=1`，主机下发 14 轴位置目标。
- ROS2 仅为可选观测旁路，不在控制命令链路中。

## 环境

已部署主机使用：

```bash
source /home/zxcx/TeleOp/.miniconda-xr/etc/profile.d/conda.sh
conda activate Teleop
cd /home/zxcx/TeleOp/xrobotoolkit-marvin-teleop
```

首次部署、PICO 配网、双网卡和故障排查统一按
[`docs/XRoboToolkit环境部署与PICO联调流程.md`](docs/XRoboToolkit环境部署与PICO联调流程.md)
执行。

## 操作顺序

### 1. PICO → MuJoCo

先启动 PC Service，并在 PICO 中开启 `Head`、`Controller` 和 `Send`，然后运行：

```bash
python scripts/simulation/teleop_marvin_mujoco.py --scale-factor 0.5
```

仿真为物理 500 Hz、控制 100 Hz，每次控制后严格执行 5 个 MuJoCo 子步。
默认自动保存 `logs/marvin_latency_*.csv` 和对应 `.summary.json`。

### 2. Marvin 只读检查

真实机械臂保持下伺服或现场规定的安全只读状态，先做 10 秒检查：

```bash
python scripts/hardware/inspect_marvin_state.py --duration-s 10
```

通过后再记录 30 分钟基线：

```bash
python scripts/hardware/inspect_marvin_state.py --duration-s 1800
```

该入口只连接和订阅，不切换模式、不修改 Tool/K/D/速度比例，也不发送运动目标。
它会校验 SDK 版本、A/B 帧更新、有限反馈、错误码和控制器状态，并保存
`logs/marvin_calibration_*.csv` 与 `.metadata.json`。

### 3. Marvin 低速遥操

只有完整现场 Go/No-Go 通过、左右 Grip 松开且急停观察员在场时，才运行：

需要按操作者臂长校正平移比例时，先单独运行 PICO-only 标定；它不连接 Marvin：

```bash
python scripts/hardware/calibrate_marvin_scale_from_pico.py
```

松开双 Grip，自然下垂按一次 `A`，水平前伸再按一次 `A`；`B` 取消重来。成功值保存到
`logs/marvin_scale_calibration.json`。独立标定入口仍在随后启动实机入口时加载；在真机进程内完成 A/A 标定时，只会在双 Grip 松开、复位结束且机械臂静止的安全窗口保存并应用，新比例从下一次按 Grip 锁存零点后生效，不会让当前 TCP 目标跳变。

```bash
python scripts/hardware/teleop_marvin_hardware.py \
  --enable-hardware \
  --confirmed-estop \
  --confirmed-joint-mapping \
  --confirmed-robot-model "从铭牌和控制器读取的准确型号"
```

首次参数：

| 项目 | 默认值 |
| --- | --- |
| XR / IK / 反馈 / 实机命令 | 200 / 100 / 200 / 50 Hz |
| PICO→TCP 平移比例 | 优先读取已保存标定；无标定时 0.5；`--scale-factor` 可显式覆盖 |
| Grip 松手复位 | 返回本次启动实测姿态；3 s 名义余弦轨迹并继续经过全部关节保护 |
| 关节速度 / 加速度 / jerk | 0.1 rad/s / 0.3 rad/s² / 2 rad/s³ |
| 关节目标临界阻尼自然频率 | 8 rad/s |
| 软限位余量 | 5°，带预测制动 |
| TCP 半径 / 线速度 / 角速度 | 0.25 m / 0.1 m/s / 0.5 rad/s |
| 控制器速度 / 加速度比例 | 10% / 10% |

程序启动时依次检查反馈、Tool、静止状态和模式回读；先同步当前关节反馈目标，再进入
关节阻抗。任何门禁失败都不会进入 TELEOP。默认只比较 Tool；只有明确的现场参数写入
任务才允许增加 `--configure-tools`。

本入口是 TCP 笛卡尔遥操，不是单关节点动工具。A/B 与左右关节的逐轴映射必须先通过
MarvinPlatform 或供应商批准的低速点动流程完成并签字。遥操验收从单臂 TCP 小于
5 cm 开始，通过后才允许双臂空载。

## 停止与数据

正常停止：先松开双 Grip，等待双臂返回启动姿态并进入 ARMED，再按 `Ctrl+C`。程序请求双臂
`state=0`、释放 SDK 并输出日志路径。出现失控趋势、碰撞风险或进程无响应时，先按物理急停。

每次实机运行自动保存：

```text
logs/marvin_hardware_<timestamp>.jsonl
logs/marvin_hardware_<timestamp>.summary.json
logs/marvin_calibration_<timestamp>.csv
logs/marvin_calibration_<timestamp>.metadata.json
```

标定 CSV 可转换为 MuJoCo 拟合数组：

```bash
python scripts/misc/prepare_marvin_mujoco_calibration.py \
  --csv-path logs/marvin_calibration_<timestamp>.csv
```

## 验证

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  python -m pytest -q tests/test_marvin_hardware.py tests/test_marvin_mujoco_model.py
```

开发设计、参数依据和未关闭风险见
[`docs/XR-Robotics天机双臂VR遥操开发计划.md`](docs/XR-Robotics天机双臂VR遥操开发计划.md)。
机械臂控制代码入口、全部参数含义和停机后逐项调参方法见
[`docs/Marvin机械臂控制与测试调参指南.md`](docs/Marvin机械臂控制与测试调参指南.md)。
