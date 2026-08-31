# PICO → Marvin 最小遥操闭环

```text
XR → 在线 A/A scale 标定/读取 → 位姿映射
   → Marvin SDK IK → 遥操目标 / B 键回位
   → set_joint_cmd_pose(A/B) 或 MuJoCo
```

项目保留一条共享控制链，提供实机与 MuJoCo 两个后端。应用层不包含 limiter；
实机限位由 Marvin 硬件控制器负责，仿真约束来自 MJCF 模型。

## 安全边界

- 优先完成离线测试和 PICO → MuJoCo 验收；
- 实机必须确认急停、A/B 关节映射、Robot 型号、Tool 和回位路径；
- 程序退出不能替代物理急停；异常运动时优先触发急停；
- MuJoCo 和离线测试不会加载 Marvin 控制 SDK，也不会连接机械臂。

## 安装

```bash
sudo apt-get install build-essential pybind11-dev libjson-c-dev
source /home/zxcx/TeleOp/.miniconda-xr/etc/profile.d/conda.sh
conda activate Teleop
cd /home/zxcx/TeleOp/xr-marvin-teleop
python -m pip install -e . --no-build-isolation
```

安装会编译项目内的原子 XR 快照 binding，并链接默认位置
`/opt/apps/roboticsservice/SDK`。SDK 位于其他目录时设置
`XROBOTOOLKIT_SDK_ROOT` 和 `XROBOTOOLKIT_SDK_LIBRARY_DIR`。

## PICO → MuJoCo

PICO 已连接且 `Controller/Send` 打开后运行；头显摘下使用时需关闭自动休眠：

```bash
python scripts/simulation/teleop_marvin_mujoco.py --scale-factor 0.5
```

## 日志回放

实机与仿真默认把控制周期写入 `logs/*.jsonl`：

```bash
python scripts/simulation/replay_marvin_log.py \
  logs/marvin_hardware_<timestamp>.jsonl \
  --source command
```

使用 `--source feedback` 回放反馈状态；无窗口验证追加 `--headless`。

## 实机启动

启动 PC Service：

```bash
bash /opt/apps/roboticsservice/runService.sh
pgrep -af RoboticsServiceProcess
ss -lnt | grep -E ':(63901|60061)\b'
```

仅在测试和现场确认全部通过后运行：

```bash
python scripts/hardware/teleop_marvin_hardware.py \
  --enable-hardware \
  --confirmed-estop \
  --confirmed-joint-mapping \
  --confirmed-robot-model "M6S-Lite-CCS-680-B"
```

默认 K 为 `5 5 5 5 4 3 3`，D 为 `0.3 0.3 0.3 0.3 0.3 0.3 0.3`。覆盖参数使用
`--left-k/--left-d/--right-k/--right-d`，未经现场批准不要调节。
实机默认按 SDK 的 PD 遥操建议使用 `200 Hz / 5 ms`，并在进入关节阻抗前为
双臂设置调试值 `velRatio=10`、`AccRatio=10`。充分测试后才能手动提高。需要覆盖时使用
`--control-hz`、`--joint-velocity-ratio` 和 `--joint-acceleration-ratio`。
控制参数、模式和 PD 前馈设置后分别等待 `0.2 s / 1 s / 1 s` 并复核反馈。

## 文档

- [测试流程](docs/testing.md)
- [MuJoCo 仿真流程](docs/simulation.md)
- [项目结构、模块职责与命名](docs/project-structure.md)
- [Marvin MuJoCo 资产说明](assets/marvin/README.md)
