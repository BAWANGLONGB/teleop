# MuJoCo 仿真流程

## 1. 数据链路

```text
PICO → XRoboToolkit PC Service → 原子 get_snapshot() → XrClient
     → 在线 A/A scale 标定/读取 → 位姿映射
     → Marvin SDK IK（ZSPType=0 + RefJoint）
     → 遥操关节目标 / B 键回位轨迹
     → MuJoCo position actuators
     → JSONL 日志 → MuJoCo 回放
```

仿真和实机共用 `MarvinHardwareTeleopController`。仿真只把最终适配器替换为
`MarvinMujocoAdapter`，因此 Grip、scale、映射、IK 和 B 键回位逻辑保持一致。
仿真不会导入 `libMarvinSDK.so`，也不会连接机械臂。

## 2. 模型与单位

- 模型：`assets/marvin/marvin_dual.mujoco.xml`；
- 关节顺序：`Joint1_L..Joint7_L, Joint1_R..Joint7_R`；
- 控制量和状态：弧度、弧度每秒；
- 物理步长：`0.002 s`（500 Hz）；
- 默认遥操频率：`50 Hz`，每个控制周期执行 10 个物理步；
- 初始姿态：A `[90,-90,-90,-20,90,0,0]°`，
  B `[-90,-90,90,-20,-90,0,0]°`。

应用层没有 limiter。MJCF 中的 joint range、actuator ctrlrange 和动力学参数属于
仿真模型本身；实机约束由 Marvin 硬件控制器执行。

## 3. 启动准备

```bash
source /home/zxcx/TeleOp/.miniconda-xr/etc/profile.d/conda.sh
conda activate Teleop
cd /home/zxcx/TeleOp/xr-marvin-teleop
python -m pip install -e . --no-build-isolation
python -m unittest discover -s tests -v
```

按[测试流程](testing.md#6-pico-only-测试)确认 PICO 数据持续递增。PICO 端必须
打开 `Head`、`Controller` 和 `Send`，并关闭 `Switch w/ A Button`。

## 4. 实时遥操

```bash
python scripts/simulation/teleop_marvin_mujoco.py --scale-factor 0.5
```

默认参数：

| 参数 | 默认值 |
| --- | --- |
| MJCF | `assets/marvin/marvin_dual.mujoco.xml` |
| Marvin 运动学 SDK | `../TJArm/tj_fx_robot-master` |
| Tool | `../TJArm/tools_cfg.json` 当前 A/B Tool |
| 控制频率 | `50 Hz` |
| B 键回位 | `3 s` 余弦轨迹 |
| scale | 命令行 > 已保存标定 > `1.0` |
| scale 文件 | `logs/marvin_scale_calibration.json` |
| 日志目录 | `logs/` |

常用覆盖：

```bash
python scripts/simulation/teleop_marvin_mujoco.py \
  --scale-factor 0.5 \
  --control-hz 100 \
  --return-duration 3.0 \
  --log-directory logs
```

控制频率必须在 `[50, 200] Hz`，且控制周期必须是 `0.002 s` 物理步长的整数倍。

无图形界面运行：

```bash
python scripts/simulation/teleop_marvin_mujoco.py --headless --scale-factor 0.5
```

headless 模式仍需要实时 PICO 数据；它用于远程运行和数据采集，不是合成 XR
测试。合成 XR 测试使用 `python -m unittest discover -s tests -v`。

## 5. 操作方式

- 左 Grip `> 0.9`：锁存左手柄与 A 臂 TCP，开始左臂遥操；
- 右 Grip `> 0.9`：锁存右手柄与 B 臂 TCP，开始右臂遥操；
- Grip 松开：对应臂保持当前关节姿态并清除遥操锚点；
- 双 Grip 松开后按 B：双臂沿 3 秒余弦轨迹返回初始姿态；
- IK 成功：发送新的 7 轴目标；
- IK 奇异、越界或无解：保持上一关节目标；
- 关闭窗口或按 `Ctrl+C`：停止仿真并关闭日志。

在线 A/A scale 标定：

1. 松开双 Grip；
2. 双臂自然下垂，按一次 A；
3. 双臂水平前伸，再按一次 A；
4. 新 scale 保存到 `logs/marvin_scale_calibration.json`。

B 只触发机器人回位，不清除尚未完成的标定采样。

## 6. 会话日志

启动后打印日志路径，文件名为：

```text
logs/marvin_mujoco_<timestamp>.jsonl
```

日志写盘使用独立线程，控制循环只把记录放入队列。每个 `control_cycle` 包含：

- `monotonic_time_ns`、`xr_frame_valid`、`xr_timestamp_ns`；
- 头显、左手柄、右手柄 OpenXR 位姿；
- Grip、A/B、当前 scale；
- A/B 帧号、状态和错误码；
- `q_feedback_rad`、`dq_feedback_rad_s`；
- 最终 `q_command_rad`。

短时 XR 掉帧记录仍保留关节反馈和指令，但 `xr_frame_valid=false`，XR 字段为
`null`。

## 7. 日志回放

按记录的最终关节命令驱动 MuJoCo actuator：

```bash
python scripts/simulation/replay_marvin_log.py \
  logs/marvin_mujoco_<timestamp>.jsonl \
  --source command
```

直接设置记录的反馈关节状态，适合复现实机轨迹：

```bash
python scripts/simulation/replay_marvin_log.py \
  logs/marvin_hardware_<timestamp>.jsonl \
  --source feedback
```

可选参数：

| 参数 | 作用 |
| --- | --- |
| `--speed 0.5` | 半速回放 |
| `--speed 2` | 两倍速回放 |
| `--max-frames 500` | 最多回放 500 帧 |
| `--headless` | 不创建窗口 |
| `--xml-path PATH` | 使用指定 MJCF |

## 8. 常见问题

| 现象 | 检查项 |
| --- | --- |
| 只显示 `server connect` | PICO 是否为 `WORKING`，Head/Controller/Send 是否开启 |
| `PICO produced no advancing...` | XR 时间戳为 0 或停止递增，保持头显唤醒 |
| MJCF 加载失败 | `assets/marvin/meshes/` 是否完整 |
| IK 保持上一目标 | 检查目标是否超工作空间、奇异或关节越界 |
| 回放无记录 | 文件是否为本项目 `control_cycle` JSONL |
| 无法创建窗口 | 使用 `--headless`，或检查图形会话和 `DISPLAY` |
