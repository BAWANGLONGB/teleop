# PICO → Marvin 最小遥操闭环

```text
XR → 在线 A/A scale 标定/读取 → 位姿映射
   → Marvin SDK IK → 遥操目标 / B 键回位
   → set_joint_cmd_pose(A/B) 或 MuJoCo
```

项目保留一条共享控制链，提供实机与 MuJoCo 两个后端。厂商 SDK 负责机械软限位；
共享 IK 边界额外要求 `J4 <= -5°`，避免遥操作跨过 `J4=0°` 奇异位形。
MuJoCo 的其余关节约束来自 MJCF 模型。

## 安全边界

- 优先完成离线测试和 PICO → MuJoCo 验收；
- 实机必须确认急停、A/B 关节映射、Robot 型号、Tool 和回位路径；
- 程序退出不能替代物理急停；异常运动时优先触发急停；
- MuJoCo 和离线测试不会加载 Marvin 控制 SDK，也不会连接机械臂。

## 安装

PC Service 与 PICO 安装文件的版本、校验和官方获取地址见
[`../pico-service-software/README.md`](../pico-service-software/README.md)。

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

左右 Trigger 和摇杆 Y 轴采用增量夹爪控制：Trigger 或后拉闭合，前推打开，
输入回中后保持；冲突时闭合优先。默认全行程约 `2 s`，夹爪目标最多按 `20 Hz`
更新。当前 MuJoCo 夹爪仍是固定视觉模型，仿真会验证和记录归一化夹爪目标。

如需让冗余构型偏向 J3，可选启用 IK_NSP。Grip 按下时，以按下瞬间的手柄横向位置
为零点，横向移动会映射为 `ZSP_Angle`；默认最大偏角为 `5°`，并按斜率渐变，避免
首次切换跳变：

```bash
python scripts/simulation/teleop_marvin_mujoco.py --headless \
  --nsp-lateral --nsp-max-angle 5 --nsp-angle-rate 20
```

默认死区为 `0.03 m`、满量程为 `0.12 m`，可用
`--nsp-lateral-deadzone` 和 `--nsp-lateral-range` 调整。左右硬件的角度方向不一致
时，可用 `--nsp-lateral-sign-left/right {-1,1}` 校准。NSP 失败、超限或单步变化过大
时回退普通 IK；不提供 `--nsp-lateral` 或旧的 `--nsp-angle-left/right` 时保持普通 IK
路径。默认左右 sign 都为 `+1`：Marvin `+X`（手柄向右）使右臂沉肘、左臂抬肘，
反向移动则相反；旧参数仍保留用于固定角度兼容场景。

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
unset LD_PRELOAD
python scripts/hardware/teleop_marvin_hardware.py \
  --enable-hardware \
  --confirmed-estop \
  --confirmed-joint-mapping \
  --confirmed-robot-model "M6S-Lite-CCS-680-B"
```

真机默认不启用夹爪。若使用 Marvin Modbus，完成厂商协议和空载验证后，为
`--gripper-config` 提供左右臂配置：

```jsonc
{
  "left": {
    "slave_id": 1,
    "position_register": "<厂商位置寄存器>",
    "open_position": "<全开值>",
    "closed_position": "<全闭值>",
    "initial_closedness": "<启动实际闭合度 0..1>",
    "channel": 2
  },
  "right": {
    "slave_id": 1,
    "position_register": "<厂商位置寄存器>",
    "open_position": "<全开值>",
    "closed_position": "<全闭值>",
    "initial_closedness": "<启动实际闭合度 0..1>",
    "channel": 2
  }
}
```

尖括号是说明文字，实际文件必须替换成整数/浮点数。`channel=2/3` 分别对应
COM1/COM2。若 PICO 摇杆前后方向相反，启动时追加 `--thumbstick-y-sign -1`。
没有准确协议时不要提供此参数，程序不会向 Marvin Modbus 夹爪发送任何帧。

### DAS Finger Controller 夹爪

DAS 夹爪控制使用官方 Python SDK，不依赖 ROS2。先按官方仓库完成安装和 USB/udev
配置，并准备 SDK checkout 路径：

```bash
git clone https://github.com/genrobot-ai/gen_finger_con_python_sdk_release.git \
  /home/zxcx/TeleOp/gen_finger_con_python_sdk_release
python -m pip install -r \
  /home/zxcx/TeleOp/gen_finger_con_python_sdk_release/requirements.txt
```

依赖必须安装到运行本 TeleOp 的同一个 `Teleop` Python 环境；不要只安装在另一个
SDK 虚拟环境中，否则 `FingerSystem` 无法被当前进程加载。

复制 [`config/das_gripper.example.json`](config/das_gripper.example.json)，按实际
空载行程修改 `closed_distance_m`、`open_distance_m` 和区间内的安全
`startup_distance_m`。示例默认最小距离为 `0.000 m`，但启动仍使用安全的
`0.050 m`，不会在连接时主动闭合到零。DAS 只在显式提供以下两个参数时启用：

```bash
unset LD_PRELOAD
python scripts/hardware/teleop_marvin_hardware.py \
  --enable-hardware \
  --confirmed-estop \
  --confirmed-joint-mapping \
  --confirmed-robot-model "M6S-Lite-CCS-680-B" \
  --das-gripper-config config/das_gripper.example.json \
  --das-sdk-root /home/zxcx/TeleOp/gen_finger_con_python_sdk_release
```

程序先完成 DAS 自检，再连接 Marvin；首个编码器请求携带配置的安全启动距离。
若仍返回 `-66.66`，清空对应夹爪并单独标定（该命令不会连接 Marvin）：

```bash
python scripts/hardware/calibrate_das_finger.py \
  --config config/das_gripper.example.json \
  --side left --confirmed-gripper-clear
```

Trigger 或摇杆后拉闭合，摇杆前推张开，输入释放后保持。

### ROS2 数据采集（可选）

ROS2 是采集数据总线，也是 PICO、DAS 与控制任务之间的边界。PICO SDK 和 DAS
SDK/双目相机分别由独立进程持有；Marvin 控制进程只订阅输入、下发关节命令和夹爪
ROS2 命令，录制与图像处理不会进入控制进程。先安装 MCAP 后端、构建消息包并 source：

```bash
sudo apt-get install ros-humble-rosbag2-storage-mcap
cd ros2_ws
colcon build --packages-select teleop_msgs \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DPYTHON_EXECUTABLE=/usr/bin/python3
source install/setup.bash
cd ..
```

以下每个终端都需要激活 `Teleop` 环境并 source 同一个 `install/setup.bash`。如果当前
终端曾设置系统 `libstdc++` 预加载，先清除：

```bash
unset LD_PRELOAD
```

推荐使用 supervisor 一条命令启动 PICO、DAS、recorder 和 Marvin 控制，并在退出时
按安全顺序收尾：

```bash
python scripts/data/run_collection.py \
  --task pick_and_place \
  --operator zxcx \
  --robot-model "M6S-Lite-CCS-680-B" \
  --enable-hardware \
  --confirmed-estop \
  --confirmed-joint-mapping \
  --das-config config/das_gripper.example.json \
  --das-sdk-root /home/zxcx/TeleOp/gen_finger_con_python_sdk_release
```

它会依次等待有效 PICO 帧、左右 DAS 编码器反馈和左右相机首帧，再启动 Marvin；并固定
Marvin、PICO、左右 DAS、recorder 的 CPU 亲和性，将 recorder 设为 `nice +10`。按一次
`Ctrl+C` 后依次停止 Marvin、DAS、完成 MCAP/manifest、停止 PICO。Pico PC Service
仍需提前独立启动。完整 SOP 见[日常操作说明](docs/操作指南.md)。如需分进程排障，
按以下顺序启动。先发布 PICO：

```bash
python scripts/data/publish_pico.py
```

再在两个终端分别启动左右 DAS 数据源：

```bash
python scripts/data/publish_das.py \
  --side left \
  --config config/das_gripper.example.json \
  --sdk-root /home/zxcx/TeleOp/gen_finger_con_python_sdk_release

# 另一个终端
python scripts/data/publish_das.py \
  --side right \
  --config config/das_gripper.example.json \
  --sdk-root /home/zxcx/TeleOp/gen_finger_con_python_sdk_release
```

随后创建 episode；recorder 会为左右相机各启动一个原生 MJPEG 写盘进程：

```bash
python scripts/data/record_episode.py \
  --task pick_and_place \
  --operator zxcx \
  --das-config config/das_gripper.example.json \
  --calibration logs/marvin_scale_calibration.json \
  --calibration config/das_gripper.example.json
```

最后启动实机，硬件确认参数保持不变，并追加：

```text
--ros2 --pico-from-ros2 --das-from-ros2
```

主要话题如下；每个流独立编号，不再生成中心化 `/teleop/sample`：

| 类别 | 话题 |
| --- | --- |
| PICO | `/raw/pico/frame` |
| Marvin | `/raw/marvin/joint_state`、`/command/marvin/joint_target` |
| DAS | `/raw/das/{left,right}/state`、`/command/das/target` |
| 触觉 | `/raw/das/{left,right}/tactile` |
| 图像 | `/raw/das/{left,right}/image/compressed` |
| 运行状态 | `/diagnostics`、`/episode/state`、`/episode/event` |

`header.stamp` 是采集机墙钟，`receive_steady_ns` 是不受校时影响的本机单调时钟，
`source_timestamp_ns` 保存设备原始时间戳；设备不提供硬件时间时该字段为 `0`。
离线统一使用消息内的 `receive_steady_ns` 对齐，禁止控制线程等待多传感器凑齐一帧。
相机原生 MJPEG 不经过解码、重编码或 ROS2 DDS，独立进程直接写入 MCAP。

默认输出为 `dataset/session_<date>/episode_<time>_<id>/`，在线阶段写入 `state/`、
`vision_left/`、`vision_right/` 三个隔离 bag；结束后自动按统一时间轴合并为完整
`data/` MCAP，并补充左右反馈/控制 TCP 6D pose。`calibration/` 保存标定快照，
`metadata.json` 保存任务、代码版本和对齐统计，`manifest.json` 保存完整性检查与文件
SHA-256。也可重新执行：

```bash
python scripts/data/validate_episode.py \
  dataset/session_<date>/episode_<time>_<id>
```

生成人工审阅视频：

```bash
/usr/bin/python3 scripts/data/review_episode.py \
  dataset/session_<date>/episode_<time>_<id>
```

输出的 `review.mp4` 包含左右画面、机械臂关节反馈/目标、TCP pose、夹爪反馈/目标及
各状态与图像的时间差。

JSONL 继续作为控制调试日志，不作为训练数据的主格式。

默认 K 为 `5 5 5 5 4 3 3`，D 为 `0.3 0.3 0.3 0.3 0.3 0.3 0.3`。覆盖参数使用
`--left-k/--left-d/--right-k/--right-d`，未经现场批准不要调节。
实机默认使用 `50 Hz / 20 ms`，并在进入关节阻抗前为
双臂设置调试值 `velRatio=10`、`AccRatio=10`。充分测试后才能手动提高。需要覆盖时使用
`--control-hz`、`--joint-velocity-ratio` 和 `--joint-acceleration-ratio`。
控制参数、模式和 PD 前馈设置后分别等待 `0.2 s / 1 s / 1 s` 并复核反馈。

## 文档

- [首次部署（PC Service 与 PICO）](docs/首次部署.md)
- [日常操作说明](docs/操作指南.md)
- [项目结构、模块职责与命名](docs/project-structure.md)
- [Marvin MuJoCo 资产说明](assets/marvin/README.md)
