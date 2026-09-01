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
路径。默认左右 sign 都为 `+1`：Marvin `+Y`（手柄向右）使右臂沉肘、左臂抬肘，
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
python scripts/hardware/teleop_marvin_hardware.py \
  --enable-hardware \
  --confirmed-estop \
  --confirmed-joint-mapping \
  --confirmed-robot-model "M6S-Lite-CCS-680-B"
```

真机夹爪默认禁用。拿到夹爪厂商的 Modbus-RTU 单寄存器位置协议并完成空载验证后，
为 `--gripper-config` 提供左右臂配置：

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
没有准确协议时不要提供此参数，程序不会向夹爪发送任何帧。

默认 K 为 `5 5 5 5 4 3 3`，D 为 `0.3 0.3 0.3 0.3 0.3 0.3 0.3`。覆盖参数使用
`--left-k/--left-d/--right-k/--right-d`，未经现场批准不要调节。
实机默认按 SDK 的 PD 遥操建议使用 `200 Hz / 5 ms`，并在进入关节阻抗前为
双臂设置调试值 `velRatio=10`、`AccRatio=10`。充分测试后才能手动提高。需要覆盖时使用
`--control-hz`、`--joint-velocity-ratio` 和 `--joint-acceleration-ratio`。
控制参数、模式和 PD 前馈设置后分别等待 `0.2 s / 1 s / 1 s` 并复核反馈。

## 文档

- [首次部署（PC Service 与 PICO）](docs/首次部署.md)
- [日常操作说明](docs/操作指南.md)
- [项目结构、模块职责与命名](docs/project-structure.md)
- [Marvin MuJoCo 资产说明](assets/marvin/README.md)
