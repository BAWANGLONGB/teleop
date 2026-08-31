# PICO → Marvin 双臂遥操

本仓库只维护 Marvin 厂家 SDK/资料和当前最小遥操工程。

## 目录

| 路径 | 内容 |
| --- | --- |
| [`TJArm/`](TJArm/) | Marvin 控制 SDK、运动学 SDK、配置、示例和厂家文档 |
| [`xr-marvin-teleop/`](xr-marvin-teleop/) | PICO → 在线 scale 标定/读取 → 位姿映射 → Marvin IK → 双臂关节目标 |

## 环境

```bash
source /home/zxcx/TeleOp/.miniconda-xr/etc/profile.d/conda.sh
conda activate Teleop
cd /home/zxcx/TeleOp/xr-marvin-teleop
python -m pip install -e .
```

## 测试

```bash
python -m unittest discover -s tests -v
```

## PICO → MuJoCo

先启动 XRoboToolkit PC Service，并在 PICO 中确认 `Network=WORKING`，打开
`Head`、`Controller` 和 `Send`：

```bash
python scripts/simulation/teleop_marvin_mujoco.py
```

## 实机

启动 PC Service：

```bash
bash /opt/apps/roboticsservice/runService.sh
pgrep -af RoboticsServiceProcess
ss -lnt | grep -E ':(63901|60061)\b'
```

PICO 端确认：

1. Network 为 `WORKING`；
2. `Controller`、`Send` 已打开；
3. `Switch w/ A Button` 关闭；
4. 头显保持唤醒，左右手柄均已连接；摘下使用时关闭自动休眠。

仅在 PICO、MuJoCo、机型、A/B 关节映射、Tool 和物理急停全部确认后运行：

```bash
python scripts/hardware/teleop_marvin_hardware.py \
  --enable-hardware \
  --confirmed-estop \
  --confirmed-joint-mapping \
  --confirmed-robot-model "M6S-Lite-CCS-680-B"
```

默认首次调试参数为 `velRatio=10`、`AccRatio=10`、`D=0.3`。完整启动、测试、
标定、日志和仿真说明见 [`xr-marvin-teleop/README.md`](xr-marvin-teleop/README.md)。

出现异常运动时优先使用物理急停；PICO 按键和程序退出不能替代急停。
