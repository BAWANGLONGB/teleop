# 测试流程

本文按风险从低到高验证 XR → 位姿映射 → Marvin SDK IK → 关节目标闭环。
未通过当前阶段时，不进入下一阶段。

## 1. 测试边界

| 阶段 | PICO | `libKine.so` | `libMarvinSDK.so` | 机械臂 |
| --- | --- | --- | --- | --- |
| 自动测试 | 否 | 是 | 否 | 否 |
| MuJoCo headless | 否 | 是 | 否 | 否 |
| PICO-only | 是 | 否 | 否 | 否 |
| PICO → MuJoCo | 是 | 是 | 否 | 否 |
| 实机验收 | 是 | 是 | 是 | 是 |

除最后一阶段外，所有流程都不得连接或控制机械臂。

## 2. 环境检查

```bash
sudo apt-get install build-essential pybind11-dev libjson-c-dev
source /home/zxcx/TeleOp/.miniconda-xr/etc/profile.d/conda.sh
conda activate Teleop
cd /home/zxcx/TeleOp/xr-marvin-teleop
python -m pip install -e . --no-build-isolation
python -c "import mujoco, numpy; print('environment OK')"
python -c "from xr_marvin_teleop import _xrobotoolkit_sdk; print(_xrobotoolkit_sdk.__file__)"
```

检查三个入口只解析参数、不连接设备：

```bash
python scripts/hardware/teleop_marvin_hardware.py --help
python scripts/simulation/teleop_marvin_mujoco.py --help
python scripts/simulation/replay_marvin_log.py --help
```

## 6. PICO-only 测试

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

只读取 XR，不创建 Marvin 控制对象：

```bash
python - <<'PY'
from contextlib import closing
from xr_marvin_teleop.common.xr_client import XrClient

with closing(XrClient()) as client:
    snapshot = client.wait_for_fresh_snapshot(timeout_seconds=2.0)
    print("timestamp_ns:", snapshot.timestamp_ns)
    print("left_controller:", snapshot.left_controller_pose)
    print("right_controller:", snapshot.right_controller_pose)
    print("grip:", snapshot.grip_values)
    print("trigger:", snapshot.trigger_values)
    print("thumbstick_y:", snapshot.thumbstick_y_values)
PY
```

必须出现 `PICO XR stream ready.`，时间戳大于 0 且持续递增。仅出现
`server connect` 表示 Python 连接了本机服务，不代表 PICO 正在发送数据。
未连接 PICO 时应得到 XR 超时，不应再出现 `libMarvinSDK.so` 段错误。

## 7. PICO → MuJoCo 验收

按[仿真流程](simulation.md)启动后依次验证：

1. 双 Grip 松开时，双臂保持当前位置；
2. 左右 Grip 分别只控制对应 A/B 臂；
3. 手柄缓慢平移时，TCP 方向和 scale 符合预期；
4. 手柄保持静止时，关节目标无可见跳变；
5. 双 Grip 松开后按 B，双臂沿 3 秒余弦轨迹回位；
6. B 回位不清除已采集的第一帧 scale 标定；
7. 双臂下垂/前伸两次按 A，在线 scale 更新并持久化；
8. 关闭 PICO `Send` 后，短时陈旧数据保持关节目标，恢复时 Grip 重新锚定；持续断流后流程终止；
9. 使用日志的 `command` 与 `feedback` 两种模式完成回放。
10. 左右 Trigger 分别增量闭合对应夹爪，松开后保持；摇杆后拉闭合、前推打开；
    Trigger 与前推冲突时闭合优先。
11. IK 返回 `J4 > -5°` 时该解被拒绝，本周期保持上一条关节命令。

默认时间戳连续 `0.5 s` 不推进时进入保持/重新锚定状态，连续 `2.0 s` 不推进时
判定连接中断并结束控制。

任一项失败，保持在 MuJoCo 阶段，不进入实机。

## 8. 实机 Go/No-Go

实机启动前至少满足：

- 全量自动测试通过；
- PICO-only 与 PICO → MuJoCo 全部通过；
- 现场确认 SDK A/B、14 轴顺序和正方向；
- 当前 Tool、初始位姿和 B 键回位路径与实物一致；
- 机械臂静止、无错误，其他 SDK 客户端已退出；
- 物理急停已测试，观察员可立即触发。

启动日志和反馈还应确认：

- 双臂 `frame_serial` 连续更新后才配置控制参数；
- 首次实机调试时双臂速度/加速度比例均为 `10/10`；
- 双臂进入 `state=3`、关节阻抗 `type=1`；
- 进入阻抗模式并设置 PD 前馈后，才发送当前反馈关节姿态；
- 遥操期间双臂 `frame_serial` 持续更新；
- 默认控制频率为 `200 Hz`，PD 前馈周期为 `5 ms`；
- 程序退出时双臂进入 `state=0` 后再释放 SDK。

实机首次验收按“只读反馈 → 单臂小位移 → 双臂小位移”逐级进行。异常运动时
优先使用物理急停，程序退出不能替代急停。
