# PICO → Marvin 双臂遥操

本仓库维护 Marvin 厂家 SDK/资料、当前遥操工程和本地数据采集 UI。

## 目录

| 路径 | 内容 |
| --- | --- |
| [`TJArm/`](TJArm/) | Marvin 控制 SDK、运动学 SDK、配置、示例和厂家文档 |
| [`xr-marvin-teleop/`](xr-marvin-teleop/) | PICO → Marvin 双臂控制，以及 ROS2 原始流与双 MCAP 数据采集 |
| [`UI/`](UI/) | 本地采集控制台、设备状态监测、实时预览和数据集管理 |
| [`xr-marvin-teleop/docs`](xr-marvin-teleop/docs) | 包含首次部署和操作说明文档 |
| [`pico-service-software/`](pico-service-software/) | PC Service/PICO 安装文件版本、SHA-256 和官方网络来源 |

## 环境

```bash
source /home/zxcx/TeleOp/.miniconda-xr/etc/profile.d/conda.sh
conda activate Teleop
cd /home/zxcx/TeleOp/xr-marvin-teleop
python -m pip install -e .
```

## 数据采集 UI

启动本地服务：

```bash
cd /home/zxcx/TeleOp
python3 UI/server.py
```

浏览器访问 <http://127.0.0.1:4173>。UI 后端会为采集进程激活 `Teleop` 环境，加载
ROS Humble 和工作区 `ros2_ws/install/setup.bash`，并清除 `LD_PRELOAD`；设备状态页
本身不查询 ROS：PICO 检查 TCP `63901/60061`、外部连接及客户端 Ping，Marvin Ping
`192.168.1.190`，DAS 只检查左右串口与相机设备节点。

PICO 端连接步骤：

1. PICO 与采集机 Wi-Fi 连接同一局域网；
2. 打开 XRoboToolkit，进入 `Data & Control → PC Service → Enter`；
3. 填写采集机 Wi-Fi IPv4，不能填写 `127.0.0.1` 或 Marvin 的 `192.168.1.190`；
4. 确认 `Network=WORKING`，打开 `Head`、`Controller`、`Send`；
5. 关闭 `Switch w/ A Button`，保持头显和左右手柄唤醒；
6. 返回 UI，确认 PICO 显示“已连接”。

UI 使用流程：

1. 在“采集作业”确认设备状态并完成现场安全检查；
2. 填写任务、采集员和机器人型号，选择视觉、分辨率及最长时长；
3. 焦点位于采集配置时按 `Enter`，或点击“开始采集”；
4. 通过左右相机 30 FPS 预览和实时错误区监督采集；
5. 点击“停止并保存”或按 `Esc` 安全收尾；
6. 在“数据集”中搜索、查看、导出 MCAP，删除操作只会移入 `.trash`。

UI 停止不是急停；发生异常运动时立即使用物理急停。

## 测试

```bash
cd /home/zxcx/TeleOp/xr-marvin-teleop
python -m unittest discover -s tests -v

cd /home/zxcx/TeleOp
python3 UI/server.py --self-test
node UI/test.mjs
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

最小指令
```bash
  source /home/zxcx/TeleOp/.miniconda-xr/etc/profile.d/conda.sh
  conda activate Teleop
  cd /home/zxcx/TeleOp/xr-marvin-teleop

  python -m pip install -e . --no-build-isolation

  python scripts/hardware/teleop_marvin_hardware.py \
    --enable-hardware \
    --confirmed-estop \
    --confirmed-joint-mapping \
    --confirmed-robot-model "M6S-Lite-CCS-680-B" \
    --das-gripper-config config/das_gripper.example.json \
    --das-sdk-root /home/zxcx/TeleOp/gen_finger_con_python_sdk_release

```

可选参数：`--nsp-lateral` `--nsp-max-angle 10`
```bash
python scripts/hardware/teleop_marvin_hardware.py \
  --enable-hardware \
  --confirmed-estop \
  --confirmed-joint-mapping \
  --confirmed-robot-model "M6S-Lite-CCS-680-B" \
  --nsp-lateral \
  --nsp-max-angle 10
```

标定、日志和仿真说明见 [`xr-marvin-teleop/README.md`](xr-marvin-teleop/README.md)。
首次部署请先阅读[部署与验收文档](xr-marvin-teleop/docs/首次部署.md)，
日常使用参见[操作说明](xr-marvin-teleop/docs/操作指南.md)。

出现异常运动时优先使用物理急停；PICO 按键和程序退出不能替代急停。
