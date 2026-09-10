# PICO → Marvin 最小遥操闭环

```text
XR → 在线 A/A scale 标定/读取 → 位姿映射
   → Marvin SDK IK → 遥操目标 / B 键回位
   → set_joint_cmd_pose(A/B) 或 MuJoCo
```

项目保留一条共享控制链，提供实机与 MuJoCo 两个后端。厂商 SDK 负责机械软限位；
共享 IK 边界额外要求 `J4 <= -5°`，避免遥操作跨过 `J4=0°` 奇异位形。
MuJoCo 的其余关节约束来自 MJCF 模型。

首次使用先读[部署步骤](docs/首次部署.md)，日常数采按[操作指南](docs/操作指南.md)执行；
修改代码前可从[模块导航](docs/project-structure.md)和[接口契约](docs/architecture-and-interface-spec.md)了解调用边界。

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
输入回中后保持；冲突时闭合优先。默认满输入的归一化全行程约 `1 s`，夹爪目标最多按 `20 Hz`
更新。当前 MuJoCo 夹爪仍是固定视觉模型，仿真会验证和记录归一化夹爪目标。

如需让冗余构型偏向 J3，可选启用 IK_NSP。Grip 按下时，以按下瞬间的手柄位置
为零点，Marvin X 位移会映射为 `ZSP_Angle`；默认最大偏角为 `5°`，并按斜率渐变，避免
首次切换跳变：

```bash
python scripts/simulation/teleop_marvin_mujoco.py --headless \
  --nsp-lateral --nsp-max-angle 5 --nsp-angle-rate 20
```

默认死区为 `0.03 m`、满量程为 `0.12 m`，可用
`--nsp-lateral-deadzone` 和 `--nsp-lateral-range` 调整。左右硬件的角度方向不一致
时，可用 `--nsp-lateral-sign-left/right {-1,1}` 校准。NSP 失败、超限或单步变化过大
时回退普通 IK；不提供 `--nsp-lateral` 或旧的 `--nsp-angle-left/right` 时保持普通 IK
路径。参数沿用 `lateral` 命名，但当前实现读取 Marvin X（对应 OpenXR Z，即手柄前后方向）；
手柄向右的 OpenXR +X 映射到 Marvin +Y。左右 sign 默认均为 `+1`，实际肘部运动需现场校准。
旧参数仍保留用于固定角度兼容场景。

## 日志回放

实机与仿真默认把控制周期写入 `logs/*.jsonl`：

```bash
python scripts/simulation/replay_marvin_log.py \
  logs/marvin_hardware_<timestamp>.jsonl \
  --source command
```

使用 `--source feedback` 回放反馈状态；无窗口验证追加 `--headless`。
JSONL 不纳入版本控制；`logs/marvin_scale_calibration.json` 是有效标定配置，不能作为日志删除。

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

相机支持 `640x480` 和 `1600x1296`。实测时间校正分别为：`640x480` 左侧
`24 ms`、右侧 `25 ms`；`1600x1296` 左侧 `22 ms`、右侧 `24 ms`。其他分辨率
未标定时配置值为 `null`。这些值仅保留在配置快照中，当前录制和导出均不应用时延校正。

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

数采设置统一放在 [`config/collection.json`](config/collection.json)：`paths` 管理路径，
`robot` 管理连接与映射，`capture` 管理模态/采样，`recording` 管理话题/缓存/ROS bag 配置，
`preview` 管理预览，`runtime` 管理 CPU/nice/超时，`export` 管理双 MCAP 与 H.264 参数。
左右相机的分辨率/FPS、触觉频率和夹爪标定仍只在主配置引用的 DAS JSON 中设置；ROS 原生 YAML 保留。
使用前必须核对 `robot.model` 与实机型号，配置不能替代本次安全确认。

```bash
# 只解析、校验和打印，不连接设备、不创建采集目录
python scripts/data/run_collection.py --print-effective-config
# 可选局部配置覆盖默认文件；显式 CLI 的优先级最高
python scripts/data/run_collection.py --config config/collection.local.json \
  --print-effective-config --no-mjpeg --h264-crf 28
```

局部 JSON 只需包含要修改的字段；未知字段、重复键和类型错误会报错。字典按字段覆盖，数组整体替换。
配置内相对路径相对于该文件，CLI 相对路径相对于当前目录。`--vision` / `--no-vision`、
`--preview` / `--no-preview`、`--nsp-lateral` / `--no-nsp-lateral` 可显式覆盖布尔值；
`--unlimited-duration` 可清除配置中的最长录制时长。

启动前复制主配置引用的 DAS、标定、URDF、ROS YAML 到快照；运行中不热更新。
Episode 的 `config/collection.json`、`config/files.json` 与文件快照会内嵌到两个最终 MCAP。
`--part devices` 运行时固定设备参数；新录制若修改机器人、夹爪或 PICO 相关设置会拒绝启动，
相机配置（由 recorder 持有）及离线编码参数允许逐段变化。互斥/核对以同一 `output_root` 为边界。

位移比例优先使用显式 `--scale-factor`（或配置 `robot.scale_factor`），否则读取已有标定。
标定文件缺失时自动以 `scale-factor=1.2` 生成并保存，再纳入采集快照；已有标定不会被覆盖。
默认文件不包含实测臂长，仍可通过 A/A 操作完成实际标定。

UI 后端通过 `python ../UI/server.py --collection-config config/collection.json` 选择同一主配置；
表单初始值由后端读取，表单修改作为本次覆盖。更换主配置/输出目录后重启 UI 后端。
默认开启实时预览（`preview.enabled=true`），使用共享内存目录 `/dev/shm/fieldnote-preview-zxcx`，最高刷新率为 `30 FPS`。
开启双目视觉并开始录制后显示画面；仅启动设备时不采集相机画面。修改预览配置后重启 UI 后端并刷新页面。
无需预览时设置 `preview.enabled=false` 或使用 CLI `--no-preview`。

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
`Ctrl+C` 后依次停止 Marvin、DAS、关闭原始 MCAP、停止 PICO。最终 MCAP 在数据集页面选择格式后打包。Pico PC Service
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
最终时间轴使用采集时的系统时间 `header.stamp`，不做单调时钟映射、不扣除相机时延。
无 header 的 Episode 消息使用 `wall_time_ns`；诊断和无采集时间的消息使用 bag 系统时间。
禁止控制线程等待多传感器凑齐一帧。
相机原生 MJPEG 不经过解码、重编码或 ROS2 DDS，独立进程直接写入 MCAP。

在线阶段在临时 `dataset/session_<date>/episode_<time>_<id>/` 中写入 `state/`、
`vision_left/`、`vision_right/` 三个隔离 bag。停止后先关闭原始文件，标记 `export_status=pending`，
UI / `run_collection.py` 不自动打包。在数据集页面勾选段落，选择 `.h264.mcap` 或 `.mjpeg.mcap`，
点击“打包并导出”并选择保存目录，此时才生成所选格式。已生成的格式直接下载，另一格式可随后补生成，已有文件不覆盖。
打包期间不能录制，设备可保持运行；失败显示错误，原始数据保留。也可在录制停止后使用命令行：

```bash
python -m pip install -e '.[h264]'
python scripts/data/postprocess_episode.py dataset/session_<date>/episode_<time>_<id>
```

Episode 的 `final/` 下默认只有两个最终文件，每个都包含完整状态/指令/触觉、FK TCP、
双路图像，以及元数据/校准附件，无需外部 MP4 或 Parquet：

```text
final/episode_<time>_<id>.mjpeg.mcap  # 原始 JPEG 字节，foxglove.CompressedImage
final/episode_<time>_<id>.h264.mcap   # Annex B，foxglove.CompressedVideo
```

三个入口 `run_collection.py`、`record_episode.py`、`postprocess_episode.py` 均支持
`--no-mjpeg` / `--no-h264`（至少开启一种）。H.264 参数：`--h264-crf 23`（1–51，越低画质越高）、
`--h264-preset veryfast`、`--h264-keyint 60`（帧）、`--h264-threads 2`（1–16）。
后处理默认继承录制参数，也可通过命令行覆盖。关闭 MJPEG 最终输出不会取消转码需要的临时 JPEG。
新 Episode 的后处理默认使用录制快照，不读取已更改的公共采集设置；历史无快照 Episode 保持兼容。
离线 `--config` 只接受 `export` 和 `runtime.export_nice` 的局部覆盖，CLI 仍支持 `--urdf` 等原有接口。
原始设置保存在 `capture_config` / `video_outputs`，实际导出参数另存为 `export_options` / `export_config`。
H.264 固定无 B 帧，每个消息一个完整 AU，每个 IDR 带 SPS/PPS。
消息布局遵循 [Foxglove CompressedVideo 规范](https://docs.foxglove.dev/docs/sdk/schemas/compressed-video)。

导出以低 CPU 优先级执行，并通过同一 `--output-root` 下的互斥锁拒绝与录制并行运行；
设备模式 `--part devices` 独立运行，不阻止保存。导出期间新录制会快速报忙，UI 打包期间启动新录制会报忙。
原始 bag 保留用于恢复和重新调参，
确认最终结果后可人工归档/清理；已有 `final/` 不会被覆盖；CLI 使用 `--add-missing` 可补生成缺失格式。
两个文件都可直接在 Foxglove 的 Image 面板选择相应 `image/compressed` 或 `video/compressed` 话题。
旧 LeRobot 解包工具保留用于历史文件；UI 按所选格式先打包，再下载 `final/<episode_id>.h264.mcap` 或 `.mjpeg.mcap`。

历史 Session 可批量迁移（先停止采集并 source ROS2 环境）：

```bash
python -m pip install -e '.[h264,lerobot]'  # pyarrow 仅用于读取旧附件包
python scripts/data/migrate_sessions.py dataset/session_2026-09-04 dataset/session_2026-09-09 \
  --replace --allow-lossy-legacy --backup-root dataset/.migration-backup-20260910
```

复用当前 `config/collection.json` 的导出设置，逐段检查 CRC、消息数及全部视频帧后替换。
原件移动到备份目录，不删除；迁移报告包含新文件 SHA-256。已完成段落会跳过。
历史附件式 MCAP 无法恢复原始 ROS 消息：保留重采样数值于 `/legacy/resampled_state`，
已有 MP4 有损转为两种 Foxglove 图像话题；时间仅能近似重建，限制写入内嵌元数据。
不带 `--replace` 则另存到 Session 的 `migrated/`；`--limit 1` 可先处理一段。
`--workers 4` 可并行处理四段（默认一段），所有工作进程结束前保持采集互斥锁。

### Session 命名、人工结果与成功数据整理

UI「采集作业台」可新建/选择命名 Session、重命名所选 Session。默认不选择时仍按日期分组。
Session 目录 ID 保持不变，显示名称存储在其 `session.json`；`--session <id>` 可供 CLI 录制使用。
每段使用独立 Episode ID，采集开始时把 Session ID/名称写入录制元数据。

作业台的「数采结果标注」在本段录制和保存结束后启用「成功 / 失败 / 清除标注」，
可通过段落下拉框补标历史数据。`review.json` 中的 `result` 分别为
`success / failure / unmarked`，独立于程序完成状态与 MCAP 校验状态；未标注显示为「成功（默认）」。
清除标注也恢复默认成功，不批量改写历史文件。异常中断/拒绝的段落不会仅因未标注就自动收录。
标注只原子更新小 JSON，不重写 MCAP；若已导出 MCAP，人工结果以外部 `review.json` 为准。

停止采集和设备后整理（使用当前配置的 `paths.output_root`，或显式传 `--output-root`）：

```bash
# 预览选中段落与缺失文件，不复制 MCAP
python scripts/data/collect_successful_mcaps.py --output dataset/successful_v1 --dry-run
# 输出必须是新目录，避免旧标注残留或覆盖文件
python scripts/data/collect_successful_mcaps.py --output dataset/successful_v1
```

脚本收录人工成功，以及未标注且状态为 `completed/validated/degraded` 的段落，复制到 `h264/` 和 `mjpeg/`，
文件名附带 Session ID，避免跨 Session 重名。`manifest.json` 保留原路径、名称、人工结果和 SHA-256。
源文件不移动；人工失败、未结束、回收站和备份不会收录。被明确关闭的格式可缺省，
仍启用但未生成的格式会报错，需先离线导出。整批复制成功后才发布输出目录；
录制/导出运行时会快速拒绝整理，整理期间也禁止改标注，避免选择结果中途变化。

启动 UI 后台、填写作业台任务/Session 并「启动设备」后，可用左手柄 **X** 开始/结束录制，
**Y** 将本次 UI 后台运行中最近一段已结束的录制整体移到 `dataset/.trash/`（原始数据和两种最终 MCAP 一起，可恢复）。
Y 不追删更早段落，重启后台后不自动选取历史数据。录制中不能删除，启动/保存中忽略按键；
长按不重复，启动/重连先松开再按，X/Y 同按不执行。A/B 原有功能不变。
手柄通过非阻塞本地 socket 交给后台线程执行，不等待 HTTP、转码或文件操作，不改变 ROS PicoFrame 格式。
浏览器修改参数会同步给后台；关闭浏览器仍使用最近同步参数，不支持脱离 UI 后台的 CLI 热键。
首次更新需重新编译原生 binding（停止设备后 `python setup.py build_ext --inplace`，使用 Teleop 环境），
再重启 UI 后台和设备、刷新页面。

JSONL 继续作为控制调试日志，不作为训练数据的主格式。

默认 K 为 `4 4 4 2 2 2 2`，D 为 `0.3 0.3 0.3 0.3 0.2 0.2 0.2`。覆盖参数使用
`--left-k/--left-d/--right-k/--right-d`，未经现场批准不要调节。
实机默认使用 `50 Hz / 20 ms`，并在进入关节阻抗前为
双臂设置调试值 `velRatio=10`、`AccRatio=10`。充分测试后才能手动提高。需要覆盖时使用
`--control-hz`、`--joint-velocity-ratio` 和 `--joint-acceleration-ratio`。
控制参数、模式和 PD 前馈设置后分别等待 `0.2 s / 1 s / 1 s` 并复核反馈。

## 离线验证

在已安装本项目的 `Teleop` 环境、项目根目录运行，不连接设备：

```bash
python -m unittest discover -s tests -v
```

完整视频集成测试还需要 ROS2、已构建的 `teleop_msgs` 和 `.[h264]` 依赖；缺少时该项会明确跳过：

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
python -m unittest discover -s tests -v
```

测试覆盖配置覆盖/快照、SDK mock、真实厂家 IK、无窗口 MuJoCo、历史附件兼容和双格式 MCAP 导出。
运行目录与保留规则见[项目结构](docs/project-structure.md#9-维护与生成文件)。

## 文档

- [首次部署（PC Service 与 PICO）](docs/首次部署.md)
- [日常操作说明](docs/操作指南.md)
- [项目结构、模块职责与命名](docs/project-structure.md)
- [架构边界与接口规范](docs/architecture-and-interface-spec.md)
- [常见报错](docs/常见报错.md)
- [Marvin MuJoCo 资产说明](assets/marvin/README.md)
