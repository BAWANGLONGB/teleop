# ROS 2 / Foxglove 消息协议 v2

新采集使用 ROS 2 标准接口与 `foxglove_msgs/Grid`，不需要构建或 source `teleop_msgs`。
Episode 的 `message_protocol_version=2`；配置文件自身的 `schema_version` 仍为 1。

```bash
sudo apt-get install ros-humble-foxglove-msgs ros-humble-rosbag2-storage-mcap
source /opt/ros/humble/setup.bash
python -m pip install -e '.[h264]'
```

## 数据契约

| 话题 | 类型 | 内容 |
| --- | --- | --- |
| `/raw/pico/poses` | `geometry_msgs/msg/PoseArray` | 两个位姿，顺序 left/right，frame=`openxr_local`，xyz+xyzw |
| `/raw/pico/joy` | `sensor_msgs/msg/Joy` | axes: left/right grip、left/right trigger、left/right thumbstick_y；buttons: A/B/X/Y |
| `/raw/marvin/joint_state` | `sensor_msgs/msg/JointState` | `Joint1_L…Joint7_L, Joint1_R…Joint7_R`，rad、rad/s，无力矩时 effort 为空 |
| `/command/marvin/joint_target` | `trajectory_msgs/msg/JointTrajectory` | 相同关节名，单个位置目标，rad |
| `/raw/das/{side}/state` | `sensor_msgs/msg/JointState` | 一个逻辑开合自由度 `{side}_gripper_width`，单位 m |
| `/command/das/target` | `trajectory_msgs/msg/JointTrajectory` | left/right_gripper_width 的归一化开启度：0 闭合、1 开启 |
| `/raw/das/{side}/tactile` | `foxglove_msgs/msg/Grid` | 448 原始字节，两行各 224 个采样索引 |
| `/raw/das/{side}/image` | `sensor_msgs/msg/Image` | 可选原始像素发布，标准 encoding、step、data |
| `/raw/das/{side}/image/compressed` | `sensor_msgs/msg/CompressedImage` | 相机原生 JPEG，直接写 bag，不经过 DDS |
| `/raw/marvin/{side}/tcp_pose` | `geometry_msgs/msg/PoseStamped` | 关节反馈 FK，world 系，xyz+xyzw |
| `/command/marvin/{side}/tcp_target` | `geometry_msgs/msg/PoseStamped` | 实际下发关节目标 FK，world 系，xyz+xyzw |
| `/episode/state`、`/episode/event` | `std_msgs/msg/String` | 带版本与系统时间的 Episode JSON |

单点轨迹的 `header.stamp` 是下发参考时间，`time_from_start=0`。SDK 网关只接受立即位置目标，
拒绝多点、延时、速度/加速度/力矩命令；没有新增轨迹插值器。Marvin 指令话题记录 SDK 下发的目标，
不表示机器人已到位。夹爪开合自由度是逻辑坐标，不冒充 URDF 中单指的位移。

## 采样诊断与控制配对

每个数据话题对应 `<topic>/status`，类型为 `diagnostic_msgs/msg/DiagnosticArray`；
PICO poses 和 joy 共用 `/raw/pico/status`。只有一个名为 `teleop_sample` 的 DiagnosticStatus，
KeyValue 的 value 使用 JSON 编码，包含：

- `protocol_version=2`、`topics`、`publisher_session_id`、`sequence_id`、`valid`；
- `source_timestamp_ns` 和 `source_clock`，无设备时钟时为 0 / unavailable；相机 PTS 标为 gstreamer_pts；
- `receive_steady_ns`，命令使用 `issue_steady_ns`；
- Marvin 的 frame_serial/arm_state/error_code/low_speed，DAS 的 target_distance_m/status_flags。

业务消息与诊断使用完全相同的采样时间。PICO 在非阻塞、有界 32 帧缓存内精确配对三个话题；
DAS 反馈与命令也必须配到有效的诊断。无效诊断立即使对应输入失效，迟到/重复/过期样本不刷新控制状态。
发布者重新启动使用新的 session；拒绝已退休 session 的迟到帧。控制的新鲜度同时检查采样系统时间
和接收进程本地的单调时钟，跨机器运行要求系统时钟同步。

当前硬件采集协议使用系统时间（CLOCK_REALTIME）。设备源时间和 PTS 不与系统时间混算。
原始 bag 时间是接收/写入时间，后处理使用 header.stamp 或 Grid.timestamp；仅 `/diagnostics`
健康汇总使用 bag 时间。通用 ROS 工具可以回放标准消息；历史回放不会被硬件输入端当成新鲜指令。

校验通过磁盘临时索引逐时间戳匹配诊断与数据，报告元数据缺失、孤立诊断、序号缺口、源时间回退、
无法解码与无效样本计数。没有采样诊断时 sequence_check 为 unavailable，不声称零丢帧。

## 触觉与坐标系

Grid 的 column_count=224、row_stride=224、cell_stride=1，field=`raw_value: UINT8`。
data 是原始 448 字节，完整保留二进制值，不在此层解释有符号压力。每个夹爪的两行是两个触觉面，
不是左右机器人手臂。frame=`das_{side}_tactile_index`、cell_size=(1,1) 仅表示索引展示间距，
不接入机器人米制 TF。SDK 的 50×10 每面展示经过重复/补点，不能作为独立传感点记录。

相机使用 `das_{side}_camera_optical_frame`（右、下、前）；没有实测外参时不生成虚假的 TF。
TCP 使用 URDF 的 world 系，不将 world 与 base_link 当成同一坐标系。

## 打包与历史迁移

原始 bag 和后处理 bag 均为 ROS 2 CDR，内含标准类型及 Foxglove Grid。
最终 `.mjpeg.mcap` 保留 `sensor_msgs/msg/CompressedImage`；`.h264.mcap` 中视频为
`foxglove.CompressedVideo` Protobuf，话题 `/raw/das/{side}/video/compressed`，对应采样诊断也重命名。
H.264 沿用 Annex B、无 B 帧、每个消息一帧、IDR 带 SPS/PPS，时间戳逐帧保持不变。
最终 MCAP 内嵌 schema、配置和标定，不声明纯 ros2 profile。

旧消息定义只为离线迁移保留在 `ros2_ws/src/teleop_msgs`，运行时不依赖它。迁移到一个新目录：

```bash
# 仅迁移终端需要旧 teleop_msgs 的生成类型
source ros2_ws/install/setup.bash
python scripts/data/migrate_messages_v2.py /path/to/old_episode /path/to/new_episode \
  --das-config /path/to/original/das_gripper.json
python scripts/data/postprocess_episode.py /path/to/new_episode
```

迁移保留原文件、采样时间、序号、诊断和配置快照，不覆盖已有目录。
旧 PICO 没有 X/Y 数据，迁移标记 unavailable；旧闭合度必须使用录制时的夹爪标定转换，不能猜。
旧已生成的 data/final 不拷贝，v2 从迁移后的原始 bag 重新生成 FK 和视频。

## 验证

### PICO 卡顿诊断

设备启动后自动生成项目 `logs/pico_timing_source_<pid>_<time>.jsonl` 和
`logs/pico_timing_receiver_<pid>_<time>.jsonl`，启动日志会显示完整路径。
不新增 ROS 话题，不改变消息内容、200 ms 输入有效期、控制频率或视频打包方式。

| 阶段 | 主要指标 | 用途 |
|---|---|---|
| `sdk` | `callback_gap_ns`、`parse_ns`、`lock_wait_ns` | SDK 回调间隔、解析及缓存锁等待 |
| `sdk_cache` | `age_ns` | Python 读取时 SDK 缓存有多旧 |
| `poll` | `late_ns`、`read_ns` | 轮询调度迟到与读取开销 |
| `source` | `new_frame_gap_ns` | 源时间戳更新的实际间隔 |
| `publish` | `queue_ns`、`duration_ns` | 入队到出队、三路消息构造及发布总耗时 |
| `callback_poses/joy/status` | `duration_ns` | 各订阅回调执行耗时；异常原因及过期/淘汰累计数 |
| `join` | `duration_ns` | 首路回调到配对、校验完成 |
| `consume` | `arrival_age_ns`、`join_to_read_ns` | 控制取帧时数据年龄；`hold/valid` 状态切换 |

每秒一条 `summary`：`milliseconds` 下的 P50/P95/P99/max **单位为毫秒**，
`latest_identity` 保留各阶段最后样本的标识。任一耗时超过 50 ms 或出现异常事件时，
额外输出 `anomaly`，每阶段最多一秒一条，其 `durations_ns` 单位为纳秒。
生产线程只向 1024 条有界队列非阻塞写入；队列满时计入 `dropped_records`，不能据此声称诊断完整。
文件写入失败只停用诊断，不中断控制。

`sdk` 是 Python 轮询所观察到的最新成功 SDK 回调，不是所有回调的完整逐帧追踪。
`sdk_callback_sequence` 可识别中间被覆盖的回调；解析失败另见现有 SDK 错误日志。
所有本机耗时使用 CLOCK_MONOTONIC，源时间戳仅用于关联，不能与主机时钟直接相减。
`publish` 和接收端通过发布会话 ID、序号、消息 stamp 关联；SDK 与发布端通过源时间戳关联。
只有同一主机、同一次启动的日志才能直接相减单调时钟。该诊断不能单独区分网络与 PC 服务停顿。

原生模块更新需重新编译；源码部署时在停止设备后执行：

```bash
conda activate Teleop
python setup.py build_ext --inplace
```

若出现 `native_timing_unavailable_rebuild_extension`，表示加载的原生模块仍是旧版。

### 离线与隔离检查

```bash
source /opt/ros/humble/setup.bash
python -m unittest discover -s tests -v
# 实际 DDS 往返检查（隔离本机 domain，不连接设备）
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=191 python tests/ros_v2_live_check.py
```

测试不连接硬件。覆盖序列化往返、关节顺序、四元数奇异位置、触觉字节完整性、PICO 配对、
夹爪标定、原始 bag→FK→双 MCAP、H.264 解码与失败恢复。
