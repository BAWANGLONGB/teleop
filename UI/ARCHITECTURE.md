# 机器人数据采集平台架构

本方案不替换现有采集代码。在 `run_collection.py` 前增加一个薄 API 适配层，复用已有的设备进程、ROS 2 话题、MCAP 后处理与校验器。

```text
浏览器控制台（本目录）
  │  HTTP：配置、Episode 查询、开始/停止
  │  WebSocket：状态、诊断、采集进度
  ▼
控制 API（单进程、无设备 SDK）
  │  启动并监管现有 run_collection.py
  │  同一采集站只允许一个活动作业
  ▼
采集编排器
  ├── publish_pico.py ─────────────── /raw/pico/frame
  ├── publish_das.py × 2 ──────────── /raw/das/{side}/state|tactile|image
  ├── teleop_marvin_hardware.py ───── /raw/marvin/* + /command/*
  └── record_episode.py ───────────── /diagnostics + /episode/*
                       │
                       ▼
          State MCAP + Vision L/R MCAP
                       │
       系统时间戳 → 校验 → 离线 AV1（默认）或兼容格式 MCAP
                       │
                       ▼
dataset/session_<稳定 ID>/
  ├── session.json                 显示名称
  └── episode_HHMMSS_ID/
        ├── metadata.json
        ├── review.json            人工成功/失败/未标注
        └── final/
              ├── episode_….av1.mcap
              ├── episode_….h264.mcap
              └── episode_….mjpeg.mcap
```

`episode_HHMMSS_ID/` 是稳定目录。录制停止后只保存原始 bag。数据集页面选择 H.264 或 MJPEG 后，POST `/api/exports/mcap` 按需打包所选格式，再通过 GET 下载；已有格式直接下载，可追加生成另一格式，不覆盖现有文件。打包期间禁止新录制，设备可保持运行；失败保留原件，日志位于 Episode 的 `export.log`。

## 模块边界

| 模块 | 职责 | 明确不做 |
| --- | --- | --- |
| Web UI | 配置、状态、启停、数据检索与导出 | 不加载机器人或 ROS SDK |
| Control API | 参数校验、互斥作业、进程监管、事件转发 | 不进入实时控制回路 |
| 采集编排器 | 预检、CPU 亲和性、启动和安全退出顺序 | 不负责页面状态 |
| ROS 2 进程 | 独占设备 SDK，发布独立编号的数据流 | 不直接写业务元数据 |
| Recorder / Postprocessor | 分流写盘、系统时间戳、离线合包 | 不控制硬件 |
| Validator | 完整率、频率、延迟、时间连续性判定 | 不修改原始数据 |

## 最小 API 契约

新增 Session 和人工结果接口：`GET/POST /api/sessions`、
`POST /api/sessions/{id}/rename`、`POST /api/episodes/{id}/review`。
Session 命名与结果按钮位于采集作业台；结果请求为 `{session, result}`，
`result` 取 `success/failure/unmarked`。结果独立存于 Episode 的 `review.json`，
未标注（含没有 review.json）显示为默认成功；正常结束的未标注段参与成功数据整理，异常中断需人工确认成功。
不覆盖机器校验状态、不重写 MCAP；录制/保存中的本段不可标注。
显示名称位于 `session.json`，重命名不移动目录。UI 为本段预分配 ID，
通过 `--session/--episode-id` 传给录制器，避免将结果误标到另一段。
批量成功数据整理见项目 `scripts/data/collect_successful_mcaps.py`，输出分为 `av1/`、`h264/`、`mjpeg/` 和来源清单。

左手柄 X 切换录制、Y 将本次 UI 后台运行中最近一段已结束录制移入回收站。
现有 PICO 发布器读取左手 primaryButton/secondaryButton，通过非阻塞 Unix datagram 发往 UI 后台独立线程；
不增加 SDK 读者、不修改 ROS 消息、不在控制线程执行 HTTP/磁盘操作。按键只检测上升沿，
启动/重连先等松开，300ms 防抖，同时 X/Y 不执行；队列满丢弃，超过 500ms 或旧设备 token 的事件不执行。
后台串行复用现有录制接口，启动/保存期间丢弃按键，删除精确绑定 Session/Episode，不向前追删历史数据。
浏览器通过 `POST /api/collection/hotkeys` 同步作业台参数；后台保留最近同步参数，关闭浏览器仍可使用手柄。
`GET /api/status` 的 `hotkeys` 返回最近操作或错误。须重编译原生 XR binding 并在设备停止后重启 UI/设备。

```text
GET    /api/status                   主机、磁盘、设备、当前作业
GET    /api/devices/pico             PICO 服务端口与 TCP 客户端状态
POST   /api/devices/pico/reconnect   启动 PC Service 并等待端口就绪
GET    /api/devices/cameras/formats  左右 V4L2 能力的交集
GET    /api/devices/hardware         Marvin、双侧夹爪与双目相机实时状态
POST   /api/devices/start            启动并监管 PICO、DAS 与 Marvin
POST   /api/devices/stop             停止设备；活动录制存在时拒绝
GET    /api/preview/{left|right}.jpg  采集中最近一帧原生 JPEG
GET    /api/episodes?status=&q=      从 MCAP 内嵌 meta/meta.json 建立列表
GET    /api/exports/mcap?episode=... 流式导出单个 Episode 的 MCAP 文件
GET    /api/episodes/{id}            内嵌 metadata + validation
POST   /api/episodes                 校验参数并启动采集
POST   /api/episodes/active/stop     SIGINT，触发现有安全收尾
POST   /api/robot/reset              仅在数采之外独立复位双臂至初始位
POST   /api/episodes/{id}/open       通过系统文件管理器打开 MCAP 所在目录
DELETE /api/episodes/{id}            原子移动至 dataset/.trash/
```

下载请求按 `format=av1|h264|mjpeg` 流式返回 `final/<episode_id>.<format>.mcap`，默认 AV1。浏览器先 POST `{episode, format}` 等待打包完成，再 GET 下载；多段依次处理，不生成整批临时副本。打包使用既有后处理脚本、录制快照和独占录制锁，低优先级运行；设备模式不占用该锁。

`POST /api/episodes` 接受 `run_collection.py` 已存在的参数：`task`、`operator`、`robot_model`、`max_duration`、`no_vision`、`nsp_lateral` 和标定文件引用。DAS SDK 使用 `camera_resolutions` 和 `camera_fps`：帧率固定设置为 60 FPS，实际采集帧率约 30 FPS；当前 TeleOp 配置为 `1600x1296@60`。UI 保留两种已知分辨率，生产 API 再通过 `v4l2-ctl --list-formats-ext` 校验左右相机均支持 60 FPS。

机器人复位仅在数采和遥操调试进程均未运行时启动一次性 `reset_marvin_hardware.py`，由它独占 Marvin SDK，以默认 3 秒余弦轨迹回到 `MARVIN_INITIAL_POSE_Q_RAD`，确认到位后释放连接并退出。

API 将选择的 `camera_resolution` 与 `camera_fps` 写入本次作业专用 DAS 配置快照，再交给现有编排器，不修改全局标定文件。“启动设备”和“开始录制”分别调用编排器的 `devices` 与 `recording` 模式；设备 ready-file 出现前禁止开始录制。

实时预览不重复打开相机：`capture_das_mjpeg.py` 写入原始帧后，以最高 30 FPS 原子更新 `/dev/shm/fieldnote-preview-<uid>/left.jpg|right.jpg`。共享内存或浏览器异常时只禁用预览，不影响采集；HTTP 服务只读快照，采集停止后清理。

Robotics Service 就绪以 TCP `63901` 和 `60061` 均处于监听状态为准；PICO 在线还要求 `63901` 存在来自非本机地址的 `ESTABLISHED` TCP 会话，且该客户端 Ping 可达，以排除设备掉线后残留的 TCP 会话。页面每 5 秒刷新。连接按钮可执行 `bash /opt/apps/roboticsservice/runService.sh` 并等待服务端口就绪，但 UI 不启动、停止或探测 `publish_pico.py` 和 ROS 话题。

机械臂、夹爪和相机状态同样不访问 ROS：机械臂仅被动读取内核 UDP socket，检查是否存在指向 `192.168.1.190` 的端口；夹爪仅检查 `/dev/ttyFingerLeft`、`/dev/ttyFingerRight`，相机仅检查 `/dev/finger_camera_left`、`/dev/finger_camera_right`。这些状态只表示端口或设备节点存在，不代表数据内容有效。

端口或设备节点异常会打印到 `server.py` 所在终端；同一错误只在首次出现或状态变化后再次出现时打印。API 请求失败同时写入浏览器开发者工具 Console。

删除接口先校验 Episode ID 格式、解析后的路径仍在 `dataset/` 下、目标不是活动 Episode，再使用同文件系统原子重命名移入 `dataset/.trash/<episode>_<deleted_at>.mcap`。接口不接受任意文件路径；永久清理由独立保留期任务完成。

## 状态机

```text
IDLE → STARTING_DEVICES → DEVICES_READY → RECORDING → FINALIZING → DEVICES_READY → IDLE
                    └── timeout ───────────────→ ABORTED
任意活动状态 ── stop/error ───────────────────→ FINALIZING → ABORTED/REJECTED
```

仅允许一个活动 Episode。停止操作向编排器发送 `SIGINT`，继续使用现有的 Marvin → DAS → Recorder → PICO 收尾顺序；浏览器断开不停止采集。

## 数据与安全原则

- 最终 MCAP 是系统间的稳定契约；列表读取其首个 `meta/meta.json` attachment，规模达到数千段并出现延迟后再引入索引。
- 最终 MCAP 不可变；原始校验结果和标定快照已经内嵌在 meta 中。
- UI 的删除操作只移入 `.trash`；默认保留 7 天，避免误删导致不可恢复的数据损失。
- 设备 SDK 仍由独立进程持有，相机编码和 UI 推流不得进入机械臂控制进程。
- UI 停止按钮不是急停；异常运动始终使用物理急停，现场安全检查独立于网页交互。
- 生产部署仅绑定实验室网段，使用反向代理完成登录、TLS 与操作审计；采集进程以非 root 用户运行。

## 已实现范围

当前脚本已接入状态、数据列表、回收站删除、PICO 探测/重连、相机能力、单作业采集启停、Session 命名、人工结果标注与按需 AV1/H.264/MJPEG MCAP 打包下载。页面内历史视频播放仍未接入；没有跨采集站调度需求前不引入消息队列、微服务或数据库。

## 运行

```bash
source /home/zxcx/TeleOp/.miniconda-xr/etc/profile.d/conda.sh
conda activate Teleop
source /home/zxcx/TeleOp/xr-marvin-teleop/ros2_ws/install/setup.bash
unset LD_PRELOAD
cd /home/zxcx/TeleOp/UI
python server.py
```

浏览器访问 `http://localhost:4173`。`server.py` 只监听本机，负责静态页面、数据集读取与回收站删除、PICO 探测/重连、相机能力查询，以及现有 `run_collection.py` 的启停。按 `Ctrl+C` 退出服务时，活动采集会先收到安全停止信号。

最小自检不会连接或删除真实硬件数据：

```bash
python server.py --self-test
node test.mjs
```
