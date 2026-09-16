# Project Memory


## Decision Record: ROS topic 契约单点归属

**日期**: 2026-09-14
**问题**: ROS topic 字符串分散在发布、订阅、校验、后处理和审阅模块，修改容易遗漏。

### 选项分析

| 选项 | 优势 | 劣势 | 复杂度 |
| --- | --- | --- | --- |
| 复用 `ros/protocol.py` | 已是消息契约入口，一次检索可定位 | 配置 JSON 仍需显式保存字符串 | 低 |
| 新建 constants/geometry 模块 | 分类更细 | 增加模块和跳转，不解决更多问题 | 中 |

### 决策

**选择**: 复用 `ros/protocol.py`。
**理由**: 这是现有代码中的规范实现位置，改动最小且调用方明确。
**Trade-offs**: `protocol.py` 顶部常量增加；用户配置 JSON 保持可读、可快照，不强行代码生成。

### 影响范围

- `xr-marvin-teleop/xr_marvin_teleop/ros/protocol.py` 及 ROS、Episode、采集调用方。
- `xr-marvin-teleop/docs/project-structure.md` 和接口规范。

### 撤销条件

当协议按版本拆包，或出现独立于 ROS 的第三种传输协议时，再拆出共享数据契约模块。

## Decision Record: AV1 最终视频存储

**日期**: 2026-09-15
**问题**: 原生 MJPEG 最终文件体积较大，同时不能让视频压缩影响实时采集。

### 选项分析

| 选项 | 优势 | 劣势 | 复杂度 |
| --- | --- | --- | --- |
| 在线 AV1 编码 | 不保留大体积 MJPEG 分片 | 编码负载进入实时链路，失败时恢复困难 | 高 |
| 离线 AV1 编码 | 采集链路不变，保留源数据并可重试 | 导出需要额外时间和临时磁盘 | 中 |
| 继续默认 MJPEG | 无新增编码成本 | 最终文件体积不降 | 低 |

### 决策

**选择**: 在线保持 V4L2 原生 MJPEG，停止录制后使用 `libsvtav1` 离线生成 AV1 MCAP。
**理由**: 将高 CPU 编码移出实时路径，并通过原始 bag 和临时目录保障失败可恢复。
**Trade-offs**: 导出有编码延迟；SVT-AV1 的缓存帧先按 PTS 写入临时 SQLite，再按原消息顺序生成 MCAP。

### 影响范围

- `xr-marvin-teleop/xr_marvin_teleop/common/episode_video.py`、采集配置、后处理、迁移及收集脚本。
- `UI` 导出接口和格式选择、视频集成测试及操作文档。

### 撤销条件

真实双路相机基准表明离线耗时或临时磁盘不可接受，或部署环境无法稳定提供 `libsvtav1` 时，恢复 H.264 默认输出并保留 AV1 可选项。
