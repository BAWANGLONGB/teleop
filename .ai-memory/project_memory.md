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
