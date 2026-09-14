## 修复：遥操失联回零、UI 请求来源、导出与停止互锁

- 控制器失联时取消回零，恢复后等待新的 B 按键；UI 写请求验证 Origin、Host 和 JSON 类型，复位需显式现场确认；独立导出锁保留录制/删除互斥并释放设备停止路径。
- 修改 UI/server.py、UI/app.js、控制器及对应 Python/浏览器测试；保留会话开始前的未提交改动。
- 验证：ROS2 环境 unittest 66 项，65 通过，1 因缺少 teleop_msgs 跳过；HTTP 专项 3 项通过；UI 自检、Chrome 交互、git diff --check 通过。未连接实机。
- 已知限制：旧消息迁移和真实机械响应未验证。未提交 Git。

## [17:45] 动作: 收口 ROS topic 契约与公开几何 helper
- 文件: xr-marvin-teleop/xr_marvin_teleop/ros/protocol.py 及全部生产调用方；episode_postprocessor.py、xr_target_mapper.py。
- 决策: 复用现有 protocol.py，不新增 constants/geometry 模块。
- 验证: compileall 通过；16 tests OK，2 ROS2 环境跳过；生产代码旧 topic 字面量仅剩 protocol.py。

## [17:49] 动作: 完成结构与可读性重构
- 文件: 控制器计算拆分；Marvin 测试按职责拆为 5 个文件；结构与安全边界文档同步。
- 决策: 保持控制器公共构造接口，不引入参数对象、ABC 或新依赖；先完成高优先级批次验证。
- 验证: 67 tests，63 通过、4 个环境依赖跳过、0 失败；29 个迁移测试方法 AST 完全一致；compileall/diff-check/引用残留扫描通过。

## [17:55] 动作: 拆分采集与后处理长流程
- 文件: scripts/data/run_collection.py、scripts/data/record_episode.py、common/episode_postprocessor.py。
- 决策: 仅提取同文件阶段函数，保留信号、异常清理和写入事务边界，不新增抽象层。
- 验证: 主流程分别缩至 14、208、184 行；全量 67 tests，63 通过、4 个可选依赖跳过、0 失败；compileall 与 diff-check 通过。
