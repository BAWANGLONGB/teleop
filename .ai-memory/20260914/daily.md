## 修复：遥操失联回零、UI 请求来源、导出与停止互锁

- 控制器失联时取消回零，恢复后等待新的 B 按键；UI 写请求验证 Origin、Host 和 JSON 类型，复位需显式现场确认；独立导出锁保留录制/删除互斥并释放设备停止路径。
- 修改 UI/server.py、UI/app.js、控制器及对应 Python/浏览器测试；保留会话开始前的未提交改动。
- 验证：ROS2 环境 unittest 66 项，65 通过，1 因缺少 teleop_msgs 跳过；HTTP 专项 3 项通过；UI 自检、Chrome 交互、git diff --check 通过。未连接实机。
- 已知限制：旧消息迁移和真实机械响应未验证。未提交 Git。
