# 工作日志 - 2026-09-15
session-id: 20260915-1047

## [10:47] 动作: 将最终视频存储切换为 AV1
- 文件: `xr-marvin-teleop` 的视频导出、配置、迁移、收集脚本、测试和文档，以及 `UI` 导出入口。
- 决策: 在线继续原生 MJPEG 采集；离线用 `libsvtav1` 写 `foxglove.CompressedVideo`，默认仅生成 AV1；编码器缓存帧以 PTS 暂存 SQLite 后按原时间顺序写 MCAP；旧配置缺少 AV1 字段时保持禁用。
- 验证: Teleop 环境全量 68 tests 通过、1 个旧消息依赖测试跳过；真实 SVT-AV1 编码与 dav1d 解码通过；Chrome UI 交互、自检、compileall 和 diff-check 通过。未连接真实相机或机械臂。
