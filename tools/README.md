# 延迟与卡顿日志分析

无需安装依赖、ROS 或连接机器人，在仓库根目录运行：

```bash
# 同一次运行的源端和接收端日志；可追加 marvin_hardware_*.jsonl
python3 tools/analyze_latency.py var/logs/pico_timing_source_具体文件.jsonl var/logs/pico_timing_receiver_具体文件.jsonl

# 扫描目录内所有 JSONL，每个文件独立报告；只关注 >=200 ms 的异常
python3 tools/analyze_latency.py var/logs --threshold-ms 200 --top 10

# 保存机器可读报告
python3 tools/analyze_latency.py var/logs --json > /tmp/latency-report.json

python3 -m unittest tests.test_analyze_latency -v
```

报告包含各阶段样本数、超阈值窗口数、最大耗时、最慢窗口 P95、疑似来源、事件计数和可定位的文件行号。阈值默认 50 ms，可按实际控制周期调整。目录只扫描当前层，不读取 MCAP 或普通文本日志。

- `source.new_frame_gap_ns` / `sdk_cache.age_ns` 大，而同窗口解析、锁等待和轮询耗时小：优先检查 SDK 上游供帧、网络、PC Service 和 gRPC；这些日志不能进一步确认其中哪个环节阻塞。
- `publish.queue_ns` / `publish.duration_ns` 大：发布线程排队或 ROS 发布调用慢；`join.duration_ns` 大：接收端等齐多个 topic 慢。
- `consume.*` 是等待或旧帧帧龄，可能是上游中断的后果，不能直接归因控制器。`consume:hold` 是状态切换次数，不是 hold 帧数或持续时间。
- 源时间在相邻汇总窗口推进超过本地时间的 1.5 倍时，标记疑似积压追赶；时间跳变也可能触发，需要原始逐帧数据确认。
- Marvin 日志只能给出控制采样间隔；没有 IK、机器人读写耗时打点，不能凭它细分根因。

P95 显示的是各窗口 P95 的最大值，不能从现有汇总恢复全程 P95。慢窗口比例不是卡顿时长比例。异常行经过限流且与汇总重复，因此只作为定位证据，不累加样本。`dropped_records` 是日志器累计丢弃计数（含指标样本溢出），不代表网络丢包；非零时统计不完整。SDK latest-only 缓存也可能覆盖大回调间隔，所以回调间隔正常不能排除上游卡顿。

各文件独立分析，不将 PICO 源时间戳和 PC 时钟相减，不推算跨主机端到端延迟。未发现慢窗口不等于全链路无延迟。无有效指标时返回退出码 1，参数或文件读取错误返回 2；坏 JSON 行会报告并跳过，便于读取仍在写入的日志。
