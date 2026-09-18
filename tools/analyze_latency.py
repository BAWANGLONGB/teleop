#!/usr/bin/env python3
"""Analyze PICO timing and Marvin control JSONL logs using only the stdlib."""

import argparse
from collections import Counter
import heapq
import json
import math
from pathlib import Path


STAGES = {
    'sdk.callback_gap_ns': 'SDK 回调到达间隔（可能漏掉被缓存覆盖的大间隔）',
    'sdk.parse_ns': 'native JSON 解析',
    'sdk.lock_wait_ns': 'native 缓存锁等待',
    'sdk_cache.age_ns': 'SDK 缓存帧龄（下游症状）',
    'poll.read_ns': 'Python 读取快照',
    'poll.late_ns': 'Python 轮询调度迟到',
    'source.new_frame_gap_ns': '新帧空窗',
    'publish.queue_ns': '发布线程队列等待',
    'publish.duration_ns': 'ROS 发布调用',
    'join.duration_ns': 'ROS 多 topic 拼帧',
    'consume.arrival_age_ns': '消费时帧龄（下游症状）',
    'consume.join_to_read_ns': '拼帧后等待消费（含旧帧重复读取）',
    'control.gap_ns': '控制采样间隔（非单次执行耗时）',
}


def number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def analyze(path, threshold_ms=50.0, top=10):
    metrics, reasons, suspects = {}, Counter(), Counter()
    worst, warnings = [], Counter()
    summaries = cycles = dropped = 0
    previous_cycle = previous_sdk = None
    bursts = 0

    def metric(name, maximum, count, line, p95=None):
        if not number(maximum) or maximum < 0 or not number(count) or count <= 0:
            raise ValueError(f'invalid metric {name}')
        item = metrics.setdefault(name, dict(samples=0, windows=0, slow_windows=0,
                                            max_ms=0.0, worst_line=line, max_window_p95_ms=None))
        item['samples'] += count
        item['windows'] += 1
        item['slow_windows'] += maximum >= threshold_ms
        if maximum >= item['max_ms']:
            item.update(max_ms=maximum, worst_line=line)
        if p95 is not None:
            if not number(p95) or not 0 <= p95 <= maximum:
                raise ValueError(f'invalid p95 {name}')
            item['max_window_p95_ms'] = max(item['max_window_p95_ms'] or 0, p95)

    def evidence(value, line, label):
        heapq.heappush(worst, (value, line, label))
        if len(worst) > top:
            heapq.heappop(worst)

    with Path(path).open(encoding='utf-8') as source:
        for line, raw in enumerate(source, 1):
            if not raw.strip():
                continue
            try:
                record = json.loads(raw)
                if not isinstance(record, dict):
                    raise ValueError('record must be an object')
            except ValueError:
                warnings['无效 JSON 行（已跳过）'] += 1
                continue
            try:
                event = record.get('event')
                if event == 'summary':
                    values = record['milliseconds']
                    if not isinstance(values, dict):
                        raise ValueError('milliseconds must be an object')
                    maxima = {}
                    for name, stats in values.items():
                        metric(name, stats['max'], stats['count'], line, stats.get('p95'))
                        maxima[name] = stats['max']
                    summaries += 1
                    reasons.update(record.get('reasons', {}))
                    dropped = max(dropped, record.get('dropped_records', 0))
                    slow = {key for key, value in maxima.items() if value >= threshold_ms}
                    direct = slow & {'sdk.parse_ns', 'sdk.lock_wait_ns', 'poll.read_ns',
                                     'poll.late_ns', 'publish.queue_ns', 'publish.duration_ns',
                                     'join.duration_ns'}
                    for name in direct:
                        suspects[STAGES[name]] += 1
                    if 'source.new_frame_gap_ns' in slow or 'sdk_cache.age_ns' in slow:
                        local = {'sdk.parse_ns', 'sdk.lock_wait_ns', 'poll.read_ns', 'poll.late_ns'}
                        if local <= maxima.keys() and not local & slow:
                            suspects['SDK 上游供帧/到达不连续（PICO、网络、PC Service、gRPC 尚无法细分）'] += 1
                        else:
                            suspects['新帧空窗：本地耗时异常或打点不足，无法单独归因上游'] += 1
                    for name in slow:
                        evidence(maxima[name], line, STAGES.get(name, name))
                    sdk = (record.get('latest_identity', {}).get('sdk') or {})
                    if sdk and previous_sdk:
                        dt = record['monotonic_ns'] - previous_sdk[0]
                        ds = sdk['source_timestamp_ns'] - previous_sdk[1]['source_timestamp_ns']
                        dc = sdk['sdk_callback_sequence'] - previous_sdk[1]['sdk_callback_sequence']
                        # ponytail: summary-level heuristic; raw per-frame traces needed for exact backlog.
                        if dt > 0 and ds > dt * 1.5 and dc > 1:
                            bursts += 1
                            evidence(ds / 1e6, line, '疑似积压追赶：源时间推进超过本地窗口 1.5 倍')
                    if sdk:
                        previous_sdk = (record['monotonic_ns'], sdk)
                elif event == 'anomaly':
                    # Anomalies duplicate summary samples: use for locations, never add to totals.
                    durations = record.get('durations_ns', {})
                    maximum = max((v / 1e6 for v in durations.values() if number(v) and v >= 0), default=0)
                    if maximum >= threshold_ms or record.get('reason') not in (None, 'valid'):
                        evidence(maximum, line, f"异常 {record.get('stage')}: {record.get('reason') or '慢调用/空窗'}")
                elif event == 'control_cycle':
                    now = record['monotonic_time_ns']
                    if not number(now) or now < 0:
                        raise ValueError('invalid control timestamp')
                    cycles += 1
                    if previous_cycle is not None:
                        gap = (now - previous_cycle) / 1e6
                        if gap < 0:
                            warnings['控制时间倒退（分段处理）'] += 1
                        else:
                            metric('control.gap_ns', gap, 1, line)
                            if gap >= threshold_ms:
                                suspects['控制采样间隔超阈值：需补 IK、机器人读写等阶段耗时才能细分'] += 1
                                evidence(gap, line, STAGES['control.gap_ns'])
                    previous_cycle = now
                    if record.get('xr_frame_valid') is False:
                        reasons['control:xr_invalid'] += 1
                else:
                    warnings['不支持的事件（已跳过）'] += 1
            except (KeyError, TypeError, ValueError, AttributeError, OverflowError):
                warnings['记录字段不完整或无效（部分字段可能已计入）'] += 1
    return dict(path=str(path), summaries=summaries, control_cycles=cycles,
                threshold_ms=threshold_ms, metrics=metrics, reasons=dict(reasons),
                suspects=dict(suspects), catchup_windows=bursts, dropped_records=dropped,
                warnings=dict(warnings), worst=[dict(value_ms=v, line=n, detail=s)
                                               for v, n, s in sorted(worst, reverse=True)])


def render(report):
    print(f"\n文件: {report['path']}")
    print(f"汇总窗口 {report['summaries']} | 控制样本 {report['control_cycles']} | "
          f"日志丢弃计数 {report['dropped_records']} | 疑似追赶窗口 {report['catchup_windows']}")
    print('指标                                          样本数  慢窗口/总窗口   最大ms  最慢窗口P95ms')
    for name, item in sorted(report['metrics'].items(), key=lambda pair: -pair[1]['max_ms']):
        p95 = item['max_window_p95_ms']
        print(f"{name:44} {item['samples']:7} {item['slow_windows']:5}/{item['windows']:<7} "
              f"{item['max_ms']:9.3f} {p95 if p95 is not None else '-'}")
    print('来源线索（按窗口计数；多个阶段可能同时出现，不能相加为总卡顿数）:')
    for label, count in sorted(report['suspects'].items(), key=lambda pair: -pair[1]):
        print(f'  {count}: {label}')
    if not report['suspects']:
        print('  没有足够证据定位来源；不代表全链路无延迟。')
    if report['reasons']:
        print('事件计数:', json.dumps(report['reasons'], ensure_ascii=False))
    for label, count in report['warnings'].items():
        print(f'注意: {label}: {count}')
    for item in report['worst']:
        print(f"  {report['path']}:{item['line']}  {item['value_ms']:.3f} ms  {item['detail']}")


def main(argv=None):
    parser = argparse.ArgumentParser(description='分析 PICO timing / Marvin control JSONL 的延迟来源')
    parser.add_argument('paths', nargs='+', type=Path, help='JSONL 文件或日志目录（目录仅扫描当前层 *.jsonl）')
    parser.add_argument('--threshold-ms', type=float, default=50, help='慢窗口阈值，默认 50 ms')
    parser.add_argument('--top', type=int, default=10, help='每文件最多显示多少条定位证据')
    parser.add_argument('--json', action='store_true', help='输出 JSON 报告')
    args = parser.parse_args(argv)
    if not math.isfinite(args.threshold_ms) or args.threshold_ms <= 0 or args.top < 1:
        parser.error('threshold-ms 必须是有限正数，top 必须 >= 1')
    files = sorted({file for path in args.paths for file in
                    (path.expanduser().glob('*.jsonl') if path.expanduser().is_dir() else [path.expanduser()])})
    if not files:
        parser.error('没有找到 JSONL 日志')
    try:
        reports = [analyze(path, args.threshold_ms, args.top) for path in files]
    except (OSError, UnicodeError) as error:
        parser.exit(2, f'无法读取日志: {error}\n')
    if args.json:
        print(json.dumps(reports, ensure_ascii=False, indent=2, allow_nan=False))
    else:
        print('阈值用于筛选线索；P95 为各窗口 P95 的最大值，不是全程 P95。')
        print('不跨时钟域计算端到端延迟；各文件独立分析。异常日志限流，慢窗口占比不等于卡顿时长占比。')
        for report in reports:
            render(report)
    return 0 if any(r['metrics'] for r in reports) else 1


if __name__ == '__main__':
    raise SystemExit(main())
