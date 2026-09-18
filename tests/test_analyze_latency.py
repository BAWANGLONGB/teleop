"""Offline latency analysis needs neither ROS nor robot hardware."""
import contextlib
import io
import json
from pathlib import Path
import tempfile
import unittest

from tools.analyze_latency import analyze, main


class LatencyAnalysisTest(unittest.TestCase):
    def test_attribution_counts_units_and_bad_lines(self):
        def summary(sequence, stamp, now, gap, queue=1):
            maxima = {'source.new_frame_gap_ns': gap, 'sdk_cache.age_ns': gap,
                      'poll.read_ns': 1, 'poll.late_ns': 1, 'sdk.parse_ns': 1,
                      'sdk.lock_wait_ns': 1, 'publish.queue_ns': queue}
            return dict(event='summary', monotonic_ns=now,
                        milliseconds={key: dict(count=10, max=value, p95=value / 2)
                                      for key, value in maxima.items()},
                        latest_identity={'sdk': dict(source_timestamp_ns=stamp,
                                                    sdk_callback_sequence=sequence)},
                        reasons={'consume:hold': 1}, dropped_records=3)
        records = [summary(1, 10**9, 10**9, 250),
                   dict(event='anomaly', stage='source', durations_ns={'gap_ns': 250_000_000}),
                   summary(200, 3 * 10**9, 2 * 10**9, 10, queue=80),
                   dict(event='control_cycle', monotonic_time_ns=1_000_000_000),
                   dict(event='control_cycle', monotonic_time_ns=1_100_000_000),
                   dict(event='control_cycle', monotonic_time_ns=1, xr_frame_valid=False)]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'test.jsonl'
            path.write_text('\n'.join(map(json.dumps, records)) + '\n{truncated\n[]\n', encoding='utf-8')
            result = analyze(path, top=3)
            metric = result['metrics']['source.new_frame_gap_ns']
            self.assertEqual(metric['samples'], 20)  # anomaly must not double-count
            self.assertEqual(metric['slow_windows'], 1)
            self.assertEqual(metric['max_ms'], 250)
            self.assertEqual(metric['max_window_p95_ms'], 125)
            self.assertEqual(result['metrics']['control.gap_ns']['max_ms'], 100)
            self.assertEqual(result['catchup_windows'], 1)
            self.assertEqual(result['dropped_records'], 3)  # cumulative, not sum
            self.assertEqual(result['reasons']['consume:hold'], 2)
            self.assertEqual(result['suspects']['发布线程队列等待'], 1)
            self.assertTrue(any('SDK 上游' in key for key in result['suspects']))
            self.assertEqual(result['warnings']['无效 JSON 行（已跳过）'], 2)
            self.assertEqual(len(result['worst']), 3)
            with contextlib.redirect_stdout(io.StringIO()) as output:
                self.assertEqual(main([str(path), '--json']), 0)
            self.assertEqual(json.loads(output.getvalue())[0]['summaries'], 2)
            path.write_text('', encoding='utf-8')
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(main([str(path)]), 1)


if __name__ == '__main__':
    unittest.main()
