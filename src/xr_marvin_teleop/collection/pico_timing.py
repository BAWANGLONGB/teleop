"""Non-blocking, bounded PICO timing diagnostics; no ROS or hardware required."""

from collections import Counter, defaultdict
import json
import math
import os
from pathlib import Path
import queue
import threading
import time


class PicoTimingLog:
    def __init__(self, name, directory=None):
        directory = (
            Path(directory)
            if directory is not None
            else Path(__file__).resolve().parents[3] / "var" / "logs"
        )
        self.path = directory / f"pico_timing_{name}_{os.getpid()}_{time.time_ns()}.jsonl"
        self._queue = queue.Queue(maxsize=1024)
        self._stop = threading.Event()
        self.dropped = 0
        self.error = None
        self._thread = threading.Thread(target=self._write, name=f"timing-{name}", daemon=True)
        self._thread.start()
        print(f"PICO timing log: {self.path}", flush=True)

    def record(self, stage, *, identity=None, event=None, **durations_ns):
        if self._stop.is_set() or self.error is not None:
            return
        try:
            self._queue.put_nowait((time.monotonic_ns(), stage, identity, event, durations_ns))
        except queue.Full:
            self.dropped += 1

    def _write(self):
        counts, metrics, events = Counter(), defaultdict(list), Counter()
        identities = {}
        last_anomaly = {}
        deadline = time.monotonic() + 1.0
        try:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            with self.path.open("x", encoding="utf-8") as output:
                def emit(record):
                    output.write(json.dumps(record, separators=(",", ":"), allow_nan=False) + "\n")

                while not self._stop.is_set() or not self._queue.empty():
                    try:
                        now, stage, identity, event, values = self._queue.get(timeout=0.05)
                    except queue.Empty:
                        pass
                    else:
                        counts[stage] += 1
                        identities[stage] = identity
                        if event:
                            events[f"{stage}:{event}"] += 1
                        for name, value in values.items():
                            # ponytail: bounded window; count overflow instead of growing during overload.
                            samples = metrics[f"{stage}.{name}"]
                            if len(samples) < 4096:
                                samples.append(value / 1e6)
                            else:
                                self.dropped += 1
                        if (event or any(v >= 50_000_000 for v in values.values())) and now - last_anomaly.get(stage, 0) >= 1_000_000_000:
                            emit(dict(event="anomaly", stage=stage, monotonic_ns=now,
                                      reason=event, identity=identity, durations_ns=values))
                            last_anomaly[stage] = now
                    if time.monotonic() >= deadline or (self._stop.is_set() and self._queue.empty()):
                        summary = {}
                        for name, samples in metrics.items():
                            samples.sort()
                            summary[name] = dict(count=len(samples), **{
                                label: samples[max(0, math.ceil(len(samples) * p) - 1)]
                                for label, p in (("p50", .5), ("p95", .95), ("p99", .99), ("max", 1.))})
                        emit(dict(event="summary", wall_time_ns=time.time_ns(), monotonic_ns=time.monotonic_ns(),
                                  counts=dict(counts), reasons=dict(events), milliseconds=summary,
                                  latest_identity=dict(identities),
                                  dropped_records=self.dropped))
                        output.flush()
                        counts.clear()
                        metrics.clear()
                        events.clear()
                        identities.clear()
                        deadline = time.monotonic() + 1.0
        except Exception as error:
            self.error = str(error)
            print(f"PICO timing log disabled: {error}", flush=True)

    def close(self):
        self._stop.set()
        self._thread.join(timeout=1.0)
