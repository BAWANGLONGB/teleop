"""Non-blocking JSONL session logging for Marvin hardware teleoperation."""

from __future__ import annotations

import json
import queue
import threading
import time
from datetime import datetime
from pathlib import Path

import numpy as np


def _json_value(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {str(key): _json_value(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_value(item) for item in value]
    return value


class MarvinSessionLogger:
    def __init__(self, output_dir, metadata):
        timestamp = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S_%f")
        output_dir = Path(output_dir).expanduser()
        output_dir.mkdir(parents=True, exist_ok=True)
        self.events_path = output_dir / f"marvin_hardware_{timestamp}.jsonl"
        self.summary_path = output_dir / f"marvin_hardware_{timestamp}.summary.json"
        self._metadata = _json_value(metadata)
        self._queue = queue.SimpleQueue()
        self._stop_token = object()
        self._error = None
        self._event_count = 0
        self._count_lock = threading.Lock()
        self._started_at = datetime.now().astimezone()
        self._started_monotonic = time.monotonic()
        self._thread = threading.Thread(
            target=self._writer,
            name="marvin-session-logger",
            daemon=True,
        )
        self._thread.start()

    def _writer(self):
        try:
            with self.events_path.open("w", encoding="utf-8") as event_file:
                last_flush = time.monotonic()
                while True:
                    event = self._queue.get()
                    if event is self._stop_token:
                        break
                    json.dump(event, event_file, ensure_ascii=False, allow_nan=False)
                    event_file.write("\n")
                    if time.monotonic() - last_flush >= 1.0:
                        event_file.flush()
                        last_flush = time.monotonic()
                event_file.flush()
        except Exception as error:
            self._error = error

    def record(self, event_type, **fields):
        event = {
            "event": event_type,
            "wall_time_ns": time.time_ns(),
            "monotonic_time_ns": time.monotonic_ns(),
            **_json_value(fields),
        }
        with self._count_lock:
            self._event_count += 1
        self._queue.put(event)

    def close(self, final_state=None):
        if self._thread is None:
            return
        self._queue.put(self._stop_token)
        self._thread.join(timeout=5.0)
        ended_at = datetime.now().astimezone()
        summary = {
            "schema_version": 1,
            "started_at": self._started_at.isoformat(),
            "ended_at": ended_at.isoformat(),
            "duration_s": time.monotonic() - self._started_monotonic,
            "event_count": self._event_count,
            "events_path": str(self.events_path.resolve()),
            "final_state": final_state,
            "writer_error": None if self._error is None else repr(self._error),
            "configuration": self._metadata,
        }
        with self.summary_path.open("w", encoding="utf-8") as summary_file:
            json.dump(summary, summary_file, indent=2, ensure_ascii=False, allow_nan=False)
            summary_file.write("\n")
        self._thread = None
