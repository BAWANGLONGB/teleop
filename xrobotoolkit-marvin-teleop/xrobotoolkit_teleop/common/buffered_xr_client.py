import threading
import time
from typing import Iterable

import numpy as np


class BufferedXrClient:
    """Poll an XR client on a worker and expose one atomic latest-value snapshot."""

    def __init__(
        self,
        client,
        pose_names: Iterable[str],
        key_names: Iterable[str],
        button_names: Iterable[str],
        poll_hz: float = 200.0,
        include_motion_trackers: bool = False,
    ):
        if poll_hz <= 0.0:
            raise ValueError("poll_hz must be positive")
        self._client = client
        self._pose_names = tuple(dict.fromkeys(pose_names))
        self._key_names = tuple(dict.fromkeys(key_names))
        self._button_names = tuple(dict.fromkeys(button_names))
        self._poll_period = 1.0 / poll_hz
        self._include_motion_trackers = include_motion_trackers
        self._lock = threading.Lock()
        self._local = threading.local()
        self._ready = threading.Event()
        self._stop = threading.Event()
        self._thread = None
        self._snapshot = None
        self._sequence = 0
        self._last_source_timestamp = None
        self._source_timestamp_change_ns = None
        self._last_error = None

    def start(self):
        if self._thread is not None:
            return
        self._thread = threading.Thread(target=self._poll_loop, name="xr-latest-value", daemon=True)
        self._thread.start()

    def wait_until_ready(self, timeout=1.0):
        return self._ready.wait(timeout)

    def _poll_loop(self):
        next_deadline = time.monotonic()
        while not self._stop.is_set():
            try:
                source_timestamp = self._client.get_timestamp_ns()
                poses = {
                    name: np.asarray(self._client.get_pose_by_name(name), dtype=float).copy()
                    for name in self._pose_names
                }
                keys = {
                    name: float(self._client.get_key_value_by_name(name))
                    for name in self._key_names
                }
                buttons = {
                    name: bool(self._client.get_button_state_by_name(name))
                    for name in self._button_names
                }
                motion_trackers = (
                    self._client.get_motion_tracker_data()
                    if self._include_motion_trackers
                    else {}
                )
                receipt_ns = time.monotonic_ns()
                with self._lock:
                    if source_timestamp != self._last_source_timestamp:
                        self._last_source_timestamp = source_timestamp
                        self._source_timestamp_change_ns = receipt_ns
                    source_age_ms_at_receipt = (
                        float("inf")
                        if self._source_timestamp_change_ns is None
                        else (receipt_ns - self._source_timestamp_change_ns) / 1e6
                    )
                    self._sequence += 1
                    self._snapshot = {
                        "sequence": self._sequence,
                        "source_timestamp_ns": source_timestamp,
                        "receipt_monotonic_ns": receipt_ns,
                        "source_age_ms_at_receipt": source_age_ms_at_receipt,
                        "poses": poses,
                        "keys": keys,
                        "buttons": buttons,
                        "motion_trackers": motion_trackers,
                    }
                    self._last_error = None
                self._ready.set()
            except Exception as error:  # Keep the last valid snapshot on transient SDK errors.
                with self._lock:
                    self._last_error = repr(error)

            next_deadline += self._poll_period
            sleep_duration = next_deadline - time.monotonic()
            if sleep_duration > 0.0:
                self._stop.wait(sleep_duration)
            else:
                next_deadline = time.monotonic()

    def _read_group(self, group, name):
        snapshot = getattr(self._local, "snapshot", None)
        if snapshot is None:
            with self._lock:
                snapshot = self._snapshot
        if snapshot is None or name not in snapshot[group]:
            raise RuntimeError(f"XR latest-value snapshot has no '{name}' in {group}")
        value = snapshot[group][name]
        return value.copy() if isinstance(value, np.ndarray) else value

    def begin_cycle(self):
        """Pin all getters on the calling thread to one complete snapshot."""
        with self._lock:
            if self._snapshot is None:
                raise RuntimeError("XR latest-value snapshot is not ready")
            self._local.snapshot = self._snapshot

    def end_cycle(self):
        if hasattr(self._local, "snapshot"):
            del self._local.snapshot

    def get_pose_by_name(self, name):
        return self._read_group("poses", name)

    def get_key_value_by_name(self, name):
        return self._read_group("keys", name)

    def get_button_state_by_name(self, name):
        return self._read_group("buttons", name)

    def get_motion_tracker_data(self):
        snapshot = getattr(self._local, "snapshot", None)
        if snapshot is not None:
            return snapshot["motion_trackers"].copy()
        with self._lock:
            if self._snapshot is None:
                return {}
            return self._snapshot["motion_trackers"].copy()

    def get_timestamp_ns(self):
        snapshot = getattr(self._local, "snapshot", None)
        if snapshot is not None:
            return snapshot["source_timestamp_ns"]
        with self._lock:
            return 0 if self._snapshot is None else self._snapshot["source_timestamp_ns"]

    def get_diagnostics(self):
        now_ns = time.monotonic_ns()
        pinned_snapshot = getattr(self._local, "snapshot", None)
        with self._lock:
            snapshot = pinned_snapshot if pinned_snapshot is not None else self._snapshot
            last_error = self._last_error
            if snapshot is None:
                return {
                    "sequence": 0,
                    "poll_age_ms": float("inf"),
                    "source_age_ms": float("inf"),
                    "source_timestamp_ns": None,
                    "last_error": last_error,
                }
            poll_age_ms = (now_ns - snapshot["receipt_monotonic_ns"]) / 1e6
            return {
                "sequence": snapshot["sequence"],
                "poll_age_ms": poll_age_ms,
                "source_age_ms": snapshot["source_age_ms_at_receipt"] + poll_age_ms,
                "source_timestamp_ns": snapshot["source_timestamp_ns"],
                "last_error": last_error,
            }

    def close(self):
        self._stop.set()
        if self._thread is not None and self._thread is not threading.current_thread():
            self._thread.join(timeout=1.0)
        self._client.close()
