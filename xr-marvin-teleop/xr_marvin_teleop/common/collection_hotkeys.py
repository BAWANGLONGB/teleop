"""Best-effort UI commands, separate from the ROS/control data path."""
import json
import os
import socket
import time


class CollectionHotkeys:
    def __init__(self):
        self.address = os.environ.get("FIELDNOTE_HOTKEY_SOCKET")
        self.token = os.environ.get("FIELDNOTE_HOTKEY_TOKEN")
        self.socket = None
        if self.address:
            try:
                self.socket = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
                self.socket.setblocking(False)
            except OSError:
                self.close()
                self.socket = None  # Optional UI commands must not prevent PICO startup.
        self.previous = None
        self.last_frame_ns = 0
        self.last_press_ns = 0

    def update(self, snapshot):
        now = time.monotonic_ns()
        if snapshot is None:
            self.previous = None
            return
        buttons = (snapshot.button_x, snapshot.button_y)
        previous = self.previous
        self.previous = buttons
        gap = now - self.last_frame_ns
        self.last_frame_ns = now
        # Require an observed release after startup/reconnect; never replay old presses.
        if previous is None or gap > 500_000_000:
            return
        edges = [pressed and not old for pressed, old in zip(buttons, previous)]
        if not any(edges):
            return
        elapsed = now - self.last_press_ns
        self.last_press_ns = now
        # Both buttons together are ambiguous; consume without executing either.
        if all(buttons) or elapsed < 300_000_000 or not self.socket:
            return
        packet = json.dumps({"button": "X" if edges[0] else "Y", "at_ns": now,
                             "token": self.token}).encode()
        try:
            self.socket.sendto(packet, self.address)
        except OSError:
            pass  # Full/unavailable UI socket: discard, never block PICO or retry a command.

    def close(self):
        if self.socket:
            self.socket.close()
