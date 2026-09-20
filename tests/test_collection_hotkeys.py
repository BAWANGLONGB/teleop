import importlib.util
import json
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from xr_marvin_teleop.collection.hotkeys import CollectionHotkeys
from xr_marvin_teleop.collection.config import write_json
from xr_marvin_teleop.collection.episode_video import activity_lock


class TestCollectionHotkeys(unittest.TestCase):
    def test_socket_to_backend_without_hardware(self):
        spec = importlib.util.spec_from_file_location("socket_ui", Path(__file__).resolve().parents[1] / "ui/server.py")
        ui = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(ui)
        ui.DEVICES = {"hotkey_token": "test", "recording_payload": {"task": "socket"}}
        stopped, started = threading.Event(), threading.Event()
        with tempfile.TemporaryDirectory() as directory, socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as channel:
            address = str(Path(directory) / "buttons.sock")
            channel.bind(address)
            with patch.dict("os.environ", {"FIELDNOTE_HOTKEY_SOCKET": address, "FIELDNOTE_HOTKEY_TOKEN": "test"}), \
                    patch.object(ui, "devices_active", return_value=True), \
                    patch.object(ui, "_start_collection", side_effect=lambda *args: started.set()) as start:
                listener = threading.Thread(target=ui.listen_controller_buttons, args=(channel, stopped))
                listener.start()
                buttons = CollectionHotkeys()
                try:
                    buttons.socket.sendto(b"invalid JSON", address)
                    buttons.update(SimpleNamespace(button_x=False, button_y=False))
                    buttons.update(SimpleNamespace(button_x=True, button_y=False))
                    self.assertTrue(started.wait(1), "X did not reach UI backend")
                    start.assert_called_once_with({"task": "socket"}, "recording")
                finally:
                    buttons.close()
                    stopped.set()
                    listener.join(2)
                self.assertFalse(listener.is_alive())

    def test_edges_and_nonblocking_transport(self):
        transport = Mock()
        with patch.dict("os.environ", {"FIELDNOTE_HOTKEY_SOCKET": "/tmp/test-buttons", "FIELDNOTE_HOTKEY_TOKEN": "test"}), \
                patch("socket.socket", return_value=transport), \
                patch("time.monotonic_ns") as clock:
            buttons = CollectionHotkeys()
            transport.setblocking.assert_called_once_with(False)
            now = 1_000_000_000

            def frame(x=False, y=False, gap=100_000_000):
                nonlocal now
                now += gap
                clock.return_value = now
                buttons.update(SimpleNamespace(button_x=x, button_y=y))

            frame(x=True)  # Held at startup.
            frame(x=True)
            self.assertEqual(transport.sendto.call_count, 0)
            frame()
            frame(x=True)
            frame(x=True)
            frame()
            frame(x=True)  # 300ms after initial edge: a deliberate new press.
            self.assertEqual(transport.sendto.call_count, 2)
            frame()
            frame(x=True)  # Bounce at 200ms: no event.
            self.assertEqual(transport.sendto.call_count, 2)
            frame()
            frame(x=True, y=True, gap=300_000_000)
            self.assertEqual(transport.sendto.call_count, 2)
            buttons.update(None)
            frame(y=True)
            frame(y=True)
            self.assertEqual(transport.sendto.call_count, 2)
            frame()
            frame(y=True)
            self.assertEqual(json.loads(transport.sendto.call_args.args[0])["button"], "Y")
            frame()
            frame(x=True, gap=600_000_000)  # Lost advancing frames without explicit None.
            self.assertEqual(transport.sendto.call_count, 3)
            frame()
            transport.sendto.side_effect = BlockingIOError("queue full")
            frame(x=True)
            buttons.close()
            transport.close.assert_called_once()

    def test_backend_toggle_delete_and_guards(self):
        spec = importlib.util.spec_from_file_location("hotkey_ui", Path(__file__).resolve().parents[1] / "ui/server.py")
        ui = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(ui)
        with tempfile.TemporaryDirectory() as directory, patch("time.monotonic_ns") as clock:
            root = Path(directory)
            ui.DATASET_ROOT = root
            episode = root / "session_test" / "episode_120000_deadbeef"
            episode.mkdir(parents=True)
            write_json(episode / "metadata.json", {"status": "completed"})
            process = Mock()
            process.poll.return_value = 0
            ui.COLLECTION = {"process": process, "status": "completed", "episode_id": episode.name,
                             "session": episode.parent.name, "episode_path": str(episode),
                             "task": "test", "started_at": 1, "log": root / "log"}
            ui.DEVICES = {"hotkey_token": "test", "recording_payload": {"task": "old"}}
            ui.update_hotkey_settings({"task": "latest", "session": "session_test"})
            now = 1_000_000_000

            def press(button, **extra):
                nonlocal now
                now += 400_000_000
                clock.return_value = now
                packet = {"button": button, "at_ns": now, "token": "test", **extra}
                ui.handle_controller_button(packet)
                return packet

            with patch.object(ui, "devices_active", return_value=True), \
                    patch.object(ui, "_start_collection") as start, patch.object(ui, "_stop_collection") as stop:
                packet = press("X")
                start.assert_called_once_with({"task": "latest", "session": "session_test"}, "recording")
                ui.handle_controller_button(packet)  # Duplicate datagram.
                press("X", token="previous-device")
                press("X", at_ns=0)
                press("X", at_ns="invalid")
                self.assertEqual(start.call_count, 1)
                with ui.START_LOCK:
                    press("X")
                self.assertEqual(start.call_count, 1)
                process.poll.return_value = None
                ui.COLLECTION["status"] = "running"
                press("X")
                stop.assert_called_once()
                press("Y")
                self.assertTrue(episode.is_dir())
                for state in ("starting", "stopping"):
                    ui.COLLECTION["status"] = state
                    ui.COLLECTION["ready_file"] = root / "absent"
                    press("X")
                    press("Y")
                self.assertEqual(stop.call_count, 1)
                self.assertEqual(start.call_count, 1)
                process.poll.return_value = 0
                ui.COLLECTION["status"] = "completed"
                with activity_lock(root, exclusive=True):
                    press("Y")
                    self.assertTrue(episode.is_dir())
                write_json(episode / "metadata.json", {"status": "recording"})
                press("Y")
                self.assertTrue(episode.is_dir())
                write_json(episode / "metadata.json", {"status": "completed"})
                # Same ID in another Session must not be removed.
                other = root / "session_other" / episode.name
                other.mkdir(parents=True)
                press("Y")
                self.assertFalse(episode.exists())
                self.assertTrue(other.is_dir())
                self.assertEqual(len(list((root / ".trash").iterdir())), 1)
                press("Y")
                self.assertTrue(other.is_dir())
                self.assertTrue(ui.HOTKEY_STATUS["error"])

    def test_y_stops_inflight_export_before_delete(self):
        spec = importlib.util.spec_from_file_location("cancel_ui", Path(__file__).resolve().parents[1] / "ui/server.py")
        ui = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(ui)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            ui.DATASET_ROOT = root / "dataset"
            ui.COLLECTION_EXPORT_ROOT = root / "collection"
            episode = ui.DATASET_ROOT / "session_test/episode_120000_deadbeef"
            episode.mkdir(parents=True)
            write_json(episode / "metadata.json", {"status": "completed"})
            (episode / "final").mkdir()
            original = episode / "final" / f"{episode.name}.h264.mcap"
            original.write_bytes(b"h264")
            published = ui.COLLECTION_EXPORT_ROOT / "2026-09-18" / original.name
            published.parent.mkdir(parents=True)
            published.hardlink_to(original)
            recording = Mock()
            recording.poll.return_value = 0
            ui.COLLECTION = {"process": recording, "status": "completed", "episode_id": episode.name,
                             "session": episode.parent.name, "episode_path": str(episode),
                             "task": "test", "started_at": 1, "log": root / "recording.log"}
            ui.DEVICES = {"hotkey_token": "test", "recording_payload": {}}
            launched = threading.Event()
            children = []
            errors = []
            real_popen = subprocess.Popen

            def slow_export(_command, **kwargs):
                process = real_popen([sys.executable, "-c", "import time; time.sleep(30)"],
                                     stdout=kwargs["stdout"], stderr=kwargs["stderr"], start_new_session=True)
                children.append(process)
                launched.set()
                return process

            def run_export():
                try:
                    ui.prepare_mcap_export({"episode": episode.name, "format": "av1"})
                except ui.ApiError as error:
                    errors.append(error)

            try:
                with patch.object(ui, "teleop_environment", return_value={}), \
                     patch.object(ui.subprocess, "Popen", side_effect=slow_export), \
                     patch.object(ui, "devices_active", return_value=True):
                    worker = threading.Thread(target=run_export)
                    worker.start()
                    self.assertTrue(launched.wait(2))
                    ui.handle_controller_button({"button": "Y", "at_ns": time.monotonic_ns(), "token": "test"})
                    worker.join(2)
                self.assertFalse(ui.HOTKEY_STATUS["error"])
                self.assertFalse(episode.exists())
                self.assertFalse(published.exists())
                self.assertEqual(children[0].poll(), -15)
                self.assertFalse(ui.EXPORT_LOCK.locked())
                self.assertFalse(worker.is_alive())
                self.assertEqual(len(errors), 1)
            finally:
                for child in children:
                    if child.poll() is None:
                        child.kill()
                        child.wait()


if __name__ == "__main__":
    unittest.main()
