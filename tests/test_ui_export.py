"""On-demand exports: selected format, reuse, and recording exclusion."""
from contextlib import nullcontext
import importlib.util
from http.client import HTTPConnection
from http.server import ThreadingHTTPServer
from pathlib import Path
import tempfile
import threading
import unittest
from unittest.mock import Mock, patch

from xr_marvin_teleop.collection.config import write_json


class TestUiExport(unittest.TestCase):
    def test_http_write_origin_and_reset_confirmation(self):
        spec = importlib.util.spec_from_file_location("secure_ui", Path(__file__).resolve().parents[1] / "ui/server.py")
        ui = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(ui)
        with patch.object(ui, "process_running", return_value=False), patch.object(ui.subprocess, "run") as run:
            for payload in ({}, {"confirmed_estop": True},
                            {"confirmed_estop": True, "confirmed_workspace_clear": "true"}):
                with self.assertRaises(ui.ApiError):
                    ui.request_robot_reset(payload)
            run.assert_not_called()
        server = ThreadingHTTPServer(("127.0.0.1", 0), ui.Handler)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        host = f"127.0.0.1:{server.server_port}"
        payload = '{"confirmed_estop":true,"confirmed_workspace_clear":true}'
        try:
            with patch.object(ui, "request_robot_reset", return_value={"completed": True}) as reset:
                for method, origin, content_type, request_host, status in (
                    ("POST", "https://untrusted.example", "application/x-www-form-urlencoded", host, 403),
                    ("POST", None, "application/json", host, 403),
                    ("POST", "null", "application/json", host, 403),
                    ("DELETE", "https://untrusted.example", "application/json", host, 403),
                    ("POST", "http://untrusted.example", "application/json", "untrusted.example", 403),
                    ("POST", "http://" + host, "text/plain", host, 415),
                    ("POST", "http://" + host, "application/json", host, 200),
                ):
                    connection = HTTPConnection("127.0.0.1", server.server_port, timeout=2)
                    try:
                        headers = {"Host": request_host, "Content-Type": content_type}
                        if origin is not None:
                            headers["Origin"] = origin
                        connection.request(method, "/api/robot/reset", payload, headers)
                        response = connection.getresponse()
                        self.assertEqual(response.status, status)
                        response.read()
                    finally:
                        connection.close()
                reset.assert_called_once_with({"confirmed_estop": True, "confirmed_workspace_clear": True})
        finally:
            server.shutdown()
            server.server_close()
            thread.join()

    def test_recording_stop_does_not_package_and_devices_do_not_block_export(self):
        from xr_marvin_teleop.collection.episode_video import activity_lock
        from xr_marvin_teleop.cli.collection import main
        runtime = main.__globals__
        for name in ("_preflight", "freeze_arguments", "check_active_devices", "_start_devices", "_finish_starting_devices"):
            runtime[name] = lambda *args, **kwargs: None
        runtime["active_devices"] = lambda *args: nullcontext()
        runtime["_build_commands"] = lambda *args: {}
        runtime["_validated_cpu_sets"] = lambda *args: {}
        runtime["_shutdown_processes"] = lambda *args, **kwargs: {}
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            runtime["_start_recording"] = lambda *args: root / "episode"
            argv = ["--part", "recording", "--task", "test", "--output-root", str(root),
                    "--enable-hardware", "--confirmed-estop", "--confirmed-joint-mapping"]
            def recording_monitor(*args):
                with self.assertRaises(RuntimeError):
                    activity_lock(root, exclusive=True)
                return "recorder", 0
            runtime["_monitor"] = recording_monitor
            with patch("subprocess.run") as run:
                self.assertEqual(main(argv), 0)
                run.assert_not_called()
            def devices_monitor(*args):
                with activity_lock(root, exclusive=True):
                    return "signal", 0
            runtime["_monitor"] = devices_monitor
            argv[1] = "devices"
            self.assertEqual(main(argv), 0)

    def test_prepare_formats_reuses_files_and_retains_sources_on_failure(self):
        spec = importlib.util.spec_from_file_location("export_ui", Path(__file__).resolve().parents[1] / "ui/server.py")
        ui = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(ui)
        with tempfile.TemporaryDirectory() as directory:
            ui.DATASET_ROOT = Path(directory)
            episode = ui.DATASET_ROOT / "session_test/episode_120000_deadbeef"
            episode.mkdir(parents=True)
            write_json(episode / "metadata.json", {"status": "completed"})
            raw = episode / "raw.mcap"
            raw.write_bytes(b"original")
            payload = {"episode": episode.name, "format": "mjpeg"}

            def pack(command, **kwargs):
                self.assertTrue(ui.EXPORT_LOCK.locked())
                self.assertTrue(ui.START_LOCK.acquire(blocking=False))
                ui.START_LOCK.release()
                with patch.object(ui, "_stop_devices", return_value={"status": "stopping"}):
                    self.assertEqual(ui.stop_devices(), {"status": "stopping"})
                with self.assertRaises(ui.ApiError):
                    ui.start_collection({})
                with self.assertRaises(ui.ApiError):
                    ui.move_episode_to_trash(ui.DATASET_ROOT, episode.name)
                self.assertIn("--add-missing", command)
                variant = next(
                    name for name in ui.VIDEO_VARIANTS if f"--{name}" in command
                )
                for name in ui.VIDEO_VARIANTS:
                    self.assertIn(
                        f"--{name}" if name == variant else f"--no-{name}",
                        command,
                    )
                for codec in ("h264", "av1"):
                    for suffix in ("crf", "preset", "keyint", "threads"):
                        name = f"{codec}_{suffix}"
                        flag = "--" + name.replace("_", "-")
                        if variant == codec:
                            self.assertEqual(command[command.index(flag) + 1], str(ui.COLLECTION_SETTINGS["export"][name]))
                        else:
                            self.assertNotIn(flag, command)
                (episode / "final").mkdir(exist_ok=True)
                (episode / "final" / f"{episode.name}.{variant}.mcap").write_bytes(variant.encode())
                return Mock(returncode=0)

            with patch.object(ui, "teleop_environment", return_value={}), patch.object(ui.subprocess, "run", side_effect=pack) as run:
                self.assertEqual(ui.prepare_mcap_export(payload), {"ready": True})
                ui.prepare_mcap_export(payload)
                self.assertEqual(run.call_count, 1)
                payload["format"] = "h264"
                ui.prepare_mcap_export(payload)
                self.assertEqual(run.call_count, 2)
                payload["format"] = "av1"
                ui.prepare_mcap_export(payload)
                self.assertEqual(run.call_count, 3)
                self.assertEqual(ui.mcap_export_files(ui.DATASET_ROOT, [episode.name], "mjpeg")[0][1].read_bytes(), b"mjpeg")
                self.assertEqual(ui.mcap_export_files(ui.DATASET_ROOT, [episode.name], "h264")[0][1].read_bytes(), b"h264")
                self.assertEqual(ui.mcap_export_files(ui.DATASET_ROOT, [episode.name], "av1")[0][1].read_bytes(), b"av1")
                with self.assertRaises(ui.ApiError):
                    ui.prepare_mcap_export({**payload, "format": "../bad"})
                av1 = episode / "final" / f"{episode.name}.av1.mcap"
                av1.unlink()
                with patch.object(ui, "collection_active", return_value=True):
                    with self.assertRaises(ui.ApiError):
                        ui.prepare_mcap_export(payload)
                self.assertEqual(run.call_count, 3)
                run.side_effect = None
                run.return_value = Mock(returncode=1)
                with self.assertRaises(ui.ApiError):
                    ui.prepare_mcap_export(payload)
                self.assertFalse(av1.exists())
                self.assertEqual(raw.read_bytes(), b"original")
                self.assertFalse(ui.START_LOCK.locked())
                self.assertFalse(ui.EXPORT_LOCK.locked())


if __name__ == "__main__":
    unittest.main()
