"""On-demand exports: selected format, reuse, and recording exclusion."""
from contextlib import nullcontext
import importlib.util
import runpy
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

from xr_marvin_teleop.common.collection_config import write_json


class TestUiExport(unittest.TestCase):
    def test_recording_stop_does_not_package_and_devices_do_not_block_export(self):
        from xr_marvin_teleop.common.episode_video import activity_lock
        main = runpy.run_path(str(Path(__file__).resolve().parents[1] / "scripts/data/run_collection.py"))["main"]
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
        spec = importlib.util.spec_from_file_location("export_ui", Path(__file__).resolve().parents[2] / "UI/server.py")
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
                self.assertTrue(ui.START_LOCK.locked())
                self.assertIn("--add-missing", command)
                variant = "mjpeg" if "--mjpeg" in command else "h264"
                self.assertIn("--no-h264" if variant == "mjpeg" else "--no-mjpeg", command)
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
                self.assertEqual(ui.mcap_export_files(ui.DATASET_ROOT, [episode.name], "mjpeg")[0][1].read_bytes(), b"mjpeg")
                self.assertEqual(ui.mcap_export_files(ui.DATASET_ROOT, [episode.name], "h264")[0][1].read_bytes(), b"h264")
                with self.assertRaises(ui.ApiError):
                    ui.prepare_mcap_export({**payload, "format": "../bad"})
                h264 = episode / "final" / f"{episode.name}.h264.mcap"
                h264.unlink()
                with patch.object(ui, "collection_active", return_value=True):
                    with self.assertRaises(ui.ApiError):
                        ui.prepare_mcap_export(payload)
                self.assertEqual(run.call_count, 2)
                run.side_effect = None
                run.return_value = Mock(returncode=1)
                with self.assertRaises(ui.ApiError):
                    ui.prepare_mcap_export(payload)
                self.assertFalse(h264.exists())
                self.assertEqual(raw.read_bytes(), b"original")
                self.assertFalse(ui.START_LOCK.locked())


if __name__ == "__main__":
    unittest.main()
