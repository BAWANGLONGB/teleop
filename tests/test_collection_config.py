import argparse
from contextlib import redirect_stdout
from copy import deepcopy
import io
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from xr_marvin_teleop.collection.config import (
    DEFAULT_CONFIG, load_config, validate_config, configure_parser, apply_config,
    snapshot_config, active_devices, check_active_devices,
)
from xr_marvin_teleop.collection.episode_video import add_video_arguments


class TestCollectionConfig(unittest.TestCase):
    def test_interpolation_config_and_legacy_capture(self):
        config = load_config()
        self.assertEqual(config["robot"]["joint_command_max_speed_deg_s"], 100.0)
        self.assertTrue(config["export"]["av1"])
        for value in (0, -1, True, None, float("nan"), float("inf")):
            invalid = deepcopy(config)
            invalid["robot"]["joint_command_max_speed_deg_s"] = value
            with self.assertRaises(ValueError):
                validate_config(invalid)
        del config["robot"]["joint_command_max_speed_deg_s"]
        restored = apply_config(argparse.Namespace(), saved=config)
        self.assertNotIn("joint_command_max_speed_deg_s", restored["robot"])
        config["export"]["h264"] = True
        for name in ("av1", "av1_crf", "av1_preset", "av1_keyint", "av1_threads"):
            config["export"].pop(name)
        restored = apply_config(argparse.Namespace(), saved=config)
        self.assertFalse(restored["export"]["av1"])

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "old-full-config.json"
            path.write_text(json.dumps(config))
            restored = load_config(path)
            validate_config(restored)
            self.assertFalse(restored["export"]["av1"])

    def test_layers_paths_switches_and_validation(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            path = root / "custom.json"
            path.write_text(json.dumps({"paths": {"output_root": "data", "calibrations": ["extra.json"]},
                                        "capture": {"vision_enabled": False},
                                        "export": {"h264": False, "h264_crf": 29}}))
            parser = argparse.ArgumentParser()
            parser.add_argument("--output-root", type=Path)
            parser.add_argument("--no-vision", action="store_true")
            parser.add_argument("--vision", dest="no_vision", action="store_false")
            add_video_arguments(parser)
            configure_parser(parser)
            args = parser.parse_args(["--config", str(path)])
            config = apply_config(args)
            self.assertEqual(args.output_root, root / "data")
            self.assertEqual(config["paths"]["calibrations"], [str(root / "extra.json")])
            self.assertTrue(args.no_vision)
            self.assertFalse(args.h264)
            self.assertTrue(args.av1)
            self.assertEqual(args.h264_crf, 29)
            self.assertEqual(config["paths"]["das_config"], str(DEFAULT_CONFIG.parent / "das_gripper.example.json"))
            self.assertEqual(config["robot"]["gripper_mode"], "binary")
            args = parser.parse_args(["--config", str(path), "--vision", "--h264", "--h264-crf", "18", "--output-root", "relative-cli"])
            apply_config(args)
            self.assertFalse(args.no_vision)
            self.assertTrue(args.h264)
            self.assertEqual(args.h264_crf, 18)
            self.assertEqual(args.output_root, Path.cwd() / "relative-cli")
            for text in ('{"export":{"h264":true,"h264":false}}',
                         '{"export":{"h264_pers et":"fast"}}',
                         '{"enable_hardware":true}', '{"paths":{"output_root":null}}',
                         '{"export":{"h264":"false"}}', '{"export":{"av1_crf":64}}',
                         '{"capture":{"pico_poll_hz":NaN}}',
                         '{"runtime":{"cpus":{"hardware":[true]}}}'):
                path.write_text(text)
                with self.assertRaises(ValueError, msg=text):
                    validate_config(load_config(path))
            # The editable default file must not double as its own schema.
            invalid_default = load_config()
            invalid_default["capture"]["vision_enabled"] = "false"
            path.write_text(json.dumps(invalid_default))
            with patch("xr_marvin_teleop.collection.config.DEFAULT_CONFIG", path):
                with self.assertRaises(ValueError):
                    validate_config(load_config())

    def test_missing_scale_is_generated_before_snapshot(self):
        from xr_marvin_teleop.control.calibration import resolve_scale_factor

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config = load_config()
            scale = root / "logs/scale.json"
            config["paths"]["scale_calibration"] = str(scale)
            for require_devices in (True, False):
                scale.unlink(missing_ok=True)
                path = snapshot_config(config, root / str(require_devices), require_devices=require_devices)
                frozen = load_config(path)
                self.assertEqual(resolve_scale_factor(None, scale), 1.2)
                self.assertEqual(Path(frozen["paths"]["scale_calibration"]).read_bytes(), scale.read_bytes())
            scale.unlink()
            self.assertEqual(resolve_scale_factor(None, scale), 1.2)
            record = json.loads(scale.read_text())
            self.assertIsNone(record["arm_lengths_m"])
            record["scale_factor"] = 0.9
            scale.write_text(json.dumps(record))
            original = scale.read_bytes()
            self.assertEqual(resolve_scale_factor(None, scale), 0.9)
            self.assertEqual(resolve_scale_factor(1.4, scale), 1.4)
            snapshot_config(config, root / "existing")
            self.assertEqual(scale.read_bytes(), original)
            scale.unlink()
            self.assertEqual(resolve_scale_factor(1.4, scale), 1.4)
            self.assertEqual(resolve_scale_factor(None, scale), 1.2)
            scale.write_text("invalid json")
            with self.assertRaises(ValueError):
                resolve_scale_factor(None, scale)
            self.assertEqual(scale.read_text(), "invalid json")

    def test_snapshot_is_portable_and_device_contract_is_checked(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config = load_config()
            scale = root / "scale.json"
            scale.write_text('{"scale":1}')
            config["paths"]["scale_calibration"] = str(scale)
            config["paths"]["output_root"] = str(root)
            path = snapshot_config(config, root / "snapshot")
            frozen = validate_config(load_config(path))
            snapshot_scale = Path(frozen["paths"]["scale_calibration"])
            scale.write_text('{"scale":2}')
            self.assertEqual(snapshot_scale.read_text(), '{"scale":1}')
            (root / "snapshot").rename(root / "moved")
            moved = load_config(root / "moved/collection.json")
            self.assertTrue(Path(moved["paths"]["urdf"]).is_file())
            with active_devices(moved):
                check_active_devices(moved)
                export_only = deepcopy(moved)
                export_only["export"]["h264_crf"] = 31
                check_active_devices(export_only)
                changed = deepcopy(moved)
                changed["robot"]["ip"] = "192.168.1.191"
                with self.assertRaisesRegex(RuntimeError, "differs"):
                    check_active_devices(changed)
                with self.assertRaises(RuntimeError):
                    with active_devices(moved):
                        pass
            check_active_devices(config)

    def test_entrypoints_print_without_devices_and_forward_parameters(self):
        project = DEFAULT_CONFIG.parent.parent
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config = root / "custom.json"
            config.write_text(json.dumps({"export": {"h264_crf": 32}, "preview": {"fps": 12},
                                          "runtime": {"camera_startup_timeout_s": 22.0},
                                          "recording": {"state_cache_bytes": 123456}}))
            for module in ("xr_marvin_teleop.cli.collection", "xr_marvin_teleop.cli.record"):
                result = subprocess.run([sys.executable, "-m", module,
                                         "--config", str(config), "--print-effective-config", "--no-h264"],
                                        capture_output=True, text=True, timeout=10)
                self.assertEqual(result.returncode, 0, result.stderr)
                effective = json.loads(result.stdout)
                self.assertFalse(effective["export"]["h264"])
                self.assertEqual(effective["export"]["h264_crf"], 32)
                self.assertEqual(effective["preview"]["fps"], 12)
            from xr_marvin_teleop.cli import collection as collection_cli
            namespace = vars(collection_cli)
            (root / "session_test").mkdir()
            args = namespace["parse_command_line_arguments"]([
                "--config", str(config), "--task", "test", "--enable-hardware",
                "--output-root", str(root), "--session", "session_test", "--episode-id", "episode_120000_deadbeef",
                "--joint-command-max-speed-deg-s", "80",
                "--confirmed-estop", "--confirmed-joint-mapping"])
            commands = namespace["_build_commands"](args)
            self.assertEqual(Path(commands["pico"][0]), project / ".venv/bin/python")
            hardware = commands["hardware"]
            self.assertEqual(hardware[hardware.index("--joint-command-max-speed-deg-s") + 1], "80.0")
            recorder = commands["recorder"]
            self.assertEqual(recorder[recorder.index("--camera-startup-timeout") + 1], "22.0")
            self.assertEqual(recorder[recorder.index("--config") + 1], str(config))
            self.assertEqual(recorder[recorder.index("--session") + 1], "session_test")
            self.assertEqual(recorder[recorder.index("--episode-id") + 1], "episode_120000_deadbeef")
            self.assertIn("--encoder-stale-timeout", commands["das_left"])
            self.assertEqual(commands["hardware"][commands["hardware"].index("--gripper-mode") + 1], "binary")

            from xr_marvin_teleop.cli import record as record_cli
            camera = record_cli._camera_command(
                project, "left",
                SimpleNamespace(camera_device="/dev/video0", camera_resolution="640x480", camera_fps=60),
                root / "camera", root / "storage.yaml", root / "ready",
            )
            self.assertEqual(Path(camera[0]), project / ".venv/bin/python")
            self.assertEqual(camera[1:3], ["-m", "xr_marvin_teleop.cli.capture"])

    def test_recorder_persists_configuration_before_capture(self):
        project = DEFAULT_CONFIG.parent.parent
        from xr_marvin_teleop.cli import record as record_cli
        namespace = vars(record_cli)
        runtime = namespace["main"].__globals__
        runtime["_require_mcap"] = lambda: None
        runtime["EpisodePublisher"] = lambda: SimpleNamespace(
            publish_state=lambda *_args: None, publish_event=lambda *_args: None,
            spin_once=lambda: None, close=lambda: None)
        commands = []
        def start(command, log_path):
            commands.append(command)
            return SimpleNamespace(poll=lambda: None, returncode=0), log_path.open("w")
        runtime["_start_recorder"] = start
        runtime["_stop_recorder"] = lambda *_args: None
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config = root / "custom.json"
            config.write_text(json.dumps({"paths": {"output_root": str(root / "data")},
                                          "preview": {"enabled": False},
                                          "recording": {"state_cache_bytes": 123456},
                                          "export": {"h264": False}}))
            session = root / "data/session_named"
            session.mkdir(parents=True)
            (session / "session.json").write_text(json.dumps({"name": "命名采集"}))
            with patch.object(sys, "argv", ["record_episode.py", "--config", str(config),
                                           "--session", session.name, "--episode-id", "episode_120000_deadbeef",
                                           "--task", "test", "--no-vision", "--max-duration", "0.001"]), redirect_stdout(io.StringIO()):
                namespace["main"]()
            self.assertEqual(len(commands), 1)
            self.assertEqual(commands[0][commands[0].index("--max-cache-size") + 1], "123456")
            metadata_path = next((root / "data").glob("session_*/episode_*/metadata.json"))
            metadata = json.loads(metadata_path.read_text())
            self.assertEqual(metadata["session"], session.name)
            self.assertEqual(metadata["session_name"], "命名采集")
            self.assertEqual(metadata["episode_id"], "episode_120000_deadbeef")
            self.assertEqual(metadata["export_status"], "pending")
            self.assertFalse(metadata["capture_config"]["export"]["h264"])
            self.assertTrue(metadata["capture_config"]["export"]["av1"])
            self.assertTrue((metadata_path.parent / metadata["config_snapshot"]).is_file())
            self.assertTrue(metadata["config_files"])


if __name__ == "__main__":
    unittest.main()
