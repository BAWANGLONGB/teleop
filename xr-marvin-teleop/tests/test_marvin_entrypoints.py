"""Hardware CLI, reset, and collection supervisor tests."""

import unittest

from tests.marvin_hardware_fakes import (
    FakeMarvinSdkAdapter,
    MARVIN_INITIAL_POSE_Q_RAD,
    np,
    Path,
    runpy,
    tempfile,
)


class TestMarvinEntrypoints(unittest.TestCase):
    def test_hardware_cli_does_not_enable_das_by_default(self):
        entry_path = (
            Path(__file__).resolve().parents[1]
            / "scripts"
            / "hardware"
            / "teleop_marvin_hardware.py"
        )
        arguments, _parser = runpy.run_path(str(entry_path))[
            "parse_command_line_arguments"
        ]([])
        self.assertIsNone(arguments.das_gripper_config)
        self.assertIsNone(arguments.das_sdk_root)
        self.assertFalse(arguments.das_from_ros2)
        self.assertEqual(arguments.gripper_mode, "binary")


    def test_standalone_reset_reuses_safe_cosine_return(self):
        entry_path = (
            Path(__file__).resolve().parents[1]
            / "scripts"
            / "hardware"
            / "reset_marvin_hardware.py"
        )
        reset_robot = runpy.run_path(str(entry_path))["reset_robot"]
        adapter = FakeMarvinSdkAdapter()
        clock = [0.0]

        def sleep(seconds):
            clock[0] += seconds

        reset_robot(
            adapter,
            None,
            duration=0.04,
            expected_sdk_version=1,
            monotonic=lambda: clock[0],
            sleep=sleep,
        )

        np.testing.assert_allclose(
            adapter.sent_commands_rad[-1], MARVIN_INITIAL_POSE_Q_RAD
        )
        self.assertEqual(
            adapter.events[:3],
            [
                "configure_control_parameters",
                "enter_joint_impedance",
                "enable_pd_feedforward",
            ],
        )
        self.assertTrue(adapter.idle)
        self.assertTrue(adapter.released)


    def test_collection_supervisor_builds_safe_job_and_shutdown_order(self):
        entry_path = (
            Path(__file__).resolve().parents[1]
            / "scripts"
            / "data"
            / "run_collection.py"
        )
        namespace = runpy.run_path(str(entry_path))
        command_line = [
            "--task",
            "pick",
            "--robot-model",
            "M6S",
            "--enable-hardware",
            "--confirmed-estop",
            "--confirmed-joint-mapping",
            "--das-config",
            "das.json",
            "--das-sdk-root",
            "das-sdk",
            "--preview-root",
            "/dev/shm/fieldnote-preview-test",
        ]
        arguments = namespace["parse_command_line_arguments"](command_line)
        commands = namespace["_build_commands"](arguments)
        self.assertEqual(arguments.part, "all")
        self.assertEqual(
            namespace["parse_command_line_arguments"](
                ["--part", "devices", *command_line]
            ).part,
            "devices",
        )
        self.assertIn("--pico-from-ros2", commands["hardware"])
        self.assertIn("--ros2", commands["hardware"])
        self.assertIn("--das-from-ros2", commands["hardware"])
        self.assertNotIn("--das-sdk-root", commands["hardware"])
        self.assertIn("--sdk-root", commands["das_left"])
        self.assertIn("--side", commands["das_right"])
        self.assertIn("--das-config", commands["recorder"])
        self.assertIn("--ready-file", commands["recorder"])
        self.assertIn("--calibration", commands["recorder"])
        self.assertIn("--preview-root", commands["recorder"])
        self.assertIn("--no-mjpeg", commands["recorder"])
        self.assertIn("--no-h264", commands["recorder"])
        self.assertIn("--av1", commands["recorder"])
        custom = namespace["parse_command_line_arguments"](
            [*command_line, "--h264", "--no-av1", "--h264-crf", "28", "--h264-threads", "1"]
        )
        custom_command = namespace["_build_commands"](custom)["recorder"]
        self.assertIn("--no-mjpeg", custom_command)
        self.assertIn("--h264", custom_command)
        self.assertIn("--no-av1", custom_command)
        self.assertEqual(custom_command[custom_command.index("--h264-crf") + 1], "28")
        self.assertEqual(custom_command[custom_command.index("--h264-threads") + 1], "1")
        self.assertEqual(
            namespace["PROCESS_CPUS"]["hardware"], (2, 3, 18, 19)
        )

        calls = []
        results = namespace["_shutdown_processes"](
            {
                "pico": object(),
                "das_left": object(),
                "das_right": object(),
                "recorder": object(),
                "hardware": object(),
            },
            stop_process=lambda process, name, timeout: (
                calls.append((name, timeout)) or 0
            ),
        )
        self.assertEqual(
            calls,
            [
                ("hardware", 15.0),
                ("das_left", 10.0),
                ("das_right", 10.0),
                ("recorder", 50.0),
                ("pico", 10.0),
            ],
        )
        self.assertEqual(
            results,
            {
                "hardware": 0,
                "das_left": 0,
                "das_right": 0,
                "recorder": 0,
                "pico": 0,
            },
        )

        calls.clear()
        runtime = namespace["main"].__globals__
        runtime["_preflight"] = lambda _arguments: None
        runtime["_build_commands"] = lambda _arguments: {}
        runtime["_validated_cpu_sets"] = lambda *_args: {}
        from contextlib import nullcontext
        runtime["freeze_arguments"] = lambda *_args, **_kwargs: None
        runtime["active_devices"] = lambda *_args: nullcontext()
        runtime["check_active_devices"] = lambda *_args: None
        runtime["_start_devices"] = lambda *_arguments: calls.append("devices")
        runtime["_start_recording"] = lambda *_arguments: calls.append("recording")
        runtime["_finish_starting_devices"] = lambda *_arguments: calls.append(
            "hardware"
        )
        runtime["_monitor"] = lambda *_arguments: ("signal", 0)
        runtime["_shutdown_processes"] = lambda *_arguments, **_kwargs: {}
        # Exercise the real lock in an isolated root, never the operator's dataset.
        with tempfile.TemporaryDirectory() as directory:
            for part, expected in (
                ("all", ["devices", "recording", "hardware"]),
                ("devices", ["devices", "hardware"]),
                ("recording", ["recording"]),
            ):
                calls.clear()
                self.assertEqual(namespace["main"]([
                    "--part", part, *command_line, "--output-root", directory,
                ]), 0)
                self.assertEqual(calls, expected)


if __name__ == "__main__":
    unittest.main()
