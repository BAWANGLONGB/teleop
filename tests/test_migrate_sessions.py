"""Run with sourced ROS2: python -m unittest tests.test_migrate_sessions."""
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from xr_marvin_teleop.cli.migrate import migrate


class TestMigration(unittest.TestCase):
    def test_verify_before_replace_and_restore_on_publish_failure(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            session = root / "session_test"
            source = session / "episode_test"
            source.mkdir(parents=True)
            original = json.dumps({"episode_id": "episode_test"})
            (source / "metadata.json").write_text(original)
            backup = root / "backup"

            def convert(source, work, config):
                (work / "metadata.json").write_text(original)
                (work / "final").mkdir()
                outputs = [work / "final" / f"episode_test.{variant}.mcap"
                           for variant in ("mjpeg", "h264", "av1")]
                for output in outputs:
                    output.write_bytes(b"test output")
                return outputs

            rename = Path.rename

            def fail_publish(path, target):
                if path.name == "ready":
                    raise OSError("publication failed")
                return rename(path, target)

            with patch("xr_marvin_teleop.cli.migrate.convert_raw", side_effect=convert):
                with patch("xr_marvin_teleop.cli.migrate.verify", side_effect=ValueError("bad CRC")):
                    with self.assertRaisesRegex(ValueError, "bad CRC"):
                        migrate(source, session, {"export": {}}, backup, True)
                self.assertEqual((source / "metadata.json").read_text(), original)
                self.assertFalse(backup.exists())
                checked = {"counts": {"/test": 1}, "bytes": 11, "sha256": "test"}
                with patch("xr_marvin_teleop.cli.migrate.verify", return_value=checked), \
                     patch.object(Path, "rename", fail_publish):
                    with self.assertRaisesRegex(OSError, "publication failed"):
                        migrate(source, session, {"export": {}}, backup, True)
                self.assertEqual((source / "metadata.json").read_text(), original)
                with patch("xr_marvin_teleop.cli.migrate.verify", return_value=checked):
                    result = migrate(source, session, {"export": {}}, backup, True)
                self.assertEqual((Path(result["backup"]) / "metadata.json").read_text(), original)
                self.assertEqual(len(list((source / "final").glob("*.mcap"))), 3)
                self.assertEqual(json.loads((source / "metadata.json").read_text())["export_status"], "completed")
                manifest = json.loads((source / "manifest.json").read_text())
                self.assertEqual(
                    set(manifest["files"]),
                    {
                        "final/episode_test.mjpeg.mcap",
                        "final/episode_test.h264.mcap",
                        "final/episode_test.av1.mcap",
                    },
                )


if __name__ == "__main__":
    unittest.main()
