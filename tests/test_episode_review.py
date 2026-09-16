import hashlib
import importlib.util
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from scripts.data.collect_successful_mcaps import collect
from xr_marvin_teleop.common.collection_config import write_json
from xr_marvin_teleop.common.episode_review import (
    annotation_lock, episode_path, read_review, save_review, save_session, session_path,
)
from xr_marvin_teleop.common.episode_video import activity_lock


class TestEpisodeReview(unittest.TestCase):
    def test_ui_session_and_review_api(self):
        location = Path(__file__).resolve().parents[2] / "UI/server.py"
        spec = importlib.util.spec_from_file_location("review_ui_server", location)
        ui = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(ui)
        with tempfile.TemporaryDirectory() as directory, patch.object(ui, "DATASET_ROOT", Path(directory)):
            def request(method, path, body=None):
                handler = object.__new__(ui.Handler)
                handler.command, handler.path = method, path
                data = json.dumps(body).encode() if body is not None else b""
                handler.headers = {"Content-Length": str(len(data))}
                handler.rfile = io.BytesIO(data)
                responses = []
                handler.send_json = lambda data, status=200: responses.append((status, data))
                getattr(handler, f"do_{method}")()
                return responses[-1]

            status, session = request("POST", "/api/sessions", {"name": "抓取测试"})
            self.assertEqual(status, 200)
            root = Path(directory)
            episode = root / session["id"] / "episode_120000_deadbeef"
            episode.mkdir()
            write_json(episode / "metadata.json", {"status": "completed", "episode_id": episode.name})
            url = f"/api/episodes/{episode.name}/review"
            body = {"session": session["id"], "result": "success"}
            self.assertEqual(request("POST", url, body)[0], 200)
            status, listing = request("GET", "/api/episodes")
            self.assertEqual(listing["episodes"][0]["review"]["result"], "success")
            self.assertEqual(listing["episodes"][0]["status"], "completed")
            self.assertEqual(request("POST", f"/api/sessions/{session['id']}/rename", {"name": "新的名称"})[0], 200)
            self.assertTrue(episode.is_dir())
            self.assertEqual(request("GET", "/api/sessions")[1]["sessions"][0]["name"], "新的名称")
            with patch.object(ui, "collection_status", return_value={"active": True, "episode_id": episode.name}):
                self.assertEqual(request("POST", url, {**body, "result": "failure"})[0], 409)
            self.assertEqual(read_review(episode)["result"], "success")
            self.assertEqual(request("POST", url, {**body, "result": "failure"})[0], 200)
            self.assertEqual(request("POST", url, {**body, "result": "unmarked"})[0], 200)
            self.assertEqual(request("POST", url, {**body, "session": "../escape"})[0], 400)
            self.assertEqual(request("POST", url, ["not an object"])[0], 400)

    def test_names_outcomes_and_success_copy(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory) / "dataset"
            session = save_session(root, "抓取杯子 / 第一批")
            other = save_session(root, "抓取杯子 / 第一批")
            self.assertNotEqual(session["id"], other["id"])
            self.assertEqual(save_session(root, "第二批", session["id"])["id"], session["id"])

            def episode(session_id, name, h264_only=False):
                path = session_path(root, session_id) / name
                (path / "final").mkdir(parents=True)
                write_json(path / "metadata.json", {"status": "completed", "episode_id": name,
                           "export_options": {"mjpeg": not h264_only, "h264": True}})
                for variant in (("h264",) if h264_only else ("h264", "mjpeg")):
                    (path / "final" / f"{name}.{variant}.mcap").write_bytes(f"{session_id}/{name}/{variant}".encode())
                return path

            success = episode(session["id"], "episode_120000_deadbeef")
            failure = episode(session["id"], "episode_120001_cafebabe")
            unmarked = episode(session["id"], "episode_120002_01234567")
            duplicate = episode(other["id"], success.name, h264_only=True)
            with activity_lock(root):
                save_review(root, session["id"], success.name, "success")
                with self.assertRaises(RuntimeError):
                    collect(root, Path(directory) / "busy")
            save_review(root, session["id"], failure.name, "failure")
            save_review(root, other["id"], duplicate.name, "success")
            self.assertEqual(read_review(unmarked)["result"], "unmarked")
            write_json(unmarked / "review.json", {"result": "success", "episode_id": "episode_999999_deadbeef"})
            with self.assertRaises(ValueError):
                read_review(unmarked)
            (unmarked / "review.json").unlink()
            self.assertEqual(json.loads((success / "metadata.json").read_text())["status"], "completed")
            with annotation_lock(root), self.assertRaises(RuntimeError):
                save_review(root, session["id"], success.name, "failure")
            with self.assertRaises(ValueError):
                save_review(root, session["id"], success.name, "completed")
            with self.assertRaises(ValueError):
                episode_path(root, "../outside", success.name)
            with self.assertRaises(ValueError):
                save_session(root, "\n")
            write_json(unmarked / "metadata.json", {"status": "recording"})
            with self.assertRaises(RuntimeError):
                save_review(root, session["id"], unmarked.name, "success")

            output = root / "successful"
            plan = collect(root, output, dry_run=True)
            self.assertFalse(output.exists())
            self.assertEqual(len(plan["episodes"]), 2)
            for state in ("aborted", "rejected", "starting", "finalizing"):
                write_json(unmarked / "metadata.json", {"status": state})
                self.assertEqual(len(collect(root, output, True)["episodes"]), 2)
            write_json(unmarked / "metadata.json", {"status": "completed"})
            self.assertEqual(len(collect(root, output, True)["episodes"]), 3)
            missing = success / "final" / f"{success.name}.mjpeg.mcap"
            original = missing.read_bytes()
            missing.unlink()
            self.assertEqual(len(collect(root, output, True)["missing_files"]), 1)
            with self.assertRaises(ValueError):
                collect(root, output)
            self.assertFalse(output.exists())
            missing.write_bytes(original)
            with patch("scripts.data.collect_successful_mcaps.write_json", side_effect=OSError("disk full")):
                with self.assertRaises(OSError):
                    collect(root, output)
            self.assertFalse(output.exists())
            self.assertFalse(list(root.glob(".collect-success-*")))
            plan = collect(root, output)
            self.assertEqual(len(list(output.rglob("*.mcap"))), 5)
            for entry in plan["episodes"]:
                for file in entry["files"]:
                    data = (output / file["destination"]).read_bytes()
                    self.assertEqual(data, Path(file["source"]).read_bytes())
                    self.assertEqual(hashlib.sha256(data).hexdigest(), file["sha256"])
            with self.assertRaises(FileExistsError):
                collect(root, output)
            with self.assertRaises(ValueError):
                collect(root, success / "nested-output")
            save_review(root, session["id"], success.name, "unmarked")
            self.assertEqual(len(collect(root, root / "new-successful", True)["episodes"]), 3)


if __name__ == "__main__":
    unittest.main()
