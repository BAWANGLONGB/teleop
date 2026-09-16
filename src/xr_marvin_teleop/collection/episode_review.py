"""Human session names and episode outcomes, separate from recorder validation."""
from contextlib import contextmanager
import fcntl
from pathlib import Path
import re
import time
import uuid

from .config import read_json, write_json
from .episode_video import activity_lock

EPISODE_ID = re.compile(r"episode_\d{6}_[0-9a-f]{8}\Z")
SESSION_ID = re.compile(r"session_[A-Za-z0-9_-]{1,100}\Z")


def new_episode_id():
    return f"episode_{time.strftime('%H%M%S')}_{uuid.uuid4().hex[:8]}"


def session_path(root, session_id):
    root = Path(root).resolve()
    if not isinstance(session_id, str) or not SESSION_ID.fullmatch(session_id):
        raise ValueError("Session ID 格式无效")
    path = root / session_id
    if path.is_symlink() or path.resolve().parent != root:
        raise ValueError("Session 路径无效")
    return path


def episode_path(root, session_id, episode_id):
    if not isinstance(episode_id, str) or not EPISODE_ID.fullmatch(episode_id):
        raise ValueError("Episode ID 格式无效")
    session = session_path(root, session_id)
    path = session / episode_id
    if path.is_symlink() or path.resolve().parent != session:
        raise ValueError("Episode 路径无效")
    if not path.is_dir():
        raise FileNotFoundError("Episode 目录不存在，请先完成采集或迁移旧数据")
    return path


@contextmanager
def annotation_lock(root):
    # ponytail: one short metadata-write lock per dataset; never taken by control threads.
    path = Path(root).resolve() / ".annotations.lock"
    if path.is_symlink():
        raise ValueError("标注锁路径无效")
    with path.open("a+b") as stream:
        try:
            fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            raise RuntimeError("正在整理数据或保存标注，请稍后重试") from error
        yield


def session_record(path):
    path = Path(path)
    if (path / "session.json").is_symlink():
        raise ValueError("Session 元数据路径无效")
    saved = read_json(path / "session.json") if (path / "session.json").is_file() else {}
    return {"id": path.name, "name": saved.get("name", path.name)}


def save_session(root, name, session_id=None):
    if not isinstance(name, str) or not 1 <= len(name.strip()) <= 80 or any(ord(c) < 32 for c in name):
        raise ValueError("Session 名称须为 1–80 个字符，且不能包含控制字符")
    root = Path(root).resolve()
    root.mkdir(parents=True, exist_ok=True)
    with annotation_lock(root):
        if session_id is None:
            session_id = f"session_{time.strftime('%Y-%m-%d_%H%M%S')}_{uuid.uuid4().hex[:8]}"
            path = session_path(root, session_id)
            path.mkdir()
        else:
            path = session_path(root, session_id)
            if not path.is_dir():
                raise FileNotFoundError("Session 不存在")
        write_json(path / "session.json", {"schema_version": 1, "id": session_id,
                   "name": name.strip(), "updated_at_ns": time.time_ns()})
        return session_record(path)


def read_review(episode):
    episode = Path(episode)
    path = episode / "review.json"
    if path.is_symlink():
        raise ValueError("标注文件路径无效")
    review = read_json(path) if path.is_file() else {"result": "unmarked"}
    if not isinstance(review, dict) or review.get("result") not in ("success", "failure", "unmarked"):
        raise ValueError(f"标注内容无效：{path}")
    if review.get("episode_id", episode.name) != episode.name or review.get("session", episode.parent.name) != episode.parent.name:
        raise ValueError(f"标注与段落不匹配：{path}")
    return review


def save_review(root, session_id, episode_id, result):
    if result not in ("success", "failure", "unmarked"):
        raise ValueError("结果只能是 success、failure 或 unmarked")
    with activity_lock(root), annotation_lock(root):
        episode = episode_path(root, session_id, episode_id)
        metadata = read_json(episode / "metadata.json")
        if metadata.get("status") not in ("completed", "aborted", "validated", "degraded", "rejected"):
            raise RuntimeError("本段尚未结束，暂不能标注")
        review = {"schema_version": 1, "episode_id": episode_id, "session": session_id,
                  "result": result, "updated_at_ns": time.time_ns()}
        write_json(episode / "review.json", review)
        return review
