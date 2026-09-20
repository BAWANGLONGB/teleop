#!/usr/bin/env python3
"""Fieldnote UI server: static files plus the minimal local control API."""

import argparse
import json
import os
import re
import shutil
import signal
import socket
import secrets
import subprocess
import sys
import tempfile
import threading
import time
from datetime import datetime
from functools import lru_cache
from http import HTTPStatus
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlsplit


UI_ROOT = Path(__file__).resolve().parent
WORKSPACE = UI_ROOT.parent
PROJECT_ROOT = WORKSPACE
SOURCE_ROOT = PROJECT_ROOT / "src"
LOG_ROOT = PROJECT_ROOT / "var" / "logs"
if str(SOURCE_ROOT) not in sys.path:
    sys.path.insert(0, str(SOURCE_ROOT))

from xr_marvin_teleop.collection.episode_package import read_attachment, write_episode_mcap
from xr_marvin_teleop.collection.config import DEFAULT_CONFIG, load_config, validate_config
from xr_marvin_teleop.collection.episode_review import (
    SESSION_ID, new_episode_id, session_path, session_record, save_session, read_review, save_review,
    episode_path as session_episode_path, annotation_lock,
)
from xr_marvin_teleop.collection.episode_video import VIDEO_VARIANTS, activity_lock

COLLECTION_CONFIG_PATH = DEFAULT_CONFIG
COLLECTION_SETTINGS = validate_config(load_config())

ROS_BASE_SETUP = Path("/opt/ros/humble/setup.bash")
DATASET_ROOT = Path(COLLECTION_SETTINGS["paths"]["output_root"])
COLLECTION_EXPORT_ROOT = WORKSPACE / "var" / "collection"
COLLECTION_MODULE = "xr_marvin_teleop.cli.collection"
RESET_MODULE = "xr_marvin_teleop.cli.reset"
POSTPROCESS_MODULE = "xr_marvin_teleop.cli.postprocess"
TELEOP_PYTHON = WORKSPACE / ".venv" / "bin" / "python"
ROBOTICS_SERVICE_SCRIPT = Path("/opt/apps/roboticsservice/runService.sh")
ROBOTICS_SERVICE_PORTS = (63901, 60061)
MARVIN_IP = COLLECTION_SETTINGS["robot"]["ip"]
PREVIEW_ROOT = Path(COLLECTION_SETTINGS["preview"]["root"]) if COLLECTION_SETTINGS["preview"]["root"] else None
DAS_CONFIG = Path(COLLECTION_SETTINGS["paths"]["das_config"])
DAS_SDK_ROOT = Path(COLLECTION_SETTINGS["paths"]["das_sdk_root"])
SCALE_CALIBRATION = Path(COLLECTION_SETTINGS["paths"]["scale_calibration"])
EPISODE_RE = re.compile(r"episode_\d{6}_[0-9a-f]{8}\Z")
KNOWN_CAMERA_FORMATS = {"640x480": (60,), "1600x1296": (60,)}
STATE_LOCK = threading.Lock()
START_LOCK = threading.Lock()
EXPORT_LOCK = threading.Lock()
EXPORT_STATE_LOCK = threading.Lock()
EXPORT_STATE = {"episode": None, "process": None, "cancelled": False, "deleting": False}
RESET_LOCK = threading.Lock()
PICO_LOCK = threading.Lock()
ERROR_LOCK = threading.Lock()
COLLECTION = None
DEVICES = None
LAST_STATUS_ERRORS = {}
HOTKEY_SOCKET_PATH = None
HOTKEY_AFTER_NS = 0
HOTKEY_STATUS = {}


class ApiError(Exception):
    def __init__(self, status, message):
        self.status = status
        self.message = message


def print_status_error(source, message):
    with ERROR_LOCK:
        if message is None:
            LAST_STATUS_ERRORS.pop(source, None)
            return False
        if LAST_STATUS_ERRORS.get(source) == message:
            return False
        LAST_STATUS_ERRORS[source] = message
    print(f"[{datetime.now():%Y-%m-%d %H:%M:%S}] ERROR {source}: {message}", file=sys.stderr, flush=True)
    return True


@lru_cache(maxsize=1)
def teleop_environment():
    environment = os.environ.copy()
    missing = [str(path) for path in (ROS_BASE_SETUP, TELEOP_PYTHON) if not path.is_file()]
    if missing:
        raise ApiError(HTTPStatus.SERVICE_UNAVAILABLE, f"遥操环境文件缺失：{', '.join(missing)}")
    try:
        result = subprocess.run(
            (
                "bash", "-c",
                'source "$1" && unset LD_PRELOAD && env -0',
                "fieldnote", str(ROS_BASE_SETUP),
            ),
            capture_output=True, timeout=5, check=False,
        )
    except (OSError, subprocess.TimeoutExpired) as error:
        raise ApiError(HTTPStatus.SERVICE_UNAVAILABLE, f"遥操环境初始化失败：{error}") from error
    if result.returncode != 0:
        detail = result.stderr.decode(errors="replace").strip()
        raise ApiError(HTTPStatus.SERVICE_UNAVAILABLE, f"遥操环境初始化失败：{detail or result.returncode}")
    for item in result.stdout.split(b"\0"):
        key, separator, value = item.partition(b"=")
        if separator:
            environment[os.fsdecode(key)] = os.fsdecode(value)
    environment["PYTHONPATH"] = os.pathsep.join(filter(None, (str(SOURCE_ROOT), environment.get("PYTHONPATH"))))
    environment.pop("LD_PRELOAD", None)
    return environment


def read_json(path):
    with path.open(encoding="utf-8") as file:
        return json.load(file)


def package_metadata(path):
    return json.loads(read_attachment(path, "meta/meta.json"))


def episode_path(dataset_root, episode_id, *, prefer_directory=False):
    if not EPISODE_RE.fullmatch(episode_id):
        raise ApiError(HTTPStatus.BAD_REQUEST, "Episode ID 格式无效")
    root = dataset_root.resolve()
    legacy = [path for path in root.glob(f"session_*/{episode_id}") if path.is_dir()]
    # ponytail: metadata is the source of truth; add an index only if thousands of files make this slow.
    packaged = []
    for pattern in ("session_*/data/chunk-*/episode_*.mcap", "data/chunk-*/episode_*.mcap"):
        for path in root.glob(pattern):
            try:
                if package_metadata(path).get("episode_id") == episode_id:
                    packaged.append(path)
            except (OSError, ValueError, KeyError, json.JSONDecodeError):
                continue
    matches = (legacy or packaged) if prefer_directory else (packaged or legacy)
    if len(matches) != 1:
        raise ApiError(HTTPStatus.NOT_FOUND, "Episode 不存在")
    path = matches[0]
    if path.is_symlink() or root not in path.resolve().parents:
        raise ApiError(HTTPStatus.BAD_REQUEST, "Episode 路径无效")
    return path


def move_episode_to_trash(dataset_root, episode_id, collection_active=False, *, session=None, export_locked=False):
    if EXPORT_LOCK.locked() and not export_locked:
        raise ApiError(HTTPStatus.CONFLICT, "正在导出，暂不能删除 Episode")
    if collection_active:
        raise ApiError(HTTPStatus.CONFLICT, "采集中不能删除 Episode")
    with activity_lock(dataset_root), annotation_lock(dataset_root):
        source = (session_episode_path(dataset_root, session, episode_id) if session
                  else episode_path(dataset_root, episode_id, prefer_directory=True))
        if source.is_dir() and (source / "metadata.json").is_file():
            if read_json(source / "metadata.json").get("status") in ("starting", "recording", "finalizing"):
                raise ApiError(HTTPStatus.CONFLICT, "本段仍在录制或保存，不能删除")
        trash = dataset_root.resolve() / ".trash"
        if trash.is_symlink():
            raise ValueError("回收站路径无效")
        trash.mkdir(exist_ok=True)
        destination = trash / f"{source.parent.name}__{episode_id}_{time.time_ns()}{source.suffix}"
        published = []
        original = source / "final" / f"{episode_id}.h264.mcap"
        if COLLECTION_EXPORT_ROOT.is_symlink():
            raise ValueError(f"输出根目录不能是符号链接：{COLLECTION_EXPORT_ROOT}")
        if source.is_dir() and original.is_file() and not original.is_symlink():
            for candidate in COLLECTION_EXPORT_ROOT.glob(f"????-??-??/{original.name}"):
                if not candidate.parent.is_symlink() and not candidate.is_symlink() and candidate.is_file() and candidate.samefile(original):
                    published.append(candidate)
        source.replace(destination)
        removed = []
        try:
            for candidate in published:
                candidate.unlink()
                removed.append(candidate)
        except OSError:
            destination.replace(source)
            for candidate in removed:
                os.link(original, candidate)
            raise
        return destination


def _begin_export(episode_id):
    with EXPORT_STATE_LOCK:
        if EXPORT_STATE["deleting"] or not EXPORT_LOCK.acquire(blocking=False):
            return False
        EXPORT_STATE.update(episode=episode_id, process=None, cancelled=False)
        return True


def _finish_export():
    with EXPORT_STATE_LOCK:
        EXPORT_STATE.update(episode=None, process=None, cancelled=False)
        EXPORT_LOCK.release()


def _export_cancelled():
    with EXPORT_STATE_LOCK:
        return EXPORT_STATE["cancelled"]


def delete_episode(dataset_root, episode_id, *, session=None, current_only=False):
    with EXPORT_STATE_LOCK:
        if EXPORT_STATE["deleting"]:
            raise ApiError(HTTPStatus.CONFLICT, "已有删除任务在进行")
        if EXPORT_LOCK.locked() and EXPORT_STATE["episode"] != episode_id:
            raise ApiError(HTTPStatus.CONFLICT, "正在导出其他 Episode，请稍后重试")
        EXPORT_STATE["deleting"] = True
        EXPORT_STATE["cancelled"] = EXPORT_LOCK.locked()
        process = EXPORT_STATE["process"]
    try:
        if process is not None and process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
        if not EXPORT_LOCK.acquire(timeout=5):
            with EXPORT_STATE_LOCK:
                process = EXPORT_STATE["process"]
            if process is not None and process.poll() is None:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
            if not EXPORT_LOCK.acquire(timeout=5):
                raise ApiError(HTTPStatus.CONFLICT, "导出进程未退出，Episode 未删除")
        try:
            with START_LOCK:
                if current_only:
                    current = collection_status()
                    if (current.get("episode_id"), current.get("session")) != (episode_id, session):
                        raise ApiError(HTTPStatus.CONFLICT, "上一段录制已改变，Episode 未删除")
                return move_episode_to_trash(dataset_root, episode_id, collection_active(),
                                             session=session, export_locked=True)
        finally:
            EXPORT_LOCK.release()
    finally:
        with EXPORT_STATE_LOCK:
            EXPORT_STATE["deleting"] = False


def open_episode_directory(dataset_root, episode_id, opener=None, launch=None):
    path = episode_path(dataset_root, episode_id)
    target = path.parent if path.is_file() else path
    opener = opener or shutil.which("xdg-open") or shutil.which("gio")
    if not opener:
        raise ApiError(HTTPStatus.SERVICE_UNAVAILABLE, "未找到系统文件管理器")
    command = (opener, "open", str(target)) if Path(opener).name == "gio" else (opener, str(target))
    (launch or subprocess.Popen)(command, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
    return path


def mcap_export_files(dataset_root, episode_ids, variant="av1"):
    if variant not in VIDEO_VARIANTS:
        raise ApiError(HTTPStatus.BAD_REQUEST, "导出格式只能是 av1、h264 或 mjpeg")
    episode_ids = list(dict.fromkeys(episode_ids))
    if len(episode_ids) != 1:
        raise ApiError(HTTPStatus.BAD_REQUEST, "每次只能导出一段 Episode")
    episode_id = episode_ids[0]
    episode = episode_path(dataset_root, episode_id, prefer_directory=True)
    if episode.is_file():
        raise ApiError(HTTPStatus.UNPROCESSABLE_ENTITY, f"{episode_id} 是旧格式，请先迁移并生成 AV1 MCAP")
    path = episode / "final" / f"{episode_id}.{variant}.mcap"
    if path.parent.is_symlink() or path.is_symlink() or path.resolve().parent != episode.resolve() / "final":
        raise ApiError(HTTPStatus.BAD_REQUEST, f"{episode_id} {variant} 数据路径无效")
    if not path.is_file():
        raise ApiError(HTTPStatus.UNPROCESSABLE_ENTITY, f"{episode_id} 尚无 {variant} MCAP，请先打包")
    return [(episode_id, path)]


def _run_mcap_export(episode_id, variant):
    if _export_cancelled():
        raise ApiError(HTTPStatus.CONFLICT, "MCAP 导出已中止")
    try:
        mcap_export_files(DATASET_ROOT, [episode_id], variant)
        return {"ready": True}
    except ApiError as error:
        if error.status != HTTPStatus.UNPROCESSABLE_ENTITY:
            raise
    with START_LOCK:
        if collection_active():
            raise ApiError(HTTPStatus.CONFLICT, "请先停止录制并等待原始数据保存完成")
    episode = episode_path(DATASET_ROOT, episode_id, prefer_directory=True)
    if not episode.is_dir():
        raise ApiError(HTTPStatus.UNPROCESSABLE_ENTITY, "旧格式请先迁移")
    if read_json(episode / "metadata.json").get("status") not in ("completed", "validated", "degraded"):
        raise ApiError(HTTPStatus.CONFLICT, "本段未正常结束，不能打包")
    log_path = episode / "export.log"
    command = [str(TELEOP_PYTHON), "-m", POSTPROCESS_MODULE,
               str(episode), "--output-root", str(DATASET_ROOT), "--add-missing"]
    command.extend(
        f"--{name}" if variant == name else f"--no-{name}"
        for name in VIDEO_VARIANTS
    )
    if variant in ("h264", "av1"):
        for suffix in ("crf", "preset", "keyint", "threads"):
            name = f"{variant}_{suffix}"
            command += ["--" + name.replace("_", "-"), str(COLLECTION_SETTINGS["export"][name])]
    with log_path.open("ab", buffering=0) as log:
        environment = teleop_environment()
        with EXPORT_STATE_LOCK:
            if EXPORT_STATE["cancelled"]:
                raise ApiError(HTTPStatus.CONFLICT, "MCAP 导出已中止")
            process = subprocess.Popen(command, cwd=PROJECT_ROOT, env=environment,
                                       stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            EXPORT_STATE["process"] = process
        try:
            returncode = process.wait()
        finally:
            with EXPORT_STATE_LOCK:
                EXPORT_STATE["process"] = None
    if _export_cancelled():
        raise ApiError(HTTPStatus.CONFLICT, "MCAP 导出已中止")
    if returncode:
        raise ApiError(HTTPStatus.UNPROCESSABLE_ENTITY,
                       collection_exit_error(log_path, returncode, "MCAP 打包") + f"；日志：{log_path}")
    mcap_export_files(DATASET_ROOT, [episode_id], variant)
    return {"ready": True}


def prepare_mcap_export(payload):
    variant = payload.get("format", "av1")
    episode_id = payload.get("episode")
    if not isinstance(episode_id, str) or not EPISODE_RE.fullmatch(episode_id):
        raise ApiError(HTTPStatus.BAD_REQUEST, "Episode ID 格式无效")
    if not _begin_export(episode_id):
        raise ApiError(HTTPStatus.CONFLICT, "正在启动或打包，请稍后重试")
    try:
        return _run_mcap_export(episode_id, variant)
    finally:
        _finish_export()


def episode_record(path):
    if path.is_file():
        metadata = package_metadata(path)
        source = metadata.get("source_metadata", {})
        started = int(source.get("started_at_ns", 0) or 0)
        features = metadata.get("features", {})
        modalities = [
            label for label, present in (
                ("关节", "observation.state" in features),
                ("PICO", "observation.pico" in features),
                ("触觉", any("tactile" in name for name in features)),
                ("视觉", any(item.get("dtype") == "video" for item in features.values())),
            ) if present
        ]
        return {
            "id": metadata.get("episode_id", path.stem),
            "session": metadata.get("session", path.parents[2].name),
            "can_review": False,
            "task": metadata.get("task", "—"),
            "operator": metadata.get("operator", "—"),
            "robot_model": metadata.get("robot_model", "—"),
            "status": metadata.get("status", "unknown"),
            "duration_seconds": int(metadata.get("duration_seconds", 0) or 0),
            "size_bytes": path.stat().st_size,
            "created_at": datetime.fromtimestamp(started / 1e9).astimezone().isoformat() if started else "",
            "modalities": modalities,
        }
    metadata_path = path / "metadata.json"
    manifest_path = path / "manifest.json"
    metadata = read_json(metadata_path) if metadata_path.is_file() else {}
    manifest = read_json(manifest_path) if manifest_path.is_file() else {}
    started = int(metadata.get("started_at_ns", 0) or 0)
    ended = int(metadata.get("ended_at_ns", 0) or 0)
    duration = max(0, (ended - started) // 1_000_000_000) if ended else 0
    size = sum(item.stat().st_size for item in path.rglob("*") if item.is_file() and not item.is_symlink())
    status = manifest.get("status") or metadata.get("status", "unknown")
    topics = {
        topic
        for bag in manifest.get("bags", {}).values()
        if isinstance(bag, dict)
        for topic in bag
    }
    modalities = [
        label for label, present in (
            ("关节", any("marvin" in topic for topic in topics)),
            ("PICO", bool({"/raw/pico/poses", "/raw/pico/frame"} & topics)),
            ("触觉", any("tactile" in topic for topic in topics)),
            ("视觉", any("image" in topic for topic in topics)),
        ) if present
    ]
    return {
        "id": metadata.get("episode_id", path.name),
        "session": path.parent.name,
        "session_name": session_record(path.parent)["name"],
        "can_review": True,
        "review": read_review(path),
        "recording_status": metadata.get("status", "unknown"),
        "task": metadata.get("task", "—"),
        "operator": metadata.get("operator", "—"),
        "robot_model": metadata.get("robot_model", "—"),
        "status": status,
        "duration_seconds": duration,
        "size_bytes": size,
        "created_at": datetime.fromtimestamp(started / 1e9).astimezone().isoformat() if started else "",
        "modalities": modalities,
    }


def list_episodes(dataset_root=None):
    dataset_root = DATASET_ROOT if dataset_root is None else dataset_root
    records = {}
    if dataset_root.is_dir():
        for path in dataset_root.glob("session_*/episode_*"):
            if path.is_dir() and EPISODE_RE.fullmatch(path.name):
                try:
                    record = episode_record(path)
                    records[record["id"]] = record
                except (OSError, ValueError, json.JSONDecodeError):
                    continue
        for pattern in ("session_*/data/chunk-*/episode_*.mcap", "data/chunk-*/episode_*.mcap"):
            for path in dataset_root.glob(pattern):
                if path.is_file() and not path.is_symlink():
                    try:
                        record = episode_record(path)
                        records.setdefault(record["id"], record)
                    except (OSError, ValueError, KeyError, json.JSONDecodeError):
                        continue
    return sorted(records.values(), key=lambda item: item["created_at"], reverse=True)


def prepare_camera_config(source, destination, resolution, fps):
    config = read_json(source)
    for side in ("left", "right"):
        if not isinstance(config.get(side), dict):
            raise ApiError(HTTPStatus.INTERNAL_SERVER_ERROR, f"DAS 配置缺少 {side}")
        if resolution is not None:
            config[side]["camera_resolution"] = resolution
        if fps is not None:
            config[side]["camera_fps"] = fps
        selected = config[side]["camera_resolution"]
        if selected not in KNOWN_CAMERA_FORMATS or config[side]["camera_fps"] not in KNOWN_CAMERA_FORMATS[selected]:
            raise ApiError(HTTPStatus.BAD_REQUEST, "相机分辨率与帧率组合不受支持")
    with destination.open("w", encoding="utf-8") as file:
        json.dump(config, file, ensure_ascii=False, indent=2)
        file.write("\n")


def parse_v4l2_formats(output):
    formats, mjpeg, resolution = {}, False, None
    for line in output.splitlines():
        match = re.search(r"\[\d+\]:\s+'([^']+)'", line)
        if match:
            mjpeg = match.group(1) in {"MJPG", "JPEG"}
            resolution = None
            continue
        match = re.search(r"Size:\s+Discrete\s+(\d+)x(\d+)", line)
        if match and mjpeg:
            resolution = f"{match.group(1)}x{match.group(2)}"
            formats.setdefault(resolution, set())
            continue
        match = re.search(r"\(([0-9.]+)\s+fps\)", line)
        if match and mjpeg and resolution:
            formats[resolution].add(round(float(match.group(1))))
    return formats


def camera_formats():
    tool = shutil.which("v4l2-ctl")
    devices = (Path("/dev/finger_camera_left"), Path("/dev/finger_camera_right"))
    if tool and all(device.exists() for device in devices):
        detected = []
        for device in devices:
            try:
                result = subprocess.run(
                    (tool, "--device", str(device), "--list-formats-ext"),
                    capture_output=True,
                    text=True,
                    timeout=3,
                    check=False,
                )
            except (OSError, subprocess.TimeoutExpired):
                break
            if result.returncode == 0:
                detected.append(parse_v4l2_formats(result.stdout))
        if len(detected) == 2:
            common = [
                {"resolution": resolution, "fps": sorted(
                    detected[0][resolution] & detected[1][resolution] & set(KNOWN_CAMERA_FORMATS[resolution])
                )}
                for resolution in sorted(detected[0].keys() & detected[1].keys())
                if resolution in KNOWN_CAMERA_FORMATS
                and detected[0][resolution] & detected[1][resolution] & set(KNOWN_CAMERA_FORMATS[resolution])
            ]
            if common:
                return {"source": "v4l2", "formats": common}
    return {
        "source": "config+sdk",
        "formats": [
            {"resolution": resolution, "fps": list(fps)}
            for resolution, fps in KNOWN_CAMERA_FORMATS.items()
        ],
    }


def process_running(program_name):
    for process in Path("/proc").iterdir():
        if not process.name.isdigit():
            continue
        if int(process.name) == os.getpid():
            continue
        try:
            arguments = (process / "cmdline").read_bytes().split(b"\0")
        except OSError:
            continue
        if any(Path(argument.decode(errors="ignore")).name == program_name for argument in arguments if argument):
            return True
    return False


def parse_robotics_service_ports(output):
    return sorted({int(port) for port in re.findall(r":(63901|60061)\b", output)})


def pico_ports_ready(ports):
    return set(ROBOTICS_SERVICE_PORTS).issubset(ports)


def robotics_service_ports():
    tool = shutil.which("ss")
    if not tool:
        return []
    try:
        result = subprocess.run(
            (tool, "-H", "-lnt"), capture_output=True, text=True, timeout=2, check=False
        )
    except (OSError, subprocess.TimeoutExpired):
        return []
    return parse_robotics_service_ports(result.stdout) if result.returncode == 0 else []


def parse_pico_clients(output):
    clients = []
    for line in output.splitlines():
        fields = line.split()
        if len(fields) < 5 or fields[0] != "ESTAB" or fields[3].rsplit(":", 1)[-1] != "63901":
            continue
        address = fields[4].rsplit(":", 1)[0].strip("[]")
        if address not in ("127.0.0.1", "::1"):
            clients.append(address.removeprefix("::ffff:"))
    return sorted(set(clients))


def pico_clients():
    tool = shutil.which("ss")
    if not tool:
        return []
    try:
        result = subprocess.run((tool, "-H", "-nt"), capture_output=True, text=True, timeout=2, check=False)
    except (OSError, subprocess.TimeoutExpired):
        return []
    return parse_pico_clients(result.stdout) if result.returncode == 0 else []


def ping_host(address, runner=subprocess.run):
    tool = shutil.which("ping")
    if not tool:
        return False, "系统缺少 ping 命令"
    try:
        result = runner(
            (tool, "-n", "-c", "1", "-W", "1", address),
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=2, check=False,
        )
    except (OSError, subprocess.TimeoutExpired):
        return False, f"Ping {address} 超时"
    return (True, None) if result.returncode == 0 else (False, f"Ping {address} 无响应")


def reachable_hosts(addresses, probe=ping_host):
    return [address for address in addresses if probe(address)[0]]


def pico_status():
    service = process_running("RoboticsServiceProcess")
    ports = robotics_service_ports()
    service_ready = pico_ports_ready(ports)
    tcp_clients = pico_clients() if service_ready else []
    clients = reachable_hosts(tcp_clients)
    connected = bool(clients)
    error = None
    if not service_ready:
        missing = ", ".join(str(port) for port in ROBOTICS_SERVICE_PORTS if port not in ports)
        error = f"PICO 服务端口未就绪：{missing}"
    elif tcp_clients and not connected:
        error = f"检测到 63901 TCP 会话，但客户端 {', '.join(tcp_clients)} Ping 不通"
    elif not connected:
        error = "服务已就绪，PICO 尚未建立 63901 TCP 连接"
    print_status_error("PICO", error)
    return {
        "connected": connected,
        "service_running": service,
        "service_ready": service_ready,
        "ports_listening": ports,
        "expected_ports": list(ROBOTICS_SERVICE_PORTS),
        "clients": clients,
        "tcp_clients": tcp_clients,
        "error": error,
    }


def hardware_status():
    try:
        config = read_json(DAS_CONFIG)
        print_status_error("DAS 配置", None)
    except (OSError, ValueError, json.JSONDecodeError) as error:
        print_status_error("DAS 配置", str(error))
        config = {}
    sides = {
        side: {
            "serial": Path(config.get(side, {}).get("serial_port", f"/dev/ttyFinger{side.title()}")),
            "camera": Path(config.get(side, {}).get("camera_device", f"/dev/finger_camera_{side}")),
        }
        for side in ("left", "right")
    }
    marvin_connected, marvin_error = ping_host(MARVIN_IP)
    serial_ready = {side: paths["serial"].exists() for side, paths in sides.items()}
    camera_ready = {side: paths["camera"].exists() for side, paths in sides.items()}
    status = {
        "collection_active": collection_active(),
        "devices_active": devices_active(),
        "processes": {
            "marvin": marvin_connected,
            "das": all(serial_ready.values()),
            "vision": all(camera_ready.values()),
        },
        "marvin": {
            "ip": MARVIN_IP,
            "connected": marvin_connected,
            "healthy": marvin_connected,
            "error": marvin_error,
        },
        "das": {
            side: {
                "device": str(paths["serial"]),
                "device_present": serial_ready[side],
                "healthy": serial_ready[side],
                "error": None if serial_ready[side] else "串口设备不存在",
            }
            for side, paths in sides.items()
        },
        "cameras": {
            **{
                side: {
                    "device": str(paths["camera"]),
                    "device_present": camera_ready[side],
                    "healthy": camera_ready[side],
                    "error": None if camera_ready[side] else "相机设备不存在",
                }
                for side, paths in sides.items()
            },
        },
    }
    print_status_error("Marvin", status["marvin"]["error"])
    for side in ("left", "right"):
        print_status_error(f"DAS {side}", status["das"][side]["error"])
        print_status_error(f"Camera {side}", status["cameras"][side]["error"])
    return status


def _job_status(job):
    if not job:
        return {"active": False, "status": "idle"}
    active = job["process"].poll() is None
    if active and job["status"] == "starting" and job["ready_file"].is_file():
        job["status"] = "running"
    episode = Path(job["episode_path"]) if job.get("episode_path") else None
    return {
        "active": active,
        "episode_id": job.get("episode_id"),
        "session": job.get("session"),
        "episode_exists": episode is not None and (episode / "metadata.json").is_file(),
        "review": read_review(episode) if episode is not None else {"result": "unmarked"},
        "status": job["status"],
        "task": job["task"],
        "started_at": job["started_at"],
        "returncode": job.get("returncode"),
        "error": job.get("error"),
        "export_status": job.get("export_status"),
        "export_error": job.get("export_error"),
        "export_path": job.get("export_path"),
        "log": str(job["log"]),
        "max_duration": job.get("max_duration"),
    }


def collection_status():
    with STATE_LOCK:
        return _job_status(COLLECTION)


def device_status():
    with STATE_LOCK:
        return _job_status(DEVICES)


def collection_active():
    return collection_status()["active"]


def devices_active(ready=False):
    status = device_status()
    return status["active"] and (not ready or status["status"] == "running")


def collection_exit_error(log_path, returncode, label="进程"):
    try:
        with log_path.open("rb") as log:
            log.seek(0, os.SEEK_END)
            log.seek(max(0, log.tell() - 16_384))
            lines = log.read().decode(errors="replace").splitlines()
    except OSError:
        lines = []
    detail = next(
        (line.strip() for line in reversed(lines) if re.search(r"error|exception|failed|traceback", line, re.IGNORECASE)),
        "",
    )
    message = f"{label}异常退出（退出码 {returncode}）"
    return f"{message}：{detail[:600]}" if detail else message


def _publish_h264(episode_id, started_at):
    source = mcap_export_files(DATASET_ROOT, [episode_id], "h264")[0][1]
    if COLLECTION_EXPORT_ROOT.is_symlink():
        raise ValueError(f"输出根目录不能是符号链接：{COLLECTION_EXPORT_ROOT}")
    directory = COLLECTION_EXPORT_ROOT / datetime.fromtimestamp(started_at).strftime("%Y-%m-%d")
    if directory.is_symlink():
        raise ValueError(f"输出日期目录不能是符号链接：{directory}")
    directory.mkdir(parents=True, exist_ok=True)
    target = directory / source.name
    if target.exists():
        if target.is_symlink() or not target.is_file() or not target.samefile(source):
            raise FileExistsError(f"输出文件已存在：{target}")
    else:
        # @decision AUTO-H264-PUBLISH MCAP is immutable; a hard link avoids a second large on-disk copy.
        os.link(source, target)
    return target


def _automatic_h264_export(job):
    try:
        _run_mcap_export(job["episode_id"], "h264")
        if _export_cancelled():
            raise ApiError(HTTPStatus.CONFLICT, "MCAP 导出已中止")
        output = _publish_h264(job["episode_id"], job["started_at"])
    except (ApiError, OSError, ValueError, RuntimeError, subprocess.SubprocessError) as error:
        message = error.message if isinstance(error, ApiError) else str(error)
        cancelled = _export_cancelled()
        with STATE_LOCK:
            job["export_status"] = "cancelled" if cancelled else "failed"
            job["export_error"] = None if cancelled else message
        if not cancelled:
            print_status_error("H264 自动转换", message)
    else:
        with STATE_LOCK:
            job["export_status"] = "completed"
            job["export_path"] = str(output)
        print_status_error("H264 自动转换", None)
    finally:
        _finish_export()


def _start_automatic_h264_export(job):
    if not _begin_export(job["episode_id"]):
        with STATE_LOCK:
            job["export_status"] = "failed"
            job["export_error"] = "已有导出任务在运行"
        print_status_error("H264 自动转换", job["export_error"])
        return
    with STATE_LOCK:
        job["export_status"] = "running"
        job["export_error"] = None
    try:
        threading.Thread(target=_automatic_h264_export, args=(job,), daemon=False).start()
    except RuntimeError as error:
        _finish_export()
        with STATE_LOCK:
            job["export_status"] = "failed"
            job["export_error"] = str(error)
        print_status_error("H264 自动转换", str(error))


def _watch_job(part, process, config_path, ready_file):
    returncode = process.wait()
    config_path.unlink(missing_ok=True)
    ready_file.unlink(missing_ok=True)
    if part == "recording" and PREVIEW_ROOT is not None:
        for side in ("left", "right"):
            try:
                (PREVIEW_ROOT / f"{side}.jpg").unlink(missing_ok=True)
            except OSError:
                pass
    completed_recording = None
    with STATE_LOCK:
        job = COLLECTION if part == "recording" else DEVICES
        if job and job["process"] is process:
            job["status"] = "completed" if returncode == 0 else "failed"
            job["returncode"] = returncode
            job["error"] = None if returncode == 0 else collection_exit_error(
                job["log"], returncode, "录制进程" if part == "recording" else "设备进程"
            )
            print_status_error("录制" if part == "recording" else "设备", job["error"])
            if part == "recording" and returncode == 0:
                completed_recording = job
    if completed_recording is not None:
        _start_automatic_h264_export(completed_recording)
    if part == "devices" and collection_active():
        stop_collection()


def start_collection(payload):
    if not START_LOCK.acquire(blocking=False):
        raise ApiError(HTTPStatus.CONFLICT, "录制任务正在启动")
    try:
        return _start_collection(payload, "recording")
    finally:
        START_LOCK.release()


def start_devices(payload):
    if not START_LOCK.acquire(blocking=False):
        raise ApiError(HTTPStatus.CONFLICT, "设备或录制任务正在启动")
    try:
        return _start_collection(payload, "devices")
    finally:
        START_LOCK.release()


def update_hotkey_settings(payload):
    with START_LOCK, STATE_LOCK:
        if DEVICES is not None:
            DEVICES["recording_payload"] = dict(payload)
    return {"saved": True}


def handle_controller_button(packet):
    global HOTKEY_AFTER_NS, HOTKEY_STATUS
    delete_target = None
    now = time.monotonic_ns()
    if not isinstance(packet, dict) or packet.get("button") not in ("X", "Y"):
        return
    stamp = packet.get("at_ns")
    if type(stamp) is not int or not 0 <= now - stamp <= 500_000_000:
        return
    if not START_LOCK.acquire(blocking=False):
        return  # Discard commands during a UI transition, never queue them for later.
    try:
        if stamp <= HOTKEY_AFTER_NS or not DEVICES or packet.get("token") != DEVICES.get("hotkey_token"):
            return
        if not devices_active(ready=True):
            return
        HOTKEY_AFTER_NS = now
        current = collection_status()
        if current["active"] and current["status"] != "running":
            raise ApiError(HTTPStatus.CONFLICT, "正在启动或保存，本次按键已忽略")
        if packet["button"] == "X":
            if current["active"]:
                _stop_collection()
                message = "X：正在结束录制并保存"
            else:
                _start_collection(dict(DEVICES["recording_payload"]), "recording")
                message = "X：正在开始录制"
        else:
            if current["active"]:
                raise ApiError(HTTPStatus.CONFLICT, "Y：请先结束录制并等待保存完成")
            if not current.get("episode_exists"):
                raise ApiError(HTTPStatus.CONFLICT, "Y：本次 UI 运行中没有可删除的上一段录制")
            delete_target = (current["episode_id"], current["session"])
        if delete_target is None:
            HOTKEY_STATUS = {"at_ns": now, "message": message, "error": False}
    except (ApiError, OSError, ValueError, RuntimeError, subprocess.SubprocessError) as error:
        message = error.message if isinstance(error, ApiError) else str(error)
        HOTKEY_STATUS = {"at_ns": now, "message": message, "error": True}
        print_status_error("手柄数采", message)
    finally:
        HOTKEY_AFTER_NS = time.monotonic_ns()
        START_LOCK.release()
    if delete_target is not None:
        episode_id, session = delete_target
        try:
            destination = delete_episode(DATASET_ROOT, episode_id, session=session, current_only=True)
            HOTKEY_STATUS = {"at_ns": now, "message": f"Y：{episode_id} 已移到回收站，可恢复：{destination}", "error": False}
        except (ApiError, OSError, ValueError, RuntimeError, subprocess.SubprocessError) as error:
            message = error.message if isinstance(error, ApiError) else str(error)
            HOTKEY_STATUS = {"at_ns": now, "message": message, "error": True}
            print_status_error("手柄数采", message)


def listen_controller_buttons(channel, stopped):
    channel.settimeout(0.2)
    while not stopped.is_set():
        try:
            packet = channel.recv(1024)
        except socket.timeout:
            continue
        try:
            handle_controller_button(json.loads(packet))
        except (ValueError, UnicodeError):
            continue


def request_robot_reset(payload):
    if any(payload.get(name) is not True for name in ("confirmed_estop", "confirmed_workspace_clear")):
        raise ApiError(HTTPStatus.BAD_REQUEST, "复位前必须确认物理急停可用、工作区无人和障碍物")
    if not RESET_LOCK.acquire(blocking=False):
        raise ApiError(HTTPStatus.CONFLICT, "机器人正在复位")
    try:
        if START_LOCK.locked() or collection_active() or devices_active() or any(
            process_running(name)
            for name in ("xr_marvin_teleop.cli.collection", "xr_marvin_teleop.cli.record")
        ):
            raise ApiError(HTTPStatus.CONFLICT, "设备或录制运行中不能单独复位机器人")
        if process_running("xr_marvin_teleop.cli.hardware"):
            raise ApiError(HTTPStatus.CONFLICT, "调试控制进程正在占用机器人")
        if not TELEOP_PYTHON.is_file():
            raise ApiError(HTTPStatus.SERVICE_UNAVAILABLE, "机器人复位脚本或 Python 环境缺失")
        environment = os.environ.copy()
        environment["PYTHONPATH"] = os.pathsep.join(
            filter(None, (str(SOURCE_ROOT), environment.get("PYTHONPATH")))
        )
        environment.pop("LD_PRELOAD", None)
        try:
            result = subprocess.run(
                (
                    str(TELEOP_PYTHON), "-m", RESET_MODULE,
                    "--enable-hardware", "--confirmed-estop",
                    "--confirmed-workspace-clear", "--confirmed-robot-model",
                    "M6S-Lite-CCS-680-B",
                ),
                cwd=PROJECT_ROOT,
                env=environment,
                capture_output=True,
                text=True,
                timeout=20,
                check=False,
            )
        except subprocess.TimeoutExpired as error:
            raise ApiError(HTTPStatus.GATEWAY_TIMEOUT, "机器人复位超时") from error
        if result.returncode != 0:
            detail = (result.stderr or result.stdout).strip().splitlines()
            raise ApiError(
                HTTPStatus.SERVICE_UNAVAILABLE,
                detail[-1] if detail else f"机器人复位失败（退出码 {result.returncode}）",
            )
        return {
            "completed": True,
            "target": "MARVIN_INITIAL_POSE_Q_RAD",
            "duration_seconds": 3,
        }
    finally:
        RESET_LOCK.release()


def _start_collection(payload, part):
    global COLLECTION, DEVICES, HOTKEY_AFTER_NS
    if part == "recording" and (EXPORT_LOCK.locked() or EXPORT_STATE["deleting"]):
        raise ApiError(HTTPStatus.CONFLICT, "正在导出，请等待完成后录制")
    if RESET_LOCK.locked():
        raise ApiError(HTTPStatus.CONFLICT, "机器人正在复位")
    if part == "devices" and devices_active():
        raise ApiError(HTTPStatus.CONFLICT, "设备已经启动")
    if part == "devices" and collection_active():
        raise ApiError(HTTPStatus.CONFLICT, "请先停止当前录制")
    if part == "recording" and collection_active():
        raise ApiError(HTTPStatus.CONFLICT, "已有录制任务正在运行")
    if part == "recording" and not devices_active(ready=True):
        raise ApiError(HTTPStatus.CONFLICT, "设备尚未就绪，请先启动设备")
    if part == "devices":
        for confirmation in ("confirmed_estop", "confirmed_joint_mapping", "confirmed_workspace_clear"):
            if payload.get(confirmation) is not True:
                raise ApiError(HTTPStatus.BAD_REQUEST, "必须逐项完成现场安全确认")
    task = str(payload.get("task", "")).strip()
    operator = str(payload.get("operator", "")).strip()
    robot = str(payload.get("robot_model", COLLECTION_SETTINGS["robot"]["model"])).strip()
    if not task or len(task) > 80 or not operator or len(operator) > 80 or not robot or len(robot) > 100:
        raise ApiError(HTTPStatus.BAD_REQUEST, "任务、采集员或机器人型号无效")
    selected_session, episode_id, episode_directory = None, None, None
    if part == "recording":
        selected_session = payload.get("session") or f"session_{time.strftime('%Y-%m-%d')}"
        directory = session_path(DATASET_ROOT, selected_session)
        if payload.get("session") and not directory.is_dir():
            raise ApiError(HTTPStatus.BAD_REQUEST, "指定的 Session 不存在")
        directory.mkdir(parents=True, exist_ok=True)
        episode_id = new_episode_id()
        episode_directory = directory / episode_id
    if not all(path.exists() for path in (DAS_CONFIG, DAS_SDK_ROOT, SCALE_CALIBRATION)):
        raise ApiError(HTTPStatus.SERVICE_UNAVAILABLE, "DAS SDK 或标定文件缺失")
    resolution = payload.get("camera_resolution")
    try:
        fps = int(payload["camera_fps"]) if "camera_fps" in payload else None
    except (TypeError, ValueError) as error:
        raise ApiError(HTTPStatus.BAD_REQUEST, "相机帧率无效") from error
    no_vision = payload.get("no_vision", not COLLECTION_SETTINGS["capture"]["vision_enabled"])
    for field in ("no_vision", "nsp_lateral"):
        if field in payload and type(payload[field]) is not bool:
            raise ApiError(HTTPStatus.BAD_REQUEST, f"{field} 必须是布尔值")
    environment = teleop_environment().copy()
    hotkey_token = secrets.token_hex(16) if part == "devices" else None
    if part == "devices" and HOTKEY_SOCKET_PATH:
        environment["FIELDNOTE_HOTKEY_SOCKET"] = HOTKEY_SOCKET_PATH
        environment["FIELDNOTE_HOTKEY_TOKEN"] = hotkey_token
    if part == "devices" and process_running("xr_marvin_teleop.cli.pico"):
        raise ApiError(HTTPStatus.CONFLICT, "外部 PICO 发布器仍在运行，请先停止以避免设备冲突")
    temporary = tempfile.NamedTemporaryFile(prefix="fieldnote-das-", suffix=".json", delete=False)
    temporary.close()
    config_path = Path(temporary.name)
    try:
        prepare_camera_config(DAS_CONFIG, config_path, resolution, fps)
    except Exception:
        config_path.unlink(missing_ok=True)
        raise
    preview_root = None
    if part == "recording" and not no_vision and COLLECTION_SETTINGS["preview"]["enabled"]:
        try:
            PREVIEW_ROOT.mkdir(mode=0o700, parents=True, exist_ok=True)
            if not PREVIEW_ROOT.is_symlink():
                PREVIEW_ROOT.chmod(0o700)
                for side in ("left", "right"):
                    (PREVIEW_ROOT / f"{side}.jpg").unlink(missing_ok=True)
                preview_root = PREVIEW_ROOT
        except OSError as error:
            print_status_error("相机预览", str(error))
    command = [
        str(TELEOP_PYTHON), "-m", COLLECTION_MODULE,
        "--part", part,
        "--config", str(COLLECTION_CONFIG_PATH), "--output-root", str(DATASET_ROOT),
        "--task", task, "--operator", operator, "--robot-model", robot,
        "--enable-hardware", "--confirmed-estop", "--confirmed-joint-mapping",
        "--das-config", str(config_path), "--das-sdk-root", str(DAS_SDK_ROOT),
        "--scale-calibration-path", str(SCALE_CALIBRATION),
    ]
    ready_file = config_path.with_suffix(".ready")
    ready_file.unlink(missing_ok=True)
    command += ["--ready-file", str(ready_file)]
    if part == "recording":
        command += ["--session", selected_session, "--episode-id", episode_id]
    if preview_root is not None:
        command += ["--preview-root", str(preview_root)]
    duration = payload.get("max_duration", COLLECTION_SETTINGS["capture"]["max_duration_s"])
    if part == "recording" and duration not in (None, ""):
        try:
            duration = float(duration)
        except (TypeError, ValueError) as error:
            config_path.unlink(missing_ok=True)
            raise ApiError(HTTPStatus.BAD_REQUEST, "最长时长无效") from error
        if not 0 < duration <= 86_400:
            config_path.unlink(missing_ok=True)
            raise ApiError(HTTPStatus.BAD_REQUEST, "最长时长必须在 0 到 24 小时内")
        command += ["--max-duration", str(duration)]
    elif part == "recording" and "max_duration" in payload:
        command.append("--unlimited-duration")
    command.append("--no-vision" if no_vision else "--vision")
    if "nsp_lateral" in payload:
        command.append("--nsp-lateral" if payload["nsp_lateral"] else "--no-nsp-lateral")
    log_path = LOG_ROOT / f"ui_{part}_{datetime.now():%Y%m%d_%H%M%S}.log"
    log_path.parent.mkdir(exist_ok=True)
    with log_path.open("ab", buffering=0) as log:
        try:
            process = subprocess.Popen(
                command,
                cwd=PROJECT_ROOT,
                env=environment,
                stdout=log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
        except OSError:
            config_path.unlink(missing_ok=True)
            ready_file.unlink(missing_ok=True)
            raise
    job = {
        "process": process,
        "hotkey_token": hotkey_token,
        "recording_payload": dict(payload),
        "session": selected_session,
        "episode_id": episode_id,
        "episode_path": str(episode_directory) if episode_directory is not None else None,
        "task": task,
        "status": "starting",
        "started_at": time.time(),
        "log": log_path,
        "ready_file": ready_file,
        "vision_enabled": not no_vision,
        "max_duration": duration,
        "export_status": None,
        "export_error": None,
        "export_path": None,
    }
    with STATE_LOCK:
        if part == "recording":
            COLLECTION = job
            DEVICES["recording_payload"] = dict(payload)
        else:
            DEVICES = job
        HOTKEY_AFTER_NS = time.monotonic_ns()
    print_status_error("录制" if part == "recording" else "设备", None)
    threading.Thread(
        target=_watch_job,
        args=(part, process, config_path, ready_file),
        daemon=True,
    ).start()
    return collection_status() if part == "recording" else device_status()


def stop_collection():
    with START_LOCK:
        return _stop_collection()


def _stop_collection():
    global HOTKEY_AFTER_NS
    with STATE_LOCK:
        job = COLLECTION
        if not job or job["process"].poll() is not None:
            raise ApiError(HTTPStatus.CONFLICT, "当前没有活动采集")
        status = _job_status(job)
        if status["status"] in ("stopping", "saving"):
            return status
        job["status"] = "stopping"
        HOTKEY_AFTER_NS = time.monotonic_ns()
        process = job["process"]
    try:
        os.killpg(process.pid, signal.SIGINT)
    except ProcessLookupError:
        pass
    return collection_status()


def stop_devices():
    with START_LOCK:
        return _stop_devices()


def _stop_devices():
    if collection_active():
        raise ApiError(HTTPStatus.CONFLICT, "请先停止录制并等待保存完成")
    with STATE_LOCK:
        job = DEVICES
        if not job or job["process"].poll() is not None:
            raise ApiError(HTTPStatus.CONFLICT, "设备尚未启动")
        job["status"] = "stopping"
        process = job["process"]
    try:
        os.killpg(process.pid, signal.SIGINT)
    except ProcessLookupError:
        pass
    return device_status()


def restart_pico():
    with PICO_LOCK:
        return _restart_pico()


def _restart_pico():
    if collection_active() or devices_active():
        raise ApiError(HTTPStatus.CONFLICT, "设备运行中不能重连 PICO")
    if not ROBOTICS_SERVICE_SCRIPT.is_file():
        raise ApiError(HTTPStatus.SERVICE_UNAVAILABLE, "Robotics Service 启动脚本不存在")
    service_started = False
    service_log = LOG_ROOT / "ui_robotics_service.log"
    service_log.parent.mkdir(exist_ok=True)
    ports = robotics_service_ports()
    if pico_ports_ready(ports):
        return {**pico_status(), "service_started": False}
    if not process_running("RoboticsServiceProcess"):
        with service_log.open("ab", buffering=0) as log:
            launcher = subprocess.Popen(
                ("bash", str(ROBOTICS_SERVICE_SCRIPT)),
                cwd=ROBOTICS_SERVICE_SCRIPT.parent,
                stdout=log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
        service_started = True
        try:
            returncode = launcher.wait(timeout=3)
        except subprocess.TimeoutExpired:
            returncode = None
        if returncode not in (None, 0):
            raise ApiError(HTTPStatus.SERVICE_UNAVAILABLE, f"Robotics Service 启动失败，日志：{service_log}")
    deadline = time.monotonic() + 10
    while time.monotonic() < deadline:
        ports = robotics_service_ports()
        if pico_ports_ready(ports):
            break
        time.sleep(0.25)
    else:
        raise ApiError(HTTPStatus.SERVICE_UNAVAILABLE, "PICO 服务端口未在 10 秒内就绪")
    return {**pico_status(), "service_started": service_started}


class Handler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(UI_ROOT), **kwargs)

    def parse_request(self):
        if not super().parse_request():
            return False
        if self.command in ("POST", "DELETE", "PUT", "PATCH"):
            host = self.headers.get("Host", "")
            try:
                hostname = urlsplit("http://" + host).hostname
            except ValueError:
                hostname = None
            allowed_hosts = {"localhost", "127.0.0.1", "::1", self.server.server_name,
                             self.connection.getsockname()[0]}
            if (hostname not in allowed_hosts
                    or self.headers.get("Origin") != "http://" + host):
                self.send_json({"error": "请求来源无效"}, HTTPStatus.FORBIDDEN)
                return False
            if self.headers.get_content_type() != "application/json":
                self.send_json({"error": "请求必须使用 application/json"}, HTTPStatus.UNSUPPORTED_MEDIA_TYPE)
                return False
        return True

    def end_headers(self):
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Referrer-Policy", "no-referrer")
        if not urlsplit(self.path).path.startswith("/api/"):
            self.send_header("Cache-Control", "no-store")
        super().end_headers()

    def send_json(self, payload, status=HTTPStatus.OK):
        body = json.dumps(payload, ensure_ascii=False).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def send_preview(self, side):
        try:
            body = (PREVIEW_ROOT / f"{side}.jpg").read_bytes() if PREVIEW_ROOT is not None else b""
        except OSError:
            body = b""
        if len(body) < 4 or not body.startswith(b"\xff\xd8") or not body.endswith(b"\xff\xd9"):
            self.send_response(HTTPStatus.NO_CONTENT)
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            return
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def send_mcap_export(self, episode_ids, variant="av1"):
        _episode_id, path = mcap_export_files(DATASET_ROOT, episode_ids, variant)[0]
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "application/octet-stream")
        self.send_header("Content-Disposition", f'attachment; filename="{path.name}"')
        self.send_header("Content-Length", str(path.stat().st_size))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        try:
            with path.open("rb") as source:
                shutil.copyfileobj(source, self.wfile, length=1024 * 1024)
        except (BrokenPipeError, ConnectionResetError, OSError):
            self.close_connection = True

    def read_body(self):
        try:
            length = int(self.headers.get("Content-Length", "0"))
        except ValueError as error:
            raise ApiError(HTTPStatus.BAD_REQUEST, "Content-Length 无效") from error
        if length > 65_536:
            raise ApiError(HTTPStatus.REQUEST_ENTITY_TOO_LARGE, "请求体过大")
        try:
            body = json.loads(self.rfile.read(length) or b"{}")
            if not isinstance(body, dict):
                raise ApiError(HTTPStatus.BAD_REQUEST, "JSON 请求体必须是对象")
            return body
        except json.JSONDecodeError as error:
            raise ApiError(HTTPStatus.BAD_REQUEST, "JSON 请求体无效") from error

    def handle_api(self, action):
        try:
            return action()
        except ApiError as error:
            print_status_error(f"API {self.command} {urlsplit(self.path).path}", error.message)
            self.send_json({"error": error.message}, error.status)
        except (ValueError, RuntimeError) as error:
            self.send_json({"error": str(error)}, HTTPStatus.BAD_REQUEST if isinstance(error, ValueError) else HTTPStatus.CONFLICT)
        except FileNotFoundError as error:
            self.send_json({"error": str(error)}, HTTPStatus.NOT_FOUND)
        except (OSError, subprocess.SubprocessError) as error:
            print_status_error(f"API {self.command} {urlsplit(self.path).path}", str(error))
            self.send_json({"error": str(error)}, HTTPStatus.INTERNAL_SERVER_ERROR)

    def do_GET(self):
        request = urlsplit(self.path)
        if request.path == "/api/sessions":
            return self.handle_api(lambda: self.send_json({"sessions": [
                session_record(session_path(DATASET_ROOT, p.name))
                for p in sorted(DATASET_ROOT.glob("session_*"), reverse=True)
                if p.is_dir() and not p.is_symlink() and SESSION_ID.fullmatch(p.name)
            ]}))
        if request.path == "/api/collection/config":
            return self.handle_api(lambda: self.send_json({
                "robot": COLLECTION_SETTINGS["robot"], "capture": COLLECTION_SETTINGS["capture"],
                "preview": COLLECTION_SETTINGS["preview"], "cameras": read_json(DAS_CONFIG),
            }))
        preview = re.fullmatch(r"/api/preview/(left|right)\.jpg", request.path)
        if preview:
            return self.send_preview(preview.group(1))
        if request.path == "/api/exports/mcap":
            query = parse_qs(request.query)
            return self.handle_api(lambda: self.send_mcap_export(query.get("episode", []), query.get("format", ["av1"])[0]))
        if request.path == "/api/status":
            usage = shutil.disk_usage(DATASET_ROOT if DATASET_ROOT.exists() else PROJECT_ROOT)
            return self.send_json({
                "devices": device_status(),
                "collection": collection_status(),
                "hotkeys": HOTKEY_STATUS,
                "disk_free_bytes": usage.free,
            })
        if request.path == "/api/devices/pico":
            return self.send_json(pico_status())
        if request.path == "/api/devices/cameras/formats":
            return self.send_json(camera_formats())
        if request.path == "/api/devices/hardware":
            return self.send_json(hardware_status())
        if request.path == "/api/episodes":
            query = parse_qs(request.query)
            records = list_episodes()
            if query.get("q"):
                term = query["q"][0].casefold()
                records = [item for item in records if term in f'{item["id"]} {item["task"]}'.casefold()]
            if query.get("status"):
                records = [item for item in records if item["status"] == query["status"][0]]
            return self.send_json({"episodes": records, "count": len(records)})
        return super().do_GET()

    def do_POST(self):
        path = urlsplit(self.path).path
        if path == "/api/exports/mcap":
            return self.handle_api(lambda: self.send_json(prepare_mcap_export(self.read_body())))
        if path == "/api/collection/hotkeys":
            return self.handle_api(lambda: self.send_json(update_hotkey_settings(self.read_body())))
        session_match = re.fullmatch(r"/api/sessions/(session_[A-Za-z0-9_-]{1,100})/rename", path)
        if path == "/api/sessions" or session_match:
            return self.handle_api(lambda: self.send_json(save_session(
                DATASET_ROOT, self.read_body().get("name"), session_match.group(1) if session_match else None,
            )))
        review_match = re.fullmatch(r"/api/episodes/(episode_\d{6}_[0-9a-f]{8})/review", path)
        if review_match:
            def review():
                body = self.read_body()
                with START_LOCK:
                    current = collection_status()
                    if current["active"] and current.get("episode_id") == review_match.group(1):
                        raise ApiError(HTTPStatus.CONFLICT, "本段仍在录制或保存，请等待结束")
                    saved = save_review(DATASET_ROOT, body.get("session"), review_match.group(1), body.get("result"))
                self.send_json(saved)
            return self.handle_api(review)
        open_match = re.fullmatch(r"/api/episodes/(episode_\d{6}_[0-9a-f]{8})/open", path)
        if open_match:
            return self.handle_api(lambda: self.send_json(
                {"opened": open_match.group(1), "path": str(open_episode_directory(DATASET_ROOT, open_match.group(1)))},
                HTTPStatus.ACCEPTED,
            ))
        if path == "/api/episodes":
            return self.handle_api(lambda: self.send_json(start_collection(self.read_body()), HTTPStatus.ACCEPTED))
        if path == "/api/episodes/active/stop":
            return self.handle_api(lambda: self.send_json(stop_collection(), HTTPStatus.ACCEPTED))
        if path == "/api/devices/start":
            return self.handle_api(lambda: self.send_json(start_devices(self.read_body()), HTTPStatus.ACCEPTED))
        if path == "/api/devices/stop":
            return self.handle_api(lambda: self.send_json(stop_devices(), HTTPStatus.ACCEPTED))
        if path == "/api/robot/reset":
            return self.handle_api(lambda: self.send_json(request_robot_reset(self.read_body())))
        if path == "/api/devices/pico/reconnect":
            return self.handle_api(lambda: self.send_json(restart_pico(), HTTPStatus.ACCEPTED))
        self.send_json({"error": "接口不存在"}, HTTPStatus.NOT_FOUND)

    def do_DELETE(self):
        path = urlsplit(self.path).path
        match = re.fullmatch(r"/api/episodes/(episode_\d{6}_[0-9a-f]{8})", path)
        if not match:
            return self.send_json({"error": "接口不存在"}, HTTPStatus.NOT_FOUND)

        def delete():
            if self.read_body().get("confirm") is not True:
                raise ApiError(HTTPStatus.BAD_REQUEST, "删除前必须明确确认")
            destination = delete_episode(DATASET_ROOT, match.group(1))
            self.send_json({"deleted": match.group(1), "trash": destination.name})

        return self.handle_api(delete)

    def log_message(self, format, *args):
        if self.path.startswith("/api/preview/"):
            return
        print(f"[{self.log_date_time_string()}] {format % args}")


def self_test():
    sample = """[0]: 'MJPG'\n Size: Discrete 640x480\n  Interval: Discrete 0.017s (60.000 fps)\n  Interval: Discrete 0.033s (30.000 fps)"""
    assert parse_v4l2_formats(sample) == {"640x480": {60, 30}}
    assert parse_robotics_service_ports("LISTEN *:63901\nLISTEN [::ffff:127.0.0.1]:60061\n") == [60061, 63901]
    assert pico_ports_ready([60061, 63901]) and not pico_ports_ready([63901])
    assert parse_pico_clients("ESTAB 0 0 192.168.1.100:63901 192.168.1.42:51234") == ["192.168.1.42"]
    assert not parse_pico_clients("ESTAB 0 0 127.0.0.1:63901 127.0.0.1:51234")
    assert ping_host(MARVIN_IP, lambda *args, **kwargs: subprocess.CompletedProcess(args[0], 0)) == (True, None)
    assert reachable_hosts(["up", "down"], lambda address: (address == "up", None)) == ["up"]
    LAST_STATUS_ERRORS["self-test"] = "same"
    assert not print_status_error("self-test", "same")
    assert not print_status_error("self-test", None) and "self-test" not in LAST_STATUS_ERRORS
    probe_name = "fieldnote_process_probe.py"
    decoy = subprocess.Popen((str(TELEOP_PYTHON), "-c", "import time; time.sleep(5)", f"prefix-{probe_name}"))
    try:
        assert not process_running(probe_name)
    finally:
        decoy.terminate()
        decoy.wait()
    probe = subprocess.Popen((str(TELEOP_PYTHON), "-c", "import time; time.sleep(5)", probe_name))
    try:
        assert process_running(probe_name)
    finally:
        probe.terminate()
        probe.wait()
    with tempfile.TemporaryDirectory() as temporary:
        root = Path(temporary) / "dataset"
        ready_file = Path(temporary) / "job.ready"
        process = type("RunningProcess", (), {"poll": lambda self: None})()
        job = {
            "process": process,
            "task": "test",
            "status": "starting",
            "started_at": 0,
            "log": Path(temporary) / "job.log",
            "ready_file": ready_file,
        }
        assert _job_status(job)["status"] == "starting"
        ready_file.touch()
        assert _job_status(job)["status"] == "running"
        job["status"] = "stopping"
        assert _job_status(job)["status"] == "stopping"
        episode = root / "session_2026-09-03" / "episode_120000_deadbeef"
        episode.mkdir(parents=True)
        (episode / "metadata.json").write_text("{}", encoding="utf-8")
        moved = move_episode_to_trash(root, episode.name)
        assert moved.is_dir() and not episode.exists() and moved.parent == root / ".trash"
        blocked = root / "session_2026-09-03" / "episode_120001_cafebabe"
        blocked.mkdir()
        (blocked / "data").mkdir()
        (blocked / "data" / "data_0.mcap").write_bytes(b"mcap")
        assert episode_record(blocked)["session"] == "session_2026-09-03"
        assert episode_record(blocked)["size_bytes"] == 4
        (blocked / "final").mkdir()
        mjpeg = blocked / "final" / f"{blocked.name}.mjpeg.mcap"
        mjpeg.write_bytes(b"mjpeg")
        try:
            mcap_export_files(root, [blocked.name])
            raise AssertionError("raw/MJPEG MCAP was exported without AV1")
        except ApiError as error:
            assert error.status == HTTPStatus.UNPROCESSABLE_ENTITY
        av1 = blocked / "final" / f"{blocked.name}.av1.mcap"
        av1.symlink_to(mjpeg)
        try:
            mcap_export_files(root, [blocked.name])
            raise AssertionError("symlink AV1 export was accepted")
        except ApiError as error:
            assert error.status == HTTPStatus.BAD_REQUEST
        av1.unlink()
        av1.write_bytes(b"av1")
        assert mcap_export_files(root, [blocked.name]) == [(blocked.name, av1)]
        packaged_mcap = root / "session_2026-09-03" / "data/chunk-000/episode_000000.mcap"
        packaged_mcap.parent.mkdir(parents=True)
        packaged_meta = Path(temporary) / "meta.json"
        packaged_data = Path(temporary) / "data.parquet"
        packaged_meta.write_text(
            '{"dataset_format":"lerobot","episode_id":"episode_120002_01234567",'
            '"session":"session_2026-09-03","features":{"observation.state":{"dtype":"float32"}},'
            '"video_paths":{},"source_metadata":{"started_at_ns":1000000000}}', encoding="utf-8"
        )
        packaged_data.write_bytes(b"PAR1")
        write_episode_mcap(packaged_mcap, (
            ("meta/meta.json", "application/json", packaged_meta),
            ("data/data.parquet", "application/vnd.apache.parquet", packaged_data),
        ))
        try:
            mcap_export_files(root, ["episode_120002_01234567"])
            raise AssertionError("legacy attachment MCAP was exported as AV1")
        except ApiError as error:
            assert error.status == HTTPStatus.UNPROCESSABLE_ENTITY
        assert episode_record(packaged_mcap)["size_bytes"] == packaged_mcap.stat().st_size
        try:
            mcap_export_files(root, [episode.name, blocked.name])
            raise AssertionError("multiple Episodes were accepted by one export")
        except ApiError as error:
            assert error.status == HTTPStatus.BAD_REQUEST
        launches = []
        opened = open_episode_directory(
            root, blocked.name, opener="/usr/bin/xdg-open",
            launch=lambda command, **options: launches.append((command, options)),
        )
        assert opened == blocked and launches[0][0] == ("/usr/bin/xdg-open", str(blocked))
        try:
            move_episode_to_trash(root, blocked.name, collection_active=True)
            raise AssertionError("active collection deletion was allowed")
        except ApiError as error:
            assert error.status == HTTPStatus.CONFLICT and blocked.is_dir()
        source = Path(temporary) / "source.json"
        output = Path(temporary) / "output.json"
        source.write_text('{"left": {}, "right": {}}', encoding="utf-8")
        prepare_camera_config(source, output, "1600x1296", 60)
        configured = read_json(output)
        assert all(configured[side]["camera_fps"] == 60 for side in ("left", "right"))
        try:
            prepare_camera_config(source, output, "640x480", 30)
            raise AssertionError("30 FPS camera setting was accepted")
        except ApiError as error:
            assert error.status == HTTPStatus.BAD_REQUEST
        failure_log = Path(temporary) / "collection.log"
        failure_log.write_text("Traceback\nModuleNotFoundError: missing driver\n", encoding="utf-8")
        assert collection_exit_error(failure_log, 1).endswith("ModuleNotFoundError: missing driver")
        duplicate_metadata = read_json(packaged_meta)
        duplicate_metadata["episode_id"] = blocked.name
        packaged_meta.write_text(json.dumps(duplicate_metadata), encoding="utf-8")
        write_episode_mcap(packaged_mcap.parent / "episode_000001.mcap", (
            ("meta/meta.json", "application/json", packaged_meta),
            ("data/data.parquet", "application/vnd.apache.parquet", packaged_data),
        ))
        assert mcap_export_files(root, [blocked.name]) == [(blocked.name, av1)]
    print("Server self-check passed")


def main():
    global COLLECTION_CONFIG_PATH, COLLECTION_SETTINGS, DATASET_ROOT, MARVIN_IP, PREVIEW_ROOT, DAS_CONFIG, DAS_SDK_ROOT, SCALE_CALIBRATION
    global HOTKEY_SOCKET_PATH
    parser = argparse.ArgumentParser(description="Serve the Fieldnote data collection console")
    parser.add_argument("--collection-config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=4173)
    parser.add_argument("--self-test", action="store_true")
    arguments = parser.parse_args()
    COLLECTION_CONFIG_PATH = arguments.collection_config.expanduser().resolve()
    COLLECTION_SETTINGS = validate_config(load_config(COLLECTION_CONFIG_PATH))
    DATASET_ROOT = Path(COLLECTION_SETTINGS["paths"]["output_root"])
    MARVIN_IP = COLLECTION_SETTINGS["robot"]["ip"]
    PREVIEW_ROOT = Path(COLLECTION_SETTINGS["preview"]["root"]) if COLLECTION_SETTINGS["preview"]["root"] else None
    DAS_CONFIG = Path(COLLECTION_SETTINGS["paths"]["das_config"])
    DAS_SDK_ROOT = Path(COLLECTION_SETTINGS["paths"]["das_sdk_root"])
    SCALE_CALIBRATION = Path(COLLECTION_SETTINGS["paths"]["scale_calibration"])
    if arguments.self_test:
        return self_test()
    server = ThreadingHTTPServer((arguments.host, arguments.port), Handler)
    hotkey_directory = tempfile.TemporaryDirectory(prefix="fieldnote-keys-")
    HOTKEY_SOCKET_PATH = str(Path(hotkey_directory.name) / "buttons.sock")
    channel = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    channel.bind(HOTKEY_SOCKET_PATH)
    os.chmod(HOTKEY_SOCKET_PATH, 0o600)
    stopped = threading.Event()
    listener = threading.Thread(target=listen_controller_buttons, args=(channel, stopped), daemon=True)
    listener.start()
    print(f"Fieldnote: http://{arguments.host}:{arguments.port}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping Fieldnote...")
    finally:
        stopped.set()
        listener.join()
        channel.close()
        hotkey_directory.cleanup()
        server.server_close()
        if collection_active():
            stop_collection()
            COLLECTION["process"].wait()
        if devices_active():
            stop_devices()


if __name__ == "__main__":
    main()
