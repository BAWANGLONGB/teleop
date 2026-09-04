#!/usr/bin/env python3
"""Fieldnote UI server: static files plus the minimal local control API."""

import argparse
import json
import os
import re
import shutil
import signal
import subprocess
import sys
import tarfile
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
PROJECT_ROOT = WORKSPACE / "xr-marvin-teleop"
CONDA_SETUP = WORKSPACE / ".miniconda-xr" / "etc" / "profile.d" / "conda.sh"
ROS_BASE_SETUP = Path("/opt/ros/humble/setup.bash")
ROS_SETUP = PROJECT_ROOT / "ros2_ws" / "install" / "setup.bash"
DATASET_ROOT = PROJECT_ROOT / "dataset"
COLLECTION_SCRIPT = PROJECT_ROOT / "scripts" / "data" / "run_collection.py"
RESET_SCRIPT = PROJECT_ROOT / "scripts" / "hardware" / "reset_marvin_hardware.py"
TELEOP_PYTHON = WORKSPACE / ".miniconda-xr" / "envs" / "Teleop" / "bin" / "python"
ROBOTICS_SERVICE_SCRIPT = Path("/opt/apps/roboticsservice/runService.sh")
ROBOTICS_SERVICE_PORTS = (63901, 60061)
MARVIN_IP = "192.168.1.190"
PREVIEW_ROOT = Path("/dev/shm") / f"fieldnote-preview-{os.getuid()}"
DAS_CONFIG = PROJECT_ROOT / "config" / "das_gripper.example.json"
DAS_SDK_ROOT = WORKSPACE / "gen_finger_con_python_sdk_release"
SCALE_CALIBRATION = PROJECT_ROOT / "logs" / "marvin_scale_calibration.json"
EPISODE_RE = re.compile(r"episode_\d{6}_[0-9a-f]{8}\Z")
KNOWN_CAMERA_FORMATS = {"640x480": (60,), "1600x1296": (60,)}
STATE_LOCK = threading.Lock()
START_LOCK = threading.Lock()
RESET_LOCK = threading.Lock()
PICO_LOCK = threading.Lock()
ERROR_LOCK = threading.Lock()
COLLECTION = None
LAST_STATUS_ERRORS = {}


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
    missing = [str(path) for path in (CONDA_SETUP, ROS_BASE_SETUP, ROS_SETUP, TELEOP_PYTHON) if not path.is_file()]
    if missing:
        raise ApiError(HTTPStatus.SERVICE_UNAVAILABLE, f"遥操环境文件缺失：{', '.join(missing)}")
    try:
        result = subprocess.run(
            (
                "bash", "-c",
                'source "$1" && conda activate Teleop && source "$2" && source "$3" && unset LD_PRELOAD && env -0',
                "fieldnote", str(CONDA_SETUP), str(ROS_BASE_SETUP), str(ROS_SETUP),
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
    environment["PYTHONPATH"] = os.pathsep.join(filter(None, (str(PROJECT_ROOT), environment.get("PYTHONPATH"))))
    environment.pop("LD_PRELOAD", None)
    return environment


def read_json(path):
    with path.open(encoding="utf-8") as file:
        return json.load(file)


def episode_path(dataset_root, episode_id):
    if not EPISODE_RE.fullmatch(episode_id):
        raise ApiError(HTTPStatus.BAD_REQUEST, "Episode ID 格式无效")
    root = dataset_root.resolve()
    matches = [path for path in root.glob(f"session_*/{episode_id}") if path.is_dir()]
    if len(matches) != 1:
        raise ApiError(HTTPStatus.NOT_FOUND, "Episode 不存在")
    path = matches[0]
    if path.is_symlink() or root not in path.resolve().parents:
        raise ApiError(HTTPStatus.BAD_REQUEST, "Episode 路径无效")
    return path


def move_episode_to_trash(dataset_root, episode_id, collection_active=False):
    if collection_active:
        raise ApiError(HTTPStatus.CONFLICT, "采集中不能删除 Episode")
    source = episode_path(dataset_root, episode_id)
    trash = dataset_root.resolve() / ".trash"
    trash.mkdir(exist_ok=True)
    destination = trash / f"{episode_id}_{time.time_ns()}"
    source.replace(destination)
    return destination


def open_episode_directory(dataset_root, episode_id, opener=None, launch=None):
    path = episode_path(dataset_root, episode_id)
    opener = opener or shutil.which("xdg-open") or shutil.which("gio")
    if not opener:
        raise ApiError(HTTPStatus.SERVICE_UNAVAILABLE, "未找到系统文件管理器")
    command = (opener, "open", str(path)) if Path(opener).name == "gio" else (opener, str(path))
    (launch or subprocess.Popen)(command, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
    return path


def mcap_export_files(dataset_root, episode_ids):
    episode_ids = list(dict.fromkeys(episode_ids))
    if not episode_ids or len(episode_ids) > 100:
        raise ApiError(HTTPStatus.BAD_REQUEST, "请选择 1 到 100 段 Episode")
    files = []
    for episode_id in episode_ids:
        episode = episode_path(dataset_root, episode_id)
        data = episode / "data"
        if data.is_symlink():
            raise ApiError(HTTPStatus.BAD_REQUEST, f"{episode_id} 数据路径无效")
        episode_files = [
            path for path in sorted(data.glob("*.mcap"))
            if path.is_file() and not path.is_symlink() and episode in path.resolve().parents
        ]
        if not episode_files:
            raise ApiError(HTTPStatus.UNPROCESSABLE_ENTITY, f"{episode_id} 没有可导出的标准化 MCAP")
        files.extend((episode_id, path) for path in episode_files)
    return files


def episode_record(path):
    metadata_path = path / "metadata.json"
    manifest_path = path / "manifest.json"
    metadata = read_json(metadata_path) if metadata_path.is_file() else {}
    manifest = read_json(manifest_path) if manifest_path.is_file() else {}
    started = int(metadata.get("started_at_ns", 0) or 0)
    ended = int(metadata.get("ended_at_ns", 0) or 0)
    duration = max(0, (ended - started) // 1_000_000_000) if ended else 0
    size = sum(item.stat().st_size for item in path.rglob("*") if item.is_file() and not item.is_symlink())
    status = manifest.get("status") or metadata.get("status", "unknown")
    quality = manifest.get("quality")
    if not isinstance(quality, (int, float)):
        quality = None
    topics = {
        topic
        for bag in manifest.get("bags", {}).values()
        if isinstance(bag, dict)
        for topic in bag
    }
    modalities = [
        label for label, present in (
            ("关节", any("marvin" in topic for topic in topics)),
            ("PICO", "/raw/pico/frame" in topics),
            ("触觉", any("tactile" in topic for topic in topics)),
            ("视觉", any("image" in topic for topic in topics)),
        ) if present
    ]
    return {
        "id": metadata.get("episode_id", path.name),
        "task": metadata.get("task", "—"),
        "operator": metadata.get("operator", "—"),
        "robot_model": metadata.get("robot_model", "—"),
        "status": status,
        "quality": quality,
        "duration_seconds": duration,
        "size_bytes": size,
        "created_at": datetime.fromtimestamp(started / 1e9).astimezone().isoformat() if started else "",
        "modalities": modalities,
    }


def list_episodes(dataset_root=DATASET_ROOT):
    records = []
    if dataset_root.is_dir():
        for path in dataset_root.glob("session_*/episode_*"):
            if path.is_dir() and EPISODE_RE.fullmatch(path.name):
                try:
                    records.append(episode_record(path))
                except (OSError, ValueError, json.JSONDecodeError):
                    continue
    return sorted(records, key=lambda item: item["created_at"], reverse=True)


def prepare_camera_config(source, destination, resolution, fps):
    if resolution not in KNOWN_CAMERA_FORMATS or fps not in KNOWN_CAMERA_FORMATS[resolution]:
        raise ApiError(HTTPStatus.BAD_REQUEST, "相机分辨率与帧率组合不受支持")
    config = read_json(source)
    for side in ("left", "right"):
        if not isinstance(config.get(side), dict):
            raise ApiError(HTTPStatus.INTERNAL_SERVER_ERROR, f"DAS 配置缺少 {side}")
        config[side]["camera_resolution"] = resolution
        config[side]["camera_fps"] = fps
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


def collection_status():
    with STATE_LOCK:
        job = COLLECTION
        if not job:
            return {"active": False, "status": "idle"}
        return {
            "active": job["process"].poll() is None,
            "status": job["status"],
            "task": job["task"],
            "started_at": job["started_at"],
            "returncode": job.get("returncode"),
            "error": job.get("error"),
            "log": str(job["log"]),
            "max_duration": job.get("max_duration"),
        }


def collection_active():
    return collection_status()["active"]


def collection_exit_error(log_path, returncode):
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
    message = f"遥操进程异常退出（退出码 {returncode}）"
    return f"{message}：{detail[:600]}" if detail else message


def _watch_collection(process, config_path):
    returncode = process.wait()
    config_path.unlink(missing_ok=True)
    for side in ("left", "right"):
        try:
            (PREVIEW_ROOT / f"{side}.jpg").unlink(missing_ok=True)
        except OSError:
            pass
    with STATE_LOCK:
        if COLLECTION and COLLECTION["process"] is process:
            COLLECTION["status"] = "completed" if returncode == 0 else "failed"
            COLLECTION["returncode"] = returncode
            COLLECTION["error"] = None if returncode == 0 else collection_exit_error(COLLECTION["log"], returncode)
            print_status_error("遥操", COLLECTION["error"])


def start_collection(payload):
    if not START_LOCK.acquire(blocking=False):
        raise ApiError(HTTPStatus.CONFLICT, "采集任务正在启动")
    try:
        return _start_collection(payload)
    finally:
        START_LOCK.release()


def request_robot_reset():
    if not RESET_LOCK.acquire(blocking=False):
        raise ApiError(HTTPStatus.CONFLICT, "机器人正在复位")
    try:
        if START_LOCK.locked() or collection_active() or any(
            process_running(name)
            for name in ("run_collection.py", "record_episode.py")
        ):
            raise ApiError(HTTPStatus.CONFLICT, "数采过程中不能单独复位机器人")
        if process_running("teleop_marvin_hardware.py"):
            raise ApiError(HTTPStatus.CONFLICT, "调试控制进程正在占用机器人")
        if not RESET_SCRIPT.is_file() or not TELEOP_PYTHON.is_file():
            raise ApiError(HTTPStatus.SERVICE_UNAVAILABLE, "机器人复位脚本或 Python 环境缺失")
        environment = os.environ.copy()
        environment["PYTHONPATH"] = os.pathsep.join(
            filter(None, (str(PROJECT_ROOT), environment.get("PYTHONPATH")))
        )
        environment.pop("LD_PRELOAD", None)
        try:
            result = subprocess.run(
                (
                    str(TELEOP_PYTHON), str(RESET_SCRIPT),
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


def _start_collection(payload):
    global COLLECTION
    if RESET_LOCK.locked():
        raise ApiError(HTTPStatus.CONFLICT, "机器人正在复位")
    if collection_active():
        raise ApiError(HTTPStatus.CONFLICT, "已有采集任务正在运行")
    for confirmation in ("confirmed_estop", "confirmed_joint_mapping", "confirmed_workspace_clear"):
        if payload.get(confirmation) is not True:
            raise ApiError(HTTPStatus.BAD_REQUEST, "必须逐项完成现场安全确认")
    task = str(payload.get("task", "")).strip()
    operator = str(payload.get("operator", "")).strip()
    robot = str(payload.get("robot_model", "")).strip()
    if not task or len(task) > 80 or not operator or len(operator) > 80 or not robot or len(robot) > 100:
        raise ApiError(HTTPStatus.BAD_REQUEST, "任务、采集员或机器人型号无效")
    if not all(path.exists() for path in (COLLECTION_SCRIPT, DAS_CONFIG, DAS_SDK_ROOT, SCALE_CALIBRATION)):
        raise ApiError(HTTPStatus.SERVICE_UNAVAILABLE, "采集脚本、DAS SDK 或标定文件缺失")
    resolution = str(payload.get("camera_resolution", "640x480"))
    try:
        fps = int(payload.get("camera_fps", 60))
    except (TypeError, ValueError) as error:
        raise ApiError(HTTPStatus.BAD_REQUEST, "相机帧率无效") from error
    if resolution not in KNOWN_CAMERA_FORMATS or fps not in KNOWN_CAMERA_FORMATS[resolution]:
        raise ApiError(HTTPStatus.BAD_REQUEST, "相机分辨率与帧率组合不受支持")
    environment = teleop_environment().copy()
    if process_running("publish_pico.py"):
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
    if payload.get("no_vision") is not True:
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
        str(TELEOP_PYTHON if TELEOP_PYTHON.is_file() else Path(sys.executable)), str(COLLECTION_SCRIPT),
        "--task", task, "--operator", operator, "--robot-model", robot,
        "--enable-hardware", "--confirmed-estop", "--confirmed-joint-mapping",
        "--das-config", str(config_path), "--das-sdk-root", str(DAS_SDK_ROOT),
        "--scale-calibration-path", str(SCALE_CALIBRATION),
    ]
    if preview_root is not None:
        command += ["--preview-root", str(preview_root)]
    duration = payload.get("max_duration")
    if duration not in (None, ""):
        try:
            duration = float(duration)
        except (TypeError, ValueError) as error:
            config_path.unlink(missing_ok=True)
            raise ApiError(HTTPStatus.BAD_REQUEST, "最长时长无效") from error
        if not 0 < duration <= 86_400:
            config_path.unlink(missing_ok=True)
            raise ApiError(HTTPStatus.BAD_REQUEST, "最长时长必须在 0 到 24 小时内")
        command += ["--max-duration", str(duration)]
    if payload.get("no_vision") is True:
        command.append("--no-vision")
    if payload.get("nsp_lateral") is True:
        command.append("--nsp-lateral")
    log_path = PROJECT_ROOT / "logs" / f"ui_collection_{datetime.now():%Y%m%d_%H%M%S}.log"
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
            raise
    with STATE_LOCK:
        COLLECTION = {
            "process": process,
            "task": task,
            "status": "starting",
            "started_at": time.time(),
            "log": log_path,
            "vision_enabled": payload.get("no_vision") is not True,
            "max_duration": duration,
        }
    print_status_error("遥操", None)
    threading.Thread(target=_watch_collection, args=(process, config_path), daemon=True).start()
    return collection_status()


def stop_collection():
    with STATE_LOCK:
        job = COLLECTION
        if not job or job["process"].poll() is not None:
            raise ApiError(HTTPStatus.CONFLICT, "当前没有活动采集")
        job["status"] = "stopping"
        process = job["process"]
    try:
        os.killpg(process.pid, signal.SIGINT)
    except ProcessLookupError:
        pass
    return collection_status()


def restart_pico():
    with PICO_LOCK:
        return _restart_pico()


def _restart_pico():
    if collection_active():
        raise ApiError(HTTPStatus.CONFLICT, "采集中不能重连 PICO")
    if not ROBOTICS_SERVICE_SCRIPT.is_file():
        raise ApiError(HTTPStatus.SERVICE_UNAVAILABLE, "Robotics Service 启动脚本不存在")
    service_started = False
    service_log = PROJECT_ROOT / "logs" / "ui_robotics_service.log"
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
            body = (PREVIEW_ROOT / f"{side}.jpg").read_bytes()
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

    def send_mcap_export(self, episode_ids):
        files = mcap_export_files(DATASET_ROOT, episode_ids)
        if len(files) == 1:
            episode_id, path = files[0]
            size = path.stat().st_size
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "application/octet-stream")
            self.send_header("Content-Disposition", f'attachment; filename="{episode_id}.mcap"')
            self.send_header("Content-Length", str(size))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            try:
                with path.open("rb") as source:
                    shutil.copyfileobj(source, self.wfile, length=1024 * 1024)
            except (BrokenPipeError, ConnectionResetError, OSError):
                self.close_connection = True
            return
        filename = f"fieldnote_mcap_{datetime.now():%Y%m%d_%H%M%S}.tar"
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "application/x-tar")
        self.send_header("Content-Disposition", f'attachment; filename="{filename}"')
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "close")
        self.end_headers()
        try:
            with tarfile.open(fileobj=self.wfile, mode="w|") as archive:
                for episode_id, path in files:
                    archive.add(path, arcname=f"{episode_id}/{path.name}", recursive=False)
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
            return json.loads(self.rfile.read(length) or b"{}")
        except json.JSONDecodeError as error:
            raise ApiError(HTTPStatus.BAD_REQUEST, "JSON 请求体无效") from error

    def handle_api(self, action):
        try:
            return action()
        except ApiError as error:
            print_status_error(f"API {self.command} {urlsplit(self.path).path}", error.message)
            self.send_json({"error": error.message}, error.status)
        except (OSError, subprocess.SubprocessError) as error:
            print_status_error(f"API {self.command} {urlsplit(self.path).path}", str(error))
            self.send_json({"error": str(error)}, HTTPStatus.INTERNAL_SERVER_ERROR)

    def do_GET(self):
        request = urlsplit(self.path)
        preview = re.fullmatch(r"/api/preview/(left|right)\.jpg", request.path)
        if preview:
            return self.send_preview(preview.group(1))
        if request.path == "/api/exports/mcap":
            query = parse_qs(request.query)
            return self.handle_api(lambda: self.send_mcap_export(query.get("episode", [])))
        if request.path == "/api/status":
            usage = shutil.disk_usage(DATASET_ROOT if DATASET_ROOT.exists() else PROJECT_ROOT)
            return self.send_json({"collection": collection_status(), "disk_free_bytes": usage.free})
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
        if path == "/api/robot/reset":
            return self.handle_api(lambda: self.send_json(request_robot_reset()))
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
            with START_LOCK:
                destination = move_episode_to_trash(DATASET_ROOT, match.group(1), collection_active())
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
    decoy = subprocess.Popen((sys.executable, "-c", "import time; time.sleep(5)", f"prefix-{probe_name}"))
    try:
        assert not process_running(probe_name)
    finally:
        decoy.terminate()
        decoy.wait()
    probe = subprocess.Popen((sys.executable, "-c", "import time; time.sleep(5)", probe_name))
    try:
        assert process_running(probe_name)
    finally:
        probe.terminate()
        probe.wait()
    with tempfile.TemporaryDirectory() as temporary:
        root = Path(temporary) / "dataset"
        episode = root / "session_2026-09-03" / "episode_120000_deadbeef"
        episode.mkdir(parents=True)
        (episode / "metadata.json").write_text("{}", encoding="utf-8")
        moved = move_episode_to_trash(root, episode.name)
        assert moved.is_dir() and not episode.exists() and moved.parent == root / ".trash"
        blocked = root / "session_2026-09-03" / "episode_120001_cafebabe"
        blocked.mkdir()
        (blocked / "data").mkdir()
        (blocked / "data" / "data_0.mcap").write_bytes(b"mcap")
        assert episode_record(blocked)["size_bytes"] == 4
        assert episode_record(blocked)["quality"] is None
        assert mcap_export_files(root, [blocked.name]) == [(blocked.name, blocked / "data" / "data_0.mcap")]
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
    print("Server self-check passed")


def main():
    parser = argparse.ArgumentParser(description="Serve the Fieldnote data collection console")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=4173)
    parser.add_argument("--self-test", action="store_true")
    arguments = parser.parse_args()
    if arguments.self_test:
        return self_test()
    server = ThreadingHTTPServer((arguments.host, arguments.port), Handler)
    print(f"Fieldnote: http://{arguments.host}:{arguments.port}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping Fieldnote...")
    finally:
        server.server_close()
        if collection_active():
            stop_collection()


if __name__ == "__main__":
    main()
