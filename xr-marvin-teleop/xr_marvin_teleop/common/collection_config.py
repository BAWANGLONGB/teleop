"""Collection configuration, explicit CLI overrides, and immutable file snapshots."""

import argparse
from contextlib import contextmanager
from copy import deepcopy
import fcntl
import hashlib
import json
import math
from pathlib import Path


from .marvin_scale_calibration import ensure_scale_calibration


DEFAULT_CONFIG = Path(__file__).resolve().parents[2] / "config/collection.json"
PATH_FIELDS = {
    "paths": {"output_root", "das_config", "das_sdk_root", "scale_calibration", "urdf", "calibrations"},
    "recording": {"state_storage", "camera_storage", "processed_storage", "qos"},
    "preview": {"root"},
}
ARG_FIELDS = {
    "output_root": ("paths", "output_root"), "das_config": ("paths", "das_config"),
    "das_sdk_root": ("paths", "das_sdk_root"), "scale_calibration_path": ("paths", "scale_calibration"),
    "calibration": ("paths", "calibrations"), "urdf": ("paths", "urdf"),
    "robot_model": ("robot", "model"), "robot_ip": ("robot", "ip"),
    **{name: ("robot", name) for name in (
        "thumbstick_y_sign", "gripper_mode", "scale_factor", "nsp_lateral", "nsp_max_angle", "nsp_angle_rate",
        "joint_command_max_speed_deg_s",
        "nsp_lateral_deadzone", "nsp_lateral_range", "nsp_lateral_sign_left", "nsp_lateral_sign_right")},
    "no_vision": ("capture", "vision_enabled"), "max_duration": ("capture", "max_duration_s"),
    "pico_poll_hz": ("capture", "pico_poll_hz"), "preview_root": ("preview", "root"),
    "preview": ("preview", "enabled"), "preview_fps": ("preview", "fps"),
    "startup_timeout": ("runtime", "startup_timeout_s"),
    "camera_startup_timeout": ("runtime", "camera_startup_timeout_s"),
    **{name: ("export", name) for name in (
        "mjpeg", "h264", "h264_crf", "h264_preset", "h264_keyint", "h264_threads")},
}
PROCESSES = ("hardware", "pico", "das_left", "das_right", "recorder")
# Types/field names are the contract; actual default values live only in JSON.
CONFIG_SCHEMA = {
    "schema_version": int,
    "paths": {key: list if key == "calibrations" else str for key in PATH_FIELDS["paths"]},
    "robot": {
        "model": str, "ip": str, "thumbstick_y_sign": int, "gripper_mode": str, "scale_factor": None,
        "joint_command_max_speed_deg_s": float,
        "nsp_lateral": bool, "nsp_max_angle": float, "nsp_angle_rate": float,
        "nsp_lateral_deadzone": float, "nsp_lateral_range": float,
        "nsp_lateral_sign_left": int, "nsp_lateral_sign_right": int,
    },
    "capture": {"vision_enabled": bool, "pico_poll_hz": float, "max_duration_s": None},
    "recording": {**{key: str for key in PATH_FIELDS["recording"]}, "state_topics": list, "state_cache_bytes": int},
    "preview": {"enabled": bool, "root": None, "fps": int},
    "runtime": {
        "startup_timeout_s": float, "camera_startup_timeout_s": float,
        "encoder_stale_timeout_s": float, "recorder_stop_timeout_s": float,
        "recorder_term_timeout_s": float, "export_nice": int,
        "cpus": {key: list for key in PROCESSES}, "nice": {key: int for key in PROCESSES},
        "shutdown_timeout_s": {key: float for key in PROCESSES},
    },
    "export": {"mjpeg": bool, "h264": bool, "h264_crf": int, "h264_preset": str,
               "h264_keyint": int, "h264_threads": int},
}


def read_json(path):
    def unique_pairs(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                raise ValueError(f"duplicate config key: {key}")
            result[key] = value
        return result
    return json.loads(Path(path).read_text(encoding="utf-8"), object_pairs_hook=unique_pairs)


def _merge(base, patch, prefix=""):
    if not isinstance(patch, dict):
        raise ValueError(f"{prefix or 'config'} must be an object")
    for key, value in patch.items():
        if key not in base:
            raise ValueError(f"unknown config key: {prefix}{key}")
        if isinstance(base[key], dict):
            _merge(base[key], value, f"{prefix}{key}.")
        else:
            base[key] = value


def _resolve_paths(config, directory):
    if not isinstance(config, dict):
        raise ValueError("config must be an object")
    for group, fields in PATH_FIELDS.items():
        if group in config and not isinstance(config[group], dict):
            raise ValueError(f"{group} must be an object")
        for field in fields:
            if field not in config.get(group, {}):
                continue
            value = config[group][field]
            def resolve(item):
                if item is None and (group, field) == ("preview", "root"):
                    return None
                if not isinstance(item, str) or not item.strip():
                    raise ValueError(f"{group}.{field} must be a nonempty path")
                path = Path(item).expanduser()
                return str((path if path.is_absolute() else directory / path).resolve())
            if field == "calibrations":
                if not isinstance(value, list):
                    raise ValueError("paths.calibrations must be a list")
                config[group][field] = [resolve(item) for item in value]
            else:
                config[group][field] = resolve(value)


def load_config(path=None, *, saved=None):
    """Merge one override into defaults or saved capture settings, without mutating either."""
    # Only one optional override file. Paths retain the base of their source file.
    config = read_json(DEFAULT_CONFIG) if saved is None else deepcopy(saved)
    if saved is None:
        _resolve_paths(config, DEFAULT_CONFIG.parent)
    if path is not None:
        path = Path(path).expanduser().resolve()
        patch = read_json(path)
        _resolve_paths(patch, path.parent)
        _merge(config, patch)
    return config


def validate_config(config):
    def check_types(value, reference, name="config"):
        if isinstance(reference, dict):
            # Older capture snapshots predate interpolation; keep their provenance intact.
            if name == "config.robot" and isinstance(value, dict) and "joint_command_max_speed_deg_s" not in value:
                reference = {k: v for k, v in reference.items() if k != "joint_command_max_speed_deg_s"}
            if not isinstance(value, dict) or value.keys() != reference.keys():
                raise ValueError(f"{name}: missing or unknown fields")
            for key in reference:
                check_types(value[key], reference[key], f"{name}.{key}")
        elif reference is not None:
            expected = reference
            valid = (
                type(value) in (int, float) and math.isfinite(value)
                if expected is float
                else type(value) is expected
            )
            if not valid:
                raise ValueError(f"{name} must be {expected.__name__}")
    check_types(config, CONFIG_SCHEMA)
    if config["schema_version"] != 1:
        raise ValueError("unsupported collection schema_version")
    def number(value, name, low, high, integer=False):
        if (
            type(value) not in ((int,) if integer else (int, float))
            or not math.isfinite(value)
            or not low <= value <= high
        ):
            raise ValueError(f"{name} must be within [{low}, {high}]")
    number(config["capture"]["pico_poll_hz"], "pico_poll_hz", 30, 240)
    if "joint_command_max_speed_deg_s" in config["robot"]:
        number(config["robot"]["joint_command_max_speed_deg_s"],
               "joint_command_max_speed_deg_s", 1e-9, 1e12)
    for value, name in ((config["capture"]["max_duration_s"], "max_duration_s"),
                        (config["robot"]["scale_factor"], "scale_factor")):
        if value is not None:
            number(value, name, 1e-9, 1e12)
    number(config["preview"]["fps"], "preview.fps", 1, 240, True)
    root = config["preview"]["root"]
    if root is not None and (not isinstance(root, str) or not root.strip()):
        raise ValueError("preview.root must be a path or null")
    if config["preview"]["enabled"] and root is None:
        raise ValueError("preview.root is required when preview is enabled")
    number(config["recording"]["state_cache_bytes"], "state_cache_bytes", 1, 16 * 1024**3, True)
    topics = config["recording"]["state_topics"]
    if any(
        not isinstance(topic, str)
        or not topic.startswith("/")
        or any(character.isspace() for character in topic)
        for topic in topics
    ) or len(topics) != len(set(topics)):
        raise ValueError("state_topics must be unique absolute ROS topic names")
    required = {"/raw/pico/poses", "/raw/pico/joy", "/raw/pico/status",
                "/raw/marvin/joint_state", "/raw/marvin/joint_state/status",
                "/command/marvin/joint_target", "/command/marvin/joint_target/status"}
    if not required.issubset(topics):
        raise ValueError("state_topics must include PICO, Marvin feedback and joint commands")
    robot = config["robot"]
    if not robot["model"].strip() or not robot["ip"].strip():
        raise ValueError("robot model and IP must not be empty")
    for name in ("thumbstick_y_sign", "nsp_lateral_sign_left", "nsp_lateral_sign_right"):
        if robot[name] not in (-1, 1):
            raise ValueError(f"{name} must be -1 or 1")
    if robot["gripper_mode"] not in ("binary", "continuous"):
        raise ValueError("gripper_mode must be binary or continuous")
    number(robot["nsp_max_angle"], "nsp_max_angle", 1e-9, 30)
    number(robot["nsp_angle_rate"], "nsp_angle_rate", 1e-9, 1e9)
    number(robot["nsp_lateral_deadzone"], "nsp_lateral_deadzone", 0, 1e9)
    number(robot["nsp_lateral_range"], "nsp_lateral_range", 1e-9, 1e9)
    if robot["nsp_lateral_range"] <= robot["nsp_lateral_deadzone"]:
        raise ValueError("NSP range must exceed deadzone")
    runtime = config["runtime"]
    for name, cpus in runtime["cpus"].items():
        if (
            not cpus
            or any(type(cpu) is not int or cpu < 0 for cpu in cpus)
            or len(cpus) != len(set(cpus))
        ):
            raise ValueError(f"runtime.cpus.{name} must contain unique CPU numbers")
    for value in (*runtime["nice"].values(), runtime["export_nice"]):
        number(value, "nice", 0, 19, True)
    for name, value in runtime.items():
        if name.endswith("_s") and not isinstance(value, dict):
            number(value, name, 0.01, 3600)
    for name, value in runtime["shutdown_timeout_s"].items():
        number(value, name, 0.01, 3600)
    from .episode_video import video_options
    video_options(saved=config["export"])
    return config


def configure_parser(parser):
    parser.add_argument("--config", type=Path)
    parser.add_argument("--print-effective-config", action="store_true")
    # SUPPRESS distinguishes an omitted flag from an explicit false/null override.
    for action in parser._actions:
        if action.dest in ARG_FIELDS:
            action.default = argparse.SUPPRESS
            action.required = False
        if action.dest == "task":
            action.required = False


def apply_config(arguments, *, saved=None):
    config = load_config(getattr(arguments, "config", None), saved=saved)
    explicit = vars(arguments).copy()
    for name, (group, field) in ARG_FIELDS.items():
        if name not in explicit:
            continue
        value = explicit[name]
        if name == "no_vision":
            value = not value
        if field in PATH_FIELDS.get(group, set()):
            if isinstance(value, list):
                value = [str(item) for item in value]
            elif value is not None:
                value = str(value)
            patch = {group: {field: value}}
            _resolve_paths(patch, Path.cwd())
            value = patch[group][field]
        config[group][field] = value
    if "preview_root" in explicit and "preview" not in explicit:
        config["preview"]["enabled"] = explicit["preview_root"] is not None
    validate_config(config)
    bind_config(arguments, config)
    return config


def bind_config(arguments, config):
    arguments.effective_config = config
    for name, (group, field) in ARG_FIELDS.items():
        if field == "joint_command_max_speed_deg_s" and field not in config[group]:
            continue
        value = config[group][field]
        if name == "no_vision":
            value = not value
        if field in PATH_FIELDS.get(group, set()):
            if isinstance(value, list):
                value = [Path(item) for item in value]
            elif value is not None:
                value = Path(value)
        if name == "preview_root" and not config["preview"]["enabled"]:
            value = None
        setattr(arguments, name, value)


def write_json(path, value):
    """Replace metadata atomically so readers never observe a partial JSON file."""
    path = Path(path)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding="utf-8")
    temporary.replace(path)


def snapshot_config(config, directory, *, require_devices=True):
    """Copy referenced files before processes start; paths in the saved JSON are portable."""
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=False)
    ensure_scale_calibration(config["paths"]["scale_calibration"])
    saved = deepcopy(config)
    entries = []
    fields = [("paths", "das_config"), ("paths", "scale_calibration"), ("paths", "urdf")]
    fields += [("recording", key) for key in PATH_FIELDS["recording"]]
    for group, field in fields:
        source = Path(config[group][field])
        if not source.is_file() and not require_devices and field in ("das_config", "scale_calibration"):
            continue
        if not source.is_file() or source.stat().st_size > 16 * 1024**2:
            raise ValueError(f"configuration must be a regular file up to 16 MiB: {source}")
        target = directory / f"{len(entries):02d}_{source.name}"
        data = source.read_bytes()
        if len(data) > 16 * 1024**2:
            raise ValueError(f"configuration attachment too large: {source}")
        target.write_bytes(data)
        saved[group][field] = target.name
        entries.append({"file": target.name, "source": str(source), "sha256": hashlib.sha256(data).hexdigest()})
    saved["paths"]["calibrations"] = []
    for source in config["paths"]["calibrations"]:
        source = Path(source)
        if not source.is_file() or source.stat().st_size > 16 * 1024**2:
            raise ValueError(f"calibration must be a regular file up to 16 MiB: {source}")
        target = directory / f"{len(entries):02d}_{source.name}"
        data = source.read_bytes()
        if len(data) > 16 * 1024**2:
            raise ValueError(f"calibration attachment too large: {source}")
        target.write_bytes(data)
        saved["paths"]["calibrations"].append(target.name)
        entries.append({"file": target.name, "source": str(source), "sha256": hashlib.sha256(data).hexdigest()})
    write_json(directory / "collection.json", saved)
    write_json(directory / "files.json", entries)
    return directory / "collection.json"


def freeze_arguments(arguments, directory, *, require_devices=True):
    path = snapshot_config(arguments.effective_config, directory, require_devices=require_devices)
    config = load_config(path)
    validate_config(config)
    bind_config(arguments, config)
    arguments.config = path
    return path


def device_contract(config):
    """Identify settings owned by persistent devices; recording/export may vary per episode."""
    from dataclasses import asdict
    from xr_marvin_teleop.hardware.interface.das_finger import load_das_finger_configurations
    configurations = load_das_finger_configurations(config["paths"]["das_config"])
    # Cameras belong to each recorder, not the persistent DAS device processes.
    hardware = {
        side: {
            key: value for key, value in asdict(configuration).items()
            if not key.startswith("camera_")
        }
        for side, configuration in zip(("left", "right"), configurations)
    }
    scale = Path(config["paths"]["scale_calibration"]).read_bytes()
    return {
        "robot": config["robot"],
        "das": hardware,
        "pico_poll_hz": config["capture"]["pico_poll_hz"],
        "encoder_stale_timeout_s": config["runtime"]["encoder_stale_timeout_s"],
        "sdk_root": config["paths"]["das_sdk_root"],
        "scale_sha256": hashlib.sha256(scale).hexdigest(),
    }


@contextmanager
def active_devices(config):
    root = Path(config["paths"]["output_root"])
    with (root / ".devices.lock").open("a+b") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError("device supervisor already active") from None
        path = root / ".devices-config.json"
        write_json(path, device_contract(config))
        try:
            yield
        finally:
            path.unlink(missing_ok=True)


def check_active_devices(config):
    root = Path(config["paths"]["output_root"])
    lock_path = root / ".devices.lock"
    if not lock_path.exists():
        return  # Standalone device scripts have no registry.
    with lock_path.open("rb") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_SH | fcntl.LOCK_NB)
        except BlockingIOError:
            if read_json(root / ".devices-config.json") != device_contract(config):
                raise RuntimeError("recording config differs from active devices; stop devices before changing hardware parameters")
