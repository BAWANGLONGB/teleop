"""Read, extract, and write legacy LeRobot MCAP attachments used by the UI."""

import json
import struct
import time
import zlib
from pathlib import Path, PurePosixPath


MAGIC = b"\x89MCAP0\r\n"
ATTACHMENT = 0x09
DATA_END = 0x0F
FOOTER = 0x02


def _string(value):
    encoded = value.encode("utf-8")
    return struct.pack("<I", len(encoded)) + encoded


def _record(output, opcode, content):
    output.write(bytes((opcode,)))
    output.write(struct.pack("<Q", len(content)))
    output.write(content)


def _safe_attachment_name(name):
    path = PurePosixPath(name)
    if path.is_absolute() or not path.parts or ".." in path.parts or "\\" in name:
        raise ValueError(f"unsafe MCAP attachment name: {name!r}")
    return path


def write_episode_mcap(output_path, attachments, timestamp_ns=None):
    """Write files as standards-compliant MCAP Attachment records."""
    output_path = Path(output_path)
    attachments = [(str(_safe_attachment_name(n)), m, Path(p)) for n, m, p in attachments]
    if not attachments or len({item[0] for item in attachments}) != len(attachments):
        raise ValueError("MCAP attachments must be non-empty and uniquely named")
    if any(not path.is_file() for _name, _media_type, path in attachments):
        raise FileNotFoundError("an MCAP attachment source file is missing")
    timestamp_ns = time.time_ns() if timestamp_ns is None else int(timestamp_ns)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("xb") as output:
        output.write(MAGIC)
        _record(
            output,
            0x01,
            _string("org.huggingface.lerobot")
            + _string("xr-marvin-teleop"),
        )
        for name, media_type, path in attachments:
            prefix = (
                struct.pack("<QQ", timestamp_ns, int(path.stat().st_mtime_ns))
                + _string(name)
                + _string(media_type)
                + struct.pack("<Q", path.stat().st_size)
            )
            content_length = len(prefix) + path.stat().st_size + 4
            output.write(bytes((ATTACHMENT,)))
            output.write(struct.pack("<Q", content_length))
            output.write(prefix)
            checksum = zlib.crc32(prefix)
            with path.open("rb") as source:
                for chunk in iter(lambda: source.read(1024 * 1024), b""):
                    output.write(chunk)
                    checksum = zlib.crc32(chunk, checksum)
            output.write(struct.pack("<I", checksum & 0xFFFFFFFF))
        _record(output, DATA_END, struct.pack("<I", 0))
        _record(output, FOOTER, struct.pack("<QQI", 0, 0, 0))
        output.write(MAGIC)


def _read_exact(source, size):
    value = source.read(size)
    if len(value) != size:
        raise ValueError("truncated MCAP")
    return value


def _read_string(source):
    size = struct.unpack("<I", _read_exact(source, 4))[0]
    return _read_exact(source, size).decode("utf-8")


def attachment_entries(path):
    """Return Attachment locations without loading their data into memory."""
    path = Path(path)
    entries = []
    with path.open("rb") as source:
        if _read_exact(source, len(MAGIC)) != MAGIC:
            raise ValueError("not an MCAP file")
        file_size = path.stat().st_size
        saw_header = saw_data_end = saw_footer = False
        while not saw_footer:
            opcode = _read_exact(source, 1)[0]
            length = struct.unpack("<Q", _read_exact(source, 8))[0]
            content_start = source.tell()
            content_end = content_start + length
            if content_end > file_size - len(MAGIC):
                raise ValueError("MCAP record exceeds file size")
            if not saw_header:
                if opcode != 0x01:
                    raise ValueError("MCAP Header is missing")
                saw_header = True
            elif opcode == ATTACHMENT:
                log_time, create_time = struct.unpack("<QQ", _read_exact(source, 16))
                name = _read_string(source)
                media_type = _read_string(source)
                data_size = struct.unpack("<Q", _read_exact(source, 8))[0]
                data_offset = source.tell()
                if data_offset + data_size + 4 != content_end:
                    raise ValueError("invalid MCAP Attachment length")
                source.seek(data_size, 1)
                checksum = struct.unpack("<I", _read_exact(source, 4))[0]
                entries.append(
                    {
                        "name": name,
                        "media_type": media_type,
                        "log_time": log_time,
                        "create_time": create_time,
                        "data_offset": data_offset,
                        "data_size": data_size,
                        "content_start": content_start,
                        "checksum": checksum,
                    }
                )
            elif opcode == DATA_END:
                saw_data_end = True
            elif opcode == FOOTER:
                if not saw_data_end or length != 20:
                    raise ValueError("invalid MCAP Footer")
                saw_footer = True
            source.seek(content_end)
        if _read_exact(source, len(MAGIC)) != MAGIC or source.read(1):
            raise ValueError("invalid MCAP trailing magic")
    if len({item["name"] for item in entries}) != len(entries):
        raise ValueError("duplicate MCAP attachment name")
    return entries


def read_attachment(path, name):
    for entry in attachment_entries(path):
        if entry["name"] == name:
            with Path(path).open("rb") as source:
                source.seek(entry["data_offset"])
                return _read_exact(source, entry["data_size"])
    raise KeyError(name)


def extract_episode_mcap(path, output_directory):
    validate_episode_mcap(path)
    output_directory = Path(output_directory).resolve()
    output_directory.mkdir(parents=True, exist_ok=True)
    for entry in attachment_entries(path):
        relative = _safe_attachment_name(entry["name"])
        destination = output_directory.joinpath(*relative.parts)
        if output_directory not in destination.resolve().parents:
            raise ValueError(f"unsafe MCAP attachment name: {entry['name']!r}")
        destination.parent.mkdir(parents=True, exist_ok=True)
        with Path(path).open("rb") as source, destination.open("xb") as target:
            source.seek(entry["data_offset"])
            remaining = entry["data_size"]
            while remaining:
                chunk = source.read(min(1024 * 1024, remaining))
                if not chunk:
                    raise ValueError("truncated MCAP Attachment")
                target.write(chunk)
                remaining -= len(chunk)
    return output_directory


def validate_episode_mcap(path):
    entries = attachment_entries(path)
    by_name = {item["name"]: item for item in entries}
    for required in ("meta/meta.json", "data/data.parquet"):
        if required not in by_name:
            raise ValueError(f"required MCAP attachment is missing: {required}")
    for entry in entries:
        with Path(path).open("rb") as source:
            source.seek(entry["content_start"])
            remaining = entry["data_offset"] + entry["data_size"] - entry["content_start"]
            checksum = 0
            while remaining:
                chunk = _read_exact(source, min(1024 * 1024, remaining))
                checksum = zlib.crc32(chunk, checksum)
                remaining -= len(chunk)
        if entry["checksum"] and checksum & 0xFFFFFFFF != entry["checksum"]:
            raise ValueError(f"MCAP attachment CRC mismatch: {entry['name']}")
    metadata = json.loads(read_attachment(path, "meta/meta.json"))
    if metadata.get("dataset_format") != "lerobot" or not metadata.get("episode_id"):
        raise ValueError("invalid LeRobot episode metadata")
    expected = {
        "data/data.parquet",
        *metadata.get("video_paths", {}).values(),
    }
    if not expected.issubset(by_name):
        raise ValueError("metadata references a missing MCAP attachment")
    return metadata
