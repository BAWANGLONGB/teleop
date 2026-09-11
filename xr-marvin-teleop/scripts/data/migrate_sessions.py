#!/usr/bin/env python3
"""Migrate historical episodes, verify both MCAPs, then archive originals on --replace."""

import argparse
import base64
from collections import Counter
from concurrent.futures import ProcessPoolExecutor
from copy import deepcopy
from fractions import Fraction
import hashlib
import json
import multiprocessing
import os
from pathlib import Path
import shutil
import sys
import tempfile
import time

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from xr_marvin_teleop.common.collection_config import load_config, write_json
from xr_marvin_teleop.common.episode_package import extract_episode_mcap, read_attachment
from xr_marvin_teleop.common.episode_postprocessor import postprocess_episode
from xr_marvin_teleop.common.episode_validator import validate_episode
from xr_marvin_teleop.common.episode_video import H264Encoder, activity_lock, export_episode, protobuf_schema


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def verify(path):
    """Read all CRC-checked messages and decode every image, including independent IDRs."""
    import av
    from mcap.reader import make_reader
    from foxglove_schemas_protobuf.CompressedImage_pb2 import CompressedImage
    from foxglove_schemas_protobuf.CompressedVideo_pb2 import CompressedVideo
    from xr_marvin_teleop.common.episode_video import h264_nal_types

    counts, decoders = Counter(), {}
    with Path(path).open("rb") as stream:
        reader = make_reader(stream, validate_crcs=True)
        for schema, channel, message in reader.iter_messages():
            counts[channel.topic] += 1
            if schema.name in ("foxglove.CompressedImage", "foxglove.CompressedVideo", "sensor_msgs/msg/CompressedImage"):
                if schema.name == "sensor_msgs/msg/CompressedImage":
                    from sensor_msgs.msg import CompressedImage as RosImage
                    from rclpy.serialization import deserialize_message
                    from xr_marvin_teleop.ros.protocol import stamp_ns
                    value = deserialize_message(message.data, RosImage)
                    timestamp = stamp_ns(value)
                else:
                    image_type = CompressedImage if schema.name.endswith("CompressedImage") else CompressedVideo
                    value = image_type.FromString(message.data)
                    timestamp = value.timestamp.ToNanoseconds()
                if timestamp != message.log_time:
                    raise ValueError("image and MCAP timestamps differ")
                codec = "mjpeg" if value.format == "jpeg" else "h264"
                if channel.topic not in decoders:
                    decoders[channel.topic] = av.CodecContext.create(codec, "r")
                    decoders[channel.topic].thread_count = 1
                frames = decoders[channel.topic].decode(av.Packet(bytes(value.data)))
                if len(frames) != 1:
                    raise ValueError("video message does not decode to exactly one frame")
                if codec == "h264":
                    if frames[0].pict_type == av.video.frame.PictureType.B:
                        raise ValueError("B frame in output")
                    if 5 in h264_nal_types(bytes(value.data)):
                        independent = av.CodecContext.create("h264", "r")
                        independent.thread_count = 1
                        if len(independent.decode(av.Packet(value.data))) != 1:
                            raise ValueError("IDR is not independently decodable")
            elif channel.message_encoding == "json":
                json.loads(message.data)
        for decoder in decoders.values():
            if decoder.decode(None):
                raise ValueError("unexpected buffered video frames")
        attachments = {a.name: a.data for a in reader.iter_attachments()}
        metadata = json.loads(attachments["meta/meta.json"])
        if dict(counts) != metadata["topic_counts"]:
            raise ValueError("message counts differ from embedded manifest")
        if not counts or reader.get_summary() is None:
            raise ValueError("empty/unindexed MCAP")
    return {"counts": dict(counts), "bytes": Path(path).stat().st_size, "sha256": sha256(path)}


def convert_raw(source, work, config):
    original = json.loads((source / "metadata.json").read_text())
    metadata = deepcopy(original)
    previous = metadata.pop("postprocessing", {})
    if previous.get("urdf_sha256") and previous["urdf_sha256"] != sha256(config["paths"]["urdf"]):
        raise ValueError("historical URDF differs; provide the matching URDF before migration")
    metadata.pop("processed_bag", None)
    metadata["timestamp_clock"] = "CLOCK_REALTIME"
    metadata["migration"] = {"source_format": "raw_ros2_mcap", "source": str(source),
                             "source_metadata": original, "latency_correction_applied": False,
                             "export_options": config["export"]}
    for bag in metadata["bags"]:
        if Path(bag).name != bag or not (source / bag / "metadata.yaml").is_file():
            raise ValueError(f"invalid/missing raw bag: {bag}")
        (work / bag).symlink_to(source / bag, target_is_directory=True)
    if (source / "calibration").exists():
        shutil.copytree(source / "calibration", work / "calibration")
    write_json(work / "metadata.json", metadata)
    postprocess_episode(work, urdf_path=config["paths"]["urdf"],
                        storage_config_path=config["recording"]["processed_storage"])
    manifest = validate_episode(work)
    if manifest["status"] == "rejected":
        raise ValueError(f"raw data rejected: {manifest['errors']}")
    return export_episode(work, config["export"])


def _json_value(value):
    if isinstance(value, bytes):
        return base64.b64encode(value).decode("ascii")
    if isinstance(value, list):
        return [_json_value(item) for item in value]
    return value


def write_final_manifest(episode, report):
    """Inventory the published files, not the intermediate bags now in the archive."""
    path = episode / "manifest.json"
    previous = json.loads(path.read_text()) if path.exists() else {"status": "degraded"}
    write_json(path, {
        "schema_version": 1, "validated_at_ns": report["verified_at_ns"],
        "status": previous["status"], "errors": previous.get("errors", []),
        "degraded_topics": previous.get("degraded_topics", []),
        "validation_scope": "final MCAP CRCs, message counts, timestamps and all decoded video frames",
        "bags": {f"final/{name}": {topic: {"count": count} for topic, count in info["counts"].items()}
                 for name, info in report["outputs"].items()},
        "files": {f"final/{name}": {"size_bytes": info["bytes"], "sha256": info["sha256"]}
                  for name, info in report["outputs"].items()},
    })


def convert_legacy(source, work, config):
    import av
    import pyarrow.parquet as pq
    from mcap.writer import Writer, CompressionType
    from foxglove_schemas_protobuf.CompressedImage_pb2 import CompressedImage
    from foxglove_schemas_protobuf.CompressedVideo_pb2 import CompressedVideo

    extracted = extract_episode_mcap(source, work / "extracted")  # Checks original attachment CRCs.
    original = json.loads((extracted / "meta/meta.json").read_text())
    base = int(original["validation"]["bags"]["data"]["/command/marvin/joint_target"]["first_bag_time_ns"])
    metadata = deepcopy(original["source_metadata"])
    metadata["episode_id"] = original["episode_id"]
    metadata["dataset_format"] = "foxglove"
    metadata["status"] = "degraded"
    metadata["timestamp_clock"] = "LEGACY_RECONSTRUCTED_WALL_TIME"
    metadata["migration"] = {
        "source_format": "lerobot_attachments", "source": str(source), "source_sha256": sha256(source),
        "lossy": True, "state_topic": "/legacy/resampled_state",
        "limitations": ["Original ROS messages, original JPEG bytes and exact acquisition timestamps are unavailable.",
                        "State rows remain resampled float32 Parquet values; binary fields are base64 in JSON.",
                        "Images are re-encoded from the existing lossy MP4, not camera-native JPEG.",
                        "Wall times are reconstructed from legacy aligned epoch and relative frame timestamps.",
                        "Known historical camera latency subtraction is reversed; clock mapping error cannot be removed."],
        "legacy_epoch_ns": base, "export_options": config["export"],
    }
    state_schema = json.dumps({"type": "object", "properties": {
        name: ({"type": "string", "contentEncoding": "base64"} if feature["dtype"] == "binary" else
               {"type": "array", "items": {"type": "number"}} if feature["shape"] else
               {"type": "boolean"} if feature["dtype"] == "bool" else {"type": "number"})
        for name, feature in original["features"].items() if feature["dtype"] != "video"
    }}).encode()
    final = work / "final"
    final.mkdir()
    outputs = []
    for variant, message_type in (("mjpeg", CompressedImage), ("h264", CompressedVideo)):
        if not config["export"][variant]:
            continue
        target = final / f"{metadata['episode_id']}.{variant}.mcap"
        counts = Counter()
        with target.open("xb") as stream:
            writer = Writer(stream, compression=CompressionType.NONE, enable_data_crcs=True)
            writer.start(library="xr-marvin-teleop historical migration")
            sid = writer.register_schema("teleop.legacy.ResampledFrame", "jsonschema", state_schema)
            state_channel = writer.register_channel("/legacy/resampled_state", "json", sid)
            for batch in pq.ParquetFile(extracted / "data/data.parquet").iter_batches(batch_size=256):
                for row in batch.to_pylist():
                    stamp = base + round(float(row["timestamp"]) * 1e9)
                    data = json.dumps({k: _json_value(v) for k, v in row.items()}, allow_nan=False).encode()
                    writer.add_message(state_channel, stamp, data, stamp, sequence=int(row["frame_index"]))
                    counts["/legacy/resampled_state"] += 1
            if counts["/legacy/resampled_state"] != original["length"]:
                raise ValueError("Parquet row count differs from legacy metadata")
            sid = writer.register_schema(message_type.DESCRIPTOR.full_name, "protobuf", protobuf_schema(message_type))
            for side in ("left", "right"):
                video = original["videos"].get(f"observation.images.{side}")
                if video is None:
                    continue
                topic = f"/legacy/das/{side}/{'image' if variant == 'mjpeg' else 'video'}/compressed"
                channel = writer.register_channel(topic, "protobuf", sid)
                times = video["timestamps"]
                if len(times) != video["frames"]:
                    raise ValueError("legacy video timestamp count mismatch")
                offset = original["source_metadata"].get("postprocessing", {}).get("alignment", {}).get("topic_time_offsets_ns", {}).get(f"/raw/das/{side}/image/compressed", 0)
                h264 = H264Encoder(config["export"], max(1, round(video["fps"]))) if variant == "h264" else None
                jpeg = None
                path = (extracted / video["path"]).resolve()
                if not path.is_relative_to(extracted):
                    raise ValueError("unsafe legacy video path")
                with av.open(str(path)) as container:
                    container.streams.video[0].codec_context.thread_count = 1
                    for index, frame in enumerate(container.decode(video=0)):
                        if index >= len(times):
                            raise ValueError("more decoded frames than timestamps")
                        if h264 is not None:
                            payload = h264.encode_frame(frame)
                        else:
                            if jpeg is None:
                                jpeg = av.CodecContext.create("mjpeg", "w")
                                jpeg.width, jpeg.height = frame.width, frame.height
                                jpeg.pix_fmt, jpeg.time_base = "yuvj420p", Fraction(1, 30)
                                jpeg.thread_count = 1
                                jpeg.options = {"qscale": "2"}
                            frame = frame.reformat(format="yuvj420p")
                            frame.pts, frame.time_base = index, Fraction(1, 30)
                            packets = jpeg.encode(frame)
                            if len(packets) != 1:
                                raise ValueError("MJPEG encoder buffered a frame")
                            payload = bytes(packets[0])
                        stamp = base + round(float(times[index]) * 1e9) + int(offset or 0)
                        value = message_type(frame_id=f"finger_{side}_camera", format="jpeg" if variant == "mjpeg" else "h264", data=payload)
                        value.timestamp.FromNanoseconds(stamp)
                        writer.add_message(channel, stamp, value.SerializeToString(), stamp, sequence=index)
                        counts[topic] += 1
                if counts[topic] != len(times):
                    raise ValueError("decoded video frame count mismatch")
                if h264 is not None:
                    h264.finish()
                elif jpeg is not None and jpeg.encode(None):
                    raise ValueError("unexpected delayed JPEG frames")
            output_metadata = {**metadata, "video_variant": variant, "export_options": config["export"], "topic_counts": dict(counts)}
            writer.add_attachment(0, 0, "meta/meta.json", "application/json", json.dumps(output_metadata, ensure_ascii=False).encode())
            writer.add_attachment(0, 0, "migration/source_meta.json", "application/json", (extracted / "meta/meta.json").read_bytes())
            writer.finish()
            stream.flush()
            os.fsync(stream.fileno())
        outputs.append(target)
    write_json(work / "metadata.json", metadata)
    return outputs


def migrate(source, session, config, backup, replace):
    raw = source.is_dir()
    source_metadata = json.loads((source / "metadata.json").read_text()) if raw else json.loads(read_attachment(source, "meta/meta.json"))
    episode_id = source_metadata["episode_id"]
    import re
    if not re.fullmatch(r"episode_[A-Za-z0-9_]+", episode_id):
        raise ValueError("unsafe episode id")
    destination = session / episode_id
    if not replace:
        destination = session / "migrated" / episode_id
    if (destination / "final").exists():
        raise FileExistsError(f"already has final output: {destination}")
    if destination.exists() and destination != source:
        raise FileExistsError(destination)
    with tempfile.TemporaryDirectory(prefix=".migration-", dir=session) as temporary:
        work = Path(temporary) / "work"
        work.mkdir()
        outputs = convert_raw(source, work, config) if raw else convert_legacy(source, work, config)
        verification = {p.name: verify(p) for p in outputs}
        if not verification:
            raise ValueError("no output files; refusing to replace original")
        ready = Path(temporary) / "ready"
        ready.mkdir()
        (work / "final").rename(ready / "final")
        metadata = json.loads((work / "metadata.json").read_text())
        metadata.update(dataset_format="foxglove", export_status="completed",
                        export_options=config["export"],
                        final_outputs=[f"final/{name}" for name in verification])
        write_json(ready / "metadata.json", metadata)
        if (work / "manifest.json").exists():
            shutil.copy2(work / "manifest.json", ready / "manifest.json")
        if raw and (source / "review.json").is_file():
            shutil.copy2(source / "review.json", ready / "review.json")
        result = {"source": str(source), "destination": str(destination), "lossy": not raw,
                  "outputs": verification, "verified_at_ns": time.time_ns()}
        write_final_manifest(ready, result)
        write_json(ready / "migration_report.json", result)
        archive = backup / session.name / source.relative_to(session)
        if replace:
            if archive.exists():
                raise FileExistsError(f"archive already exists: {archive}")
            archive.parent.mkdir(parents=True, exist_ok=True)
            source.rename(archive)
        try:
            destination.parent.mkdir(parents=True, exist_ok=True)
            ready.rename(destination)
        except BaseException:
            if replace:
                archive.rename(source)
            raise
        result["backup"] = str(archive) if replace else None
        write_json(destination / "migration_report.json", result)
        return result


def run_job(job):
    index, total, source, session, config, backup, replace = job
    started = time.monotonic()
    print(json.dumps({"event": "start", "index": index, "total": total, "source": str(source)}), flush=True)
    try:
        result = migrate(source, session, config, backup, replace)
        print(json.dumps({"event": "complete", "index": index, "total": total,
                          "seconds": round(time.monotonic() - started, 2), **result}), flush=True)
        return True
    except Exception as error:
        print(json.dumps({"event": "failed", "source": str(source), "error": str(error)}), flush=True)
        return False


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("sessions", nargs="+", type=Path)
    parser.add_argument("--replace", action="store_true", help="replace active episodes, preserving originals in a recovery archive")
    parser.add_argument("--allow-lossy-legacy", action="store_true")
    parser.add_argument("--limit", type=int)
    parser.add_argument("--workers", type=int, default=1, choices=range(1, 5), help="offline episode workers; collection stays exclusively locked")
    parser.add_argument("--backup-root", type=Path, required=True)
    args = parser.parse_args()
    if args.limit is not None and args.limit < 1:
        parser.error("limit must be positive")
    sessions = list(dict.fromkeys(p.expanduser().resolve() for p in args.sessions))
    root = sessions[0].parent
    if any(s.parent != root or not s.name.startswith("session_") or not s.is_dir() for s in sessions):
        parser.error("sessions must be existing sibling session_* directories")
    backup = args.backup_root.expanduser().resolve()
    if any(backup == s or backup.is_relative_to(s) for s in sessions):
        parser.error("backup must be outside the source sessions")
    config = load_config()
    with activity_lock(root, exclusive=True):
        os.nice(config["runtime"]["export_nice"])
        jobs = [(p.parent, s) for s in sessions for p in sorted(s.glob("episode_*/metadata.json")) if not (p.parent / "final").exists()]
        if args.allow_lossy_legacy:
            jobs += [(p, s) for s in sessions for p in sorted(s.glob("data/chunk-*/episode_*.mcap"))]
        if args.limit is not None:
            jobs = jobs[:args.limit]
        # Spawn avoids sharing initialized ROS/codec state; parent holds the lock until all workers exit.
        with ProcessPoolExecutor(max_workers=args.workers, mp_context=multiprocessing.get_context("spawn")) as pool:
            failed = sum(not success for success in pool.map(run_job, (
                (index, len(jobs), source, session, config, backup, args.replace)
                for index, (source, session) in enumerate(jobs, 1))))
        if failed:
            raise SystemExit(1)


if __name__ == "__main__":
    main()
