"""Offline, bounded-memory Foxglove MCAP exports; never run in a control loop."""

import argparse
from collections import Counter
from contextlib import ExitStack
import fcntl
from fractions import Fraction
import heapq
import json
import os
from pathlib import Path
import re
import tempfile

from .collection_config import DEFAULT_CONFIG, read_json

VIDEO_DEFAULTS = read_json(DEFAULT_CONFIG)["export"]
PRESETS = ("ultrafast", "superfast", "veryfast", "faster", "fast", "medium", "slow")


def add_video_arguments(parser, inherit=False):
    for name in ("mjpeg", "h264"):
        parser.add_argument(
            f"--{name}", action=argparse.BooleanOptionalAction,
            default=None if inherit else VIDEO_DEFAULTS[name],
            help=f"enable/disable the final {name} MCAP (not temporary camera capture)",
        )
    for name in ("h264_crf", "h264_preset", "h264_keyint", "h264_threads"):
        parser.add_argument(
            "--" + name.replace("_", "-"),
            type=str if name == "h264_preset" else int,
            default=None if inherit else VIDEO_DEFAULTS[name],
        )


def video_options(arguments=None, saved=None):
    result = {**VIDEO_DEFAULTS, **(saved or {})}
    if arguments is not None:
        result.update({key: getattr(arguments, key) for key in VIDEO_DEFAULTS
                       if getattr(arguments, key, None) is not None})
    if not all(isinstance(result[key], bool) for key in ("mjpeg", "h264")):
        raise ValueError("mjpeg and h264 switches must be boolean")
    if not result["mjpeg"] and not result["h264"]:
        raise ValueError("at least one of --mjpeg / --h264 must be enabled")
    for key, low, high in (("h264_crf", 1, 51), ("h264_keyint", 1, 10000),
                           ("h264_threads", 1, 16)):
        if type(result[key]) is not int or not low <= result[key] <= high:
            raise ValueError(f"{key} must be an integer within [{low}, {high}]")
    if result["h264_preset"] not in PRESETS:
        raise ValueError(f"h264_preset must be one of {PRESETS}")
    return result


def activity_lock(output_root, *, exclusive=False):
    """Nonblocking: recorders share a lock; export is exclusive, persistent devices are independent."""
    root = Path(output_root).expanduser().resolve()
    root.mkdir(parents=True, exist_ok=True)
    lock = (root / ".collection.lock").open("a+b")
    try:
        fcntl.flock(lock, (fcntl.LOCK_EX if exclusive else fcntl.LOCK_SH) | fcntl.LOCK_NB)
    except BlockingIOError:
        lock.close()
        raise RuntimeError("collection or offline export is active; retry after it stops") from None
    return lock


def protobuf_schema(message_type):
    from google.protobuf.descriptor_pb2 import FileDescriptorSet

    result = FileDescriptorSet()
    seen = set()

    def append(descriptor):
        if descriptor.name in seen:
            return
        seen.add(descriptor.name)
        for dependency in descriptor.dependencies:
            append(dependency)
        descriptor.CopyToProto(result.file.add())

    append(message_type.DESCRIPTOR.file)
    return result.SerializeToString()


def h264_nal_types(payload):
    return {part[0] & 31 for part in re.split(b"\x00\x00\x00?\x01", payload)[1:] if part}


class H264Encoder:
    """One JPEG -> one Annex B AU; no reorder/lookahead queues or timestamp guessing."""

    def __init__(self, options, fps=30):
        import av

        self.av = av
        self.options = options
        if type(fps) is not int or not 1 <= fps <= 240:
            raise ValueError("camera FPS must be an integer within [1, 240]")
        self.fps = fps
        self.decoder = av.CodecContext.create("mjpeg", "r")
        self.decoder.thread_count = 1
        self.encoder = None
        self.index = 0

    def encode(self, jpeg):
        frames = self.decoder.decode(self.av.Packet(jpeg))
        if len(frames) != 1:
            raise ValueError("JPEG must decode to exactly one image")
        return self.encode_frame(frames[0])

    def encode_frame(self, frame):
        """Also accept a decoded legacy video frame without a JPEG round trip."""
        if self.encoder is None:
            if frame.width % 2 or frame.height % 2:
                raise ValueError("H.264 yuv420p requires even image dimensions")
            codec = self.av.CodecContext.create("libx264", "w")
            codec.width, codec.height = frame.width, frame.height
            codec.pix_fmt = "yuv420p"
            codec.time_base = Fraction(1, self.fps)
            codec.framerate = Fraction(self.fps, 1)
            codec.thread_count = self.options["h264_threads"]
            codec.max_b_frames = 0
            codec.gop_size = self.options["h264_keyint"]
            codec.options = {
                "crf": str(self.options["h264_crf"]),
                "preset": self.options["h264_preset"],
                "tune": "zerolatency",
                "profile": "baseline",
                "x264-params": "annexb=1:repeat-headers=1:bframes=0:rc-lookahead=0:sync-lookahead=0",
            }
            codec.open()
            self.encoder = codec
        if (frame.width, frame.height) != (self.encoder.width, self.encoder.height):
            raise ValueError("camera resolution changed within an episode")
        frame = frame.reformat(format="yuv420p")
        # MJPEG decoding marks every input as I; let x264 select P/IDR frames.
        frame.pict_type = self.av.video.frame.PictureType.NONE
        # Internal codec ticks only. MCAP/protobuf retain the exact acquisition ns.
        frame.pts, frame.time_base = self.index, self.encoder.time_base
        packets = self.encoder.encode(frame)
        if len(packets) != 1 or packets[0].pts != self.index or packets[0].dts != self.index:
            raise RuntimeError("H.264 encoder buffered/reordered a frame")
        payload = bytes(packets[0])
        nals = h264_nal_types(payload)
        if not nals.intersection((1, 5)) or (5 in nals and not {7, 8}.issubset(nals)):
            raise RuntimeError("H.264 AU is missing its frame or IDR SPS/PPS")
        if self.index == 0 and 5 not in nals:
            raise RuntimeError("H.264 stream must start at an IDR")
        self.index += 1
        return payload

    def finish(self):
        if self.encoder is not None and self.encoder.encode(None):
            raise RuntimeError("unexpected delayed H.264 frames")


def export_episode(episode_directory, options=None, *, add_missing=False):
    """Publish final/ atomically only after every selected variant passes CRC/count checks.

    With add_missing, publish only absent variants and retain existing exports.
    Source bags are deliberately retained for recovery and parameter retuning.
    Caller must hold the exclusive collection activity lock throughout processing.
    """
    from mcap.reader import make_reader
    from mcap.writer import CompressionType, Writer
    from foxglove_schemas_protobuf.CompressedImage_pb2 import CompressedImage
    from foxglove_schemas_protobuf.CompressedVideo_pb2 import CompressedVideo
    from rclpy.serialization import deserialize_message
    from teleop_msgs.msg import CompressedImageFrame

    episode = Path(episode_directory).expanduser().resolve()
    metadata = json.loads((episode / "metadata.json").read_text(encoding="utf-8"))
    options = video_options(saved=options or metadata.get("video_outputs"))
    manifest = json.loads((episode / "manifest.json").read_text(encoding="utf-8"))
    if manifest["status"] == "rejected":
        raise ValueError("refusing to export a rejected episode; source bags retained")
    if metadata.get("postprocessing", {}).get("alignment", {}).get("clock") != "CLOCK_REALTIME":
        raise ValueError("processed bag must use system time; reprocess old aligned data first")
    inputs = sorted((episode / metadata.get("processed_bag", "data")).glob("*.mcap"))
    if not inputs:
        raise FileNotFoundError("processed MCAP bag is missing")
    final = episode / "final"
    if final.is_symlink() or (final.exists() and not final.is_dir()):
        raise ValueError(f"invalid export directory: {final}")
    if final.exists() and not add_missing:
        raise FileExistsError(f"export already exists: {final}")
    variants = [name for name in ("mjpeg", "h264") if options[name]]
    episode_id = metadata["episode_id"]
    if not re.fullmatch(r"episode_[A-Za-z0-9_]+", episode_id):
        raise ValueError("unsafe episode_id")
    requested = [final / f"{episode_id}.{variant}.mcap" for variant in variants]
    if add_missing:
        for target in requested:
            if target.is_symlink() or (target.exists() and not target.is_file()):
                raise ValueError(f"invalid export file: {target}")
        variants = [variant for variant in variants if not (final / f"{episode_id}.{variant}.mcap").exists()]
    # A temporary directory also prevents readers from observing half a dual export.
    with tempfile.TemporaryDirectory(prefix=".video-export-", dir=episode) as directory:
        staging = Path(directory) / "final"
        staging.mkdir()
        for variant in variants:
            target = staging / f"{episode_id}.{variant}.mcap"
            message_type = CompressedImage if variant == "mjpeg" else CompressedVideo
            encoders = {}
            counts = Counter()
            with ExitStack() as stack:
                output = stack.enter_context(target.open("xb"))
                writer = Writer(output, compression=CompressionType.NONE, enable_data_crcs=True)
                # Mixed ROS2 CDR + Foxglove protobuf: do not claim the ros2 profile.
                writer.start(library="xr-marvin-teleop")
                image_schema = writer.register_schema(
                    name=message_type.DESCRIPTOR.full_name, encoding="protobuf",
                    data=protobuf_schema(message_type),
                )
                schemas, channels = {}, {}
                readers = [make_reader(stack.enter_context(path.open("rb")), validate_crcs=True)
                           for path in inputs]
                messages = heapq.merge(*(r.iter_messages() for r in readers),
                                       key=lambda item: item[2].log_time)
                for schema, channel, message in messages:
                    topic = channel.topic
                    camera = topic in ("/raw/das/left/image/compressed", "/raw/das/right/image/compressed")
                    if camera:
                        if schema is None or schema.name != "teleop_msgs/msg/CompressedImageFrame":
                            raise ValueError(f"unsupported camera schema for {topic}")
                        source = deserialize_message(message.data, CompressedImageFrame)
                        frame = source.image
                        payload = bytes(frame.data)
                        if not payload.startswith(b"\xff\xd8") or b"\xff\xd9" not in payload[-64:]:
                            raise ValueError(f"invalid JPEG in {topic}")
                        output_topic = topic if variant == "mjpeg" else topic.replace("/image/compressed", "/video/compressed")
                        if output_topic not in channels:
                            channels[output_topic] = writer.register_channel(
                                topic=output_topic, message_encoding="protobuf", schema_id=image_schema,
                            )
                            if variant == "h264":
                                side = topic.split("/")[3]
                                fps = metadata.get("camera_profiles", {}).get(side, {}).get("fps", 30)
                                encoders[topic] = H264Encoder(options, fps)
                        if variant == "h264":
                            payload = encoders[topic].encode(payload)
                        converted = message_type(frame_id=frame.header.frame_id, data=payload,
                                                 format="jpeg" if variant == "mjpeg" else "h264")
                        converted.timestamp.FromNanoseconds(message.log_time)
                        writer.add_message(channels[output_topic], message.log_time,
                                           converted.SerializeToString(), message.log_time,
                                           sequence=source.sequence_id & 0xFFFFFFFF)
                        counts[output_topic] += 1
                    else:
                        if topic not in channels:
                            if schema is None or not schema.data:
                                raise ValueError(f"missing embedded schema for {topic}")
                            key = (schema.name, schema.encoding, schema.data)
                            if key not in schemas:
                                schemas[key] = writer.register_schema(schema.name, schema.encoding, schema.data)
                            channels[topic] = writer.register_channel(
                                topic, channel.message_encoding, schemas[key], metadata=channel.metadata,
                            )
                        writer.add_message(channels[topic], message.log_time, message.data,
                                           message.log_time, sequence=message.sequence)
                        counts[topic] += 1
                for encoder in encoders.values():
                    encoder.finish()
                package_metadata = {**metadata, "dataset_format": "foxglove", "video_variant": variant,
                                    "export_status": "completed",
                                    "export_options": options,
                                    "export_config": {"export": options, "urdf": metadata.get("postprocessing", {}).get("urdf")},
                                    "validation": manifest, "topic_counts": dict(counts)}
                writer.add_attachment(0, 0, "meta/meta.json", "application/json",
                                      json.dumps(package_metadata, ensure_ascii=False).encode())
                for calibration in [*metadata.get("calibrations", []), *metadata.get("config_files", [])]:
                    path = (episode / calibration["snapshot"]).resolve()
                    if not path.is_relative_to(episode) or path.stat().st_size > 16 * 1024 * 1024:
                        raise ValueError("unsafe or oversized calibration attachment")
                    writer.add_attachment(0, 0, path.relative_to(episode).as_posix(),
                                          "application/octet-stream", path.read_bytes())
                writer.finish()
                output.flush()
                os.fsync(output.fileno())
            with target.open("rb") as source:
                reader = make_reader(source, validate_crcs=True)
                actual = Counter(channel.topic for _, channel, _ in reader.iter_messages(log_time_order=False))
                if actual != counts or reader.get_summary() is None:
                    raise ValueError(f"MCAP verification failed: {target}")
                list(reader.iter_attachments())  # Validate attachment CRCs as well.
        if final.exists():
            for target in staging.iterdir():
                os.link(target, final / target.name)  # Publish complete files without overwriting existing exports.
        else:
            staging.rename(final)
    return requested
