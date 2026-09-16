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
import sqlite3
import tempfile

from .collection_config import DEFAULT_CONFIG, read_json
from xr_marvin_teleop.ros.protocol import (
    DAS_COMPRESSED_IMAGE_STATUS_TOPICS,
    DAS_COMPRESSED_IMAGE_TOPICS,
)

VIDEO_DEFAULTS = read_json(DEFAULT_CONFIG)["export"]
VIDEO_VARIANTS = ("mjpeg", "h264", "av1")
ENCODED_VIDEO_VARIANTS = ("h264", "av1")
H264_PRESETS = (
    "ultrafast", "superfast", "veryfast", "faster", "fast", "medium", "slow"
)
LEGACY_AV1_DEFAULTS = {
    "av1": False,
    "av1_crf": VIDEO_DEFAULTS["av1_crf"],
    "av1_preset": VIDEO_DEFAULTS["av1_preset"],
    "av1_keyint": VIDEO_DEFAULTS["av1_keyint"],
    "av1_threads": VIDEO_DEFAULTS["av1_threads"],
}


def add_video_arguments(parser, inherit=False):
    for name in VIDEO_VARIANTS:
        parser.add_argument(
            f"--{name}", action=argparse.BooleanOptionalAction,
            default=None if inherit else VIDEO_DEFAULTS[name],
            help=f"enable/disable the final {name} MCAP (not temporary camera capture)",
        )
    for name in (
        "h264_crf",
        "h264_preset",
        "h264_keyint",
        "h264_threads",
        "av1_crf",
        "av1_preset",
        "av1_keyint",
        "av1_threads",
    ):
        parser.add_argument(
            "--" + name.replace("_", "-"),
            type=str if name == "h264_preset" else int,
            default=None if inherit else VIDEO_DEFAULTS[name],
        )


def video_options(arguments=None, saved=None):
    saved = dict(saved or {})
    if saved and "av1" not in saved:
        saved.update(LEGACY_AV1_DEFAULTS)
    result = {**VIDEO_DEFAULTS, **saved}
    if arguments is not None:
        result.update({key: getattr(arguments, key) for key in VIDEO_DEFAULTS
                       if getattr(arguments, key, None) is not None})
    if not all(isinstance(result[key], bool) for key in VIDEO_VARIANTS):
        raise ValueError("mjpeg, h264 and av1 switches must be boolean")
    if not any(result[name] for name in VIDEO_VARIANTS):
        raise ValueError("at least one video export must be enabled")
    for key, low, high in (("h264_crf", 1, 51), ("h264_keyint", 1, 10000),
                           ("h264_threads", 1, 16), ("av1_crf", 0, 63),
                           ("av1_preset", 0, 13), ("av1_keyint", 1, 10000),
                           ("av1_threads", 1, 16)):
        if type(result[key]) is not int or not low <= result[key] <= high:
            raise ValueError(f"{key} must be an integer within [{low}, {high}]")
    if result["h264_preset"] not in H264_PRESETS:
        raise ValueError(f"h264_preset must be one of {H264_PRESETS}")
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
                "profile": "High",
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


def _read_leb128(payload, offset):
    value = 0
    for shift in range(0, 56, 7):
        if offset >= len(payload):
            raise ValueError("truncated AV1 OBU size")
        byte = payload[offset]
        offset += 1
        value |= (byte & 0x7F) << shift
        if not byte & 0x80:
            return value, offset
    raise ValueError("invalid AV1 OBU size")


def _av1_obus(payload):
    """Return low-overhead AV1 OBUs as (type, complete bytes)."""
    payload = bytes(payload)
    offset = 0
    result = []
    while offset < len(payload):
        start = offset
        header = payload[offset]
        offset += 1
        if header & 0x81:
            raise ValueError("invalid AV1 OBU header")
        obu_type = (header >> 3) & 0x0F
        if header & 0x04:
            if offset >= len(payload):
                raise ValueError("truncated AV1 OBU extension")
            offset += 1
        if not header & 0x02:
            raise ValueError("AV1 output is not low-overhead OBU format")
        size, offset = _read_leb128(payload, offset)
        end = offset + size
        if end > len(payload):
            raise ValueError("truncated AV1 OBU payload")
        result.append((obu_type, payload[start:end]))
        offset = end
    if not result:
        raise ValueError("empty AV1 access unit")
    return result


def av1_obu_types(payload):
    return {obu_type for obu_type, _obu in _av1_obus(payload)}


class Av1Encoder:
    """JPEG to low-overhead AV1 access units, preserving input PTS."""

    def __init__(self, options, fps=30):
        try:
            import av
        except (ImportError, OSError) as error:
            raise RuntimeError("AV1 export requires PyAV") from error

        if type(fps) is not int or not 1 <= fps <= 240:
            raise ValueError("camera FPS must be an integer within [1, 240]")
        self.av = av
        self.options = options
        self.fps = fps
        self.decoder = av.CodecContext.create("mjpeg", "r")
        self.decoder.thread_count = 1
        self.encoder = None
        self.index = 0
        self._packets = 0
        self._sequence_header = None

    def encode(self, jpeg):
        frames = self.decoder.decode(self.av.Packet(jpeg))
        if len(frames) != 1:
            raise ValueError("JPEG must decode to exactly one image")
        return self.encode_frame(frames[0])

    def encode_frame(self, frame):
        if self.encoder is None:
            if frame.width % 2 or frame.height % 2:
                raise ValueError("AV1 yuv420p requires even image dimensions")
            try:
                codec = self.av.CodecContext.create("libsvtav1", "w")
            except Exception as error:
                raise RuntimeError(
                    "AV1 export requires FFmpeg with libsvtav1"
                ) from error
            codec.width, codec.height = frame.width, frame.height
            codec.pix_fmt = "yuv420p"
            codec.time_base = Fraction(1, self.fps)
            codec.framerate = Fraction(self.fps, 1)
            codec.thread_count = self.options["av1_threads"]
            codec.max_b_frames = 0
            codec.gop_size = self.options["av1_keyint"]
            codec.options = {
                "crf": str(self.options["av1_crf"]),
                "preset": str(self.options["av1_preset"]),
                "la_depth": "0",
                "svtav1-params": f"lp={self.options['av1_threads']}",
            }
            try:
                codec.open()
            except Exception as error:
                raise RuntimeError(
                    "failed to initialize libsvtav1 with the configured options"
                ) from error
            self.encoder = codec
        if (frame.width, frame.height) != (
            self.encoder.width,
            self.encoder.height,
        ):
            raise ValueError("camera resolution changed within an episode")
        frame = frame.reformat(format="yuv420p")
        frame.pict_type = self.av.video.frame.PictureType.NONE
        frame.pts, frame.time_base = self.index, self.encoder.time_base
        self.index += 1
        return [self._packet(packet) for packet in self.encoder.encode(frame)]

    def _packet(self, packet):
        if packet.pts is None:
            raise RuntimeError("AV1 packet has no presentation timestamp")
        obus = _av1_obus(packet)
        sequence = next((obu for obu_type, obu in obus if obu_type == 1), None)
        if sequence is not None:
            self._sequence_header = sequence
        if packet.is_keyframe and sequence is None:
            if self._sequence_header is None:
                raise RuntimeError("AV1 keyframe is missing its Sequence Header OBU")
            insert_at = 1 if obus[0][0] == 2 else 0
            obus.insert(insert_at, (1, self._sequence_header))
        if not {3, 6}.intersection(obu_type for obu_type, _obu in obus):
            raise RuntimeError("AV1 access unit does not contain a frame")
        if self._packets == 0 and not packet.is_keyframe:
            raise RuntimeError("AV1 stream must start at a keyframe")
        self._packets += 1
        return int(packet.pts), b"".join(obu for _obu_type, obu in obus)

    def finish(self):
        if self.encoder is None:
            return []
        packets = [self._packet(packet) for packet in self.encoder.encode(None)]
        if self._packets != self.index:
            raise RuntimeError("AV1 encoder changed the frame count")
        return packets


def _prepare_av1_frames(inputs, metadata, options, database_path):
    """Transcode cameras into a disk-backed PTS map before the ordered MCAP pass."""
    from mcap.reader import make_reader
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import CompressedImage as RosCompressedImage

    database = sqlite3.connect(database_path)
    try:
        database.execute(
            "CREATE TABLE frames (topic TEXT, frame_index INTEGER, data BLOB, "
            "PRIMARY KEY (topic, frame_index)) WITHOUT ROWID"
        )
        encoders = {}
        counts = Counter()

        def store(topic, packets):
            for frame_index, payload in packets:
                database.execute(
                    "INSERT INTO frames VALUES (?, ?, ?)",
                    (topic, frame_index, payload),
                )

        with ExitStack() as stack:
            readers = [
                make_reader(stack.enter_context(path.open("rb")), validate_crcs=True)
                for path in inputs
            ]
            messages = heapq.merge(
                *(reader.iter_messages() for reader in readers),
                key=lambda item: item[2].log_time,
            )
            for schema, channel, message in messages:
                topic = channel.topic
                if topic not in DAS_COMPRESSED_IMAGE_TOPICS:
                    continue
                if schema is None or schema.name != "sensor_msgs/msg/CompressedImage":
                    raise ValueError(f"unsupported camera schema for {topic}")
                frame = deserialize_message(message.data, RosCompressedImage)
                jpeg = bytes(frame.data)
                if not jpeg.startswith(b"\xff\xd8") or b"\xff\xd9" not in jpeg[-64:]:
                    raise ValueError(f"invalid JPEG in {topic}")
                if topic not in encoders:
                    side = topic.split("/")[3]
                    fps = (
                        metadata.get("camera_profiles", {})
                        .get(side, {})
                        .get("fps", 30)
                    )
                    encoders[topic] = Av1Encoder(options, fps)
                store(topic, encoders[topic].encode(jpeg))
                counts[topic] += 1
        for topic, encoder in encoders.items():
            store(topic, encoder.finish())
        database.commit()
        for topic, count in counts.items():
            stored = database.execute(
                "SELECT COUNT(*) FROM frames WHERE topic = ?", (topic,)
            ).fetchone()[0]
            if stored != count:
                raise RuntimeError(f"AV1 frame count mismatch for {topic}")
        return database
    except Exception:
        database.close()
        raise


def export_episode(episode_directory, options=None, *, add_missing=False):
    """Publish final/ atomically only after every selected variant passes CRC/count checks.

    With add_missing, publish only absent variants and retain existing exports.
    Source bags are deliberately retained for recovery and parameter retuning.
    Caller must hold the exclusive collection activity lock throughout processing.
    """
    from mcap.reader import make_reader
    from mcap.writer import CompressionType, Writer
    from foxglove_schemas_protobuf.CompressedVideo_pb2 import CompressedVideo
    from rclpy.serialization import deserialize_message, serialize_message
    from sensor_msgs.msg import CompressedImage as RosCompressedImage
    from diagnostic_msgs.msg import DiagnosticArray

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
    variants = [name for name in VIDEO_VARIANTS if options[name]]
    episode_id = metadata["episode_id"]
    if not re.fullmatch(r"episode_[A-Za-z0-9_]+", episode_id):
        raise ValueError("unsafe episode_id")
    requested = [final / f"{episode_id}.{variant}.mcap" for variant in variants]
    if add_missing:
        for target in requested:
            if target.is_symlink() or (target.exists() and not target.is_file()):
                raise ValueError(f"invalid export file: {target}")
        variants = [variant for variant in variants if not (final / f"{episode_id}.{variant}.mcap").exists()]
    # A temporary directory prevents readers from observing incomplete exports.
    with tempfile.TemporaryDirectory(prefix=".video-export-", dir=episode) as directory:
        staging = Path(directory) / "final"
        staging.mkdir()
        for variant in variants:
            target = staging / f"{episode_id}.{variant}.mcap"
            encoders = {}
            counts = Counter()
            camera_indices = Counter()
            with ExitStack() as stack:
                av1_frames = None
                if variant == "av1":
                    av1_frames = _prepare_av1_frames(
                        inputs,
                        metadata,
                        options,
                        Path(directory) / "av1.sqlite",
                    )
                    stack.callback(av1_frames.close)
                output = stack.enter_context(target.open("xb"))
                writer = Writer(output, compression=CompressionType.NONE, enable_data_crcs=True)
                # Mixed ROS2 CDR + Foxglove protobuf: do not claim the ros2 profile.
                writer.start(library="xr-marvin-teleop")
                image_schema = writer.register_schema(
                    name=CompressedVideo.DESCRIPTOR.full_name, encoding="protobuf",
                    data=protobuf_schema(CompressedVideo),
                ) if variant in ENCODED_VIDEO_VARIANTS else None
                schemas, channels = {}, {}
                readers = [make_reader(stack.enter_context(path.open("rb")), validate_crcs=True)
                           for path in inputs]
                messages = heapq.merge(*(r.iter_messages() for r in readers),
                                       key=lambda item: item[2].log_time)
                for schema, channel, message in messages:
                    topic = channel.topic
                    camera = topic in DAS_COMPRESSED_IMAGE_TOPICS
                    if camera:
                        if schema is None or schema.name != "sensor_msgs/msg/CompressedImage":
                            raise ValueError(f"unsupported camera schema for {topic}")
                        frame = deserialize_message(message.data, RosCompressedImage)
                        payload = bytes(frame.data)
                        if not payload.startswith(b"\xff\xd8") or b"\xff\xd9" not in payload[-64:]:
                            raise ValueError(f"invalid JPEG in {topic}")
                        output_topic = (
                            topic
                            if variant == "mjpeg"
                            else topic.replace("/image/compressed", "/video/compressed")
                        )
                        if output_topic not in channels:
                            camera_schema = image_schema if variant in ENCODED_VIDEO_VARIANTS else writer.register_schema(
                                name=schema.name, encoding=schema.encoding, data=schema.data)
                            channels[output_topic] = writer.register_channel(
                                topic=output_topic, message_encoding="protobuf" if variant in ENCODED_VIDEO_VARIANTS else "cdr",
                                schema_id=camera_schema,
                            )
                            if variant == "h264":
                                side = topic.split("/")[3]
                                fps = metadata.get("camera_profiles", {}).get(side, {}).get("fps", 30)
                                encoders[topic] = H264Encoder(options, fps)
                        if variant == "h264":
                            payload = encoders[topic].encode(payload)
                        elif variant == "av1":
                            frame_index = camera_indices[topic]
                            row = av1_frames.execute(
                                "SELECT data FROM frames WHERE topic = ? AND frame_index = ?",
                                (topic, frame_index),
                            ).fetchone()
                            if row is None:
                                raise RuntimeError(
                                    f"missing AV1 frame {frame_index} for {topic}"
                                )
                            payload = bytes(row[0])
                            camera_indices[topic] += 1
                        if variant in ENCODED_VIDEO_VARIANTS:
                            converted = CompressedVideo(frame_id=frame.header.frame_id, data=payload, format=variant)
                            converted.timestamp.FromNanoseconds(message.log_time)
                            serialized = converted.SerializeToString()
                        else:
                            serialized = message.data
                        writer.add_message(channels[output_topic], message.log_time,
                                           serialized, message.log_time,
                                           sequence=message.sequence)
                        counts[output_topic] += 1
                    else:
                        payload = message.data
                        if variant in ENCODED_VIDEO_VARIANTS and topic in DAS_COMPRESSED_IMAGE_STATUS_TOPICS:
                            status = deserialize_message(payload, DiagnosticArray)
                            for entry in status.status[0].values:
                                if entry.key == "topics":
                                    entry.value = json.dumps([name.replace("/image/compressed", "/video/compressed")
                                                             for name in json.loads(entry.value)])
                            payload = serialize_message(status)
                            topic = topic.replace("/image/compressed", "/video/compressed")
                        if topic not in channels:
                            if schema is None or not schema.data:
                                raise ValueError(f"missing embedded schema for {topic}")
                            key = (schema.name, schema.encoding, schema.data)
                            if key not in schemas:
                                schemas[key] = writer.register_schema(schema.name, schema.encoding, schema.data)
                            channels[topic] = writer.register_channel(
                                topic, channel.message_encoding, schemas[key], metadata=channel.metadata,
                            )
                        writer.add_message(channels[topic], message.log_time, payload,
                                           message.log_time, sequence=message.sequence)
                        counts[topic] += 1
                for encoder in encoders.values():
                    encoder.finish()
                if av1_frames is not None:
                    stored = av1_frames.execute(
                        "SELECT COUNT(*) FROM frames"
                    ).fetchone()[0]
                    if stored != sum(camera_indices.values()):
                        raise RuntimeError("not all AV1 frames were written")
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
