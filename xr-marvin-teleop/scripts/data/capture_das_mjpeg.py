#!/usr/bin/env python3
"""Write one camera's native V4L2 MJPEG frames directly to an MCAP bag."""

import argparse
import os
import signal
import threading
import time
from pathlib import Path


PREVIEW_FPS = 30


def parse_resolution(value):
    try:
        width, height = (int(item) for item in value.lower().split("x", 1))
    except (AttributeError, TypeError, ValueError) as error:
        raise ValueError(f"invalid camera resolution: {value!r}") from error
    if width <= 0 or height <= 0:
        raise ValueError("camera resolution must be positive")
    return width, height


def pipeline_description(device, width, height, fps):
    if '"' in device or "\\" in device:
        raise ValueError("camera device path contains unsupported characters")
    return (
        f'v4l2src device="{device}" do-timestamp=true '
        f'! image/jpeg,width={width},height={height},framerate={fps}/1 '
        '! appsink name=sink emit-signals=false sync=false max-buffers=2 drop=true'
    )


def is_jpeg(payload):
    return (
        len(payload) >= 4
        and payload.startswith(b"\xff\xd8")
        and payload.rfind(b"\xff\xd9") >= len(payload) - 64
    )


def write_preview_file(path, payload):
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    try:
        temporary.write_bytes(payload)
        temporary.replace(path)
    finally:
        temporary.unlink(missing_ok=True)


class NativeMjpegWriter:
    def __init__(
        self,
        side,
        device,
        resolution,
        fps,
        output,
        storage_config,
        ready_file=None,
        preview_file=None,
    ):
        try:
            import gi

            gi.require_version("Gst", "1.0")
            from gi.repository import Gst
            import rosbag2_py
            from rclpy.serialization import serialize_message
            from teleop_msgs.msg import CompressedImageFrame
        except (ImportError, OSError, ValueError) as error:
            raise RuntimeError(
                "native MJPEG recording requires GStreamer, rosbag2_py, and "
                "the latest teleop_msgs"
            ) from error

        self.side = side
        self.device = str(device)
        self.width, self.height = parse_resolution(resolution)
        self.fps = int(fps)
        if self.fps <= 0:
            raise ValueError("camera FPS must be positive")
        self.output = Path(output).expanduser().resolve()
        self.storage_config = Path(storage_config).expanduser().resolve()
        self.ready_file = (
            None if ready_file is None else Path(ready_file).expanduser().resolve()
        )
        self.preview_file = (
            None if preview_file is None else Path(preview_file).expanduser().resolve()
        )
        self._next_preview_ns = 0
        self._preview_disabled = False
        if self.output.exists():
            raise FileExistsError(f"camera bag already exists: {self.output}")
        if not self.storage_config.is_file():
            raise FileNotFoundError(
                f"MCAP storage configuration not found: {self.storage_config}"
            )

        self._Gst = Gst
        self._rosbag2_py = rosbag2_py
        self._serialize_message = serialize_message
        self._message_type = CompressedImageFrame
        self._stop_event = threading.Event()
        self._completed = False
        self._sequence = 0
        self._last_buffer_offset = None
        self._temporary_output = self.output.parent / (
            f".{self.output.name}.{os.getpid()}.tmp"
        )
        Gst.init(None)
        self._pipeline = Gst.parse_launch(
            pipeline_description(
                self.device, self.width, self.height, self.fps
            )
        )
        self._sink = self._pipeline.get_by_name("sink")
        if self._sink is None:
            raise RuntimeError("GStreamer pipeline has no appsink")
        self._writer = rosbag2_py.SequentialWriter()
        self._writer.open(
            rosbag2_py.StorageOptions(
                uri=str(self._temporary_output),
                storage_id="mcap",
                storage_config_uri=str(self.storage_config),
            ),
            rosbag2_py.ConverterOptions("", ""),
        )
        self.topic = f"/raw/das/{side}/image/compressed"
        self._writer.create_topic(
            rosbag2_py.TopicMetadata(
                name=self.topic,
                type="teleop_msgs/msg/CompressedImageFrame",
                serialization_format="cdr",
            )
        )

    def _write_preview(self, payload, steady_ns):
        if (
            self.preview_file is None
            or self._preview_disabled
            or steady_ns < self._next_preview_ns
        ):
            return
        self._next_preview_ns = steady_ns + 1_000_000_000 // PREVIEW_FPS
        try:
            write_preview_file(self.preview_file, payload)
        except OSError as error:
            self._preview_disabled = True
            print(f"Preview disabled for {self.side}: {error}", flush=True)

    def request_stop(self, _signal_number=None, _frame=None):
        self._stop_event.set()

    def _check_caps(self, sample):
        structure = sample.get_caps().get_structure(0)
        media_type = structure.get_name()
        width = structure.get_value("width")
        height = structure.get_value("height")
        if media_type != "image/jpeg" or (width, height) != (
            self.width,
            self.height,
        ):
            raise RuntimeError(
                "camera did not negotiate native MJPEG at the requested "
                f"resolution: {media_type} {width}x{height}"
            )

    def _sequence_id(self, buffer):
        none_value = self._Gst.BUFFER_OFFSET_NONE
        offset = int(buffer.offset)
        if offset != none_value and (
            self._last_buffer_offset is None or offset > self._last_buffer_offset
        ):
            self._last_buffer_offset = offset
            self._sequence = max(self._sequence + 1, offset + 1)
            return self._sequence
        self._sequence += 1
        return self._sequence

    def _write_sample(self, sample):
        receive_steady_ns = time.monotonic_ns()
        wall_time_ns = time.time_ns()
        self._check_caps(sample)
        buffer = sample.get_buffer()
        mapped, map_info = buffer.map(self._Gst.MapFlags.READ)
        if not mapped:
            raise RuntimeError("failed to map MJPEG camera buffer")
        try:
            payload = bytes(map_info.data)
        finally:
            buffer.unmap(map_info)
        if not is_jpeg(payload):
            raise RuntimeError("camera returned a non-JPEG payload")
        self._write_preview(payload, receive_steady_ns)

        message = self._message_type()
        message.image.header.stamp.sec, message.image.header.stamp.nanosec = divmod(
            wall_time_ns, 1_000_000_000
        )
        message.image.header.frame_id = f"finger_{self.side}_camera"
        message.image.format = "jpeg"
        message.image.data = payload
        message.sequence_id = self._sequence_id(buffer)
        message.source_timestamp_ns = (
            0
            if buffer.pts == self._Gst.CLOCK_TIME_NONE
            else int(buffer.pts)
        )
        message.receive_steady_ns = receive_steady_ns
        self._writer.write(
            self.topic,
            self._serialize_message(message),
            wall_time_ns,
        )

    def run(self):
        state_change = self._pipeline.set_state(self._Gst.State.PLAYING)
        if state_change == self._Gst.StateChangeReturn.FAILURE:
            raise RuntimeError(f"failed to start MJPEG camera: {self.device}")
        self._pipeline.get_state(5 * self._Gst.SECOND)
        bus = self._pipeline.get_bus()
        announced_ready = False
        while not self._stop_event.is_set():
            sample = self._sink.emit("try-pull-sample", 100 * self._Gst.MSECOND)
            if sample is not None:
                self._write_sample(sample)
                if not announced_ready:
                    announced_ready = True
                    if self.ready_file is not None:
                        self.ready_file.write_text("ready\n", encoding="utf-8")
                    print(
                        f"MJPEG {self.side} ready: {self.device} "
                        f"{self.width}x{self.height}@{self.fps}",
                        flush=True,
                    )
                continue
            event = bus.timed_pop_filtered(
                0, self._Gst.MessageType.ERROR | self._Gst.MessageType.EOS
            )
            if event is None:
                continue
            if event.type == self._Gst.MessageType.ERROR:
                error, details = event.parse_error()
                raise RuntimeError(f"GStreamer camera error: {error}; {details}")
            raise RuntimeError("GStreamer camera stream ended")
        self._completed = True

    def close(self):
        pipeline, self._pipeline = getattr(self, "_pipeline", None), None
        if pipeline is not None:
            pipeline.set_state(self._Gst.State.NULL)
        writer, self._writer = getattr(self, "_writer", None), None
        if writer is not None:
            writer.close()
        if (
            self._completed
            and self._temporary_output.exists()
            and not self.output.exists()
        ):
            self._temporary_output.replace(self.output)
        if self.preview_file is not None:
            try:
                self.preview_file.unlink(missing_ok=True)
            except OSError:
                pass


def main(arguments=None):
    parser = argparse.ArgumentParser(description="Record native DAS MJPEG to MCAP")
    parser.add_argument("--side", required=True, choices=("left", "right"))
    parser.add_argument("--device", required=True)
    parser.add_argument("--resolution", default="640x480")
    parser.add_argument("--fps", type=int, default=60)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--storage-config", required=True, type=Path)
    parser.add_argument("--ready-file", type=Path)
    parser.add_argument("--preview-file", type=Path)
    parsed = parser.parse_args(arguments)

    writer = NativeMjpegWriter(
        parsed.side,
        parsed.device,
        parsed.resolution,
        parsed.fps,
        parsed.output,
        parsed.storage_config,
        parsed.ready_file,
        parsed.preview_file,
    )
    previous_handlers = {
        signal_number: signal.signal(signal_number, writer.request_stop)
        for signal_number in (signal.SIGINT, signal.SIGTERM)
    }
    try:
        writer.run()
    finally:
        writer.close()
        for signal_number, handler in previous_handlers.items():
            signal.signal(signal_number, handler)


if __name__ == "__main__":
    main()
