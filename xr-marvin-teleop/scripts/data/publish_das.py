#!/usr/bin/env python3
"""Own one DAS serial device and expose its control/state through ROS2."""

import argparse
import math
import queue
import threading
import time
from pathlib import Path

from xr_marvin_teleop.hardware.interface.das_finger import (
    ARM_NAMES,
    DASFingerCalibrationRequired,
    _decode_encoder_value,
    closedness_to_das_distances,
    load_das_finger_configurations,
    load_das_sdk,
)


class DasSidePublisher:
    """Keep camera and opposite-side work out of one DAS serial process."""

    def __init__(
        self,
        side,
        configurations,
        sdk_root,
        ready_timeout_seconds=10.0,
        encoder_stale_timeout_seconds=0.5,
    ):
        try:
            import rclpy
            from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
            from rclpy.qos import (
                DurabilityPolicy,
                HistoryPolicy,
                QoSProfile,
                ReliabilityPolicy,
            )
            from teleop_msgs.msg import DasState, GripperCommand, TactileFrame
        except (ImportError, OSError) as error:
            raise RuntimeError(
                "DAS publishing requires sourced ROS2 and teleop_msgs"
            ) from error

        self.side = side
        self.arm_index = ARM_NAMES.index(side)
        self.configurations = tuple(configurations)
        self.configuration = self.configurations[self.arm_index]
        self.ready_timeout_seconds = float(ready_timeout_seconds)
        self.encoder_stale_timeout_seconds = float(
            encoder_stale_timeout_seconds
        )
        if (
            self.ready_timeout_seconds <= 0.0
            or self.encoder_stale_timeout_seconds <= 0.0
        ):
            raise ValueError("DAS timeouts must be positive")

        critical_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._rclpy = rclpy
        self._types = {
            "DasState": DasState,
            "TactileFrame": TactileFrame,
            "DiagnosticArray": DiagnosticArray,
            "DiagnosticStatus": DiagnosticStatus,
            "KeyValue": KeyValue,
        }
        self._owns_context = not rclpy.ok()
        if self._owns_context:
            rclpy.init()
        self._node = rclpy.create_node(f"das_{side}_source")
        self._state_publisher = self._node.create_publisher(
            DasState, f"/raw/das/{side}/state", critical_qos
        )
        self._tactile_publisher = self._node.create_publisher(
            TactileFrame, f"/raw/das/{side}/tactile", sensor_qos
        )
        self._diagnostic_publisher = self._node.create_publisher(
            DiagnosticArray, "/diagnostics", critical_qos
        )
        self._command_subscription = self._node.create_subscription(
            GripperCommand,
            "/command/das/target",
            self._handle_command,
            critical_qos,
        )

        self._state_queue = queue.Queue(maxsize=128)
        self._tactile_queue = queue.Queue(maxsize=32)
        self._target_lock = threading.Lock()
        self._target_m = self.configuration.startup_distance_m
        self._last_encoder_steady_ns = 0
        self._encoder_ready = threading.Event()
        self._calibration_required = False
        self._error = None
        self._state_sequence = 0
        self._tactile_sequence = 0
        self._last_command_sequence = 0
        self._drops = {"state": 0, "tactile": 0}
        self._bus = None

        try:
            data_bus_type = load_das_sdk(sdk_root).DataBus
            self._bus = data_bus_type(
                tty_port=self.configuration.serial_port,
                baudrate=921600,
                encoder_freq=30,
                tactile_freq=self.configuration.tactile_hz,
                encoder_callback=self._handle_encoder,
                tactile_callback=self._handle_tactile,
                initial_distance_m=self.configuration.startup_distance_m,
            )
            if self._encoder_ready.is_set():
                self._bus.set_target_distance(self._target_m)
        except Exception:
            self.close()
            raise

    def _put_latest(self, target_queue, value, stream):
        try:
            target_queue.put_nowait(value)
            return
        except queue.Full:
            self._drops[stream] += 1
        try:
            target_queue.get_nowait()
        except queue.Empty:
            pass
        target_queue.put_nowait(value)

    def _handle_encoder(self, record_data):
        wall_time_ns = time.time_ns()
        steady_ns = time.monotonic_ns()
        try:
            distance_m = _decode_encoder_value(record_data)
            first_valid = not self._encoder_ready.is_set()
            if first_valid:
                with self._target_lock:
                    self._target_m = distance_m
                if self._bus is not None:
                    self._bus.set_target_distance(distance_m)
            with self._target_lock:
                target_m = self._target_m
            self._last_encoder_steady_ns = steady_ns
            self._encoder_ready.set()
            self._calibration_required = False
            valid = True
            status_flags = 0
        except DASFingerCalibrationRequired:
            distance_m = math.nan
            with self._target_lock:
                target_m = self._target_m
            self._calibration_required = True
            valid = False
            status_flags = 1
        except Exception as error:
            distance_m = math.nan
            with self._target_lock:
                target_m = self._target_m
            self._error = error
            valid = False
            status_flags = 2
        self._state_sequence += 1
        self._put_latest(
            self._state_queue,
            (
                self._state_sequence,
                wall_time_ns,
                steady_ns,
                valid,
                distance_m,
                target_m,
                status_flags,
            ),
            "state",
        )

    def _handle_tactile(self, record_data):
        self._tactile_sequence += 1
        self._put_latest(
            self._tactile_queue,
            (
                self._tactile_sequence,
                time.time_ns(),
                time.monotonic_ns(),
                bytes(record_data),
            ),
            "tactile",
        )

    def _handle_command(self, message):
        sequence_id = int(message.sequence_id)
        if sequence_id <= self._last_command_sequence and not (
            sequence_id == 1 and self._last_command_sequence > 1
        ):
            return
        if not self._encoder_ready.is_set():
            return
        try:
            target_m = closedness_to_das_distances(
                message.closedness, self.configurations
            )[self.arm_index]
            self._bus.set_target_distance(target_m)
        except Exception as error:
            self._error = error
            return
        self._last_command_sequence = sequence_id
        with self._target_lock:
            self._target_m = target_m

    @staticmethod
    def _stamp(header, wall_time_ns, frame_id):
        header.stamp.sec, header.stamp.nanosec = divmod(
            int(wall_time_ns), 1_000_000_000
        )
        header.frame_id = frame_id

    def _publish_pending(self):
        while True:
            try:
                item = self._state_queue.get_nowait()
            except queue.Empty:
                break
            sequence, wall, steady, valid, distance, target, flags = item
            message = self._types["DasState"]()
            self._stamp(message.header, wall, f"finger_{self.side}")
            message.sequence_id = sequence
            message.source_timestamp_ns = 0
            message.receive_steady_ns = steady
            message.valid = valid
            message.side = self.side
            message.distance_m = distance
            message.target_distance_m = target
            message.status_flags = flags
            self._state_publisher.publish(message)
        while True:
            try:
                sequence, wall, steady, payload = self._tactile_queue.get_nowait()
            except queue.Empty:
                break
            message = self._types["TactileFrame"]()
            self._stamp(message.header, wall, f"finger_{self.side}")
            message.sequence_id = sequence
            message.source_timestamp_ns = 0
            message.receive_steady_ns = steady
            message.valid = True
            message.side = self.side
            message.data = payload
            self._tactile_publisher.publish(message)

    def _publish_diagnostics(self):
        message = self._types["DiagnosticArray"]()
        self._stamp(message.header, time.time_ns(), "")
        status = self._types["DiagnosticStatus"]()
        status.name = f"das_{self.side}_source"
        status.hardware_id = self.configuration.serial_port
        status.level = status.ERROR if self._error else status.OK
        status.message = str(self._error) if self._error else "running"
        age_ns = (
            0
            if not self._last_encoder_steady_ns
            else time.monotonic_ns() - self._last_encoder_steady_ns
        )
        status.values = [
            self._types["KeyValue"](
                key="last_encoder_age_ms", value=f"{age_ns / 1e6:.3f}"
            ),
            *(
                self._types["KeyValue"](
                    key=f"dropped_{name}", value=str(count)
                )
                for name, count in self._drops.items()
            ),
        ]
        message.status = [status]
        self._diagnostic_publisher.publish(message)

    def run(self):
        ready_deadline = time.monotonic() + self.ready_timeout_seconds
        announced_ready = False
        next_diagnostic_time = time.monotonic()
        while self._rclpy.ok():
            self._rclpy.spin_once(self._node, timeout_sec=0.01)
            self._publish_pending()
            if self._error is not None:
                raise RuntimeError(
                    f"DAS {self.side} source failed: {self._error}"
                ) from self._error
            now = time.monotonic()
            if not self._encoder_ready.is_set() and now >= ready_deadline:
                if self._calibration_required:
                    raise TimeoutError(
                        f"DAS encoder calibration is still required for {self.side} "
                        "(returned -66.66)"
                    )
                raise TimeoutError(
                    f"DAS did not provide encoder feedback for {self.side} within "
                    f"{self.ready_timeout_seconds:g} seconds"
                )
            if self._encoder_ready.is_set() and (
                time.monotonic_ns() - self._last_encoder_steady_ns
                > self.encoder_stale_timeout_seconds * 1e9
            ):
                raise TimeoutError(
                    f"DAS encoder feedback stale for {self.side}"
                )
            if now >= next_diagnostic_time:
                self._publish_diagnostics()
                next_diagnostic_time = now + 1.0
            if self._encoder_ready.is_set() and not announced_ready:
                announced_ready = True
                print(
                    f"DAS {self.side} source ready; encoder={self._target_m:.6f} m",
                    flush=True,
                )

    def close(self):
        bus, self._bus = self._bus, None
        if bus is not None:
            bus.stop()
        node, self._node = getattr(self, "_node", None), None
        if node is not None:
            node.destroy_node()
        if getattr(self, "_owns_context", False) and self._rclpy.ok():
            self._rclpy.shutdown()


def main(arguments=None):
    parser = argparse.ArgumentParser(description="Publish one DAS side through ROS2")
    parser.add_argument("--side", required=True, choices=ARM_NAMES)
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--sdk-root", required=True, type=Path)
    parser.add_argument("--ready-timeout", type=float, default=10.0)
    parser.add_argument("--encoder-stale-timeout", type=float, default=0.5)
    parsed = parser.parse_args(arguments)

    publisher = DasSidePublisher(
        parsed.side,
        load_das_finger_configurations(parsed.config),
        parsed.sdk_root,
        parsed.ready_timeout,
        parsed.encoder_stale_timeout,
    )
    try:
        publisher.run()
    except KeyboardInterrupt:
        pass
    finally:
        publisher.close()


if __name__ == "__main__":
    main()
