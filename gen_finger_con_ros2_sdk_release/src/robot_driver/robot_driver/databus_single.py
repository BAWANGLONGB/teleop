#!/usr/bin/env python3
# ROS2 port of databus_single.py (Gen Finger)
import sys
import os
import serial
import threading
import time
import queue
import traceback
import struct
import subprocess

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int8MultiArray

from .pack import CmdPack, MessagePack, Opcode, RecordType
from .das_protocol import DASProtocol

TOPIC_LEFT_TACTILE = '/das_controller/tactile_single_l'
TOPIC_RIGHT_TACTILE = '/das_controller/tactile_single_r'
TOPIC_ENCODER = '/das_controller/encoder_data'
TOPIC_TARGET_DISTANCE = '/das_controller/target_dis'


class DataBusNode(Node):
    """ROS2 node for Gen Finger DAS serial interface."""

    def __init__(self,
                 tty_port="",
                 baudrate=115200,
                 timeout=0.5,
                 is_calib_cmd=False,
                 calib_cmd_name: str = None,
                 encoder_freq: float = None,
                 tactile_freq: float = None):
        super().__init__('das_ros_interface')

        self.declare_parameter('topic_left_tactile', TOPIC_LEFT_TACTILE)
        self.declare_parameter('topic_right_tactile', TOPIC_RIGHT_TACTILE)
        self.declare_parameter('topic_encoder', TOPIC_ENCODER)
        self.declare_parameter('topic_target_distance', TOPIC_TARGET_DISTANCE)
        self.declare_parameter('serial_port', '')
        self.declare_parameter('side', '')

        self.topic_left_tactile = self.get_parameter('topic_left_tactile').value
        self.topic_right_tactile = self.get_parameter('topic_right_tactile').value
        self.topic_encoder = self.get_parameter('topic_encoder').value
        self.topic_target_distance = self.get_parameter('topic_target_distance').value
        param_serial_port = self.get_parameter('serial_port').value or ''
        param_side = self.get_parameter('side').value or ''

        if not tty_port:
            tty_port = param_serial_port
        if not tty_port and param_side in ('left', 'right'):
            tty_port = find_finger_serial_by_side(param_side, verbose=False)
        if not tty_port:
            tty_port = find_serial_port(side=param_side or None)
        if not tty_port:
            raise RuntimeError("No serial port; check device connection or set serial_port")

        self.pub_tactile_left = self.create_publisher(Int8MultiArray, self.topic_left_tactile, 10)
        self.pub_tactile_right = self.create_publisher(Int8MultiArray, self.topic_right_tactile, 10)
        self.pub_encoder = self.create_publisher(Float32, self.topic_encoder, 10)

        self.tty_port = tty_port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None
        self.is_running = False

        self._open_serial_success = False
        self.protocol: DASProtocol = DASProtocol()
        self.data_buffer: bytes = b""
        self.data_buffer_lock = threading.Lock()
        self.serial_lock = threading.Lock()

        self.cmd_queue = queue.Queue(1000)

        self.read_thread: threading.Thread = None
        self.parse_thread: threading.Thread = None
        self.send_thread: threading.Thread = None

        self.encoder_freq = encoder_freq
        self.tactile_freq = tactile_freq
        self.encoder_thread: threading.Thread = None
        self.tactile_thread: threading.Thread = None

        self.finger_dis = 0.0
        self.angle_lock = threading.Lock()
        self.is_calib_cmd = is_calib_cmd
        self.calib_cmd_name = calib_cmd_name
        if calib_cmd_name:
            os.environ["CALIB_CMD_NAME"] = calib_cmd_name

        self.motor_cmd_subscriber = self.create_subscription(
            Float32,
            self.topic_target_distance,
            self._motor_command_callback,
            10,
        )

        self._open_serial()
        self.is_running = True
        self._start_reading()
        self._start_parsing()
        self._start_sending()

        if self.encoder_freq:
            self._start_encoder_loop()
        if self.tactile_freq:
            self._start_tactile_loop()

    def _motor_command_callback(self, msg):
        try:
            with self.angle_lock:
                self.finger_dis = msg.data
        except Exception as e:
            self.get_logger().error(f"Motor command handling error: {e}")

    def tactile_callback(self, record_data: bytes):
        if len(record_data) != 448:
            self.get_logger().warn(f"Bad data length: expected 448 bytes, got {len(record_data)}")
            return

        try:
            raw_left_224 = [struct.unpack("B", record_data[i:i + 1])[0] for i in range(0, 224)]
            raw_right_224 = [struct.unpack("B", record_data[i:i + 1])[0] for i in range(224, 448)]

            left_expanded_448 = []
            for val in raw_left_224:
                left_expanded_448.append(val)
                left_expanded_448.append(val)

            right_expanded_448 = []
            for val in raw_right_224:
                right_expanded_448.append(val)
                right_expanded_448.append(val)

            total_grid = [[0 for _ in range(10)] for _ in range(100)]

            left_neg_coords = [
                (0, 0), (0, 1), (0, 2), (1, 0), (1, 1), (2, 0),
                (0, 7), (0, 8), (0, 9), (1, 8), (1, 9), (2, 9),
                (49, 0), (49, 1), (49, 2), (49, 3),
                (48, 0), (48, 1), (48, 2), (48, 3),
                (47, 0), (47, 1), (47, 2),
                (46, 0), (46, 1), (46, 2),
                (45, 0), (45, 1), (45, 2),
                (44, 0), (44, 1),
                (43, 0),
                (49, 6), (49, 7), (49, 8), (49, 9),
                (48, 6), (48, 7), (48, 8), (48, 9),
                (47, 7), (47, 8), (47, 9),
                (46, 7), (46, 8), (46, 9),
                (45, 7), (45, 8), (45, 9),
                (44, 8), (44, 9),
                (43, 9),
            ]

            right_neg_coords = [
                (50, 0), (50, 1), (50, 2), (51, 0), (51, 1), (52, 0),
                (50, 7), (50, 8), (50, 9), (51, 8), (51, 9), (52, 9),
                (99, 0), (99, 1), (99, 2), (99, 3),
                (98, 0), (98, 1), (98, 2), (98, 3),
                (97, 0), (97, 1), (97, 2),
                (96, 0), (96, 1), (96, 2),
                (95, 0), (95, 1), (95, 2),
                (94, 0), (94, 1),
                (93, 0),
                (99, 6), (99, 7), (99, 8), (99, 9),
                (98, 6), (98, 7), (98, 8), (98, 9),
                (97, 7), (97, 8), (97, 9),
                (96, 7), (96, 8), (96, 9),
                (95, 7), (95, 8), (95, 9),
                (94, 8), (94, 9),
                (93, 9),
            ]

            for (r, c) in left_neg_coords:
                total_grid[r][c] = -1
            for (r, c) in right_neg_coords:
                total_grid[r][c] = -1

            left_idx = 0
            for row in range(50):
                for col in range(10):
                    if total_grid[row][col] != -1 and left_idx < len(left_expanded_448):
                        total_grid[row][col] = left_expanded_448[left_idx]
                        left_idx += 1

            right_idx = 0
            for row in range(50, 100):
                for col in range(10):
                    if total_grid[row][col] != -1 and right_idx < len(right_expanded_448):
                        total_grid[row][col] = right_expanded_448[right_idx]
                        right_idx += 1

            left_flat = []
            for row in range(50):
                left_flat.extend(total_grid[row])

            right_flat = []
            for row in range(50, 100):
                right_flat.extend(total_grid[row])

            msg_left = Int8MultiArray()
            msg_left.data = [x if x == -1 else (x if x < 128 else x - 256) for x in left_flat]

            msg_right = Int8MultiArray()
            msg_right.data = [x if x == -1 else (x if x < 128 else x - 256) for x in right_flat]

            self.pub_tactile_left.publish(msg_left)
            self.pub_tactile_right.publish(msg_right)

        except Exception as e:
            self.get_logger().error(f"Tactile processing error: {e}")

    def encoder_callback(self, record_data: bytes):
        encoder_value = struct.unpack(">f", record_data)[0]
        try:
            msg = Float32()
            msg.data = float(encoder_value)
            self.pub_encoder.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Error publishing encoder: {e}")

    def echo_callback(self, record_data: bytes):
        self.get_logger().debug("echo data: {}".format(record_data))

    def camera_calib_callback(self, camera_pack):
        pass

    def drive_motor(self, angle_dgree: float):
        self.add_cmd(
            CmdPack.pack(
                opcode=Opcode.WriteDrive,
                record_type=RecordType.Drive,
                record=struct.pack(">f", angle_dgree),
            )
        )

    def disable_motor(self):
        self.add_cmd(
            CmdPack.pack(
                opcode=Opcode.DisableDrive,
                record_type=RecordType.Drive,
            )
        )

    def calib_encoder(self):
        self.add_cmd(
            CmdPack.pack(
                opcode=Opcode.CalibEncoder,
                record_type=RecordType.Drive,
            )
        )

    def send_camera_calib_cmd(self, camera_cmd: str):
        try:
            cmd = CmdPack.pack_calib(record=camera_cmd.encode('utf-8'))
            success = self.add_cmd(cmd)
            if success:
                self.get_logger().debug(f"Sent camera calib command: {camera_cmd}")
            else:
                self.get_logger().warn(f"Failed to queue camera calib command: {camera_cmd}")
            return success
        except Exception as e:
            self.get_logger().error(f"Error sending camera calib command: {e}")
            return False

    def add_cmd(self, cmd: CmdPack) -> bool:
        try:
            self.cmd_queue.put(cmd, block=True, timeout=1)
            return True
        except queue.Full:
            self.get_logger().warn("Command queue full, add failed")
            return False

    def is_opend(self):
        return self._open_serial_success

    def _open_serial(self):
        try:
            self.ser = serial.Serial()
            self.ser.port = self.tty_port
            self.ser.baudrate = self.baudrate
            self.ser.timeout = self.timeout
            self.ser.parity = serial.PARITY_NONE
            self.ser.stopbits = serial.STOPBITS_ONE
            self.ser.bytesize = serial.EIGHTBITS
            self.ser.dsrdtr = False
            self.ser.dtr = True
            self.ser.rts = False
            self.ser.open()

            if self.ser.is_open:
                self.get_logger().debug(f"open {self.tty_port} success, baudrate: {self.baudrate}")
                self._open_serial_success = True
            else:
                self.get_logger().error(f"open {self.tty_port} failed, baudrate: {self.baudrate}")
                self._open_serial_success = False
        except Exception as e:
            self.get_logger().error(f"Serial open error: {e}")
            self._open_serial_success = False

    def _start_reading(self):
        self.read_thread = threading.Thread(target=self._reading_loop)
        self.read_thread.daemon = True
        self.read_thread.start()

    def _start_parsing(self):
        self.parse_thread = threading.Thread(target=self._parsing_loop)
        self.parse_thread.daemon = True
        self.parse_thread.start()

    def _start_encoder_loop(self):
        self.encoder_thread = threading.Thread(target=self._send_encoder_loop)
        self.encoder_thread.daemon = True
        self.encoder_thread.start()

    def _start_tactile_loop(self):
        self.tactile_thread = threading.Thread(target=self._send_tactile_loop)
        self.tactile_thread.daemon = True
        self.tactile_thread.start()

    def _start_sending(self):
        self.send_thread = threading.Thread(target=self._sending_loop)
        self.send_thread.daemon = True
        self.send_thread.start()

    def _sending_loop(self):
        while self.is_running and rclpy.ok():
            try:
                cmd: CmdPack = self.cmd_queue.get(block=True, timeout=0.1)
                with self.serial_lock:
                    if self.ser and self.ser.is_open:
                        self.ser.write(cmd.data)
                        self.ser.flush()
            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().debug(f"Send error: {e}")
                time.sleep(0.01)

    def _reading_loop(self):
        while self.is_running and rclpy.ok():
            try:
                with self.serial_lock:
                    if self.ser and self.ser.is_open:
                        n = self.ser.inWaiting()
                        if n:
                            data = self.ser.read(n)
                            with self.data_buffer_lock:
                                self.data_buffer = self.data_buffer + data
            except Exception as e:
                self.get_logger().debug(f"Read loop error: {e}")
                time.sleep(0.1)

            time.sleep(0.001)

    def _parsing_loop(self):
        while self.is_running and rclpy.ok():
            with self.data_buffer_lock:
                if len(self.data_buffer) > 0:
                    packets, remain = DASProtocol.find_packet(self.data_buffer)
                    self.data_buffer = remain

                    for packet in packets:
                        if self.is_calib_cmd:
                            magic = DASProtocol.MAGIC
                            if (
                                len(packet) > 2 * len(magic)
                                and packet.startswith(magic)
                                and packet.endswith(magic)
                            ):
                                middle = packet[len(magic):-len(magic)]
                                try:
                                    text = middle.decode("ascii")
                                except Exception:
                                    text = middle.hex()
                                if self.calib_cmd_name == "MCUID":
                                    print("MCUID:", text)
                                else:
                                    print(f"Device response ({self.calib_cmd_name}): {text}")
                                self.is_calib_cmd = False
                                continue

                            camera_pack = MessagePack.unpack_camera_calib(packet)
                            if camera_pack:
                                if self.camera_calib_callback:
                                    self.camera_calib_callback(camera_pack)
                                self.is_calib_cmd = False
                                continue

                            pack = MessagePack.unpack(packet)
                            if pack:
                                for record in pack.records_:
                                    if record.record_type == RecordType.Echo:
                                        try:
                                            text = record.record_data.decode("utf-8")
                                        except Exception:
                                            text = record.record_data.hex()
                                        print(f"Device response ({self.calib_cmd_name}): {text}")
                                        self.is_calib_cmd = False
                                        break
                        else:
                            pack = MessagePack.unpack(packet)
                            if not pack:
                                continue

                            for record in pack.records_:
                                if record.record_type == RecordType.Tactile:
                                    self.tactile_callback(record.record_data)
                                elif record.record_type == RecordType.Encoder:
                                    self.encoder_callback(record.record_data)
                                elif record.record_type == RecordType.Echo:
                                    self.echo_callback(record.record_data)
                                else:
                                    self.get_logger().error(
                                        "record type:{} invalid !".format(record.record_type)
                                    )

            time.sleep(0.01)

    def _send_encoder_loop(self):
        if not self.encoder_freq:
            return

        interval = 1.0 / self.encoder_freq

        while self.is_running and rclpy.ok():
            start_time = time.time()

            with self.angle_lock:
                dis_target = self.finger_dis

            self.add_cmd(
                CmdPack.pack(
                    opcode=Opcode.ReadBatch,
                    record_type=RecordType.Encoder,
                    record=struct.pack(">f", dis_target),
                ),
            )

            elapsed = time.time() - start_time
            sleep_time = max(0, interval - elapsed)
            if sleep_time > 0:
                time.sleep(sleep_time)

    def _send_tactile_loop(self):
        if not self.tactile_freq:
            return

        interval = 1.0 / self.tactile_freq
        self.get_logger().debug(f"Tactile loop started, {self.tactile_freq} Hz, interval {interval:.3f}s")

        while self.is_running and rclpy.ok():
            start_time = time.time()
            self.add_cmd(
                CmdPack.pack(
                    opcode=Opcode.ReadSingle,
                    record_type=RecordType.Tactile,
                    record=struct.pack(">f", 0.0),
                )
            )

            elapsed = time.time() - start_time
            sleep_time = max(0, interval - elapsed)
            if sleep_time > 0:
                time.sleep(sleep_time)

    def wait_for_calib_response(self, timeout=3.0, poll_interval=0.05):
        if not self.is_calib_cmd:
            return True
        print("Waiting for device response...")
        deadline = time.time() + timeout
        while time.time() < deadline:
            if not self.is_calib_cmd:
                return True
            time.sleep(poll_interval)
        return not self.is_calib_cmd

    def stop(self):
        self.is_running = False

        threads_to_join = []
        for t in (self.read_thread, self.send_thread, self.parse_thread,
                  self.encoder_thread, self.tactile_thread):
            if t and t.is_alive():
                threads_to_join.append(t)

        for thread in threads_to_join:
            thread.join(timeout=2)

        if self.ser and self.ser.is_open:
            self.ser.close()


def check_and_fix_permission(port):
    if not os.path.exists(port):
        return False

    if os.access(port, os.R_OK | os.W_OK):
        return True

    print(f"Trying to fix permissions on {port}...")
    try:
        subprocess.run(['sudo', 'chmod', '666', port], check=True)
        print(f"Permissions fixed: {port}")
        return True
    except subprocess.CalledProcessError:
        print(f"Permission fix failed; run manually: sudo chmod 666 {port}")
        return False


def find_configured_serial_port(verbose=True):
    ports = []
    dev_dir = "/dev"
    if os.path.isdir(dev_dir):
        for name in os.listdir(dev_dir):
            if name.startswith("ttyFinger"):
                ports.append(os.path.join(dev_dir, name))
    ports.sort()
    for port in ports:
        if check_and_fix_permission(port):
            if verbose:
                print(f"Using configured serial device: {port}")
            return port
    return ports[0] if ports else None


def find_finger_serial_by_side(side, verbose=True):
    if side not in ("left", "right"):
        if verbose:
            print("side must be left or right")
        return None

    dev = "/dev/ttyFingerRight" if side == "right" else "/dev/ttyFingerLeft"
    if not os.path.exists(dev):
        if verbose:
            print(f"Serial device not found: {dev}")
        return None
    return dev if check_and_fix_permission(dev) else None


def find_serial_port(pattern="ttyUSB", max_retries=3, retry_interval=2, side=None, verbose=True):
    del pattern, max_retries, retry_interval
    if side in ("left", "right"):
        return find_finger_serial_by_side(side, verbose=verbose)

    port = find_configured_serial_port(verbose=verbose)
    if port:
        return port
    if verbose:
        print("No configured /dev/ttyFinger* serial device found")
    return None


def main(args=None):
    ros_args = ['__name:=', '__log:=', '__master:=', '__ip:=']
    filtered_args = []
    for arg in sys.argv[1:]:
        if not any(arg.startswith(ros_arg) for ros_arg in ros_args):
            filtered_args.append(arg)
    sys.argv = [sys.argv[0]] + filtered_args

    import argparse
    parser = argparse.ArgumentParser(description="DAS interface (ROS2, Gen Finger)")
    parser.add_argument("--serial-port", type=str, default="",
                        help="Serial port device (e.g., /dev/ttyFingerLeft)")
    parser.add_argument("--camera-cmd", type=str, default="",
                        help="Camera calibration command (e.g., MCUID)")
    parser.add_argument("--side", type=str, default="", choices=["", "left", "right"],
                        help="Device side: left or right (uses mapped ports)")
    cli_args, _unknown = parser.parse_known_args()

    rclpy.init(args=args)

    side = cli_args.side
    serial_port = ''
    if cli_args.serial_port and cli_args.serial_port != "":
        serial_port = cli_args.serial_port
    elif side:
        serial_port = find_finger_serial_by_side(side) or ''

    try:
        node = DataBusNode(
            tty_port=serial_port,
            baudrate=921600,
            encoder_freq=30,
            is_calib_cmd=False,
        )
    except Exception as e:
        print(f"Failed to start DataBusNode: {e}", file=sys.stderr)
        rclpy.shutdown()
        return

    time.sleep(1)

    if cli_args.camera_cmd and cli_args.camera_cmd != "":
        node.send_camera_calib_cmd(cli_args.camera_cmd)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        traceback.print_exc()
    finally:
        node.stop()
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
