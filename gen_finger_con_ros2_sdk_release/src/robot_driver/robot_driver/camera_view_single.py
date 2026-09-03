#!/usr/bin/env python3
# ROS2 port of camera_view_single.py
import sys
import os

os.environ.setdefault('LIBGL_ALWAYS_SOFTWARE', '1')
os.environ.setdefault('MESA_LOADER_DRIVER_OVERRIDE', 'llvmpipe')
os.environ.setdefault('QT_OPENGL', 'software')
os.environ.setdefault('QT_XCB_GL_INTEGRATION', 'none')

import cv2
import time
import glob
import subprocess
import signal
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


def _parse_reliability(value):
    value = str(value).strip().lower()
    if value in ('reliable', 'reliability_reliable'):
        return ReliabilityPolicy.RELIABLE
    return ReliabilityPolicy.BEST_EFFORT


class CameraCaptureROS(Node):
    def __init__(self):
        super().__init__('camera_capture_node')

        self._declare_and_load_params()

        self.node_name = self.get_name().replace('/', '_')
        if not self.node_name:
            self.node_name = 'camera_default'

        self.cameras = []
        self.running = True
        self._shutdown_event = threading.Event()
        self._release_lock = threading.Lock()
        self._released = False
        self._grab_thread = None
        self.image_publishers = []

        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

        self._init_cameras()
        self._init_ros_publishers()

    def _declare_and_load_params(self):
        self.declare_parameter('show_preview', True)
        self.declare_parameter(
            'resolutions',
            "640x480,320x240,800x600, 1024x768,1280x720,1280x1024,1280x960,1600x1296 ",
        )
        self.declare_parameter('topic_base', '/camera_fisheye')
        self.declare_parameter('camera_count', 1)
        self.declare_parameter('fps', 60)
        self.declare_parameter('publish_resolution', '')
        self.declare_parameter('usb_port', '')
        self.declare_parameter('video_device', '')
        self.declare_parameter('video_0_main', '')
        self.declare_parameter('video_0_sec', '')
        self.declare_parameter('video_1_main', '')
        self.declare_parameter('video_1_sec', '')
        self.declare_parameter('video_2_main', '')
        self.declare_parameter('video_2_sec', '')
        self.declare_parameter('image_qos_reliability', 'best_effort')
        self.declare_parameter('image_qos_depth', 1)
        self.declare_parameter('enable_fps_log', False)

        self.show_preview = self.get_parameter('show_preview').value
        resolutions_str = self.get_parameter('resolutions').value
        self.resolutions = []
        for res_str in resolutions_str.split(','):
            try:
                width, height = map(int, res_str.strip().split('x'))
                self.resolutions.append((width, height))
            except Exception:
                self.get_logger().warn(f"Cannot parse resolution string: {res_str}")

        if not self.resolutions:
            self.resolutions = [(640, 480), (320, 240), (800, 600), (1024, 768),
                                (1280, 720), (1280, 1024), (1280, 960), (1600, 1296)]

        self.topic_base = self.get_parameter('topic_base').value
        self.max_cameras = int(self.get_parameter('camera_count').value)
        self.fps = int(self.get_parameter('fps').value)

        publish_res_str = self.get_parameter('publish_resolution').value
        self.publish_resolution = None
        if publish_res_str:
            try:
                pw, ph = map(int, publish_res_str.strip().split('x'))
                self.publish_resolution = (pw, ph)
            except Exception:
                self.get_logger().warn(f"Cannot parse publish_resolution: {publish_res_str}")

        self.usb_port = self.get_parameter('usb_port').value
        self.video_device = self.get_parameter('video_device').value
        self.video0_main = self.get_parameter('video_0_main').value
        self.video0_sec = self.get_parameter('video_0_sec').value
        self.video1_main = self.get_parameter('video_1_main').value
        self.video1_sec = self.get_parameter('video_1_sec').value
        self.video2_main = self.get_parameter('video_2_main').value
        self.video2_sec = self.get_parameter('video_2_sec').value
        self.image_qos_reliability = _parse_reliability(
            self.get_parameter('image_qos_reliability').value
        )
        self.image_qos_depth = max(1, int(self.get_parameter('image_qos_depth').value))
        self.enable_fps_log = bool(self.get_parameter('enable_fps_log').value)

    def _signal_handler(self, signum, frame):
        # Only set flags here — OpenCV/rclpy cleanup must run on the main thread.
        self.running = False
        self._shutdown_event.set()

    def _request_stop(self):
        self.running = False
        self._shutdown_event.set()

    def _interruptible_sleep(self, duration):
        end = time.monotonic() + duration
        while self.running and not self._shutdown_event.is_set() and time.monotonic() < end:
            time.sleep(min(0.05, end - time.monotonic()))

    def _get_physical_devices(self):
        try:
            result = subprocess.run(['v4l2-ctl', '--list-devices'],
                                    capture_output=True, text=True)
            devices = []
            current_dev = ""
            device_names = {}

            for line in result.stdout.split('\n'):
                if not line.strip():
                    continue
                if ':' in line and not line.startswith('/dev/'):
                    current_dev = line.split(':')[0].strip()
                elif line.startswith('/dev/video'):
                    dev_path = line.strip()
                    if os.path.exists(dev_path):
                        devices.append(dev_path)
                        device_names[dev_path] = current_dev

            if self.usb_port and self.usb_port != "":
                usb_number = None
                try:
                    if 'ttyUSB' in self.usb_port:
                        usb_number = int(self.usb_port.replace('/dev/ttyUSB', ''))
                    elif 'ttyACM' in self.usb_port:
                        usb_number = int(self.usb_port.replace('/dev/ttyACM', ''))
                except Exception:
                    pass

                filtered_devices = []

                for dev in devices:
                    device_name = device_names.get(dev, '')

                    if self.usb_port in device_name or device_name in self.usb_port:
                        filtered_devices.append(dev)
                        continue

                    if usb_number is not None:
                        try:
                            video_number = int(dev.replace('/dev/video', ''))
                            if video_number in (usb_number, usb_number + 1, usb_number - 1):
                                filtered_devices.append(dev)
                                continue
                        except Exception:
                            pass

                    try:
                        udev_cmd = ['udevadm', 'info', '-q', 'path', '-n', dev]
                        udev_result = subprocess.run(udev_cmd, capture_output=True, text=True)
                        udev_path = udev_result.stdout.strip()

                        if udev_path and 'usb' in udev_path:
                            usb_info = udev_path.split('/')
                            for part in usb_info:
                                if 'usb' in part and len(part) > 3:
                                    usb_cmd = ['udevadm', 'info', '-q', 'path', '-n', self.usb_port]
                                    usb_result = subprocess.run(usb_cmd, capture_output=True, text=True)
                                    usb_path = usb_result.stdout.strip()
                                    if part in usb_path:
                                        filtered_devices.append(dev)
                                        self.get_logger().debug(f"Matched video device by USB bus: {dev}")
                                        break
                    except Exception:
                        pass

                if filtered_devices:
                    devices = filtered_devices
                    self.get_logger().debug(f"Filtered video devices: {devices}")

            if len(devices) > self.max_cameras:
                devices = devices[:self.max_cameras]

            return sorted(list(set(devices))) if devices else sorted(glob.glob('/dev/video*'))
        except Exception:
            return sorted(glob.glob('/dev/video*'))

    def _try_reset_device(self, dev_path):
        try:
            udev_info = subprocess.run(
                ['udevadm', 'info', '-q', 'path', '-n', dev_path],
                capture_output=True, text=True
            ).stdout.strip()

            if udev_info:
                usb_path = f"/sys{udev_info}/../reset"
                if os.path.exists(usb_path):
                    with open(usb_path, 'w') as f:
                        f.write('1')
                    time.sleep(2)
                    return True
        except Exception:
            pass
        return False

    def _init_camera(self, dev_path, cam_id):
        for attempt in range(3):
            try:
                if not os.path.exists(dev_path):
                    self.get_logger().warn(f"Device {dev_path} does not exist")
                    continue

                if attempt > 0:
                    self._try_reset_device(dev_path)
                    os.system(f'sudo chmod 666 {dev_path}')
                    os.system(f'sudo fuser -k {dev_path} 2>/dev/null')

                unique_cam_id = f"{self.node_name}_cam{cam_id}"
                cap = cv2.VideoCapture(dev_path, cv2.CAP_V4L2)
                if not cap.isOpened():
                    return False

                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
                cap.set(cv2.CAP_PROP_FPS, self.fps)

                success = False
                actual_width = 0
                actual_height = 0

                for res in self.resolutions:
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH, res[0])
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, res[1])
                    actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                    actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

                    if actual_width == res[0] and actual_height == res[1]:
                        success = True
                        break

                if not success:
                    pass

                for _ in range(5):
                    cap.grab()
                    time.sleep(0.01)

                actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

                window_name = f'{self.node_name}_{cam_id}_{actual_width}x{actual_height}'

                self.cameras.append({
                    'id': cam_id,
                    'unique_id': unique_cam_id,
                    'cap': cap,
                    'dev': dev_path,
                    'frame_count': 0,
                    'width': actual_width,
                    'height': actual_height,
                    'window_name': window_name,
                    'lock': threading.Lock(),
                    'latest_frame': None,
                    'latest_ts_ns': 0,
                    'cap_fps_ts': [],
                    'cap_fps_val': 0.0,
                    'last_cap_log_ts': 0.0,
                    'pub_frame_count': 0,
                    'pub_fps_ts': [],
                    'pub_fps_val': 0.0,
                    'last_pub_log_ts': 0.0,
                    'disp_fps_ts': [],
                    'disp_fps_val': 0.0,
                })
                return True

            except Exception as e:
                self.get_logger().error(f"Attempt #{attempt+1} init {dev_path} failed: {str(e)}")
                if 'cap' in locals() and cap.isOpened():
                    cap.release()
                time.sleep(1)
        return False

    def _init_cameras(self):
        if self.video_device:
            if not os.path.exists(self.video_device):
                self.get_logger().error(f"Camera device {self.video_device} does not exist")
                sys.exit(1)
            if not self._init_camera(self.video_device, 0):
                self.get_logger().error(f"Failed to open camera {self.video_device}")
                sys.exit(1)
            return

        self._get_physical_devices()
        if self.max_cameras >= 1:
            self._init_main_or_second_camera(self.video0_main, self.video0_sec, 0)
        if self.max_cameras >= 2:
            self._init_main_or_second_camera(self.video1_main, self.video1_sec, 1)
        if self.max_cameras >= 3:
            self._init_main_or_second_camera(self.video2_main, self.video2_sec, 2)

        if not self.cameras:
            sys.exit(1)

    def _init_main_or_second_camera(self, dev_main, dev_second, index):
        if self._init_camera(dev_main, index):
            return
        if self._init_camera(dev_second, index):
            return

    def _init_ros_publishers(self):
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=self.image_qos_depth,
            reliability=self.image_qos_reliability,
        )
        self.get_logger().debug(
            f"Image QoS: reliability={self.image_qos_reliability.name}, depth={self.image_qos_depth}"
        )
        for cam in self.cameras:
            if self.topic_base:
                if cam['id'] == 0:
                    topic_name = f'{self.topic_base}/color/image_raw'
                else:
                    topic_name = f'{self.topic_base}_{cam["id"]}/color/image_raw'
            else:
                if cam['id'] == 0:
                    topic_name = '/camera_fisheye/color/image_raw'
                else:
                    topic_name = f'/camera_fisheye/color/image_raw_{cam["id"] + 1}'

            publisher = self.create_publisher(Image, topic_name, qos)
            self.image_publishers.append({
                'publisher': publisher,
                'cam_id': cam['id'],
                'unique_id': cam['unique_id'],
                'topic_name': topic_name,
            })

    def _sync_grab_loop(self):
        """Single background thread for capture/publish; GUI stays on main thread."""
        while self.running and rclpy.ok() and not self._shutdown_event.is_set():
            try:
                grab_results = {}
                for cam in self.cameras:
                    cap = cam.get('cap')
                    if cap is None or not cap.isOpened():
                        grab_results[cam['id']] = False
                        continue
                    grab_results[cam['id']] = cap.grab()

                if self._shutdown_event.is_set():
                    break

                now = time.monotonic()
                ts_ns = time.time_ns()

                for cam in self.cameras:
                    if not grab_results.get(cam['id']):
                        continue

                    cap = cam.get('cap')
                    if cap is None or not cap.isOpened():
                        continue

                    ret, frame = cap.retrieve()
                    if not ret or frame is None:
                        continue

                    self._publish_frame(cam['id'], frame, ts_ns)

                    cap_log = None
                    with cam['lock']:
                        cam['frame_count'] += 1
                        cam['latest_frame'] = frame
                        cam['latest_ts_ns'] = ts_ns

                        cam['cap_fps_ts'].append(now)
                        if len(cam['cap_fps_ts']) > 30:
                            cam['cap_fps_ts'] = cam['cap_fps_ts'][-30:]
                        if len(cam['cap_fps_ts']) >= 2:
                            dt = cam['cap_fps_ts'][-1] - cam['cap_fps_ts'][0]
                            if dt > 0:
                                cam['cap_fps_val'] = (len(cam['cap_fps_ts']) - 1) / dt
                        if self.enable_fps_log and now - cam['last_cap_log_ts'] >= 1.0:
                            cam['last_cap_log_ts'] = now
                            cap_log = (
                                cam['id'],
                                cam['dev'],
                                cam['cap_fps_val'],
                                cam['frame_count'],
                            )
                    if cap_log is not None:
                        cam_id, dev_path, cap_fps, frame_count = cap_log
                        self.get_logger().info(
                            f"Cap camera {cam_id} ({dev_path}): {cap_fps:.1f} FPS, frames={frame_count}"
                        )

                if not any(grab_results.values()):
                    time.sleep(0.001)
            except Exception as e:
                if self.running and not self._shutdown_event.is_set():
                    self.get_logger().warn(f"Grab loop error: {e}")
                break

    def _start_grab_threads(self):
        self._grab_thread = threading.Thread(
            target=self._sync_grab_loop,
            name='camera_grab_sync',
            daemon=True,
        )
        self._grab_thread.start()

    def _stop_grab_threads(self):
        t = self._grab_thread
        if t and t.is_alive():
            t.join(timeout=1.0)
        self._grab_thread = None

    def _get_latest(self, cam):
        with cam['lock']:
            frame = cam['latest_frame']
            ts_ns = cam['latest_ts_ns']
            cam['latest_frame'] = None
        return frame, ts_ns

    def _publish_frame(self, cam_id, frame, timestamp_ns):
        try:
            if frame is None or frame.size == 0:
                return

            if len(frame.shape) != 3 or frame.shape[2] != 3:
                return

            ros_image = Image()
            sec = int(timestamp_ns // 1_000_000_000)
            nanosec = int(timestamp_ns % 1_000_000_000)
            ros_image.header.stamp.sec = sec
            ros_image.header.stamp.nanosec = nanosec

            for cam in self.cameras:
                if cam['id'] == cam_id:
                    ros_image.header.frame_id = cam['unique_id']
                    break

            height, width = frame.shape[:2]
            ros_image.height = height
            ros_image.width = width
            ros_image.encoding = 'bgr8'
            ros_image.step = width * 3
            ros_image.is_bigendian = 0

            if not frame.flags['C_CONTIGUOUS']:
                frame = np.ascontiguousarray(frame)

            ros_image.data = frame.tobytes()

            for pub_info in self.image_publishers:
                if pub_info['cam_id'] == cam_id:
                    pub_info['publisher'].publish(ros_image)
                    self._record_publish_stats(cam_id)
                    break

        except Exception:
            pass

    def _record_publish_stats(self, cam_id):
        now = time.monotonic()
        pub_log = None

        for cam in self.cameras:
            if cam['id'] != cam_id:
                continue

            with cam['lock']:
                cam['pub_frame_count'] += 1
                cam['pub_fps_ts'].append(now)
                if len(cam['pub_fps_ts']) > 30:
                    cam['pub_fps_ts'] = cam['pub_fps_ts'][-30:]
                if len(cam['pub_fps_ts']) >= 2:
                    dt = cam['pub_fps_ts'][-1] - cam['pub_fps_ts'][0]
                    if dt > 0:
                        cam['pub_fps_val'] = (len(cam['pub_fps_ts']) - 1) / dt
                if self.enable_fps_log and now - cam['last_pub_log_ts'] >= 1.0:
                    cam['last_pub_log_ts'] = now
                    pub_log = (
                        cam['id'],
                        cam['dev'],
                        cam['pub_fps_val'],
                        cam['pub_frame_count'],
                    )
            break

        if pub_log is not None:
            cam_id, dev_path, pub_fps, frame_count = pub_log
            self.get_logger().info(
                f"Pub camera {cam_id} ({dev_path}): {pub_fps:.1f} FPS, frames={frame_count}"
            )

    def _display_frames(self, frames_data):
        if not self.running or self._shutdown_event.is_set():
            return

        for cam, frame in frames_data:
            if frame is not None:
                timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
                info_text = f"{cam['unique_id']} | {timestamp} | Frames: {cam['frame_count']}"
                cv2.putText(frame, info_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                            0.7, (0, 255, 0), 2)
                fps_text = f"Cap: {cam['cap_fps_val']:.1f}  Disp: {cam['disp_fps_val']:.1f}"
                cv2.putText(frame, fps_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX,
                            0.7, (0, 255, 255), 2)

                cv2.imshow(cam['window_name'], frame)

        if cv2.waitKey(1) == 27:
            self._request_stop()

    def capture_frames(self):
        if self.show_preview:
            for cam in self.cameras:
                RESIZE_WIDTH = 640
                RESIZE_HEIGHT = 480
                cv2.namedWindow(cam['window_name'], cv2.WINDOW_NORMAL)
                cv2.resizeWindow(cam['window_name'], RESIZE_WIDTH, RESIZE_HEIGHT)

        self._start_grab_threads()

        period = 1.0 / self.fps if self.fps > 0 else 0.03

        try:
            while self.running and rclpy.ok() and not self._shutdown_event.is_set():
                frames_data = []

                for cam in self.cameras:
                    frame, _ts_ns = self._get_latest(cam)
                    if frame is not None:
                        now = time.monotonic()
                        cam['disp_fps_ts'].append(now)
                        if len(cam['disp_fps_ts']) > 30:
                            cam['disp_fps_ts'] = cam['disp_fps_ts'][-30:]
                        if len(cam['disp_fps_ts']) >= 2:
                            dt = cam['disp_fps_ts'][-1] - cam['disp_fps_ts'][0]
                            if dt > 0:
                                cam['disp_fps_val'] = (len(cam['disp_fps_ts']) - 1) / dt

                    frames_data.append((cam, frame))

                if self.show_preview:
                    self._display_frames(frames_data)

                rclpy.spin_once(self, timeout_sec=0.0)
                self._interruptible_sleep(period)

        except Exception as e:
            self.get_logger().error(f"Capture error: {e}")
        finally:
            self._release_resources()

    def _release_resources(self):
        with self._release_lock:
            if self._released:
                return

            self.running = False
            self._shutdown_event.set()

            for cam in self.cameras:
                cap = cam.get('cap')
                if cap is None:
                    continue
                try:
                    if cap.isOpened():
                        cap.release()
                except Exception:
                    pass
                cam['cap'] = None

            self._stop_grab_threads()

            if self.show_preview:
                try:
                    cv2.destroyAllWindows()
                except Exception:
                    for cam in self.cameras:
                        try:
                            cv2.destroyWindow(cam['window_name'])
                        except Exception:
                            pass

            self._released = True
            self.get_logger().info("Camera resources released")


def main(args=None):
    try:
        os.nice(-20)
    except Exception:
        pass

    cv2.setNumThreads(1)
    cv2.setUseOptimized(True)

    rclpy.init(args=args)
    node = None
    try:
        node = CameraCaptureROS()
        node.capture_frames()
    except KeyboardInterrupt:
        if node is not None:
            node._request_stop()
    except Exception as e:
        if node is not None:
            node.get_logger().error(f"Fatal error: {str(e)}")
        else:
            print(f"Fatal error: {str(e)}", file=sys.stderr)
    finally:
        if node is not None:
            try:
                node._release_resources()
            except Exception:
                pass
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
