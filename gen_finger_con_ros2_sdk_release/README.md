# Gen Finger Controller ROS2 SDK

> ROS2 driver for Gen Finger controller — single-camera streaming, tactile sensing, encoder feedback, and distance control.

[中文](README_CN.md)

License: [MIT License](LICENSE.txt)

## 1 Features

- ROS2 Jazzy driver (`robot_driver` package) for Gen Finger devices
- Single-camera image streaming with optional live preview 
- Tactile sensor data publishing (left / right)
- Encoder feedback for finger opening distance
- Finger distance control via ROS topics
- Single-finger and dual-finger launch files
- Utility scripts for calibration, device ID, and tactile debugging
- Demo scripts bridging model `PoseStamped` commands to finger topics

## 2 Requirements

| Item     | Requirement                   |
| -------- | ----------------------------- |
| OS       | Ubuntu 24.04 (recommended)    |
| ROS      | ROS2 Jazzy                    |
| Python   | 3.10+                         |
| USB      | USB 3.0 port                  |
| Hardware | Gen Finger controller device  |

## 3 Quick Start

> First-time users must complete [USB configuration](docs/usb-setup.md) before launching the driver.

```shell
git clone https://github.com/genrobot-ai/gen_finger_con_ros2_sdk_release.git
cd gen_finger_con_ros2_sdk_release
source /opt/ros/jazzy/setup.bash
pip3 install -r requirements.txt
colcon build --symlink-install --base-paths src/robot_driver
source install/setup.bash
ros2 launch robot_driver single_finger_start.launch.py
```

Verify feedback and send a control command:

```shell
# Terminal 2 — read encoder feedback
ros2 topic echo /encoder

# Terminal 3 — set target opening to 5 cm (range: [0.0, 0.2] m)
ros2 topic pub /target_distance std_msgs/msg/Float32 "{data: 0.05}" --once
```

After startup, one camera preview window appears.

## 4 ROS Topics

### 4.1 Single Finger

| Topic                     | Type                      | Direction | Description                                   |
| ------------------------- | ------------------------- | --------- | --------------------------------------------- |
| `/camera/color/image_raw` | `sensor_msgs/Image`       | publish   | Finger camera image                           |
| `/encoder`                | `std_msgs/Float32`        | publish   | Finger opening distance feedback (m)          |
| `/tactile/left`           | `std_msgs/Int8MultiArray` | publish   | Left tactile sensor                           |
| `/tactile/right`          | `std_msgs/Int8MultiArray` | publish   | Right tactile sensor                          |
| `/target_distance`        | `std_msgs/Float32`        | subscribe | Target opening distance, range `[0.0, 0.2]` m |

Example control command:

```shell
ros2 topic pub /target_distance std_msgs/msg/Float32 "{data: 0.05}" --once
```

### 4.2 Dual Finger

All topics are prefixed with `/left_finger` or `/right_finger`.

| Topic                            | Type               | Direction | Description                    |
| -------------------------------- | ------------------ | --------- | ------------------------------ |
| `/left_finger/encoder`          | `std_msgs/Float32` | publish   | Left finger opening feedback   |
| `/left_finger/target_distance`  | `std_msgs/Float32` | subscribe | Left finger target distance    |
| `/right_finger/encoder`         | `std_msgs/Float32` | publish   | Right finger opening feedback  |
| `/right_finger/target_distance` | `std_msgs/Float32` | subscribe | Right finger target distance   |

Camera and tactile topics follow the same namespace rules (e.g. `/left_finger/camera/...`, `/left_finger/tactile/...`).

### 4.3 Launch Parameters

| Parameter            | Default                       | Description                          |
| -------------------- | ----------------------------- | ------------------------------------ |
| `serial`             | `/dev/ttyFingerLeft`          | Serial port (single-finger launch)   |
| `video_device`       | `/dev/finger_camera_left`     | Camera device (single-finger launch) |
| `left_serial`        | `/dev/ttyFingerLeft`          | Left finger serial port              |
| `right_serial`       | `/dev/ttyFingerRight`         | Right finger serial port             |
| `left_video_device`  | `/dev/finger_camera_left`     | Left finger camera                   |
| `right_video_device` | `/dev/finger_camera_right`    | Right finger camera                  |
| `camera_resolutions` | `1600x1296`                   | Camera resolution                    |
| `show_preview`       | `true`                        | Show OpenCV preview window           |
| `fps`                | `60`                          | Camera frame rate                    |

## 5 Installation

### 5.1 System and Python Dependencies

```shell
sudo apt update
sudo apt install ros-jazzy-desktop python3-pip v4l-utils
pip3 install -r requirements.txt
```

### 5.2 Clone and Build

```shell
cd gen_finger_con_ros2_sdk_release
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --base-paths src/robot_driver
source install/setup.bash
```

## 6 USB Configuration

Configure udev rules before first use. Template: [config/99-usb-serial.rules](./config/99-usb-serial.rules).

See [USB Configuration Guide (EN)](docs/usb-setup.md).

Default serial symlinks: `/dev/ttyFingerLeft`, `/dev/ttyFingerRight`  
Default camera symlinks: `/dev/finger_camera_left`, `/dev/finger_camera_right`

## 7 Usage

### 7.1 Single Finger Demo

```shell
source install/setup.bash
ros2 launch robot_driver single_finger_start.launch.py
```

Optional launch parameters:

```shell
ros2 launch robot_driver single_finger_start.launch.py show_preview:=false
ros2 launch robot_driver single_finger_start.launch.py serial:=/dev/ttyFingerLeft video_device:=/dev/finger_camera_left
```

### 7.2 Dual Finger Demo

```shell
source install/setup.bash
ros2 launch robot_driver dual_finger_start.launch.py
```

Bridge model commands to finger topics:

```shell
ros2 run robot_driver left_das_controller_infer
ros2 run robot_driver right_das_controller_infer
```

### 7.3 Device Utility Commands

Do not run utilities while `ros2 launch` or other control nodes are using the same USB device.

**Single device:**

```shell
cd src/robot_driver/scripts/
bash camera_cmd.sh camerarc
bash camera_cmd.sh MCUID
ros2 run robot_driver tactile_dual_print
```

**Dual device (left / right):**

```shell
cd src/robot_driver/scripts/

bash camera_cmd.sh left camerarc
bash camera_cmd.sh left MCUID
ros2 run robot_driver tactile_dual_print --ros-args -p finger_ns:=left_finger

bash camera_cmd.sh right camerarc
bash camera_cmd.sh right MCUID
ros2 run robot_driver tactile_dual_print --ros-args -p finger_ns:=right_finger
```

## 8 Troubleshooting

| Issue                  | Solution                                                      |
| ---------------------- | ------------------------------------------------------------- |
| Serial port not found  | Run `sudo apt remove brltty`, reconnect device                |
| Wrong device paths     | Check udev rules in [docs/usb-setup.md](docs/usb-setup.md)    |
| Low camera frame rate  | Keep `fps:=60` in launch                                      |
| No ROS data            | Verify udev rules and node startup                            |
| `colcon build` fails   | Source ROS2 first: `source /opt/ros/jazzy/setup.bash`         |
| No camera preview      | Check udev camera symlinks; run `v4l2-ctl --list-devices`     |
| Utility command fails  | Stop `ros2 launch` and other control nodes first              |

## 9 Documentation Index

| Description        | Link                                                                                 |
| ------------------ | ------------------------------------------------------------------------------------ |
| USB setup (EN)     | [docs/usb-setup.md](docs/usb-setup.md)                                               |
| USB 配置 (ZH)      | [docs/usb-setup_CN.md](docs/usb-setup_CN.md)                                         |
| udev rules template| [config/99-usb-serial.rules](config/99-usb-serial.rules)                             |
| Single-finger launch | [single_finger_start.launch.py](src/robot_driver/launch/single_finger_start.launch.py) |
| Dual-finger launch | [dual_finger_start.launch.py](src/robot_driver/launch/dual_finger_start.launch.py) |
