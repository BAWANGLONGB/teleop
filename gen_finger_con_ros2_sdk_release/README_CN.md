# Gen Finger Controller ROS2 SDK

> 用于 Gen Finger 单相机设备的 ROS2 驱动 SDK，支持相机图像、触觉传感、编码器反馈及开合距离控制。

[English](README.md)

License: [MIT License](LICENSE.txt)

## 1 功能特性

- ROS2 Jazzy 驱动（`robot_driver` 包），用于 Gen Finger 设备
- 单相机图像流，支持实时预览
- 触觉数据发布（左 / 右）
- finger 开合距离编码器反馈
- 通过 ROS topic 控制 finger 开合
- 单指 / 双指 launch 文件
- 标定、设备 ID、触觉调试等工具脚本
- Demo 脚本：将模型 `PoseStamped` 指令转换为夹爪 topic

## 2 环境要求

| 项目     | 要求                       |
| -------- | -------------------------- |
| 系统     | Ubuntu 24.04（推荐）       |
| ROS      | ROS2 Jazzy                 |
| Python   | 3.10+                      |
| USB      | USB 3.0 接口               |
| 硬件     | Gen Finger controller 设备 |

## 3 快速开始

> 首次使用请先完成 [USB 配置](docs/usb-setup_CN.md)。

```shell
git clone https://github.com/genrobot-ai/gen_finger_con_ros2_sdk_release.git
cd gen_finger_con_ros2_sdk_release
source /opt/ros/jazzy/setup.bash
pip3 install -r requirements.txt
colcon build --symlink-install --base-paths src/robot_driver
source install/setup.bash
ros2 launch robot_driver single_finger_start.launch.py
```

验证反馈并发送控制指令：

```shell
# 终端 2 — 读取编码器反馈
ros2 topic echo /encoder

# 终端 3 — 设置目标开合 5 cm（范围 [0.0, 0.2] m）
ros2 topic pub /target_distance std_msgs/msg/Float32 "{data: 0.05}" --once
```

启动后会弹出一个相机预览窗口。

## 4 ROS Topic 接口

### 4.1 单指

| Topic                     | 类型                        | 方向  | 说明                       |
| ------------------------- | --------------------------- | ----- | -------------------------- |
| `/camera/color/image_raw` | `sensor_msgs/Image`         | 发布  | finger 相机图像            |
| `/encoder`                | `std_msgs/Float32`          | 发布  | finger 开合距离反馈（m）   |
| `/tactile/left`           | `std_msgs/Int8MultiArray`   | 发布  | 左侧触觉传感器             |
| `/tactile/right`          | `std_msgs/Int8MultiArray`   | 发布  | 右侧触觉传感器             |
| `/target_distance`        | `std_msgs/Float32`          | 订阅  | 目标开合距离，范围 `[0.0, 0.2]` m |

控制示例：

```shell
ros2 topic pub /target_distance std_msgs/msg/Float32 "{data: 0.05}" --once
```

### 4.2 双指

所有 topic 以 `/left_finger` 或 `/right_finger` 为命名空间前缀。

| Topic                            | 类型                 | 方向  | 说明               |
| -------------------------------- | -------------------- | ----- | ------------------ |
| `/left_finger/encoder`          | `std_msgs/Float32`   | 发布  | 左 finger 开合反馈 |
| `/left_finger/target_distance`  | `std_msgs/Float32`   | 订阅  | 左 finger 目标距离 |
| `/right_finger/encoder`         | `std_msgs/Float32`   | 发布  | 右 finger 开合反馈 |
| `/right_finger/target_distance` | `std_msgs/Float32`   | 订阅  | 右 finger 目标距离 |

相机与触觉 topic 遵循相同命名空间规则（如 `/left_finger/camera/...`、`/left_finger/tactile/...` 等）。

### 4.3 Launch 参数

| 参数                   | 默认值                        | 说明                   |
| ---------------------- | ----------------------------- | ---------------------- |
| `serial`               | `/dev/ttyFingerLeft`          | 串口（单指 launch）    |
| `video_device`         | `/dev/finger_camera_left`     | 相机设备（单指 launch）|
| `left_serial`          | `/dev/ttyFingerLeft`          | 左 finger 串口         |
| `right_serial`         | `/dev/ttyFingerRight`         | 右 finger 串口         |
| `left_video_device`    | `/dev/finger_camera_left`     | 左 finger 相机         |
| `right_video_device`   | `/dev/finger_camera_right`    | 右 finger 相机         |
| `camera_resolutions`   | `1600x1296`                   | 相机分辨率             |
| `show_preview`         | `true`                        | 是否显示 OpenCV 预览   |
| `fps`                  | `60`                          | 相机帧率               |

## 5 安装

### 5.1 安装系统与 Python 依赖

```shell
sudo apt update
sudo apt install ros-jazzy-desktop python3-pip v4l-utils
pip3 install -r requirements.txt
```

### 5.2 拉取仓库并编译

```shell
cd gen_finger_con_ros2_sdk_release
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --base-paths src/robot_driver
source install/setup.bash
```

## 6 USB 配置

首次使用前需为每个 USB 口配置 udev 规则，模板见 [config/99-usb-serial.rules](./config/99-usb-serial.rules)。

详细步骤见 [USB 配置指南 (ZH)](docs/usb-setup_CN.md)。

默认串口软链接：`/dev/ttyFingerLeft`、`/dev/ttyFingerRight`  
默认相机软链接：`/dev/finger_camera_left`、`/dev/finger_camera_right`

## 7 使用方法

### 7.1 单指 Demo

```shell
source install/setup.bash
ros2 launch robot_driver single_finger_start.launch.py
```

可选 launch 参数：

```shell
ros2 launch robot_driver single_finger_start.launch.py show_preview:=false
ros2 launch robot_driver single_finger_start.launch.py serial:=/dev/ttyFingerLeft video_device:=/dev/finger_camera_left
```

### 7.2 双指 Demo

```shell
source install/setup.bash
ros2 launch robot_driver dual_finger_start.launch.py
```

运行 demo 脚本，将模型指令桥接到 finger topic：

```shell
ros2 run robot_driver left_das_controller_infer
ros2 run robot_driver right_das_controller_infer
```

### 7.3 设备工具命令

运行工具前**不要**同时启动 `ros2 launch` 或其他控制节点（同一 USB 设备不能多进程共享）。

**单设备：**

```shell
cd src/robot_driver/scripts/
bash camera_cmd.sh camerarc
bash camera_cmd.sh MCUID
ros2 run robot_driver tactile_dual_print
```

**双设备（左 / 右）：**

```shell
cd src/robot_driver/scripts/

bash camera_cmd.sh left camerarc
bash camera_cmd.sh left MCUID
ros2 run robot_driver tactile_dual_print --ros-args -p finger_ns:=left_finger

bash camera_cmd.sh right camerarc
bash camera_cmd.sh right MCUID
ros2 run robot_driver tactile_dual_print --ros-args -p finger_ns:=right_finger
```

## 8 常见问题

| 问题               | 解决方法                                                      |
| ------------------ | ------------------------------------------------------------- |
| 找不到串口         | 执行 `sudo apt remove brltty`，重新插拔设备                   |
| 相机或串口路径不对 | 检查 udev 规则，见 [docs/usb-setup_CN.md](docs/usb-setup_CN.md) |
| 相机帧率偏低       | 保持 launch 中 `fps:=60`                                      |
| 无 ROS 数据        | 确认 udev 规则已生效，节点是否正常启动                        |
| `colcon build` 失败| 先 source ROS2：`source /opt/ros/jazzy/setup.bash`            |
| 无相机预览         | 检查 udev 相机软链接；用 `v4l2-ctl --list-devices` 验证       |
| 设备工具命令失败   | 运行工具前先停止 `ros2 launch` 及其他控制节点               |

## 9 文档索引

| 说明             | 链接                                                                                 |
| ---------------- | ------------------------------------------------------------------------------------ |
| USB 配置 (ZH)    | [docs/usb-setup_CN.md](docs/usb-setup_CN.md)                                         |
| USB setup (EN)   | [docs/usb-setup.md](docs/usb-setup.md)                                               |
| udev 规则模板    | [config/99-usb-serial.rules](config/99-usb-serial.rules)                             |
| 单指 launch      | [single_finger_start.launch.py](src/robot_driver/launch/single_finger_start.launch.py) |
| 双指 launch      | [dual_finger_start.launch.py](src/robot_driver/launch/dual_finger_start.launch.py) |
