# USB Configuration Guide

This guide explains how to configure udev rules so each Gen Finger USB port maps to stable serial and camera device names.

The template file is located at [config/99-usb-serial.rules](../config/99-usb-serial.rules).

After configuration, the corresponding USB port can recognize Gen Finger devices without further setup.

[中文](usb-setup_CN.md)

## 1 Single Finger USB Port

Each finger requires only **one serial port** and **one camera** (unlike the Gen Controller finger with three cameras).

The final configuration should look like the figure below.

![Single finger udev example](../image/image_1.png)

Fields that must be modified:

![Fields to modify](../image/image_2.png)

### 1.1 Parameter 1 — Serial Port KERNELS

```shell
cd /dev && ls | grep ttyUSB
udevadm info -a -n /dev/ttyUSB* | grep -E "KERNELS|DRIVERS"
```

If the serial port cannot be detected:

```shell
sudo apt remove brltty
```

Configure the **second** `KERNELS` value from the output to Parameter 1:

![Serial KERNELS example](../image/image_3.png)

### 1.2 Parameter 2 — Camera KERNELS

```shell
v4l2-ctl --list-devices
```

Example output:

![v4l2 device list](../image/image_4.png)

For the camera of this USB device, run:

```shell
udevadm info -a -n /dev/video* | grep -E "KERNELS|SUBSYSTEMS"
```

Configure the **first** `KERNELS` value from the output to Parameter 2:

![Camera KERNELS example](../image/image_5.png)

### 1.3 Apply Rules

```shell
sudo cp config/99-usb-serial.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Default symlinks after single-finger configuration:

| Device | Symlink |
|--------|---------|
| Serial | `/dev/ttyFingerLeft` |
| Camera | `/dev/finger_camera_left` |

## 2 Dual Finger USB Ports

The final configuration should look like the figure below.

![Dual finger udev example](../image/image_6.png)

Fields to modify:

![Dual finger fields](../image/image_7.png)

Steps:

1. Plug in the **left** finger and configure its serial port and camera using the single finger method above.
2. Unplug the left finger, plug in the **right** finger, and configure it the same way.
3. Reload udev rules.

Default symlinks after dual configuration:

| Device | Symlink |
|--------|---------|
| Left finger serial | `/dev/ttyFingerLeft` |
| Left finger camera | `/dev/finger_camera_left` |
| Right finger serial | `/dev/ttyFingerRight` |
| Right finger camera | `/dev/finger_camera_right` |

## 3 Multiple Fingers

Add one serial rule and one camera rule per finger to `99-usb-serial.rules`, following the same pattern.

## 4 Verify Device ID (Optional)

**Do not** run `roslaunch robot_driver single_finger_start.launch` at the same time.

```shell
cd src/robot_driver/scripts
bash camera_cmd.sh left MCUID
```

Dual finger setup:

```shell
bash camera_cmd.sh left MCUID
bash camera_cmd.sh right MCUID
```

Example output:

![Device ID example](../image/image_8.jpg)
