#!/usr/bin/env python3
"""ROS2 dual-finger launch for Gen Finger."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    left_serial = DeclareLaunchArgument('left_serial', default_value='/dev/ttyFingerLeft')
    right_serial = DeclareLaunchArgument('right_serial', default_value='/dev/ttyFingerRight')
    left_video = DeclareLaunchArgument('left_video_device', default_value='/dev/finger_camera_left')
    right_video = DeclareLaunchArgument('right_video_device', default_value='/dev/finger_camera_right')
    res_arg = DeclareLaunchArgument('camera_resolutions', default_value='1600x1296')
    preview_arg = DeclareLaunchArgument('show_preview', default_value='true')
    qos_reliability_arg = DeclareLaunchArgument('image_qos_reliability', default_value='best_effort')
    qos_depth_arg = DeclareLaunchArgument('image_qos_depth', default_value='1')

    show_preview = ParameterValue(LaunchConfiguration('show_preview'), value_type=bool)
    image_qos_depth = ParameterValue(LaunchConfiguration('image_qos_depth'), value_type=int)
    res = LaunchConfiguration('camera_resolutions')

    def side_group(ns, serial_cfg, video_cfg, das_delay):
        camera = Node(
            package='robot_driver',
            executable='camera_view_single',
            name='camera',
            output='screen',
            additional_env={
                'LIBGL_ALWAYS_SOFTWARE': '1',
                'MESA_LOADER_DRIVER_OVERRIDE': 'llvmpipe',
                'QT_OPENGL': 'software',
                'QT_XCB_GL_INTEGRATION': 'none',
            },
            parameters=[{
                'resolutions': res,
                'topic_base': 'camera',
                'show_preview': show_preview,
                'usb_port': serial_cfg,
                'camera_count': 1,
                'fps': 60,
                'video_device': video_cfg,
                'image_qos_reliability': LaunchConfiguration('image_qos_reliability'),
                'image_qos_depth': image_qos_depth,
            }],
        )
        das = Node(
            package='robot_driver',
            executable='databus_single',
            name='das_node',
            output='screen',
            parameters=[{
                'serial_port': serial_cfg,
                'topic_left_tactile': 'tactile/left',
                'topic_right_tactile': 'tactile/right',
                'topic_encoder': 'encoder',
                'topic_target_distance': 'target_distance',
            }],
        )
        return GroupAction(actions=[
            PushRosNamespace(ns),
            camera,
            TimerAction(period=das_delay, actions=[das]),
        ])

    left_group = side_group(
        'left_finger', LaunchConfiguration('left_serial'),
        LaunchConfiguration('left_video_device'), das_delay=3.0,
    )
    right_group = side_group(
        'right_finger', LaunchConfiguration('right_serial'),
        LaunchConfiguration('right_video_device'), das_delay=5.0,
    )

    return LaunchDescription([
        left_serial, right_serial, left_video, right_video,
        res_arg, preview_arg, qos_reliability_arg, qos_depth_arg,
        left_group, right_group,
    ])
