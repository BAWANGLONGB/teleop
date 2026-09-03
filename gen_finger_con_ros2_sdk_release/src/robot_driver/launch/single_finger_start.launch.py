#!/usr/bin/env python3
"""ROS2 single-finger launch for Gen Finger."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    serial_arg = DeclareLaunchArgument('serial', default_value='/dev/ttyFingerLeft')
    video_arg = DeclareLaunchArgument('video_device', default_value='/dev/finger_camera_left')
    res_arg = DeclareLaunchArgument('camera_resolutions', default_value='1600x1296')
    preview_arg = DeclareLaunchArgument('show_preview', default_value='true')
    qos_reliability_arg = DeclareLaunchArgument('image_qos_reliability', default_value='best_effort')
    qos_depth_arg = DeclareLaunchArgument('image_qos_depth', default_value='1')

    show_preview = ParameterValue(LaunchConfiguration('show_preview'), value_type=bool)
    image_qos_depth = ParameterValue(LaunchConfiguration('image_qos_depth'), value_type=int)

    camera_node = Node(
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
            'resolutions': LaunchConfiguration('camera_resolutions'),
            'topic_base': 'camera',
            'show_preview': show_preview,
            'usb_port': LaunchConfiguration('serial'),
            'camera_count': 1,
            'fps': 60,
            'video_device': LaunchConfiguration('video_device'),
            'image_qos_reliability': LaunchConfiguration('image_qos_reliability'),
            'image_qos_depth': image_qos_depth,
        }],
    )

    das_node = Node(
        package='robot_driver',
        executable='databus_single',
        name='das_node',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial'),
            'topic_left_tactile': 'tactile/left',
            'topic_right_tactile': 'tactile/right',
            'topic_encoder': 'encoder',
            'topic_target_distance': 'target_distance',
        }],
    )

    return LaunchDescription([
        serial_arg, video_arg, res_arg, preview_arg, qos_reliability_arg, qos_depth_arg,
        camera_node,
        TimerAction(period=3.0, actions=[das_node]),
    ])
