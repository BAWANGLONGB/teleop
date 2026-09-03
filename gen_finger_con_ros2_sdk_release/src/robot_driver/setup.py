from setuptools import setup
from glob import glob
import os

package_name = 'robot_driver'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@example.com',
    description='Gen Finger robot driver for camera and DAS tactile system (ROS2)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_view_single = robot_driver.camera_view_single:main',
            'databus_single = robot_driver.databus_single:main',
            'tactile_dual_print = robot_driver.tactile_dual_print:main',
            'left_das_controller_infer = robot_driver.left_das_controller_infer:main',
            'right_das_controller_infer = robot_driver.right_das_controller_infer:main',
            'camera_calib_cmd = robot_driver.camera_calib_cmd:main',
        ],
    },
)
