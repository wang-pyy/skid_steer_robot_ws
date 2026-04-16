"""
slam.launch.py – Base subsystems + RPLIDAR A1 + slam_toolbox for 2D SLAM.

Usage:
  ros2 launch robot_bringup slam.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup_pkg = get_package_share_directory('robot_bringup')

    # ---- Launch arguments ----
    serial_port_arg = DeclareLaunchArgument(
        'serial_port', default_value='/dev/ttyUSB0',
        description='RPLIDAR serial port')

    # ---- Include base.launch.py ----
    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'base.launch.py')
        )
    )

    # ---- RPLIDAR A1 ----
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'serial_baudrate': 115200,       # A1 default baud rate
            'frame_id': 'laser',
            'angle_compensate': True,
            'scan_mode': 'Standard',
        }],
        output='screen',
    )

    # ---- slam_toolbox (online async) ----
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[
            os.path.join(bringup_pkg, 'config', 'slam_toolbox_params.yaml')
        ],
        output='screen',
    )

    return LaunchDescription([
        serial_port_arg,
        base_launch,
        rplidar_node,
        slam_node,
    ])
