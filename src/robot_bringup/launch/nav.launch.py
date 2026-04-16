"""
nav.launch.py – Base subsystems + RPLIDAR + Nav2 navigation with a pre-built map.

Usage:
  ros2 launch robot_bringup nav.launch.py map:=/path/to/map.yaml

Save a map from slam.launch.py:
  ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
"""

import os
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup_pkg = get_package_share_directory('robot_bringup')

    # ---- Launch arguments ----
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(bringup_pkg, 'config', 'map.yaml'),
        description='Full path to the map YAML file')

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
            'serial_baudrate': 115200,
            'frame_id': 'laser',
            'angle_compensate': True,
            'scan_mode': 'Standard',
        }],
        output='screen',
    )

    # ---- Nav2 bringup ----
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch', 'bringup_launch.py'
            )
        ),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'params_file': os.path.join(bringup_pkg, 'config', 'nav2_params.yaml'),
            'use_sim_time': 'false',
        }.items(),
    )

    return LaunchDescription([
        map_arg,
        serial_port_arg,
        base_launch,
        rplidar_node,
        nav2_launch,
    ])
