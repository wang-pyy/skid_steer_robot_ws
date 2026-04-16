"""
base.launch.py – Starts the core robot subsystems:
  - robot_state_publisher (URDF → TF)
  - robot_driver           (cmd_vel → motors)
  - robot_odom             (encoders → wheel odometry)
  - robot_bno055           (IMU data)
  - robot_localization EKF (fuses wheel odom + IMU → /odometry/filtered + odom→base_link TF)
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    desc_pkg  = get_package_share_directory('robot_description')
    driver_pkg = get_package_share_directory('robot_driver')
    odom_pkg  = get_package_share_directory('robot_odom')
    imu_pkg   = get_package_share_directory('robot_bno055')
    bringup_pkg = get_package_share_directory('robot_bringup')

    # 1. Robot description (URDF → TF tree)
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(desc_pkg, 'launch', 'description.launch.py')
        )
    )

    # 2. Motor driver
    driver_node = Node(
        package='robot_driver',
        executable='driver_node',
        name='driver_node',
        parameters=[os.path.join(driver_pkg, 'config', 'driver_params.yaml')],
        output='screen',
    )

    # 3. Encoder odometry
    odom_node = Node(
        package='robot_odom',
        executable='odom_node',
        name='odom_node',
        parameters=[os.path.join(odom_pkg, 'config', 'odom_params.yaml')],
        output='screen',
    )

    # 4. BNO055 IMU
    imu_node = Node(
        package='robot_bno055',
        executable='bno055_node',
        name='bno055_node',
        parameters=[os.path.join(imu_pkg, 'config', 'bno055_params.yaml')],
        output='screen',
    )

    # 5. EKF (robot_localization)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[os.path.join(bringup_pkg, 'config', 'ekf.yaml')],
        output='screen',
    )

    return LaunchDescription([
        description_launch,
        driver_node,
        odom_node,
        imu_node,
        ekf_node,
    ])
