import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro


def generate_launch_description():
    pkg_dir = get_package_share_directory('robot_description')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'robot.urdf.xacro')

    robot_description_content = xacro.process_file(urdf_file).toxml()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_content,
            'publish_frequency': 50.0,
        }],
        output='screen',
    )

    return LaunchDescription([
        robot_state_publisher,
    ])
