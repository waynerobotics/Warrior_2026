import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('insta360_camera')
    params = os.path.join(pkg_share, 'config', 'insta360.yaml')

    return LaunchDescription([
        Node(
            package='insta360_camera',
            executable='insta360_node',
            name='insta360_node',
            output='screen',
            parameters=[params],
        ),
    ])
