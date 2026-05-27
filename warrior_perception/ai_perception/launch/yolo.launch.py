import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('ai_perception')
    params = os.path.join(pkg_share, 'config', 'yolo.yaml')

    return LaunchDescription([
        Node(
            package='ai_perception',
            executable='yolo_node',
            name='yolo_node',
            output='screen',
            parameters=[params],
        ),
    ])
