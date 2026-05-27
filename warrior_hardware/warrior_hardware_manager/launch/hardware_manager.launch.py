import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("warrior_hardware_manager")
    config = os.path.join(pkg_share, "config", "hardware_manager.yaml")

    return LaunchDescription([
        Node(
            package="warrior_hardware_manager",
            executable="warrior_hardware_manager_node",
            name="warrior_hardware_manager",
            output="screen",
            parameters=[config],
        )
    ])
