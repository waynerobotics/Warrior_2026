import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("warrior_motor_manager")
    config = os.path.join(pkg_share, "config", "motor_manager.yaml")

    return LaunchDescription([
        Node(
            package="warrior_motor_manager",
            executable="warrior_motor_manager_node",
            name="warrior_motor_manager",
            output="screen",
            parameters=[config],
        )
    ])
