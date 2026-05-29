import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("warrior_driver")
    warrior_driver_config = os.path.join(pkg_share, "config", "warrior_driver.yaml")

    return LaunchDescription([
        Node(
            package="warrior_driver",
            executable="warrior_driver_node",
            name="warrior_driver",
            output="screen",
            parameters=[warrior_driver_config],
        )
    ])
