import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("warrior_gps_dual"),
        "config", "gps_dual.yaml",
    )

    return LaunchDescription([
        Node(
            package="warrior_gps_dual",
            executable="gps_dual_node",
            name="gps_dual_node",
            output="screen",
            parameters=[config],
        )
    ])
