"""Launch the upstream unitree_lidar_ros2 driver with our config.

Host network prerequisite (Ethernet mode, the default):
  sudo nmcli con modify "<your-eth-conn>" \\
      ipv4.method manual ipv4.addresses 192.168.1.2/24 \\
      ipv4.gateway "" ipv4.never-default yes connection.autoconnect yes
  sudo nmcli con up "<your-eth-conn>"
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('unitree_l2_lidar')
    default_params = os.path.join(pkg_share, 'config', 'unitree_l2.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='YAML overriding unitree_lidar_ros2_node parameters'),
        Node(
            package='unitree_lidar_ros2',
            executable='unitree_lidar_ros2_node',
            name='unitree_lidar_ros2_node',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),
    ])
