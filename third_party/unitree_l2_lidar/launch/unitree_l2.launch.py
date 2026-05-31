"""Launch the upstream unitree_lidar_ros2 driver with our config.

Host network prerequisite (Ethernet mode, the default):
  Main LiDAR: 192.168.1.62 — host NIC (enp2s0) needs 192.168.1.2/24.
  Backup LiDAR: 192.168.123.110 — host NIC needs 192.168.123.2/24.

  One-time setup for main LiDAR (persists across reboots):
    sudo nmcli con modify "<your-eth-conn>" \\
        ipv4.method manual ipv4.addresses 192.168.1.2/24 \\
        ipv4.gateway "" ipv4.never-default yes connection.autoconnect yes
    sudo nmcli con up "<your-eth-conn>"

  To switch to the backup LiDAR, change lidar_ip/local_ip in
  unitree_l2.yaml and add 192.168.123.2/24 as a secondary address:
    ETH_CON=$(nmcli -g NAME con show --active | grep enp2s0)
    sudo nmcli con modify "$ETH_CON" +ipv4.addresses 192.168.123.2/24
    sudo nmcli con up "$ETH_CON"
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
