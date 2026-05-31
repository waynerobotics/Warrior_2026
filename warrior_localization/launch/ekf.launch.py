from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('warrior_localization')
    config_file = os.path.join(pkg_share, 'config', 'ekf_config.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node_odom',
        output='screen',
        parameters=[config_file, {'use_sim_time': use_sim_time}],
        remappings=[
            ('odometry/filtered', '/odometry/local'),
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation/Gazebo clock'
        ),
        ekf_node,
    ])
