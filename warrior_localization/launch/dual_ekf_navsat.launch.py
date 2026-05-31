from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('warrior_localization')
    config_file = os.path.join(pkg_share, 'config', 'ekf_config.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    ekf_node_odom = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node_odom',
        output='screen',
        parameters=[config_file, {'use_sim_time': use_sim_time}],
        remappings=[
            ('odometry/filtered', 'odometry/local'),
        ]
    )

    ekf_node_map = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node_map',
        output='screen',
        parameters=[config_file, {'use_sim_time': use_sim_time}],
        remappings=[
            ('odometry/filtered', 'odometry/global'),
        ]
    )

    navsat_transform = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        output='screen',
        parameters=[config_file, {'use_sim_time': use_sim_time}],
        remappings=[
            ('imu/data', 'imu'),
            ('gps/fix', 'shanti/gps/fix'),
            ('gps/filtered', 'gps/filtered'),
            ('odometry/gps', 'odometry/gps'),
            ('odometry/filtered', 'odometry/global')
        ]
    )

    return LaunchDescription([
        ekf_node_odom,
        ekf_node_map,
        navsat_transform
    ])