"""Fuse simulated wheel odometry, ENU IMU heading, and one GPS antenna."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Start both EKFs and expose navsat_transform's /fromLL converter."""
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    parameters = [params_file, {'use_sim_time': use_sim_time}]

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('warrior_localization'),
                'config', 'gps_sim_ekf.yaml',
            ]),
            description='Dual EKF and navsat parameters for Gazebo sensors',
        ),
        Node(
            package='robot_localization', executable='ekf_node',
            name='ekf_node_odom', output='screen', parameters=parameters,
            remappings=[('odometry/filtered', '/odometry/local')],
        ),
        Node(
            package='robot_localization', executable='ekf_node',
            name='ekf_node_map', output='screen', parameters=parameters,
            remappings=[('odometry/filtered', '/odometry/global')],
        ),
        Node(
            package='robot_localization', executable='navsat_transform_node',
            name='navsat_transform', output='screen', parameters=parameters,
            remappings=[
                ('imu', '/mobile_sensor/imu'),
                # Accommodate robot_localization releases using imu/data.
                ('imu/data', '/mobile_sensor/imu'),
                ('gps/fix', '/gps1/fix'),
                ('gps/filtered', '/gps/filtered'),
                ('odometry/gps', '/odometry/gps'),
                ('odometry/filtered', '/odometry/global'),
                ('fromLL', '/fromLL'),
            ],
        ),
    ])
