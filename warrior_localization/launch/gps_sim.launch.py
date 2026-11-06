"""Fuse MTi-680G GNSS/INS with optional wheel odometry."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml

from warrior_localization.gps_fusion import fusion_parameters


def _localization_nodes(context):
    use_wheels = LaunchConfiguration('use_wheel_odometry').perform(context).lower()
    if use_wheels not in ('true', 'false'):
        raise ValueError('use_wheel_odometry must be true or false')
    with open(LaunchConfiguration('params_file').perform(context), encoding='utf-8') as stream:
        parameters = fusion_parameters(yaml.safe_load(stream), use_wheels == 'true')
    time = {'use_sim_time': LaunchConfiguration('use_sim_time')}
    nodes = [
        Node(
            package='robot_localization', executable='ekf_node',
            name='ekf_node_odom', output='screen',
            parameters=[parameters['ekf_node_odom']['ros__parameters'], time],
            remappings=[('odometry/filtered', '/odometry/local')],
        ),
        Node(
            package='robot_localization', executable='ekf_node',
            name='ekf_node_map', output='screen',
            parameters=[parameters['ekf_node_map']['ros__parameters'], time],
            remappings=[('odometry/filtered', '/odometry/global')],
        ),
        Node(
            package='robot_localization', executable='navsat_transform_node',
            name='navsat_transform', output='screen',
            parameters=[parameters['navsat_transform']['ros__parameters'], time],
            remappings=[
                ('imu', '/imu/data'),
                # Accommodate robot_localization releases using imu/data.
                ('imu/data', '/imu/data'),
                ('gps/fix', '/gnss/fix'),
                ('gps/filtered', '/gps/filtered'),
                ('odometry/gps', '/odometry/gps'),
                ('odometry/filtered', '/odometry/global'),
                ('fromLL', '/fromLL'),
            ],
        ),
    ]
    if use_wheels == 'false':
        nodes.append(Node(
            package='warrior_localization', executable='gnss_velocity_to_body',
            name='gnss_velocity_to_body', output='screen',
            parameters=[parameters.get('gnss_velocity_to_body', {}).get('ros__parameters', {}),
                        {'base_frame': parameters['ekf_node_odom']['ros__parameters'][
                            'base_link_frame']}, time],
        ))
    return nodes


def generate_launch_description():
    """Start both EKFs and expose navsat_transform's /fromLL converter."""
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'use_wheel_odometry', default_value='true', choices=['true', 'false'],
            description='Use wheel velocities; false uses GNSS/INS velocity and no wheel input',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('warrior_localization'),
                'config', 'gps_sim_ekf.yaml',
            ]),
            description='Dual EKF/navsat tuning; mode always controls wheel input selection',
        ),
        OpaqueFunction(function=_localization_nodes),
    ])
