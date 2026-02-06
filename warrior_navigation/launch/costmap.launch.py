from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Absolute path to: <install>/share/warrior_navigation/config/costmaps_only.yaml
    default_params_file = os.path.join(
        get_package_share_directory('warrior_navigation'),
        'config',
        'costmaps_only.yaml'
    )

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Full path to the ROS2 parameters file'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true'
        ),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
        ),

        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            # Hard-pass these too so lifecycle_manager can’t crash even if YAML has issues
            parameters=[
                params_file,
                {
                    'use_sim_time': use_sim_time,
                    'autostart': True,
                    'node_names': ['planner_server', 'controller_server'],
                }
            ],
        ),
    ])
