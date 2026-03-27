from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
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
            package='nav2_costmap_2d',
            executable='nav2_costmap_2d',
            name='global_costmap',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
        ),

        Node(
            package='nav2_costmap_2d',
            executable='nav2_costmap_2d',
            name='local_costmap',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_costmaps',
            output='screen',
            parameters=[
                params_file,
                {
                    'use_sim_time': use_sim_time,
                    'autostart': True,
                    'node_names': ['global_costmap', 'local_costmap'],
                }
            ],
        ),
    ])
