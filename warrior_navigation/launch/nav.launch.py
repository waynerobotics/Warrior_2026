from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

import os


def launch_setup(context, *args, **kwargs):

    use_sim = LaunchConfiguration('use_sim')
    waypoint_file = LaunchConfiguration('waypoint_file').perform(context)

    warrior_nav_pkg = get_package_share_directory('warrior_navigation')
    gps_localization_pkg = get_package_share_directory('gps_localization')

    practice_waypoints_file = os.path.join(
        warrior_nav_pkg,
        'config',
        'practice_waypoints.yaml'
    )

    real_waypoints_file = os.path.join(
        warrior_nav_pkg,
        'config',
        'real_waypoints.yaml'
    )

    demo_waypoints_file = os.path.join(
        warrior_nav_pkg,
        'config',
        'demo_waypoints.yaml'
    )

    competition_waypoints_real_file = os.path.join(
        gps_localization_pkg,
        'config',
        'competition_waypoints.yaml'
    )

    practice_waypoints_real_file = os.path.join(
        gps_localization_pkg,
        'config',
        'practice_waypoints.yaml'
    )

    waypoint_files = {
        'practice_nav': practice_waypoints_file,
        'real_nav': real_waypoints_file,
        'demo_nav': demo_waypoints_file,
        'competition_gps': competition_waypoints_real_file,
        'practice_gps': practice_waypoints_real_file,
    }

    if waypoint_file not in waypoint_files:
        raise RuntimeError(
            f"Unknown waypoint_file '{waypoint_file}'. "
            f"Expected one of: {list(waypoint_files.keys())}"
        )

    selected_waypoint_file = waypoint_files[waypoint_file]

    nav2_gps_waypoint_node = Node(
        package='warrior_navigation',
        executable='nav2_gps_waypoint_follower',
        name='nav2_gps_waypoint_follower',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim.perform(context),
            'waypoint_file': selected_waypoint_file,
            'action_name': 'navigate_to_pose',
        }],
    )

    return [
        TimerAction(
            period=10.0,
            actions=[nav2_gps_waypoint_node],
        )
    ]


def generate_launch_description():

    use_sim = LaunchConfiguration('use_sim')

    declare_use_sim = DeclareLaunchArgument(
        'use_sim',
        default_value='true',
        description='Run in simulation or on real robot'
    )

    declare_waypoint_file = DeclareLaunchArgument(
        'waypoint_file',
        default_value='real_nav',
        description='Warrior_nav waypoint: practice_nav, real_nav, demo_nav,' \
        'gps_localizatoin waypoints: competition_gps, practice_gps'
    )

    warrior_nav = FindPackageShare('warrior_navigation')
    nav2_bringup = FindPackageShare('nav2_bringup')

    nav2_params_file = PathJoinSubstitution([
        warrior_nav,
        'config',
        'nav2_params.yaml'
    ])

    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('warrior_localization'),
                'launch',
                'dual_ekf_navsat.launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': use_sim
        }.items()
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('warrior_localization'),
                'launch',
                'online_async_launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': use_sim
        }.items()
    )

    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                nav2_bringup,
                'launch',
                'bringup_launch.py'
            ])
        ),
        launch_arguments={
            'slam': 'False',
            'use_localization': 'False',
            'use_sim_time': use_sim,
            'params_file': nav2_params_file,
            'autostart': 'true',
            'use_composition': 'True',
            'use_respawn': 'False',
        }.items()
    )

    return LaunchDescription([
        declare_use_sim,
        declare_waypoint_file,

        ekf_launch,
        slam_launch,
        nav2_bringup_launch,

        OpaqueFunction(function=launch_setup),
    ])