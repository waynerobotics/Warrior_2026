from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    OpaqueFunction,
    GroupAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

import os


def launch_setup(context, *args, **kwargs):

    use_sim = LaunchConfiguration('use_sim')
    simulation = use_sim.perform(context).lower() == 'true'
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
            'use_sim_time': simulation,
            'waypoint_file': selected_waypoint_file,
            'action_name': 'navigate_to_pose',
            'conversion_mode': 'from_ll' if simulation else 'projection',
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
        default_value='false',
        description='Run in simulation or on real robot'
    )

    declare_use_wheel_odometry = DeclareLaunchArgument(
        'use_wheel_odometry', default_value='true', choices=['true', 'false'],
        description='Simulation only: fuse wheel odometry; false uses MTi GNSS/INS velocity',
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
    
    map_file = os.path.join(
        get_package_share_directory('warrior_navigation'),
        'maps',
        'gz_world_save.yaml'
    )

    pc2ls_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim,
            'min_height': 0.5,
            'range_min': 0.2,
            'target_frame': 'base_footprint',
            'use_inf': True
        }],
        remappings=[
            ('cloud_in', 'unilidar/cloud'),
            ('scan', 'unilidar/scan')
        ]
    )

    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('warrior_localization'),
                'launch',
                'dual_ekf_navsat.launch.py'
            ])
        ),
        condition=UnlessCondition(use_sim),
        launch_arguments={
            'use_sim_time': use_sim
        }.items()
    )

    simulation_ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('warrior_localization'), 'launch', 'gps_sim.launch.py',
        ])),
        condition=IfCondition(use_sim),
        launch_arguments={
            'use_sim_time': use_sim,
            'use_wheel_odometry': LaunchConfiguration('use_wheel_odometry'),
        }.items(),
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
            'map': map_file,
            'params_file': nav2_params_file,
            'autostart': 'True',
            'use_composition': 'False',
            'use_respawn': 'False',
        }.items()
    )

    return LaunchDescription([
        declare_use_sim,
        declare_use_wheel_odometry,
        declare_waypoint_file,
        pc2ls_node,
        ekf_launch,
        simulation_ekf_launch,
        # slam_launch,
        # Remap all Nav2 odometry subscribers, including nodes using the
        # upstream default topic; the real-robot configuration is untouched.
        GroupAction(actions=[
            SetRemap(src='/odom', dst='/odometry/local', condition=IfCondition(use_sim)),
            nav2_bringup_launch,
        ]),

        OpaqueFunction(function=launch_setup),
    ])
