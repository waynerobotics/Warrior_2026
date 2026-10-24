#!/usr/bin/env python3
"""Launch the Warrior swerve simulation with odom-relative Nav2."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    world_name = LaunchConfiguration('world_name')
    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_gui = LaunchConfiguration('launch_gui')
    gazebo_gui_delay = LaunchConfiguration('gazebo_gui_delay')
    launch_rviz = LaunchConfiguration('launch_rviz')
    launch_nav_rviz = LaunchConfiguration('launch_nav_rviz')
    nav_ready_timeout = LaunchConfiguration('nav_ready_timeout')
    nav2_activation_timeout = LaunchConfiguration('nav2_activation_timeout')
    nav2_discovery_settle = LaunchConfiguration('nav2_discovery_settle')
    params_file = LaunchConfiguration('params_file')

    warrior_bringup = FindPackageShare('warrior_bringup')
    warrior_control = FindPackageShare('warrior_control')
    warrior_navigation = FindPackageShare('warrior_navigation')
    nav2_bringup = FindPackageShare('nav2_bringup')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [warrior_control, 'launch', 'swerve_drive.gazebo.launch.py']
            )
        ),
        launch_arguments={
            'world_name': world_name,
            'use_sim_time': use_sim_time,
            'launch_gui': launch_gui,
            'gui_delay': gazebo_gui_delay,
            # The Nav2 view below replaces the basic robot-only RViz session.
            'launch_rviz': 'false',
        }.items(),
    )

    twist_stamper = Node(
        package='twist_stamper',
        executable='twist_stamper',
        name='nav2_twist_stamper',
        output='screen',
        parameters=[{
            'frame_id': 'base_footprint',
            'use_sim_time': use_sim_time,
        }],
        remappings=[
            ('cmd_vel_in', '/cmd_vel'),
            ('cmd_vel_out', '/cmd_vel_stamped'),
        ],
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [nav2_bringup, 'launch', 'navigation_launch.py']
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            # Lifecycle startup is requested after DDS discovery settles.
            'autostart': 'false',
            # Humble's navigation_launch.py evaluates this as Python text.
            'use_composition': 'False',
        }.items(),
    )

    nav2_rviz_config = PathJoinSubstitution(
        [warrior_bringup, 'rviz', 'warrior.nav2.rviz']
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='nav2_rviz',
        condition=IfCondition(launch_nav_rviz),
        output='screen',
        arguments=['-d', nav2_rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    wait_for_nav = Node(
        package='warrior_bringup',
        executable='wait_for_nav_ready.py',
        name='wait_for_nav_ready',
        output='screen',
        parameters=[{
            'odom_topic': '/odom',
            'global_frame': 'odom',
            'robot_base_frame': 'base_footprint',
            'timeout_sec': ParameterValue(nav_ready_timeout, value_type=float),
        }],
    )

    activate_nav2 = Node(
        package='warrior_bringup',
        executable='activate_nav2.py',
        name='activate_nav2',
        output='screen',
        parameters=[{
            'timeout_sec': ParameterValue(
                nav2_activation_timeout, value_type=float),
            'discovery_settle_sec': ParameterValue(
                nav2_discovery_settle, value_type=float),
        }],
    )

    def start_navigation(event, _context):
        if event.returncode == 0:
            return [navigation, activate_nav2]
        return [
            EmitEvent(event=Shutdown(reason='Nav2 readiness check failed')),
        ]

    def start_rviz(event, _context):
        if event.returncode == 0:
            return [rviz]
        return [
            EmitEvent(event=Shutdown(reason='Nav2 activation failed')),
        ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'world_name',
            default_value='competition.world',
            description='Name of the Gazebo world file to load',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use the Gazebo simulation clock',
        ),
        DeclareLaunchArgument(
            'launch_gui',
            default_value='true',
            description='Launch the Gazebo graphical client',
        ),
        DeclareLaunchArgument(
            'gazebo_gui_delay',
            default_value='12.0',
            description='Seconds to defer Gazebo GUI during Nav2 startup',
        ),
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='true',
            description='Launch RViz with the Nav2 panels and goal tool',
        ),
        DeclareLaunchArgument(
            'nav_ready_timeout',
            default_value='120.0',
            description='Seconds to wait for controller odometry and base TF',
        ),
        DeclareLaunchArgument(
            'nav2_activation_timeout',
            default_value='90.0',
            description='Seconds to wait for Nav2 to become active',
        ),
        DeclareLaunchArgument(
            'nav2_discovery_settle',
            default_value='2.0',
            description='DDS settling delay before Nav2 lifecycle startup',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution(
                [warrior_navigation, 'config', 'nav2_sim_params.yaml']
            ),
            description='Full path to the odom-relative Nav2 parameter file',
        ),
        # Preserve the user's choice under a private key before the nested
        # simulator launch sets its own same-named argument to false.
        SetLaunchConfiguration('launch_nav_rviz', launch_rviz),
        sim,
        twist_stamper,
        RegisterEventHandler(
            OnProcessExit(
                target_action=activate_nav2,
                on_exit=start_rviz,
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=wait_for_nav,
                on_exit=start_navigation,
            )
        ),
        wait_for_nav,
    ])
