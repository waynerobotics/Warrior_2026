#!/usr/bin/env python3
"""Run the schematic IGVC 2027 no-man's-land GPS simulation."""

import math

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, GroupAction, IncludeLaunchDescription,
    RegisterEventHandler, SetLaunchConfiguration,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    def share(package, *parts):
        return PathJoinSubstitution([FindPackageShare(package), *parts])

    def include(package, filename, arguments, scoped=True):
        launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(share(package, 'launch', filename)),
            launch_arguments=arguments.items(),
        )
        return GroupAction(actions=[launch]) if scoped else launch

    use_sim_time = LaunchConfiguration('use_sim_time')
    # The local EKF owns odom -> base_footprint; wheel odometry is still published.
    controllers = RewrittenYaml(
        source_file=share('warrior_control', 'config', 'warrior_controllers_sim.yaml'),
        param_rewrites={
            'swerve_drive_controller.ros__parameters.enable_odom_tf': 'false',
        },
        convert_types=True,
    )
    simulation = include('warrior_control', 'swerve_drive.gazebo.launch.py', {
        'world_name': LaunchConfiguration('world_name'),
        'use_sim_time': use_sim_time,
        'launch_gui': LaunchConfiguration('launch_gui'),
        'gui_delay': LaunchConfiguration('gazebo_gui_delay'),
        'launch_rviz': 'false',
        'controller_params_file': controllers,
        **{key: LaunchConfiguration(key) for key in
           ('spawn_x', 'spawn_y', 'spawn_z', 'spawn_yaw')},
    }, scoped=False)  # Its delayed GUI/controller callbacks need the sim context.
    localization = include('warrior_localization', 'gps_sim.launch.py', {
        'use_sim_time': use_sim_time,
        'params_file': share('warrior_localization', 'config', 'gps_sim_ekf.yaml'),
    })
    navigation = include('nav2_bringup', 'navigation_launch.py', {
        'use_sim_time': use_sim_time,
        'params_file': LaunchConfiguration('params_file'),
        'autostart': 'false',
        'use_composition': 'False',
    })
    stamper = Node(
        package='twist_stamper', executable='twist_stamper',
        name='nav2_twist_stamper', output='screen',
        parameters=[{'frame_id': 'base_footprint', 'use_sim_time': use_sim_time}],
        remappings=[('cmd_vel_in', '/cmd_vel'), ('cmd_vel_out', '/cmd_vel_stamped')],
    )
    obstacle_scan = Node(
        package='warrior_navigation', executable='ground_obstacle_scan',
        name='navigation_obstacle_scan', output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            # Fit local ground from lidar returns in the upright base frame.
            # A fixed Z cutoff mistakes sloping parking-lot pavement for obstacles.
            'target_frame': 'base_footprint',
            'ground_height': 0.0,
            'min_height': 0.15, 'max_height': 1.5,
            'range_min': 0.9, 'range_max': 15.0,
            'angle_min': -math.pi, 'angle_max': math.pi,
            'angle_increment': math.pi / 180.0,
            'scan_time': 0.1,
            'use_inf': True,
        }],
        remappings=[('cloud_in', '/unilidar/cloud'), ('scan', '/navigation/scan')],
    )
    readiness = Node(
        package='warrior_bringup', executable='wait_for_nav_ready.py',
        name='wait_for_nav_ready', output='screen',
        parameters=[{
            'odom_topic': '/odometry/local',
            'global_frame': 'map',
            'robot_base_frame': 'base_footprint',
            'scan_topic': '/navigation/scan',
            'gps_odom_topic': '/odometry/gps',
            'timeout_sec': ParameterValue(
                LaunchConfiguration('nav_ready_timeout'), value_type=float),
        }],
    )
    activate = Node(
        package='warrior_bringup', executable='activate_nav2.py',
        name='activate_nav2', output='screen',
        parameters=[{
            'timeout_sec': ParameterValue(
                LaunchConfiguration('nav2_activation_timeout'), value_type=float),
            'discovery_settle_sec': 2.0,
        }],
    )
    follower = Node(
        package='warrior_navigation', executable='nav2_gps_waypoint_follower',
        name='nav2_gps_waypoint_follower', output='screen',
        condition=IfCondition(LaunchConfiguration('autostart_waypoints')),
        parameters=[{
            'use_sim_time': use_sim_time,
            'waypoint_file': LaunchConfiguration('waypoint_file'),
            'conversion_mode': 'from_ll',
            'global_frame': 'map',
            'robot_base_frame': 'base_footprint',
            'from_ll_service': '/fromLL',
            'navsat_odom_topic': '/odometry/gps',
            'costmap_topic': '/global_costmap/costmap',
            'intermediate_goal_distance': ParameterValue(
                LaunchConfiguration('intermediate_goal_distance'), value_type=float),
            'wait_for_action_timeout': 120.0,
        }],
    )
    rviz = Node(
        package='rviz2', executable='rviz2', name='nav2_rviz', output='screen',
        condition=IfCondition(LaunchConfiguration('launch_gps_rviz')),
        arguments=['-d', share('warrior_bringup', 'rviz', 'warrior.gps_nav.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    def start_navigation(event, _context):
        if event.returncode == 0:
            return [navigation, activate]
        return [EmitEvent(event=Shutdown(reason='GPS navigation readiness failed'))]

    def start_mission(event, _context):
        if event.returncode == 0:
            return [rviz, follower]
        return [EmitEvent(event=Shutdown(reason='Nav2 activation failed'))]

    arguments = [
        ('world_name', 'igvc2027_no_mans_land.world',
         'Gazebo world; the default is a schematic approximation, not a surveyed course'),
        ('use_sim_time', 'true', 'Use the Gazebo clock'),
        ('launch_gui', 'true', 'Launch the Gazebo graphical client'),
        ('launch_rviz', 'true', 'Launch the GPS navigation RViz view'),
        ('gazebo_gui_delay', '12.0', 'Delay the Gazebo GUI during startup (seconds)'),
        ('spawn_x', '0.0', 'Spawn east in Gazebo world coordinates (meters)'),
        ('spawn_y', '-2.0', 'Spawn north in Gazebo world coordinates (meters)'),
        ('spawn_z', '0.1', 'Spawn height (meters)'),
        ('spawn_yaw', '1.5707963267948966', 'Spawn heading (radians, ENU)'),
        ('nav_ready_timeout', '180.0', 'Wait for localization and lidar (seconds)'),
        ('nav2_activation_timeout', '90.0', 'Wait for Nav2 activation (seconds)'),
        ('autostart_waypoints', 'true', 'Start the YAML mission after Nav2 activation'),
        ('intermediate_goal_distance', '20.0',
         'Subgoal distance; keep below half the global costmap width minus margin'),
        ('waypoint_file', share('warrior_navigation', 'config', 'igvc2027_no_mans_land_waypoints.yaml'),
         'YAML containing waypoints: latitude, longitude, yaw (ENU radians)'),
        ('params_file', share('warrior_navigation', 'config', 'nav2_gps_sim_params.yaml'),
         'Nav2 mapless GPS simulation parameters'),
    ]
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=default, description=description)
          for name, default, description in arguments],
        # Nested simulation also declares launch_rviz; preserve the user's choice.
        SetLaunchConfiguration('launch_gps_rviz', LaunchConfiguration('launch_rviz')),
        simulation, localization, obstacle_scan, stamper,
        RegisterEventHandler(OnProcessExit(target_action=readiness, on_exit=start_navigation)),
        RegisterEventHandler(OnProcessExit(target_action=activate, on_exit=start_mission)),
        readiness,
    ])
