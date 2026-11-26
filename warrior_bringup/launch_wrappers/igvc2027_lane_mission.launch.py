#!/usr/bin/env python3
"""IGVC 2027 AutoNav lap with no waypoints in the lanes.

Gazebo P37 course + GPS navigation (no waypoint mission), camera lane
segmentation, lane following, and the mission sequencer that drives the four
no-man's-land GPS targets. Only those four targets are given, as at the
competition.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup = Path(get_package_share_directory('warrior_bringup')) / 'launch'
    lanes = Path(get_package_share_directory('warrior_lane_detection')) / 'launch'
    nav_cfg = Path(get_package_share_directory('warrior_navigation')) / 'config'
    use_sim_time = {'use_sim_time': True}

    arguments = [
        ('launch_gui', 'true', 'Gazebo graphical client'),
        ('launch_rviz', 'true', 'Navigation RViz view'),
        # Lap start: 2 m behind centreline sample 0, inside the east straight.
        ('spawn_x', '56.416', 'Spawn east (m)'),
        ('spawn_y', '10.263', 'Spawn north (m)'),
        ('spawn_z', '-2.347', 'Spawn height (m), measured terrain + 0.1'),
        ('spawn_yaw', '1.5918', 'Spawn heading (rad, ENU), along the lane'),
        ('nml_waypoint_file', str(nav_cfg / 'igvc2027_autonav_nml_waypoints.yaml'),
         'No-man\'s-land GPS targets: entrance, interior..., exit'),
    ]
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(bringup / 'igvc2027_autonav.launch.py')),
        launch_arguments={
            'autostart_waypoints': 'false',
            'launch_gui': LaunchConfiguration('launch_gui'),
            'launch_rviz': LaunchConfiguration('launch_rviz'),
            'spawn_x': LaunchConfiguration('spawn_x'),
            'spawn_y': LaunchConfiguration('spawn_y'),
            'spawn_z': LaunchConfiguration('spawn_z'),
            'spawn_yaw': LaunchConfiguration('spawn_yaw'),
        }.items())
    segmentation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(lanes / 'lane_segmentation.launch.py')))
    follower = Node(package='warrior_navigation', executable='lane_follower',
                    name='lane_follower', output='screen',
                    parameters=[{**use_sim_time, 'enabled': False}])
    mission = Node(package='warrior_navigation', executable='lane_mission',
                   name='lane_mission', output='screen',
                   parameters=[{**use_sim_time,
                                'waypoint_file': LaunchConfiguration('nml_waypoint_file')}])
    return LaunchDescription([
        *[DeclareLaunchArgument(n, default_value=d, description=t) for n, d, t in arguments],
        sim, segmentation, follower, mission,
    ])
