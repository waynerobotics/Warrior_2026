#!/usr/bin/env python3
"""
Launch real TurtleBot3 Burger hardware.

Usage:
  ros2 launch warrior_bringup turtlebot_real.launch.py
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_warrior_bringup = FindPackageShare('warrior_bringup')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    pkg_warrior_bringup, 'launch', 'main.launch.py'
                ])
            ),
            launch_arguments={
                'robot_type': 'turtlebot_real',
                'use_sim_time': 'false',
            }.items(),
        ),
    ])
