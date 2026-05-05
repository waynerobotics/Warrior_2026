#!/usr/bin/env python3
"""
Launch real Warrior hardware robot with ROS 2 control stack.

Usage:
  ros2 launch warrior_bringup warrior_real.launch.py
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_warrior_bringup = FindPackageShare('warrior_bringup')
    
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(pkg_warrior_bringup) + '/launch/main.launch.py'
            ),
            launch_arguments={
                'robot_type': 'warrior_real',
                'use_sim_time': 'false',
            }.items(),
        ),
    ])
