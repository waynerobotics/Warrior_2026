#!/usr/bin/env python3
"""
Launch simulated Differential-drive Warrior robot in empty world.

Usage:
  ros2 launch warrior_bringup diff_sim.launch.py
  ros2 launch warrior_bringup diff_sim.launch.py world_name:=competition.world
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_warrior_bringup = FindPackageShare('warrior_bringup')
    world_name = LaunchConfiguration('world_name')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'world_name',
            default_value='empty.world',
            description='Name of the world file to load'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(pkg_warrior_bringup) + '/launch/main.launch.py'
            ),
            launch_arguments={
                'robot_type': 'diff_sim',
                'world_name': world_name,
                'use_sim_time': 'true',
            }.items(),
        ),
    ])
