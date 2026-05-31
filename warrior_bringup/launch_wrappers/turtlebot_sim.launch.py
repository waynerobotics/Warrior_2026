#!/usr/bin/env python3
"""
Launch simulated TurtleBot3 Burger with GPS localization.

Usage:
  ros2 launch warrior_bringup turtlebot_sim.launch.py
"""

from click import launch

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution   

def generate_launch_description():
    pkg_warrior_bringup = FindPackageShare('warrior_bringup')
    
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_warrior_bringup, "launch", "main.launch.py"])
            ),
            launch_arguments={
                'robot_type': 'turtlebot_sim',
                'use_sim_time': 'true',
            }.items(),
        ),
    ])
