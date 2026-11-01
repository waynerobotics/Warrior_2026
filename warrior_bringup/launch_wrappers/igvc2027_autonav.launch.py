#!/usr/bin/env python3
"""Full P37 course with Warrior's GPS/IMU/odometry no-man's-land mission."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import yaml


def generate_launch_description():
    navigation = Path(get_package_share_directory('warrior_navigation')) / 'config'
    bringup = Path(get_package_share_directory('warrior_bringup')) / 'launch'
    profile = yaml.safe_load((navigation / 'igvc2027_autonav_sim.yaml').read_text())
    profile['waypoint_file'] = str(navigation / profile['waypoint_file'])
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=str(value))
          for name, value in profile.items()],
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(bringup / 'swerve_gps_nav_sim.launch.py')),
            launch_arguments={name: LaunchConfiguration(name) for name in profile}.items(),
        ),
    ])
