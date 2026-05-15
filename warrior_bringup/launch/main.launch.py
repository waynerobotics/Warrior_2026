#!/usr/bin/env python3
"""
Master launch file for Warrior robot bringup.

This file provides a unified entry point for launching all robot configurations:
- Simulated Swerve drive robot (IGVC competition world)
- Simulated Differential drive robot (empty world)
- Real Warrior hardware
- Simulated TurtleBot3 (with GPS)
- Real TurtleBot3 hardware

Usage examples:
  ros2 launch warrior_bringup main.launch.py robot_type:=swerve_sim
  ros2 launch warrior_bringup main.launch.py robot_type:=swerve_sim world_name:=empty.world
  ros2 launch warrior_bringup main.launch.py robot_type:=diff_sim world_name:=competition.world
  ros2 launch warrior_bringup main.launch.py robot_type:=warrior_real
  ros2 launch warrior_bringup main.launch.py robot_type:=turtlebot_sim
  ros2 launch warrior_bringup main.launch.py robot_type:=turtlebot_real

Supported robot_type values:
  - swerve_sim: Simulated swerve-drive Warrior robot (default world: competition.world)
  - diff_sim: Simulated differential-drive Warrior robot (default world: empty.world)
  - warrior_real: Real Warrior robot hardware
  - turtlebot_sim: Simulated TurtleBot3 Burger (default world: turtlebot3_world_gps)
  - turtlebot_real: Real TurtleBot3 Burger hardware
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def launch_robot(context, *args, **kwargs):
    """
    Conditional launch based on robot_type argument.
    This function is called after all launch arguments are resolved.
    """
    robot_type = LaunchConfiguration('robot_type').perform(context)
    world_name = LaunchConfiguration('world_name').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    namespace = LaunchConfiguration('namespace').perform(context)
    
    launch_actions = []
    
    # ===================== SWERVE SIMULATION =====================
    if robot_type == 'swerve_sim':
        pkg_warrior_control = FindPackageShare('warrior_control')
        swerve_launcher = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_warrior_control, "launch", "swerve_drive.gazebo.launch.py"])
            ),
            launch_arguments={
                'world_name': world_name,
                'use_sim_time': use_sim_time,
            }.items(),
        )
        launch_actions.append(swerve_launcher)
    
    # ===================== DIFFERENTIAL SIMULATION =====================
    elif robot_type == 'diff_sim':
        pkg_warrior_control = FindPackageShare('warrior_control')
        diff_launcher = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_warrior_control, "launch", "diff_drive.gazebo.launch.py"])
            ),
            launch_arguments={
                'world_name': world_name,
                'use_sim_time': use_sim_time,
            }.items(),
        )
        launch_actions.append(diff_launcher)
    
    # ===================== REAL WARRIOR HARDWARE =====================
    elif robot_type == 'warrior_real':
        pkg_warrior_bringup = FindPackageShare('warrior_bringup')
        warrior_real_launcher = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_warrior_bringup, "launch", "warrior.launch.py"])
            ),
            launch_arguments={
                'use_sim_time': 'false',
                'namespace': namespace,
            }.items(),
        )
        launch_actions.append(warrior_real_launcher)
    
    # ===================== SIMULATED TURTLEBOT3 =====================
    elif robot_type == 'turtlebot_sim':
        pkg_warrior_navigation = FindPackageShare('warrior_navigation')
        turtlebot_launcher = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_warrior_navigation, "launch", "nav_utils", "turtlebot3_world_gps.launch.py"])
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
            }.items(),
        )
        launch_actions.append(turtlebot_launcher)
    
    # ===================== REAL TURTLEBOT3 HARDWARE =====================
    elif robot_type == 'turtlebot_real':
        # Set environment variable and include TurtleBot3 bringup
        os.environ['TURTLEBOT3_MODEL'] = 'burger'
        pkg_turtlebot3_bringup = FindPackageShare('turtlebot3_bringup')
        turtlebot_real_launcher = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_turtlebot3_bringup, "launch", "robot.launch.py"])
            ),
            launch_arguments={
                'use_sim_time': 'false',
            }.items(),
        )
        launch_actions.append(turtlebot_real_launcher)
    
    else:
        raise ValueError(
            f"Unknown robot_type: '{robot_type}'. "
            "Must be one of: swerve_sim, diff_sim, warrior_real, turtlebot_sim, turtlebot_real"
        )
    
    return launch_actions


def generate_launch_description():
    """Generate the launch description."""
    
    return LaunchDescription([
        # ===================== LAUNCH ARGUMENTS =====================
        DeclareLaunchArgument(
            'robot_type',
            default_value='swerve_sim',
            description='Type of robot to launch. Options: swerve_sim, diff_sim, warrior_real, turtlebot_sim, turtlebot_real',
            choices=['swerve_sim', 'diff_sim', 'warrior_real', 'turtlebot_sim', 'turtlebot_real']
        ),
        DeclareLaunchArgument(
            'world_name',
            default_value='competition.world',
            description='Name of the Gazebo world file. '
                        'Defaults: competition.world (swerve_sim), empty.world (diff_sim), turtlebot3_world_gps.world (turtlebot_sim). '
                        'Ignored for real robot launches.'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time (true for simulation, false for real hardware)',
            choices=['true', 'false']
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace for robot (typically empty or /robot1 for multi-robot setup)'
        ),
        
        # ===================== OPAQUE FUNCTION FOR CONDITIONAL LAUNCH =====================
        # OpaqueFunction allows runtime decision-making based on argument values
        OpaqueFunction(function=launch_robot),
    ])
