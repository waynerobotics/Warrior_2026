#!/usr/bin/env python3
"""
warrior_swerve_teleop.launch.py
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Drive the REAL Warrior swerve robot from an Xbox controller.

Pipeline:

  joy_node  →  teleop_twist_joy  →  /cmd_vel (TwistStamped)
    →  swerve_drive_controller            (ros2_control)
    →  warrior_system/SwerveTopicBridge   →  /warrior_swerve_command
    →  warrior_driver                     →  USB (drive Arduinos + SPARK MAX steering)

This launch file OWNS warrior_driver — do not run it (or steer_calibration_node)
separately, or two processes will fight over the USB ports.

Prerequisites:
  * Robot powered (12 V on) and all USB connected (expect ~7 /dev/ttyACM*).
  * Steering already calibrated (steer_offset_rad in warrior_driver.yaml).
  * Xbox controller paired BEFORE launch — joy_node blocks waiting for it.

Usage:
  ros2 launch warrior_bringup warrior_swerve_teleop.launch.py

Pad mapping lives in warrior_joy/config/joystick.yaml (teleop_node section):
hold the enable (deadman) button to drive; left stick = forward/back + yaw,
second button = turbo. publish_stamped_twist is true there because
swerve_drive_controller subscribes to /cmd_vel as a TwistStamped.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    desc_pkg = FindPackageShare('warrior_description')
    control_pkg = FindPackageShare('warrior_control')
    driver_pkg = FindPackageShare('warrior_driver')
    joy_pkg = FindPackageShare('warrior_joy')

    # ------------------------------------------------------------------ args
    joy_dev_arg = DeclareLaunchArgument(
        'joy_dev', default_value='0',
        description='Joystick device ID (usually 0)')
    joy_dev = LaunchConfiguration('joy_dev')

    # Real-robot description: loads the warrior_system/SwerveTopicBridge
    # ros2_control hardware (NOT the gazebo plugin).
    xacro_file = PathJoinSubstitution([desc_pkg, 'urdf', 'warrior.urdf.xacro'])
    robot_description = Command(['xacro ', xacro_file])

    controllers_yaml = PathJoinSubstitution(
        [control_pkg, 'config', 'warrior_controllers_real.yaml'])

    joy_config = PathJoinSubstitution(
        [joy_pkg, 'config', 'joystick.yaml'])

    # -------------------------------------------------------- control + driver

    # ros2_control: owns the SwerveTopicBridge hardware + the controllers.
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description}, controllers_yaml],
        remappings=[('~/robot_description', '/robot_description')],
        output='both',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager-timeout', '120'],
        output='screen',
    )

    swerve_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['swerve_drive_controller', '--controller-manager-timeout', '120'],
        output='screen',
    )

    # Hardware driver — owns the USB serial connections.
    warrior_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([driver_pkg, 'launch', 'warrior_driver.launch.py'])),
    )

    # --------------------------------------------------------------- joystick

    # joy_node — reads the Xbox controller, publishes sensor_msgs/Joy.
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[joy_config, {'device_id': ParameterValue(joy_dev, value_type=int)}],
    )

    # teleop_twist_joy — Joy → /cmd_vel as TwistStamped (publish_stamped_twist
    # is set in joystick.yaml). Node name 'teleop_node' matches the yaml block.
    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_node',
        output='screen',
        parameters=[joy_config],
    )

    return LaunchDescription([
        joy_dev_arg,
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,
        # Bring up swerve_drive_controller only after the broadcaster is active.
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[swerve_drive_controller_spawner],
            )
        ),
        warrior_driver,
        joy_node,
        teleop_node,
    ])
