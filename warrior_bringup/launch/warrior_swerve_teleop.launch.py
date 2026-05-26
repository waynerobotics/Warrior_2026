#!/usr/bin/env python3
"""
warrior_swerve_teleop.launch.py
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Brings up the full Xbox → swerve pipeline:

  joy_node  →  joy_swerve  →  /motor_cmd  →  warrior_swerve_driver (x3)

The warrior_base_driver (00_base) is NOT launched here — this launch file is
for Xbox-only control.  If you want to use the physical 00_base board instead,
use warrior_serial/warrior_drivers.launch.py.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ------------------------------------------------------------------ args
    joy_dev_arg = DeclareLaunchArgument(
        'joy_dev', default_value='0',
        description='Joystick device ID (usually 0)')

    deadzone_arg = DeclareLaunchArgument(
        'deadzone', default_value='0.15',
        description='Joystick deadzone (0.0 – 1.0)')

    baud_arg = DeclareLaunchArgument(
        'baud_rate', default_value='115200',
        description='Serial baud rate for swerve Arduinos')

    discovery_retry_arg = DeclareLaunchArgument(
        'discovery_retry_period_s', default_value='2.0',
        description='Seconds between discovery retries when a swerve is missing')

    joy_dev    = LaunchConfiguration('joy_dev')
    deadzone   = LaunchConfiguration('deadzone')
    baud       = LaunchConfiguration('baud_rate')
    retry      = LaunchConfiguration('discovery_retry_period_s')

    joy_config = PathJoinSubstitution(
        [FindPackageShare('warrior_joy'), 'config', 'joystick.yaml'])

    # ------------------------------------------------------------------ nodes

    # 1. joy_node — reads the Xbox controller and publishes sensor_msgs/Joy
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[
            joy_config,
            {
                'device_id':      joy_dev,
                'deadzone':       deadzone,
                'autorepeat_rate': 20.0,
            },
        ],
    )

    # 2. joy_swerve — converts Joy → /cmd_vel (geometry_msgs/Twist)
    joy_swerve_node = Node(
        package='warrior_joy',
        executable='joy_swerve',
        name='joy_swerve',
        output='screen',
    )

    # 3. twist_to_motor — converts /cmd_vel → /motor_cmd (MotorCommand x3)
    twist_to_motor_node = Node(
        package='warrior_serial',
        executable='twist_to_motor',
        name='twist_to_motor',
        output='screen',
        parameters=[{
            'targets': ['02_swerve', '03_swerve', '04_swerve'],
            'scale_spark':   100.0,
            'scale_flipsky': 100.0,
        }],
    )

    # 4. motor_manager — discovery + serial I/O for all three swerve modules
    motor_manager_node = Node(
        package='warrior_serial',
        executable='motor_manager',
        name='motor_manager',
        output='screen',
        parameters=[{
            'targets': ['02_swerve', '03_swerve', '04_swerve'],
            'baud_rate':                 baud,
            'discovery_retry_period_s':  retry,
        }],
    )

    return LaunchDescription([
        joy_dev_arg,
        deadzone_arg,
        baud_arg,
        discovery_retry_arg,
        joy_node,
        joy_swerve_node,
        twist_to_motor_node,
        motor_manager_node,
    ])
