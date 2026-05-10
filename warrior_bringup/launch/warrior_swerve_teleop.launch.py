#!/usr/bin/env python3
"""
warrior_swerve_teleop.launch.py
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Full Xbox → swerve pipeline (position-driven steering):

  joy_node → joy_swerve → /cmd_vel ─┬→ twist_to_motor → /motor_cmd → motor_manager → Flipsky Arduinos
                                     └→ twist_to_spark → /spark_cmd → spark_max_driver(s) → SPARK MAX

Flipsky (drive speed)  : motor_manager over ASCII serial to 02/03/04_swerve Arduinos
SPARK MAX (steer angle): spark_max_driver C++ node over USB CDC binary protocol
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
        description='Serial baud rate for swerve Arduinos (Flipsky side)')

    discovery_retry_arg = DeclareLaunchArgument(
        'discovery_retry_period_s', default_value='2.0',
        description='Seconds between discovery retries when a device is missing')

    rate_scale_arg = DeclareLaunchArgument(
        'rate_scale', default_value='2.0',
        description='Steering rate: rotations/sec at full joystick deflection')

    max_position_arg = DeclareLaunchArgument(
        'max_position', default_value='5.0',
        description='Soft position clamp for SPARK MAX steering (±rotations)')

    joy_dev      = LaunchConfiguration('joy_dev')
    deadzone     = LaunchConfiguration('deadzone')
    baud         = LaunchConfiguration('baud_rate')
    retry        = LaunchConfiguration('discovery_retry_period_s')
    rate_scale   = LaunchConfiguration('rate_scale')
    max_position = LaunchConfiguration('max_position')

    joy_config = PathJoinSubstitution(
        [FindPackageShare('warrior_joy'), 'config', 'joystick.yaml'])

    # ------------------------------------------------------------------ nodes

    # 1. joy_node — reads the Xbox controller → sensor_msgs/Joy
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[
            joy_config,
            {
                'device_id':       joy_dev,
                'deadzone':        deadzone,
                'autorepeat_rate': 20.0,
            },
        ],
    )

    # 2. joy_swerve — Joy → /cmd_vel (Twist)
    joy_swerve_node = Node(
        package='warrior_joy',
        executable='joy_swerve',
        name='joy_swerve',
        output='screen',
    )

    # 3. twist_to_motor — /cmd_vel → /motor_cmd (MotorCommand, Flipsky drive speed)
    twist_to_motor_node = Node(
        package='warrior_serial',
        executable='twist_to_motor',
        name='twist_to_motor',
        output='screen',
        parameters=[{
            'targets':       ['02_swerve', '03_swerve', '04_swerve'],
            'scale_spark':   0.0,    # spark field unused — steering now on SPARK MAX
            'scale_flipsky': 100.0,  # linear.x → flipsky drive speed
        }],
    )

    # 4. twist_to_spark — /cmd_vel → /spark_cmd (SparkCommand, position setpoint)
    twist_to_spark_node = Node(
        package='warrior_serial',
        executable='twist_to_spark',
        name='twist_to_spark',
        output='screen',
        parameters=[{
            'targets':          ['02_spark', '03_spark', '04_spark'],
            'rate_scale':       rate_scale,
            'max_position':     max_position,
            'update_rate_hz':   20.0,
        }],
    )

    # 5. motor_manager — ASCII serial to Flipsky Arduinos (drive)
    motor_manager_node = Node(
        package='warrior_serial',
        executable='motor_manager',
        name='motor_manager',
        output='screen',
        parameters=[{
            'targets':                   ['02_swerve', '03_swerve', '04_swerve'],
            'baud_rate':                 baud,
            'discovery_retry_period_s':  retry,
        }],
    )

    # 6–8. spark_max_driver — one C++ node per SPARK MAX (steer position)
    #      device_id must match the CAN ID set in REV Hardware Client (1–62)
    spark_nodes = [
        Node(
            package='warrior_sparkmax',
            executable='spark_max_driver',
            name=f'spark_max_{idx}_driver',
            output='screen',
            parameters=[{
                'device_name':              f'0{idx}_spark',
                'device_id':               idx,   # CAN ID: 2, 3, 4
                'discovery_retry_period_s': retry,
                'heartbeat_ms':            50,
            }],
        )
        for idx in (2, 3, 4)
    ]

    return LaunchDescription([
        joy_dev_arg,
        deadzone_arg,
        baud_arg,
        discovery_retry_arg,
        rate_scale_arg,
        max_position_arg,
        joy_node,
        joy_swerve_node,
        twist_to_motor_node,
        twist_to_spark_node,
        motor_manager_node,
        *spark_nodes,
    ])
        joy_dev_arg,
        deadzone_arg,
        baud_arg,
        discovery_retry_arg,
        joy_node,
        joy_swerve_node,
        twist_to_motor_node,
        motor_manager_node,
    ])
