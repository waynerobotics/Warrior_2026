"""
warrior_drivers.launch.py
~~~~~~~~~~~~~~~~~~~~~~~~~
Brings up the full serial driver stack:
  - warrior_base_driver  (reads 00_base, publishes /motor_cmd)
  - warrior_swerve_driver x3  (02_swerve, 03_swerve, 04_swerve)

All parameters can be overridden from the command line, e.g.:
  ros2 launch warrior_serial warrior_drivers.launch.py baud_rate:=115200
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    baud_arg = DeclareLaunchArgument(
        'baud_rate', default_value='115200',
        description='Serial baud rate for all Arduino devices')

    discovery_retry_arg = DeclareLaunchArgument(
        'discovery_retry_period_s', default_value='2.0',
        description='Seconds between discovery attempts on disconnect')

    baud = LaunchConfiguration('baud_rate')
    retry = LaunchConfiguration('discovery_retry_period_s')

    base_driver = Node(
        package='warrior_serial',
        executable='warrior_base_driver',
        name='warrior_base_driver',
        parameters=[{
            'device_name': '00_base',
            'baud_rate': baud,
            'discovery_retry_period_s': retry,
        }],
        output='screen',
    )

    swerve_nodes = [
        Node(
            package='warrior_serial',
            executable='warrior_swerve_driver',
            name=f'warrior_swerve_{idx}_driver',
            parameters=[{
                'device_name': f'0{idx}_swerve',
                'baud_rate': baud,
                'discovery_retry_period_s': retry,
            }],
            output='screen',
        )
        for idx in (2, 3, 4)
    ]

    return LaunchDescription([
        baud_arg,
        discovery_retry_arg,
        base_driver,
        *swerve_nodes,
    ])
