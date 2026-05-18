from launch import LaunchDescription
from launch_ros.actions import Node


_WHEELS = [2, 3, 4]


def generate_launch_description():
    targets = [f'{w:02d}_swerve' for w in _WHEELS]
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            arguments=['--ros-args', '--log-level', 'WARN'],
        ),
        # /joy -> /cmd_vel (Twist).
        Node(
            package='warrior_joy',
            executable='joy_swerve',
            name='joy_swerve',
            output='screen',
        ),
        # /cmd_vel -> one /motor_cmd per swerve target.
        Node(
            package='warrior_serial',
            executable='twist_to_motor',
            name='twist_to_motor',
            output='screen',
            parameters=[{'targets': targets}],
        ),
        # /motor_cmd -> Flipsky-side Arduinos (selected via RB/LB).
        Node(
            package='warrior_serial',
            executable='motor_manager',
            name='motor_manager',
            output='screen',
            parameters=[{'targets': targets}],
        ),
        # Direct SLCAN -> SPARK MAX steering controllers.
        Node(
            package='warrior_serial',
            executable='test_swerve_module',
            name='swerve_coordinator',
            output='screen',
            parameters=[{'wheels': _WHEELS}],
        ),
    ])
