from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
    )

    test_node = Node(
        package='warrior_serial',
        executable='test_swerve_module',
        name='test_swerve_module',
        output='screen',
    )

    return LaunchDescription([
        joy_node,
        test_node,
    ])
