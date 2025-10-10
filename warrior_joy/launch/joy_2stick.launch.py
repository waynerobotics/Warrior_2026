

def generate_launch_description():
    from launch import LaunchDescription
    from launch_ros.actions import Node

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        # parameters=[{'dev': '/dev/input/js0'}]
    )

    joy_2stick_node = Node(
        package='warrior_joy',
        executable='joy_2stick',
        name='joy_2stick',
        output='screen',
        parameters=[{'left_stick_output': 'left_cmd_vel', 'right_stick_output': 'right_cmd_vel'}]

    )

    return LaunchDescription([
        joy_node,
        joy_2stick_node
    ])