

# Launch file to work with turtlebot3 burger for a controller (for cmd_vel_unstamped)
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    joy_params = os.path.join(get_package_share_directory('warrior_joy'),'config','joystick.yaml')

    joy_node = Node(
        package='joy',
        executable='joy_node',
        parameters=[joy_params]
    )

    joy_teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_node',
        output='screen',
        parameters=[joy_params]

    )

    twist_stamper_node = Node( # turltebot3 requires stamped cmd_vel
        package='twist_stamper',
        executable='twist_stamper',
        name='twist_stamper',
        output='screen',
        remappings=[('cmd_vel_in', 'cmd_vel_unstamped'), ('cmd_vel_out', 'cmd_vel') ]
    )

    return LaunchDescription([
        joy_node,
        joy_teleop_node,
        # twist_stamper_node,
    ])