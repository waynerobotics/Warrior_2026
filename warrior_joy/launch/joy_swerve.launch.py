from launch import LaunchDescription
from launch_ros.actions import Node



def generate_launch_description():


    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
    )   

    joy_swerve_node = Node(
        package='warrior_joy',
        executable='joy_swerve',
        name='joy_swerve',
        output='screen',
    )
    return LaunchDescription([
        joy_node,
        joy_swerve_node,
    ])