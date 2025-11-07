from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    bringup_pkg = get_package_share_directory('warrior_bringup')
    description_pkg = get_package_share_directory('warrior_description')
    drive_pkg = get_package_share_directory('warrior_drive')
    warrior_joy = get_package_share_directory('warrior_joy')

    robot_description_dir = os.path.join(description_pkg, 'urdf/robot', 'warrior.urdf')
    robot_description = xacro.process_file(robot_description_dir).toxml()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

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

    diff_drive_arduino = Node(
        package='warrior_drive',
        executable='arduino_drive',
        name='arduino_drive',
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher,

        joy_node,
        joy_2stick_node,
        diff_drive_arduino
    ])