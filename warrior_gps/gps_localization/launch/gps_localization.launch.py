from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('gps_localization'),
        'config',
        'gps_config.yaml'
    )
    mode = LaunchConfiguration('mode')

    return LaunchDescription([
        DeclareLaunchArgument(
            'mode',
            default_value='outdoor',
            description='Mode: outdoor | indoor | gazebo'
        ),
        Node(
            package='gps_localization',
            executable='gps_hardware_node',
            name='gps_hardware_node',
            output='screen',
            parameters=[config],
            condition=IfCondition(PythonExpression(["'",mode,"' == 'outdoor'"]))
        ),
        Node(
            package='gps_localization',
            executable='apriltag_gps_bridge',
            name='apriltag_gps_bridge',
            output='screen',
            parameters=[config],
            condition=IfCondition(PythonExpression(
                ["'",mode,"' == 'outdoor' or '",mode,"' == 'indoor'"]))
        ),
         Node(
            package='gps_localization',
            executable='robot_frame_node',
            name='robot_frame_node',
            output='screen',
            parameters=[config]
        ),
         Node(
            package='gps_localization',
            executable='world_gps_node',
            name='world_gps_node',
            output='screen',
            parameters=[config]
        ),
         Node(
            package='gps_localization',
            executable='waypoint_navigator_node',
            name='waypoint_navigator_node',
            output='screen',
            parameters=[config]
            ),
        ])
    