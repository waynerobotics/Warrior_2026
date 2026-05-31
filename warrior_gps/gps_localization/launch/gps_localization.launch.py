from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare('gps_localization'),
        'config',
        'gps_config.yaml'
    ])

    waypoints_file = PathJoinSubstitution([
        FindPackageShare('gps_localization'),
        'config',
        [LaunchConfiguration('waypoints'), '_waypoints.yaml']
    ])

    mode = LaunchConfiguration('mode')

    return LaunchDescription([
        DeclareLaunchArgument(
            'mode',
            default_value='outdoor',
            description='Mode: outdoor | indoor | gazebo'
        ),
        DeclareLaunchArgument(
            'waypoints',
            default_value='practice',
            description='Waypoint set: practice | competition'
        ),
        Node(
            package='gps_localization',
            executable='gps_hardware_node',
            name='gps_hardware_node',
            output='screen',
            parameters=[config],
            condition=IfCondition(PythonExpression(["'", mode, "' == 'outdoor'"]))
        ),
        Node(
            package='gps_localization',
            executable='apriltag_gps_bridge',
            name='apriltag_gps_bridge',
            output='screen',
            parameters=[config],
            condition=IfCondition(PythonExpression(["'", mode, "' == 'indoor'"]))
        ),
        Node(
            package='gps_localization',
            executable='simulated_gps_node',
            name='simulated_gps_node',
            output='screen',
            parameters=[config],
            condition=IfCondition(PythonExpression(["'", mode, "' == 'gazebo'"]))
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
            parameters=[config, waypoints_file]
        ),
    ])