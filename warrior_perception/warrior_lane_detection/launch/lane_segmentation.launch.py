from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=PathJoinSubstitution(
            [FindPackageShare('warrior_lane_detection'), 'config', 'lane_segmentation.yaml'])),
        Node(package='warrior_lane_detection', executable='lane_segmentation',
             name='lane_segmentation', output='screen',
             parameters=[LaunchConfiguration('params_file'),
                         {'use_sim_time': LaunchConfiguration('use_sim_time')}]),
    ])
