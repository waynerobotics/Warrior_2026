from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = PathJoinSubstitution(
        [FindPackageShare("neural_engine"), "config", "multitask.yaml"]
    )

    multitask_node = Node(
        package="neural_engine",
        executable="multitask_node",
        name="multitask_node",
        parameters=[
            params_file,
        ],
        output="screen",
    )

    return LaunchDescription([
        multitask_node,
    ])
