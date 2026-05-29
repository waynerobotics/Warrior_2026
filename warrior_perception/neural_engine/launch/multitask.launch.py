from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    engine_path_arg = DeclareLaunchArgument(
        "engine_path",
        default_value=PathJoinSubstitution(
            [FindPackageShare("neural_engine"), "inference", "resnet18.engine"]
        ),
        description="Path to the TensorRT engine file (defaults to the repo-bundled resnet18.engine)",
    )

    params_file = PathJoinSubstitution(
        [FindPackageShare("neural_engine"), "config", "multitask.yaml"]
    )

    multitask_node = Node(
        package="neural_engine",
        executable="multitask_node",
        name="multitask_node",
        parameters=[
            params_file,
            {"engine_path": LaunchConfiguration("engine_path")},
        ],
        output="screen",
    )

    return LaunchDescription([
        engine_path_arg,
        multitask_node,
    ])
