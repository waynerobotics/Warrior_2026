import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    # launch argments
    use_sim_time=LaunchConfiguration('use_sim_time')
    world_name=LaunchConfiguration('world_name')

    # packages
    gz_ros_pkg = FindPackageShare('ros_gz_sim')
    