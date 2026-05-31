from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    # parameters
    use_sim = LaunchConfiguration('use_sim')
    viz = LaunchConfiguration('viz')

    declare_use_sim = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='whether to run in Gazebo'
    )

    declare_viz = DeclareLaunchArgument(
        'viz',
        default_value='false',
        description='whether to launch rviz'
    )

    # package locations
    desc_pkg = FindPackageShare("warrior_description")
    control_pkg = FindPackageShare("warrior_control")
    nav_pkg = FindPackageShare("warrior_navigation")
    driver_pkg = FindPackageShare("warrior_driver")
    gps_pkg = FindPackageShare("gps_localization")

    # robot description
    xacro_file = PathJoinSubstitution([desc_pkg, "urdf", "gzsim.urdf.xacro"])
    robot_description = Command(["xacro ", xacro_file])

    # if !sim hardware nodes
    # swerve controller
    
    # if auto localization + navigation
    # if !auto teleop
    # if sim sim
    return