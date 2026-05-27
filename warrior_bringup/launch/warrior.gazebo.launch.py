#!/usr/bin/env python3
"""
DEPRECATED: This launcher is maintained for backward compatibility only.
For new projects, use the centralized launcher system:

  Use: ros2 launch warrior_bringup main.launch.py robot_type:=swerve_sim [world_name:=...]
  Or:  ros2 launch warrior_bringup swerve_sim.launch.py [world_name:=...]
  Or:  ros2 launch warrior_bringup diff_sim.launch.py [world_name:=...]

See warrior_bringup/README.md for complete documentation.
"""
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


def launch_setup(context, *args, **kwargs):

    namespace = LaunchConfiguration("namespace").perform(context)

    # ---- FIX use_sim_time (convert string → bool) ----
    use_sim_time_raw = LaunchConfiguration("use_sim_time").perform(context)
    use_sim_time = (use_sim_time_raw.lower() == "true")

    controller_type = LaunchConfiguration("controller_type").perform(context)

    warrior_description_pkg = FindPackageShare("warrior_description")
    
    warrior_control_pkg = FindPackageShare("warrior_control")
    
    world_name = LaunchConfiguration("world_name").perform(context)

    xacro_file = PathJoinSubstitution([warrior_description_pkg, "urdf", "gzsim.urdf.xacro"])
    robot_description = Command(["xacro ", xacro_file])

    # Choose controller
    if controller_type == "diff_drive_controller":
        controller_file = "diff_drive.gazebo.launch.py"
    else:
        controller_file = "swerve_drive.gazebo.launch.py"

    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([warrior_control_pkg, "launch", controller_file])
        ),
        launch_arguments={
            "use_sim_time": str(use_sim_time).lower(),
            "robot_description": robot_description,
            "world_name": world_name,
        }.items()
    )

    # Joy node
    joy_node = Node(
        package="joy",
        executable="joy_node",
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    # Teleop node
    teleop = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_twist_joy",
        parameters=[
            {
            "publish_stamped_twist": True,
            "use_sim_time": use_sim_time,
            "axis_linear.x": 1,
            "axis_linear.y": 0,
            "axis_angular.yaw": 3,
            "scale_linear.x": 1.0,
            "scale_linear.y": 1.0,
            "scale_angular.yaw": 2.0
        }],
        # remappings=[("cmd_vel", f"/{controller_type}/cmd_vel")],
        output="screen",
    )

    return [
        controller_launch,
        joy_node,
        teleop,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="warrior"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("controller_type", default_value="swerve_drive_controller"),
        DeclareLaunchArgument("world_name", default_value="competition.world"),
        
        OpaqueFunction(function=launch_setup),
    ])
