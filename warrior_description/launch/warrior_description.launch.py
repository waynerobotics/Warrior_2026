#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_warrior_description = FindPackageShare("warrior_description")

    declare_model_arg = DeclareLaunchArgument(
        name="model",
        default_value=PathJoinSubstitution([pkg_warrior_description, "xacro", "robot.xacro"]),
        description="Path to robot.xacro file"
    )

    declare_use_gui_arg = DeclareLaunchArgument(
        name="use_gui",
        default_value="true",
        description="Whether to use joint_state_publisher_gui"
    )

    declare_rviz_arg = DeclareLaunchArgument(
        name="rvizconfig",
        default_value=PathJoinSubstitution([pkg_warrior_description, "rviz", "warrior_urdf.rviz"]),
        description="Path to RViz config file"
    )

    model_file = LaunchConfiguration("model")
    robot_description_content = Command(["xacro ", model_file])

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description_content}]
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rvizconfig")],
    )

    return LaunchDescription([
        declare_model_arg,
        declare_use_gui_arg,
        declare_rviz_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node
    ])
