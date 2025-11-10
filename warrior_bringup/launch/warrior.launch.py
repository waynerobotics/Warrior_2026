#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess


def generate_launch_description():
    # ----------------- path -----------------
    pkg_warrior_description = FindPackageShare("warrior_description")
    pkg_warrior_control = FindPackageShare("warrior_control")

    xacro_file = PathJoinSubstitution([pkg_warrior_description, "urdf", "warrior.urdf.xacro"])
    controller_yaml = PathJoinSubstitution([pkg_warrior_control, "config", "warrior_controllers.yaml"])

    rviz2_config_file = PathJoinSubstitution(
        [pkg_warrior_description, "rviz", "warrior.rviz"]
    )


    robot_description = Command(["xacro ", xacro_file])

    # ----------------- nodes -----------------
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="screen",
        parameters=[{"use_gui": False}],
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            {"robot_description": robot_description},
            controller_yaml  # ← This line actually loads the YAML file
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager-timeout", "10"],
        output="screen",
    )

    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--controller-manager-timeout", "10"],
        output="screen",
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz2_config_file],
        parameters=[{"robot_description": robot_description}, 
                    {"use_sim_time": False}],
    )
    
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        prefix='gnome-terminal --', 
        remappings=[
            ('/cmd_vel', '/diff_drive_controller/cmd_vel')
        ],
        parameters=[
            {'stamped': True}
        ]
    )

    # ----------------- launch order -----------------
    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher,
        ros2_control_node,
        TimerAction(period=2.0, actions=[joint_state_broadcaster_spawner]),
        TimerAction(period=2.0, actions=[diff_drive_controller_spawner]),
        rviz2,
        teleop_node,
    ])
