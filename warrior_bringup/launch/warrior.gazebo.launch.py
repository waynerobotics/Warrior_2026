#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess
from launch.event_handlers import OnProcessExit


def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    
    # ----------------- path -----------------
    pkg_gz_ros = FindPackageShare("ros_gz_sim")
    pkg_warrior_description = FindPackageShare("warrior_description")
    pkg_warrior_control = FindPackageShare("warrior_control")
    pkg_warrior_bringup = FindPackageShare("warrior_bringup")

    world_file = PathJoinSubstitution([pkg_warrior_description, "worlds", "empty.world"])
    xacro_file = PathJoinSubstitution([pkg_warrior_description, "urdf", "gzsim.urdf.xacro"])
    controller_yaml = PathJoinSubstitution([pkg_warrior_control, "config", "warrior_controllers.yaml"])
    gazebo_bridge_yaml = PathJoinSubstitution([pkg_warrior_bringup, "config", "diff_gz_bridge.yaml"])

    rviz2_config_file = PathJoinSubstitution(
        [pkg_warrior_bringup, "rviz", "warrior.rviz"]
    )

    robot_description = Command(["xacro ", xacro_file])

    # ----------------- nodes -----------------
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}, 
                    {"use_sim_time": use_sim_time}],
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="screen",
        parameters=[{"use_gui": True}],
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
                    {"use_sim_time": use_sim_time}],
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
            {'stamped': True},
            {'use_sim_time': use_sim_time}
        ]
    )
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_gz_ros, "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={
            "gz_args": ["-r -v 4 ", world_file]
        }.items(),
    )
    
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "robot_description",
            "-name", "warrior",
            "-allow_renaming", "true"
        ],
        output="screen",
        # parameters=[{"use_sim_time": use_sim_time}],
    )
    
    
    # gz_bridge = Node(
    #     package="ros_gz_bridge",
    #     executable="parameter_bridge",
    #     parameters=[gazebo_bridge_yaml],
    #     output="screen"
    # )
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )


    # ----------------- launch order -----------------
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),
        gazebo,
        robot_state_publisher,
        # joint_state_publisher,
        # ros2_control_node,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[diff_drive_controller_spawner],
            )
        ),
        
        gz_bridge,
        gz_spawn_entity,
        rviz2,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=diff_drive_controller_spawner,
                on_exit=[teleop_node],
            )
        ),
        
    ])
