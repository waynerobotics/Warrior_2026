#!/usr/bin/env python3
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
    # Launch Arguments
    
    # ----------------- path -----------------
    pkg_warrior_description = FindPackageShare("warrior_description")
    pkg_warrior_control = FindPackageShare("warrior_control")
    pkg_warrior_bringup = FindPackageShare("warrior_bringup")

    xacro_file = PathJoinSubstitution([pkg_warrior_description, "urdf", "warrior.urdf.xacro"])
    controller_yaml  = PathJoinSubstitution([pkg_warrior_control, "config", "warrior_controllers_real.yaml"])

    # Set GAZEBO model path
    pkg_gazebo_path = get_package_share_directory("warrior_gazebo")
    model_resource_path = os.path.join(pkg_gazebo_path, "models")

    os.environ["IGN_GAZEBO_RESOURCE_PATH"] = \
        os.environ.get("IGN_GAZEBO_RESOURCE_PATH", "") + ":" + model_resource_path

    os.environ["GZ_SIM_RESOURCE_PATH"] = \
        os.environ.get("GZ_SIM_RESOURCE_PATH", "") + ":" + model_resource_path

    rviz2_config_file = PathJoinSubstitution(
        [pkg_warrior_bringup, "rviz", "warrior.real.rviz"]
    )

    robot_description = Command(["xacro ", xacro_file])

    # ----------------- nodes -----------------
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description},
            controller_yaml,
        ],
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
        output="both",
    )
        
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager-timeout", "120"],
        output="screen",
    )
    
    swerve_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["swerve_drive_controller", "--controller-manager-timeout", "120"],
        output="screen",
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz2_config_file],
        parameters=[{"robot_description": robot_description}],
    )
    
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        prefix='gnome-terminal --', 
        # remappings=[
        #     ('/cmd_vel', '/swerve_drive_controller/cmd_vel')
        # ],
        parameters=[
            {'stamped': True},
        ]
    )
    

    # ----------------- launch order -----------------
    return LaunchDescription([
        
        robot_state_publisher,
        # ros2_control_node,
        controller_manager,
        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=controller_manager,
        #         on_exit=[joint_state_broadcaster_spawner],
        #     )
        # ),
        joint_state_broadcaster_spawner,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[swerve_drive_controller_spawner],
            )
        ),
        
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=swerve_drive_controller_spawner,
                on_exit=[rviz2],
            )
        ),

        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=swerve_drive_controller_spawner,
        #         on_exit=[teleop_node],
        #     )
        # ),
        
    ])
