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


def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    gazebo_world = LaunchConfiguration('world_name', default='competition.world')
    
    # ----------------- path -----------------
    pkg_gz_ros = FindPackageShare("ros_gz_sim")
    pkg_warrior_description = FindPackageShare("warrior_description")
    pkg_warrior_control = FindPackageShare("warrior_control")
    pkg_warrior_bringup = FindPackageShare("warrior_bringup")
    pkg_gazebo = FindPackageShare("warrior_gazebo")

    world_file = PathJoinSubstitution([pkg_gazebo, "worlds", gazebo_world])
    xacro_file = PathJoinSubstitution([pkg_warrior_description, "urdf", "gzsim.urdf.xacro"])
    controller_yaml = PathJoinSubstitution([pkg_warrior_control, "config", "warrior_controllers.yaml"])
    gazebo_bridge_yaml = PathJoinSubstitution([pkg_warrior_bringup, "config", "diff_gz_bridge.yaml"])


    # Set GAZEBO model path
    pkg_gazebo_path = get_package_share_directory("warrior_gazebo")
    model_resource_path = os.path.join(pkg_gazebo_path, "models")

    os.environ["IGN_GAZEBO_RESOURCE_PATH"] = \
        os.environ.get("IGN_GAZEBO_RESOURCE_PATH", "") + ":" + model_resource_path

    os.environ["GZ_SIM_RESOURCE_PATH"] = \
        os.environ.get("GZ_SIM_RESOURCE_PATH", "") + ":" + model_resource_path

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
            ('/cmd_vel', '/swerve_drive_controller/cmd_vel')
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
            "-allow_renaming", "true",
            '-x', '0.', '-y', '0.', '-z', '0.3'
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
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'],
        parameters=[{"use_sim_time": use_sim_time}],
        output='screen',
    )

    # ----------------- launch order -----------------
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'
        ),
        DeclareLaunchArgument(
            'world_name',
            default_value=world_file,
            description='Gazebo world file to load'
        ),
        
        gazebo,
        robot_state_publisher,
        # joint_state_publisher,
        # ros2_control_node,
        gz_bridge,
        gz_spawn_entity,
        
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
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

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=swerve_drive_controller_spawner,
                on_exit=[teleop_node],
            )
        ),
        
    ])
