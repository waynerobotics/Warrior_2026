
import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    TimerAction,
    RegisterEventHandler,
    DeclareLaunchArgument,
    ExecuteProcess,
    SetEnvironmentVariable,
    Shutdown,
)
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    gazebo_world = LaunchConfiguration('world_name')
    launch_gui = LaunchConfiguration('launch_gui')
    gui_delay = LaunchConfiguration('gui_delay')
    launch_rviz = LaunchConfiguration('launch_rviz')
    
    # ----------------- path -----------------
    pkg_warrior_description = FindPackageShare("warrior_description")
    pkg_warrior_control = FindPackageShare("warrior_control")
    pkg_warrior_bringup = FindPackageShare("warrior_bringup")
    pkg_gazebo = FindPackageShare("warrior_gazebo")

    world_file = PathJoinSubstitution([pkg_gazebo, "worlds", gazebo_world])
    xacro_file = PathJoinSubstitution([pkg_warrior_description, "urdf", "gzsim.urdf.xacro"])
    controller_yaml  = PathJoinSubstitution([pkg_warrior_control, "config", "warrior_controllers_sim.yaml"])
    gazebo_bridge_yaml = PathJoinSubstitution([pkg_warrior_bringup, "config", "diff_gz_bridge.yaml"])

    model_resource_path = os.path.join(
        get_package_share_directory("warrior_gazebo"), "models"
    )
    pixi_prefix = os.environ.get("CONDA_PREFIX", "")
    pixi_library_path = os.path.join(pixi_prefix, "lib") if pixi_prefix else ""
    system_plugin_path = os.pathsep.join(filter(None, [
        pixi_library_path,
        os.environ.get("IGN_GAZEBO_SYSTEM_PLUGIN_PATH", ""),
        os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", ""),
        os.environ.get("LD_LIBRARY_PATH", ""),
    ]))
    gazebo_env = {
        "IGN_GAZEBO_RESOURCE_PATH": os.pathsep.join(filter(None, [
            model_resource_path,
            os.environ.get("IGN_GAZEBO_RESOURCE_PATH", ""),
        ])),
        "GZ_SIM_RESOURCE_PATH": os.pathsep.join(filter(None, [
            model_resource_path,
            os.environ.get("GZ_SIM_RESOURCE_PATH", ""),
        ])),
        "IGN_GAZEBO_SYSTEM_PLUGIN_PATH": system_plugin_path,
        "GZ_SIM_SYSTEM_PLUGIN_PATH": system_plugin_path,
    }

    rviz2_config_file = PathJoinSubstitution(
        [pkg_warrior_bringup, "rviz", "warrior.gazebo.rviz"]
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
        condition=IfCondition(launch_rviz),
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
        remappings=[('cmd_vel', '/cmd_vel_stamped')],
        parameters=[
            {'stamped': True},
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # Fortress can leave an unsupervised server behind when GUI and server are
    # started together. Separate processes keep shutdown and logging reliable.
    gazebo_server = ExecuteProcess(
        cmd=[
            "ign", "gazebo", "--force-version", "6",
            "-s", "-r", "-v", "4", world_file,
        ],
        additional_env=gazebo_env,
        output="screen",
        on_exit=Shutdown(),
    )

    gazebo_gui = ExecuteProcess(
        condition=IfCondition(launch_gui),
        cmd=[
            "ign", "gazebo", "--force-version", "6",
            "-g", "-v", "4",
        ],
        additional_env=gazebo_env,
        output="screen",
    )
    
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "robot_description",
            "-name", "warrior",
            "-allow_renaming", "true",
            '-x', '-9.11', '-y', '35.92', '-z', '0.1'
        ],
        output="screen",
        # parameters=[{"use_sim_time": use_sim_time}],
    )
    
    bridge_params = os.path.join(
        get_package_share_directory('warrior_bringup'),
        'config',
        'warrior_bridge.yaml'
    )

    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        #arguments=[            
        #     # clock
        #    '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        #
        #    # RGB camera
        #    '/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
        #    '/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',

            # Depth camera
        #    '/depth_camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
        #    '/depth_camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
        #    '/depth_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
        #    '/depth_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',

            # lidar
        #    '/L1_lidar/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
        #    '/L1_lidar/scan/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',

            # GPS
        #    '/navsat@sensor_msgs/msg/NavSatFix@gz.msgs.NavSat',

            # odom
        #    '/odom_gt@nav_msgs/msg/Odometry@gz.msgs.Odometry',

            # imu
        #    '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
        #],
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        #remappings=[
        #    ('/L1_lidar/scan', '/scan'),
        #    ('/L1_lidar/scan/points', '/scan/points'),
        #],
        output='screen',
    )

    # ----------------- launch order -----------------
    return LaunchDescription([
        # Keep every Gazebo Transport participant on the same local interface.
        # On multi-interface hosts, automatic selection can split discovery.
        SetEnvironmentVariable("IGN_IP", "127.0.0.1"),
        SetEnvironmentVariable("GZ_IP", "127.0.0.1"),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='If true, use simulated clock'
        ),
        DeclareLaunchArgument(
            'world_name',
            default_value='competition.world',
            description='Name of the Gazebo world file to load'
        ),
        DeclareLaunchArgument(
            'launch_gui',
            default_value='true',
            description='Launch the Gazebo graphical client'
        ),
        DeclareLaunchArgument(
            'gui_delay',
            default_value='3.0',
            description='Seconds to delay the Gazebo graphical client'
        ),
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='true',
            description='Launch the basic robot RViz view'
        ),
        
        gazebo_server,
        TimerAction(period=gui_delay, actions=[gazebo_gui]),
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

        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=swerve_drive_controller_spawner,
        #         on_exit=[teleop_node],
        #     )
        # ),
        
    ])
