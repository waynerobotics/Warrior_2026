from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

# Swerve drive launch file

def generate_launch_description():

    bringup_pkg = get_package_share_directory('warrior_bringup')
    description_pkg = get_package_share_directory('warrior_description')
    drive_pkg = get_package_share_directory('warrior_drive')
    warrior_joy = get_package_share_directory('warrior_joy')

    robot_description_dir = os.path.join(description_pkg, 'urdf','robot', 'warrior_swerve.urdf')
    robot_description = xacro.process_file(robot_description_dir).toxml()

    default_world = os.path.join(description_pkg,'worlds','empty.world')    
    world = LaunchConfiguration('world')
    
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='World to load'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
                    launch_arguments={'gz_args': ['-r -v4 ', world], 'on_exit_shutdown': 'true'}.items()
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
    )
    # Run the spawner node from the ros_gz_sim package.
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                    '-name', 'warrior','-z', '1.0'],
        output='screen'
    )

    bridge_params = os.path.join(bringup_pkg,'config','swerve_gz_bridge.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ]
    )

    swerve_drive_sim = Node(
        package='warrior_simulation',
        executable='swerve_drive_sim',
        name='swerve_drive_sim',
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher,

        world_arg,
        gazebo,
        rviz2,

        spawn_entity,
        ros_gz_bridge,

        swerve_drive_sim
    ])