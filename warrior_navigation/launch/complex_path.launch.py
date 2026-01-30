from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():

    # --- Gazebo world (TurtleBot3) ---
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('turtlebot3_gazebo'),
                'launch',
                'turtlebot3_world.launch.py'
            ])
        )
    )

    # --- SLAM Toolbox ---
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('warrior_localization'),
                'launch',
                'online_async_launch.py'
            ])
        )
    )

    costmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('warrior_navigation'),
                'launch',
                'costmap.launch.py'
            ])
        )
    )

    rviz_launch = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
    )
    
    robot_map_pose_node = Node(  # publishes map -> robot transform
        package='warrior_localization',
        executable='map_robot_pose',
        name='map_robot_pose',
        output='screen',
    )

    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('warrior_localization'),
                'launch',
                'ekf.launch.py'
            ])
        )
    )

    complex_path_generator_node = Node(   
        package='warrior_navigation',
        executable='complex_path_generator',
        name='complex_path_generator',
        output='screen',
    )
    


    complex_path_controller_node = Node(   
        package='warrior_navigation',
        executable='complex_path_controller',
        name='complex_path_controller',
        output='screen',
    )

    return LaunchDescription([
        gazebo_launch,
        slam_launch,
        costmap_launch,
        ekf_launch,
        complex_path_generator_node,
        complex_path_controller_node,
        rviz_launch
        # nav2_launch,
        # rviz_launch
    ])
