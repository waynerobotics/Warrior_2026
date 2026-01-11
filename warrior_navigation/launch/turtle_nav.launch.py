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
                FindPackageShare('warrior_navigation'),
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

    robot_map_pose_node = Node(  # publishes map -> robot transform
        package='warrior_navigation',
        executable='map_robot_pose',
        name='map_robot_pose',
        output='screen',
    )

    complex_path_generator_node = Node(   
        package='warrior_navigation',
        executable='complex_path_generator',
        name='complex_path_generator',
        output='screen',
    )

    # --- Nav2 bringup ---
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'bringup_launch.py'
            ])
        )
    )

    # --- RViz ---
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('turtlebot3_bringup'),
                'launch',
                'rviz2.launch.py'
            ])
        )
    )

    return LaunchDescription([
        # gazebo_launch,
        slam_launch,
        costmap_launch,
        robot_map_pose_node,
        # complex_path_generator_node
        # nav2_launch,
        # rviz_launch
    ])
