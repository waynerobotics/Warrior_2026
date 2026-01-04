from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():



    # --- Robot bringup (drivers, robot_state_publisher, TF) ---
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('turtlebot3_bringup'),
                'launch',
                'robot.launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': 'true'
        }.items()
    )

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

    # --- SLAM Toolbox (your wrapper launch) ---
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('warrior_navigation'),
                'launch',
                'online_async_launch.py'
            ])
        )
    )

    # --- Nav2 bringup ---
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'bringup_launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': 'true',
            # 'slam': 'false'
        }.items()
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
        gazebo_launch,
        slam_launch,
        nav2_launch,
        rviz_launch
    ])
