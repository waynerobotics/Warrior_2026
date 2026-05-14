from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim = LaunchConfiguration('use_sim')

    declare_use_sim = DeclareLaunchArgument(
        'use_sim',
        default_value='true',
        description='Run in simulation or on real robot'
    )

    warrior_nav = FindPackageShare('warrior_navigation')
    nav2_bringup = FindPackageShare('nav2_bringup')

    rviz2_config_file = PathJoinSubstitution(
        [warrior_nav, 'rviz2', 'complex_path.rviz']
    )

    nav2_params_file = PathJoinSubstitution(
        [warrior_nav, 'config', 'nav2_params.yaml']
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                warrior_nav,
                'launch',
                'turtlebot3_world_gps.launch.py',
            ])
        ),
        condition=IfCondition(use_sim)
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('warrior_localization'),
                'launch',
                'online_async_launch.py',
            ])
        ),
        launch_arguments={'use_sim_time': use_sim}.items()
    )

    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('warrior_localization'),
                'launch',
                'dual_ekf_navsat.launch.py',
            ])
        ),
        launch_arguments={'use_sim_time': use_sim}.items()
    )

    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                nav2_bringup,
                'launch',
                'bringup_launch.py',
            ])
        ),
        launch_arguments={
            'slam': 'False',
            'use_localization': 'False',
            'use_sim_time': use_sim,
            'params_file': nav2_params_file,
            'autostart': 'true',
            'use_composition': 'True',
            'use_respawn': 'False',
        }.items()
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz2_config_file],
        parameters=[{'use_sim_time': use_sim}],
    )

    return LaunchDescription([
        declare_use_sim,
        gazebo_launch,
        slam_launch,
        ekf_launch,
        TimerAction(
            period=10.0,
            actions=[nav2_bringup_launch]
        ),
        rviz2,
    ])
