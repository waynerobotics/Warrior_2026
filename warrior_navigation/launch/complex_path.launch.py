from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():

    use_sim = LaunchConfiguration('use_sim')

    declare_use_sim = DeclareLaunchArgument(
        'use_sim',
        default_value='true',
        description='Run in simulation or on real robot'
    )

    warrior_nav = FindPackageShare("warrior_navigation")

    rviz2_config_file = PathJoinSubstitution(
        [warrior_nav, "rviz2", "complex_path.rviz"]
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('warrior_navigation'),
                'launch',
                'turtlebot3_world_gps.launch.py'
            ])
        #     FindPackageShare('turtlebot3_gazebo'),
        #         'launch',
        #         'turtlebot3_world.launch.py'
        #     ])
        ),
        condition=IfCondition(use_sim)
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('warrior_localization'),
                'launch',
                'online_async_launch.py'
            ])
        ),
        launch_arguments={'use_sim_time': use_sim}.items()
    )

    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('warrior_localization'),
                'launch',
                'dual_ekf_navsat.launch.py'
            ])
        ),
        launch_arguments={'use_sim_time': use_sim}.items()
    )

    costmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('warrior_navigation'),
                'launch',
                'costmap.launch.py'
            ])
        ),
        launch_arguments={'use_sim_time': use_sim}.items()
    )


    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz2_config_file],
        parameters=[{"use_sim_time": use_sim}],
    )

    path_to_pose_server_node = Node(
        package='warrior_navigation',
        executable='path_to_pose_server',
        name='path_to_pose_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim,
            'action_name': 'compute_path_to_pose_core',
            'costmap_topic': '/costmap',
        }],
    )

    recovery_manager_node = Node(
        package='warrior_navigation',
        executable='recovery_manager',
        name='recovery_manager',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim,
            'action_name': 'path_to_pose',
            'planner_action_name': 'compute_path_to_pose_core',
            'enable_rviz_goal_bridge': True,
        }],
    )

    follow_path_node = Node(
        package='warrior_navigation',
        executable='follow_path',
        name='follow_path',
        output='screen',
        parameters=[{'use_sim_time': use_sim}],
    )

    return LaunchDescription([
        declare_use_sim,
        gazebo_launch,
        slam_launch,
        ekf_launch,
        costmap_launch,
        path_to_pose_server_node,
        recovery_manager_node,
        follow_path_node,
        rviz2
    ])
