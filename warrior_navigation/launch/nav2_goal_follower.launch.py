from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim = LaunchConfiguration('use_sim')
    use_gps_waypoints = LaunchConfiguration('use_gps_waypoints')
    waypoint_file = LaunchConfiguration('waypoint_file')
    map_origin_latitude = LaunchConfiguration('map_origin_latitude')
    map_origin_longitude = LaunchConfiguration('map_origin_longitude')
    map_origin_yaw = LaunchConfiguration('map_origin_yaw')
    utm_zone = LaunchConfiguration('utm_zone')

    declare_use_sim = DeclareLaunchArgument(
        'use_sim',
        default_value='true',
        description='Run in simulation or on real robot'
    )

    declare_use_gps_waypoints = DeclareLaunchArgument(
        'use_gps_waypoints',
        default_value='false',
        description='Use GPS waypoints instead of RViz goal for navigation'
    )

    declare_waypoint_file = DeclareLaunchArgument(
        'waypoint_file',
        default_value='',
        description='Path to GPS waypoint YAML file. Leave empty to use the package default.'
    )

    declare_map_origin_latitude = DeclareLaunchArgument(
        'map_origin_latitude',
        default_value='42.35911527890909',
        description='Latitude of map origin (where robot starts)'
    )

    declare_map_origin_longitude = DeclareLaunchArgument(
        'map_origin_longitude',
        default_value='-83.06651728263228',
        description='Longitude of map origin (where robot starts)'
    )

    declare_map_origin_yaw = DeclareLaunchArgument(
        'map_origin_yaw',
        default_value='0.0',
        description='Yaw angle of map frame at origin (radians)'
    )

    declare_utm_zone = DeclareLaunchArgument(
        'utm_zone',
        default_value='10',
        description='UTM zone used for GPS to map conversion'
    )

    warrior_nav = FindPackageShare('warrior_navigation')
    nav2_bringup = FindPackageShare('nav2_bringup')

    rviz2_config_file = PathJoinSubstitution(
        [warrior_nav, 'rviz2', 'goal_follower.rviz']
    )

    nav2_params_file = PathJoinSubstitution(
        [warrior_nav, 'config', 'nav2_params.yaml']
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                warrior_nav,
                'launch',
                'nav_utils',
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

    nav2_gps_waypoint_node = Node(
        package='warrior_navigation',
        executable='nav2_gps_waypoint_follower',
        name='nav2_gps_waypoint_follower',
        output='screen',
        parameters=[{
            'waypoint_file': waypoint_file,
            'map_origin_latitude': map_origin_latitude,
            'map_origin_longitude': map_origin_longitude,
            'map_origin_yaw': map_origin_yaw,
            'utm_zone': utm_zone,
            'action_name': 'navigate_to_pose',
            'use_sim_time': use_sim,

        }],
        condition=IfCondition(LaunchConfiguration('use_gps_waypoints'))
    )

    return LaunchDescription([
        declare_use_sim,
        declare_use_gps_waypoints,
        declare_waypoint_file,

        declare_map_origin_latitude,
        declare_map_origin_longitude,
        declare_map_origin_yaw,
        declare_utm_zone,

        gazebo_launch,
        slam_launch,
        ekf_launch,

        nav2_bringup_launch,
        TimerAction(period=10.0, actions=[nav2_gps_waypoint_node]),  # Delay GPS waypoint node to ensure Nav2 is ready
        rviz2,
    ])
