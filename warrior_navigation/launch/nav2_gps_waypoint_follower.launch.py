from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():
    use_sim = LaunchConfiguration('use_sim')
    waypoint_file = LaunchConfiguration('waypoint_file')
    map_origin_latitude = LaunchConfiguration('map_origin_latitude')
    map_origin_longitude = LaunchConfiguration('map_origin_longitude')
    map_origin_yaw = LaunchConfiguration('map_origin_yaw')
    utm_zone = LaunchConfiguration('utm_zone')
    utm_hemisphere = LaunchConfiguration('utm_hemisphere')

    declare_use_sim = DeclareLaunchArgument(
        'use_sim',
        default_value='true',
        description='Run in simulation or on real robot'
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

    declare_utm_hemisphere = DeclareLaunchArgument(
        'utm_hemisphere',
        default_value='N',
        description='UTM hemisphere for GPS conversion (N or S)'
    )

    warrior_nav = FindPackageShare('warrior_navigation')
    nav2_bringup = FindPackageShare('nav2_bringup')

    nav2_params_file = PathJoinSubstitution([
        warrior_nav,
        'config',
        'nav2_params.yaml'
    ])

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                warrior_nav,
                'launch',
                'turtlebot3_world_gps.launch.py'
            ])
        ),
        condition=IfCondition(use_sim)
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

    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                nav2_bringup,
                'launch',
                'bringup_launch.py'
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
            'utm_hemisphere': utm_hemisphere,
            'action_name': 'navigate_to_pose',
        }],
    )

    return LaunchDescription([
        declare_use_sim,
        declare_waypoint_file,
        declare_map_origin_latitude,
        declare_map_origin_longitude,
        declare_map_origin_yaw,
        declare_utm_zone,
        declare_utm_hemisphere,
        gazebo_launch,
        ekf_launch,
        slam_launch,
        nav2_bringup_launch,
        TimerAction(
            period=10.0,
            actions=[nav2_gps_waypoint_node],
        ),
    ])
