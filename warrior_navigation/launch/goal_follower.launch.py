from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():


    use_sim = LaunchConfiguration('use_sim')

    use_gps_waypoints = LaunchConfiguration('use_gps_waypoints')
    waypoint_file = LaunchConfiguration('waypoint_file')
    map_origin_latitude = LaunchConfiguration('map_origin_latitude')
    map_origin_longitude = LaunchConfiguration('map_origin_longitude')
    map_origin_yaw = LaunchConfiguration('map_origin_yaw')

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
        description='Full path to GPS waypoint YAML file. Leave empty to use default (turtlebot_sim_waypoints.yaml)'
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

    warrior_nav = FindPackageShare("warrior_navigation")

    rviz2_config_file = PathJoinSubstitution(
        [warrior_nav, "rviz2", "goal_follower.rviz"]
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('warrior_navigation'),
                'launch',
                'nav_utils',
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
                'nav_utils',
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

        # GPS Waypoint Manager - sequences GPS waypoints and sends to recovery_manager
    gps_waypoint_manager_node = Node(
        package='warrior_navigation',
        executable='gps_waypoint_manager',
        name='gps_waypoint_manager',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim,
            'action_name': 'path_to_pose',
            'waypoint_file': waypoint_file,
            'map_origin_latitude': map_origin_latitude,
            'map_origin_longitude': map_origin_longitude,
            'map_origin_yaw': map_origin_yaw,
            'utm_zone': 10,  # Adjust based on your location
            # utm_hemisphere uses default 'N' for Northern Hemisphere
        }],
        condition=IfCondition(use_gps_waypoints)
    )

    return LaunchDescription([
        declare_use_sim,
        declare_use_gps_waypoints,
        declare_waypoint_file,
        declare_map_origin_latitude,
        declare_map_origin_longitude,
        declare_map_origin_yaw,

        gazebo_launch,
        rviz2,

        slam_launch,
        ekf_launch,
        costmap_launch,

        path_to_pose_server_node,
        recovery_manager_node,
        follow_path_node,
        
        gps_waypoint_manager_node
    ])
