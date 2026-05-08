from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    """
    Launch file for GPS waypoint following using custom path planner/follower.
    
    This launch file sets up:
    1. Gazebo simulation (optional)
    2. Localization (EKF + SLAM)
    3. Costmap for path planning
    4. Path planner and follower
    5. Recovery manager
    6. GPS waypoint manager
    """

    use_sim = LaunchConfiguration('use_sim')
    waypoint_file = LaunchConfiguration('waypoint_file')
    map_origin_latitude = LaunchConfiguration('map_origin_latitude')
    map_origin_longitude = LaunchConfiguration('map_origin_longitude')
    map_origin_yaw = LaunchConfiguration('map_origin_yaw')

    declare_use_sim = DeclareLaunchArgument(
        'use_sim',
        default_value='true',
        description='Run in simulation or on real robot'
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

    # Gazebo simulation (optional)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('warrior_navigation'),
                'launch',
                'turtlebot3_world_gps.launch.py'
            ])
        ),
        condition=IfCondition(use_sim)
    )

    # Localization (EKF + NavSat transform for GPS integration)
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

    # SLAM for mapping
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

    # Costmap for path planning
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

    # Path planner (A* based)
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

    # Recovery manager (handles replanning with error recovery)
    recovery_manager_node = Node(
        package='warrior_navigation',
        executable='recovery_manager',
        name='recovery_manager',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim,
            'action_name': 'path_to_pose',
            'planner_action_name': 'compute_path_to_pose_core',
            'enable_rviz_goal_bridge': False,  # Disable RViz goal bridge for mission mode
        }],
    )

    # Path follower (pure pursuit)
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
    )

    return LaunchDescription([
        declare_use_sim,
        declare_waypoint_file,
        declare_map_origin_latitude,
        declare_map_origin_longitude,
        declare_map_origin_yaw,
        gazebo_launch,
        ekf_launch,
        slam_launch,
        costmap_launch,
        path_to_pose_server_node,
        recovery_manager_node,
        follow_path_node,
        # Give all nodes time to initialize before starting GPS waypoint manager
        # Costmap lifecycle manager times out after 4s, so wait 10s total
        TimerAction(
            period=10.0,
            actions=[gps_waypoint_manager_node]
        )
    ])
