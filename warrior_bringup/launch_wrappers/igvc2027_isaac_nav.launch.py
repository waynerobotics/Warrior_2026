#!/usr/bin/env python3
"""Run Warrior's physical Isaac articulation and the sensor-only GPS mission."""
from pathlib import Path
import subprocess
import sys
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, GroupAction, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    root = Path(__file__).resolve().parents[2]
    isaac = root / 'warrior_simulation/warrior_isaac'

    def share(package, *parts):
        return PathJoinSubstitution([FindPackageShare(package), *parts])

    def include(package, filename, arguments):
        return GroupAction(actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(share(package, 'launch', filename)),
            launch_arguments=arguments.items())])

    def simulation(context):
        urdf = root / 'build/isaac/warrior.urdf'
        subprocess.run([sys.executable, str(isaac / 'prepare_robot.py'), '--output', str(urdf)], check=True)
        profile = yaml.safe_load(Path(share('warrior_description', 'config', 'mti680g.yaml').perform(context)).read_text())['mti680g_sim']['ros__parameters']
        command = [str(isaac / 'launch_robot.sh'), '--stage', LaunchConfiguration('stage_file').perform(context),
                   '--scene', LaunchConfiguration('scene_file').perform(context), '--urdf', str(urdf),
                   '--port', LaunchConfiguration('port').perform(context),
                   '--capture-time', LaunchConfiguration('capture_time').perform(context),
                   '--seconds', LaunchConfiguration('seconds').perform(context)]
        command.extend(['--imu-frame', profile['imu_frame'], '--antenna-frame', profile['antenna_frame']])
        if LaunchConfiguration('headless').perform(context).lower() in ('true', '1', 'yes'):
            command.append('--headless')
        for name in ('capture', 'report'):
            value = LaunchConfiguration(name).perform(context)
            if value:
                command.extend(['--' + name, value])
        debug_cloud = LaunchConfiguration('debug_cloud').perform(context)
        if debug_cloud:
            command.extend(['--debug-cloud', debug_cloud])
        process = ExecuteProcess(cmd=command, output='screen', sigterm_timeout='15', sigkill_timeout='10')
        state = Node(package='robot_state_publisher', executable='robot_state_publisher',
                     parameters=[{'robot_description': urdf.read_text(), 'use_sim_time': True}], output='screen')
        return [state, RegisterEventHandler(OnProcessExit(target_action=process,
                 on_exit=[EmitEvent(event=Shutdown(reason='Isaac simulator closed'))])), process]

    bridge = Node(package='warrior_bringup', executable='isaac_ros_bridge.py', output='screen', parameters=[{
        'scene_file': LaunchConfiguration('scene_file'),
        'port': ParameterValue(LaunchConfiguration('port'), value_type=int),
        'gps_noise_stddev': ParameterValue(LaunchConfiguration('gps_noise_stddev'), value_type=float),
        'use_wheel_odometry': ParameterValue(LaunchConfiguration('use_wheel_odometry'), value_type=bool),
    }])
    localization = include('warrior_localization', 'gps_sim.launch.py', {
        'use_sim_time': 'true', 'use_wheel_odometry': LaunchConfiguration('use_wheel_odometry'),
        'params_file': share('warrior_localization', 'config', 'gps_sim_ekf.yaml')})
    navigation = include('nav2_bringup', 'navigation_launch.py', {
        'use_sim_time': 'true', 'params_file': LaunchConfiguration('params_file'),
        'autostart': 'false', 'use_composition': 'False'})
    stamper = Node(package='twist_stamper', executable='twist_stamper', name='nav2_twist_stamper',
        output='screen', parameters=[{'frame_id': 'base_footprint', 'use_sim_time': True}],
        remappings=[('cmd_vel_in', '/cmd_vel'), ('cmd_vel_out', '/cmd_vel_stamped')])
    readiness = Node(package='warrior_bringup', executable='wait_for_nav_ready.py', output='screen', parameters=[{
        'odom_topic': '/odometry/local', 'global_frame': 'map', 'robot_base_frame': 'base_footprint',
        'scan_topic': '/navigation/scan', 'gps_odom_topic': '/odometry/gps',
        'timeout_sec': ParameterValue(LaunchConfiguration('nav_ready_timeout'), value_type=float)}])
    activate = Node(package='warrior_bringup', executable='activate_nav2.py', output='screen', parameters=[{
        'timeout_sec': 90.0, 'discovery_settle_sec': 2.0}])
    follower = Node(package='warrior_navigation', executable='nav2_gps_waypoint_follower', output='screen',
        condition=IfCondition(LaunchConfiguration('autostart_waypoints')), parameters=[{
        'use_sim_time': True, 'waypoint_file': LaunchConfiguration('waypoint_file'),
        'conversion_mode': 'from_ll', 'global_frame': 'map', 'robot_base_frame': 'base_footprint',
        'from_ll_service': '/fromLL', 'navsat_odom_topic': '/odometry/gps',
        'costmap_topic': '/global_costmap/costmap', 'intermediate_goal_distance': 20.0,
        'wait_for_action_timeout': 120.0}])
    rviz = Node(package='rviz2', executable='rviz2', name='isaac_navigation_rviz', output='screen',
        condition=IfCondition(LaunchConfiguration('launch_rviz')),
        arguments=['-d', share('warrior_bringup', 'rviz', 'warrior.gps_nav.rviz')], parameters=[{'use_sim_time': True}])

    def start_navigation(event, _context):
        return [navigation, activate] if event.returncode == 0 else [EmitEvent(event=Shutdown(reason='Isaac navigation readiness failed'))]

    def start_mission(event, _context):
        return [rviz, follower] if event.returncode == 0 else [EmitEvent(event=Shutdown(reason='Nav2 activation failed'))]

    arguments = [
        ('stage_file', str(isaac / 'worlds/igvc2027_autonav.usda'), 'Generated Isaac course USD'),
        ('scene_file', str(root / 'warrior_simulation/igvc2027/scene.json'), 'Shared spawn and GPS datum'),
        ('headless', 'false', 'Run without the Isaac GUI'),
        ('seconds', '0', 'Stop after this many simulation seconds; 0 runs until closed'),
        ('port', '17427', 'Local UDP telemetry port; commands use the next port'),
        ('gps_noise_stddev', '-1.0', 'Optional isotropic GNSS sigma override in meters; -1 uses MTi-680G RTK defaults'),
        ('use_wheel_odometry', 'true', 'Fuse wheel encoders with GNSS/INS; false uses GNSS/INS alone'),
        ('launch_rviz', 'false', 'Also open the ROS navigation view'),
        ('autostart_waypoints', 'true', 'Begin the GPS mission after sensors and Nav2 are ready'),
        ('nav_ready_timeout', '240.0', 'Sensor/localization startup timeout in wall seconds'),
        ('waypoint_file', share('warrior_navigation', 'config', 'igvc2027_autonav_nml_waypoints.yaml'), 'GPS waypoint YAML'),
        ('params_file', share('warrior_navigation', 'config', 'nav2_gps_sim_params.yaml'), 'Nav2 configuration'),
        ('capture', '', 'Optional final rendered PNG path'),
        ('capture_time', '0', 'Capture at this simulation second; 0 captures the final frame'),
        ('report', '', 'Optional final physics and sensor validation JSON path'),
        ('debug_cloud', '', 'Optional final measured lidar cloud NPZ path'),
    ]
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value, description=description) for name, value, description in arguments],
        bridge, localization, stamper,
        RegisterEventHandler(OnProcessExit(target_action=bridge, on_exit=[EmitEvent(event=Shutdown(reason='Isaac ROS adapter closed'))])),
        RegisterEventHandler(OnProcessExit(target_action=readiness, on_exit=start_navigation)),
        RegisterEventHandler(OnProcessExit(target_action=activate, on_exit=start_mission)),
        OpaqueFunction(function=simulation), readiness,
    ])
