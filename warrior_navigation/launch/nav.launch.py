from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    slam_params = PathJoinSubstitution([
        FindPackageShare('warrior_navigation'),
        'config',
        'mapper_params_online_async.yaml'
    ])

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('slam_toolbox'),
                'launch',
                'online_async_launch.py'
            ])
        ]),
        launch_arguments={
            'slam_params_file': slam_params
        }.items()
    )

    return LaunchDescription([
        slam_toolbox_launch
    ])
