from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _spawn_wheel_nodes(context):
    wheels_str = LaunchConfiguration('wheels').perform(context)
    device_ids = [int(s.strip()) for s in wheels_str.split(',') if s.strip()]
    actions = []
    for i, dev_id in enumerate(device_ids):
        # Stagger node startup so concurrent USB port scans don't race for
        # exclusive opens on the same /dev/ttyACM*.
        actions.append(TimerAction(
            period=float(i) * 2.0,
            actions=[Node(
                package='warrior_serial',
                executable='test_swerve_module',
                name=f'test_swerve_module_{dev_id}',
                output='screen',
                parameters=[{'device_id': dev_id}],
            )],
        ))
    return actions


def generate_launch_description():
    wheels_arg = DeclareLaunchArgument(
        'wheels', default_value='2,3,4',
        description='Comma-separated SPARK MAX device_ids to drive.')

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        arguments=['--ros-args', '--log-level', 'WARN'],
    )

    return LaunchDescription([
        wheels_arg,
        joy_node,
        OpaqueFunction(function=_spawn_wheel_nodes),
    ])
