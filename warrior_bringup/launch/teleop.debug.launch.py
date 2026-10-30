from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():

    # package share paths
    description_pkg = FindPackageShare('warrior_description')
    control_pkg = FindPackageShare('warrior_control')

    # file paths
    xacro_file = PathJoinSubstitution([description_pkg, 'urdf', 'gzsim.urdf.xacro'])
    robot_description = Command(["xacro ", xacro_file])

    # controller launch file
    controller_launch = IncludeLaunchDescription(
        PathJoinSubstitution([control_pkg, 'launch', 'swerve_drive.gazebo.launch.py']),
        launch_arguments={
            'use_sim_time': 'False',
            'robot_description': robot_description,
            'world_name': 'competition.world'
        }.items()
    )

    # nodes

    # uncomment these for joystick control, keyboard teleop needs to be run in a separate terminal
    # joy_node = Node(
    #     package="joy",
    #     executable="joy_node",
    #     parameters=[{"use_sim_time": use_sim_time}],
    #     output="screen",
    # )

    # teleop_node = Node(
    #     package="teleop_twist_joy",
    #     executable="teleop_node",
    #     name="teleop_twist_joy",
    #     parameters=[
    #         {
    #         "publish_stamped_twist": True,
    #         "use_sim_time": use_sim_time,
    #         "axis_linear.x": 1,
    #         "axis_linear.y": 0,
    #         "axis_angular.yaw": 3,
    #         "scale_linear.x": 1.0,
    #         "scale_linear.y": 1.0,
    #         "scale_angular.yaw": 2.0
    #     }],
    #     # remappings=[("cmd_vel", f"/{controller_type}/cmd_vel")],
    #     output="screen",
    # )

    return LaunchDescription([
        controller_launch,
        # teleop_node
    ])
