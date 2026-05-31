"""
Full perception pipeline:
  insta360_camera      →  camera/image_raw
  unitree_lidar_ros2   →  /unilidar/cloud
  neural_engine        →  ai/seg_mask, ai/detections
  mask_to_pointcloud   →  obstacle_pointcloud  (PointCloud2)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    engine_path_arg = DeclareLaunchArgument(
        'engine_path',
        default_value=PathJoinSubstitution(
            [FindPackageShare('neural_engine'), 'inference', 'resnet18.engine']
        ),
        description='Path to TensorRT engine file',
    )

    insta360 = Node(
        package='insta360_camera',
        executable='insta360_node',
        name='insta360_node',
        output='screen',
        parameters=[PathJoinSubstitution(
            [FindPackageShare('insta360_camera'), 'config', 'insta360.yaml']
        )],
    )

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('unitree_lidar_ros2'), 'launch.py']
            )
        )
    )

    neural_engine = Node(
        package='neural_engine',
        executable='multitask_node',
        name='multitask_node',
        output='screen',
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare('neural_engine'), 'config', 'multitask.yaml']
            ),
            {'engine_path': LaunchConfiguration('engine_path')},
        ],
    )

    mask_to_laserscan = Node(
        package='omnivision',
        executable='mask_to_laserscan',
        name='mask_to_laserscan',
        output='screen',
        parameters=[{
            'mask_topic': 'ai/seg_mask',
            'pointcloud_topic': '/unilidar/cloud',
            'obstacle_pointcloud': 'obstacle_pointcloud',
        }],
    )

    return LaunchDescription([
        engine_path_arg,
        insta360,
        lidar,
        neural_engine,
        mask_to_laserscan,
    ])
