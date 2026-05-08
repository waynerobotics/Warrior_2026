from glob import glob
from setuptools import find_packages, setup

package_name = 'warrior_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/maps', glob('maps/*.yaml')),
        ('share/' + package_name + '/rviz2', glob('rviz2/*.rviz')),
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alimyust',
    maintainer_email='alimyust@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'direct_path_generator = warrior_navigation.direct_path_generator:main',
            'map_robot_pose = warrior_navigation.map_robot_pose:main',
            'linear_path_controller = warrior_navigation.linear_path_controller:main',
            'path_to_pose_server = warrior_navigation.compute_path_to_pose:main',
            'recovery_manager = warrior_navigation.recovery_manager:main',
            'astar_planner = warrior_navigation.astar_planner:main',
            'follow_path = warrior_navigation.follow_path:main',
            'waypoint_follower = warrior_navigation.logged_waypoint_follower:main',
            'gps_waypoint_manager = warrior_navigation.gps_waypoint_manager:main'
        ],
    },
)
