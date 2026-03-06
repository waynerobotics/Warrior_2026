from setuptools import setup
import os
from glob import glob

package_name = 'gps_localization'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), 
            glob('launch/*.py')),
        # Include config files
        (os.path.join('share', package_name, 'config'), 
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Agustin_Garcia',
    maintainer_email='agustinjr.1549@gmail.com',
    description='GPS localization system using AprilTags and ROS 2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_frame_node = gps_localization.robot_frame_node:main',
            'world_gps_node = gps_localization.world_gps_node:main',
            'apriltag_gps_bridge = gps_localization.apriltag_gps_bridge:main',
            'waypoint_navigator_node = gps_localization.waypoint_navigator_node:main',
        ],
    },
)
