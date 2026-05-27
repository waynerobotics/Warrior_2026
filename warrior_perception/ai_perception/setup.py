import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ai_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fire',
    maintainer_email='lord.daniel.w@hotmail.com',
    description='YOLO-based ROS 2 wrapper: image -> bitmask + 2D detections',
    license='Apache License 2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'yolo_node = ai_perception.yolo_node:main',
        ],
    },
)
