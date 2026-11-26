import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'warrior_lane_detection'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Charlescai123',
    maintainer_email='yihaocai007@gmail.com',
    description='Classical RGB-D white-lane segmentation',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'lane_segmentation = warrior_lane_detection.lane_segmentation_node:main',
        ],
    },
)
