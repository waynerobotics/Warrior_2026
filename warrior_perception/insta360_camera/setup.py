import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'insta360_camera'

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
    description='ROS 2 driver for the Insta360 X4 / X5 in USB / UVC webcam mode',
    license='Apache License 2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'insta360_node = insta360_camera.insta360_node:main',
        ],
    },
)
