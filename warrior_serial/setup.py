import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'warrior_serial'

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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fire',
    maintainer_email='lord.daniel.w@hotmail.com',
    description='ROS 2 serial bridge for the Warrior microcontroller stack',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'warrior_base_driver = warrior_serial.base_driver:main',
            'twist_to_motor = warrior_serial.twist_to_motor:main',
            'twist_to_spark = warrior_serial.twist_to_spark:main',
            'motor_manager = warrior_serial.motor_manager:main',
        ],
    },
)
