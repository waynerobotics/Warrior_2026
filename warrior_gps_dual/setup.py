from glob import glob

from setuptools import find_packages, setup

package_name = 'warrior_gps_dual'

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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Warrior',
    maintainer_email='caffeineaddiction.ai@gmail.com',
    description='Dual u-blox NMEA GPS driver: two NavSatFix topics for EKF fusion.',
    license='MIT',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'gps_dual_node = warrior_gps_dual.gps_dual_node:main',
            'enable_waas = warrior_gps_dual.enable_waas:main',
        ],
    },
)
