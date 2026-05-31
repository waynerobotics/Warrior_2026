import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'omnivision'

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
    description='360-degree camera + LiDAR fusion and yaw calibration tools',
    license='GPL-3.0-only',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'fusion = omnivision.fusion:main',
            'mask_to_pointcloud = omnivision.mask_to_pointcloud:main',
            'transformation_reconfigure = omnivision.transformation_reconfigure:main',
            'transformation_calibrator = omnivision.transformation_calibrator:main',
            'transformation_gui = omnivision.transformation_gui:main',
            'mask_to_laserscan = omnivision.mask_to_laserscan:main',
        ],
    },
)
