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
            'complex_path_generator = warrior_navigation.complex_path_generator:main',
            'astar_planner = warrior_navigation.astar_planner:main',
            'complex_path_controller = warrior_navigation.complex_path_controller:main',
            
        ],
    },
)
