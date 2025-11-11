from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'warrior_simulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/worlds',
        [f for f in glob('worlds/**/*', recursive=True) if os.path.isfile(f)]),        
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
            'diff_drive_sim = warrior_simulation.diff_drive_sim:main',
            'swerve_drive_sim = warrior_simulation.swerve_drive_sim:main',
        ],
    },
)
