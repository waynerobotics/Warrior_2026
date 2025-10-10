from setuptools import find_packages, setup

package_name = 'warrior_drive'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'diff_drive = warrior_drive.diff_drive:main',
            'arduino_drive = warrior_drive.arduino_drive:main',
        ],
    },
)
