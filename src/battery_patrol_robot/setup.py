from glob import glob
import os

from setuptools import setup


package_name = 'battery_patrol_robot'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'),
         glob('worlds/*')),
        (os.path.join('share', package_name, 'urdf'),
         glob('urdf/*')),
        (os.path.join('share', package_name, 'rviz'),
         glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'battery_node = battery_patrol_robot.battery_node:main',
            'patrol_node = battery_patrol_robot.patrol_node:main',
        ],
    },
    zip_safe=True,
    maintainer='battery patrol robot',
    maintainer_email='developer@example.com',
    description='Battery-aware autonomous patrol robot simulation.',
    license='Apache-2.0',
)
