import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'perceptbot_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.wbt') + glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'worlds', 'textures'),
            glob('worlds/textures/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'resource'), glob('resource/*.urdf')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AIR26 Workshop',
    maintainer_email='luciuspertis@gmail.com',
    description='AIR26 workshop 05 Webots simulation of the perception rover.',
    license='Apache-2.0',
    tests_require=['pytest'],
    # perceptbot_driver is a webots_ros2 plugin (imported by WebotsController), not a node
    entry_points={
        'console_scripts': [
            'mujoco_driver = perceptbot_sim.mujoco_driver:main',
            'scan_to_range = perceptbot_sim.scan_to_range:main',
        ],
    },
)
