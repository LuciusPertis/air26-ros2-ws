import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'microbot_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AIR26 Workshop',
    maintainer_email='luciuspertis@gmail.com',
    description='AIR26 workshop 02 simulators (MuJoCo + Gazebo) for the obstacle-avoider.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mujoco_driver = microbot_sim.mujoco_driver:main',
            'scan_to_range = microbot_sim.scan_to_range:main',
            'range_viz_bridge = microbot_sim.range_viz_bridge:main',
        ],
    },
)
