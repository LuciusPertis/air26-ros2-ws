import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'microbot_behaviors'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AIR26 Workshop',
    maintainer_email='luciuspertis@gmail.com',
    description='AIR26 workshop 02 obstacle-avoidance behaviours (pub-sub / service / action).',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'behavior_manager = microbot_behaviors.behavior_manager:main',
            'obstacle_services = microbot_behaviors.obstacle_services:main',
        ],
    },
)
