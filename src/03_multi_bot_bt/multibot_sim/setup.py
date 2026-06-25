import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'multibot_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AIR26 Workshop',
    maintainer_email='luciuspertis@gmail.com',
    description='AIR26 workshop 03 Webots multi-bot driver + bringup.',
    license='Apache-2.0',
    tests_require=['pytest'],
    # multibot_driver is a webots_ros2 plugin (imported by WebotsController), not a node
    entry_points={'console_scripts': []},
)
