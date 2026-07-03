import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'perceptlive_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AIR26 Workshop',
    maintainer_email='luciuspertis@gmail.com',
    description='AIR26 workshop 05_perception_live: real-hardware camera + dead-reckoned '
                'odometry stack for multi-viewer RViz (camera_processor, aruco_detector, '
                'mjpeg_bridge, cmd_vel_odometry).',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_processor = perceptlive_perception.camera_processor:main',
            'aruco_detector = perceptlive_perception.aruco_detector:main',
            'mjpeg_bridge = perceptlive_perception.mjpeg_bridge:main',
            'cmd_vel_odometry = perceptlive_perception.cmd_vel_odometry:main',
        ],
    },
)
