import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'vla_demo'

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
    description='AIR26 workshop 07 mini-VLA: instruction -> delta-theta -> arm in RViz/Gazebo/MuJoCo.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vla_brain = vla_demo.vla_brain:main',
            'theta_integrator = vla_demo.theta_integrator:main',
            'mujoco_driver = vla_demo.mujoco_driver:main',
            'gz_command_relay = vla_demo.gz_command_relay:main',
        ],
    },
)
