import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'vla_so101_description'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'mjcf'), glob('mjcf/*.xml')),
        (os.path.join('share', package_name, 'mjcf', 'assets'), glob('mjcf/assets/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AIR26 Workshop',
    maintainer_email='luciuspertis@gmail.com',
    description='SO-ARM100/101 tabletop scene for the SmolVLA demo.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': []},
)
