import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'multibot_bt'

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
    description='AIR26 workshop 03 py_trees patrol behaviour + formation anchor.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'patrol_bt = multibot_bt.patrol_bt:main',
            'formation_anchor = multibot_bt.formation_anchor:main',
        ],
    },
)
