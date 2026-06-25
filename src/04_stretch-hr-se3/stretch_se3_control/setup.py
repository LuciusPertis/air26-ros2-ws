from setuptools import find_packages, setup

package_name = 'stretch_se3_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AIR26 Workshop',
    maintainer_email='luciuspertis@gmail.com',
    description='AIR26 workshop Part B basic-control demo nodes for the Stretch SE3 sim.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'square_drive = stretch_se3_control.square_drive:main',
            'lift_arm = stretch_se3_control.lift_arm:main',
            'head_scan = stretch_se3_control.head_scan:main',
            'wrist_gripper = stretch_se3_control.wrist_gripper:main',
            'wake_up = stretch_se3_control.wake_up:main',
        ],
    },
)
