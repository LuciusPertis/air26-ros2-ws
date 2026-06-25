from setuptools import find_packages, setup

package_name = 'basics_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lsp',
    maintainer_email='lsp@todo.todo',
    description='AIR26 workshop — basics (Python): topics, services, actions',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'topic_talker   = basics_py.topic_talker:main',
            'topic_listener = basics_py.topic_listener:main',
            'service_server = basics_py.service_server:main',
            'service_client = basics_py.service_client:main',
            'action_server  = basics_py.action_server:main',
            'action_client  = basics_py.action_client:main',
            'combined_node  = basics_py.combined_node:main',
        ],
    },
)
