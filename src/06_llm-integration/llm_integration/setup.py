import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'llm_integration'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lsp',
    maintainer_email='luciuspertis@gmail.com',
    description='AIR26 Workshop 06 — natural-language robot control via local Ollama LLM tool-calling.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # ROS robot controllers
            'llm_micro = llm_integration.robots.micro:main',
            'llm_stretch = llm_integration.robots.stretch:main',
            # standalone (no-ROS) Ollama warm-up demos
            'ollama_api_demo = llm_integration.demos.ollama_api_demo:main',
            'chat_terminal = llm_integration.demos.chat_terminal:main',
        ],
    },
)
