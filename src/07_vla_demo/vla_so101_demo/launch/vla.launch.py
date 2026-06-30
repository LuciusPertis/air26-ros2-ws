"""Bring up the SmolVLA SO-101 tabletop demo.

  mujoco_driver (system python) -> SO-101 tabletop in MuJoCo; publishes /camera/front +
                                   /joint_states, applies /joint_command
  smolvla_node  (venv python!)  -> real SmolVLA-450M: text + camera + state -> /joint_command
  instruction_pub               -> the task string (override with the `instruction:=` arg)
  rviz2                         -> shows the VLA's camera feed

SmolVLA needs torch+lerobot from the isolated venv, so it's launched with that interpreter
(ExecuteProcess). The driver + rviz are plain ROS python.

  ros2 launch vla_so101_demo vla.launch.py
  ros2 launch vla_so101_demo vla.launch.py instruction:='stack the blue cube on the red cube'
"""

import os

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

VENV_PYTHON = '/home/lsp/vla_venv/bin/python'


def generate_launch_description():
    instruction = LaunchConfiguration('instruction')
    checkpoint = LaunchConfiguration('checkpoint')
    use_rviz = LaunchConfiguration('use_rviz')

    smolvla_script = os.path.join(get_package_prefix('vla_so101_demo'),
                                  'lib', 'vla_so101_demo', 'smolvla_node')
    rviz_cfg = os.path.join(get_package_prefix('vla_so101_demo'),
                            'share', 'vla_so101_demo', 'rviz', 'vla.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('instruction', default_value='pick up the red cube'),
        DeclareLaunchArgument('checkpoint', default_value='lerobot/smolvla_base'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        SetEnvironmentVariable('MUJOCO_GL', 'egl'),

        Node(package='vla_so101_demo', executable='mujoco_driver', output='screen'),

        Node(package='vla_so101_demo', executable='instruction_pub', output='screen',
             parameters=[{'instruction': instruction}]),

        # SmolVLA runs in the venv (torch/lerobot) -> launch with the venv interpreter
        ExecuteProcess(
            cmd=[VENV_PYTHON, smolvla_script, '--ros-args',
                 '-p', ['instruction:=', instruction],
                 '-p', ['checkpoint:=', checkpoint]],
            output='screen'),

        Node(package='rviz2', executable='rviz2', condition=IfCondition(use_rviz),
             arguments=['-d', rviz_cfg], output='log'),
    ])
