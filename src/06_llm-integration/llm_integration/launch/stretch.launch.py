# AIR26 Workshop 06 — LLM integration on the project-04 Stretch SE3 robot (MuJoCo).
#
# One launch brings up the whole demo:
#   * the project-04 Stretch MuJoCo sim (base + lift + arm + head + gripper)  <-- from 04
#   * the LLM motion controller (llm_stretch)                                  <-- /llm/command
#
#   ros2 launch llm_integration stretch.launch.py
#   ros2 launch llm_integration stretch.launch.py backend:=mock          # no Ollama needed
#   ros2 launch llm_integration stretch.launch.py instruction:='raise the lift and look down'
#
# Then publish a natural-language command:
#   ros2 topic pub --once /llm/command std_msgs/String "data: 'extend the arm halfway'"
#
# This file *includes* stretch_se3_bringup's sim.launch.py — the dependency on
# project 04 (see exec_depend in package.xml). The sim starts in its default
# 'position' mode (joints move immediately); the controller switches to
# 'navigation' for base drive and back to 'position' for joint goals as needed.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    bringup = get_package_share_directory('stretch_se3_bringup')

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument('backend', default_value='ollama',
                                        choices=['ollama', 'mock']))
    ld.add_action(DeclareLaunchArgument('model', default_value='qwen3:1.7b'))
    ld.add_action(DeclareLaunchArgument('ollama_host', default_value=''))
    ld.add_action(DeclareLaunchArgument('use_rviz', default_value='true',
                                        choices=['true', 'false']))
    ld.add_action(DeclareLaunchArgument('use_mujoco_viewer', default_value='true',
                                        choices=['true', 'false']))
    ld.add_action(DeclareLaunchArgument(
        'instruction', default_value='',
        description='optional NL command auto-published once after startup'))

    # --- project 04: Stretch in MuJoCo -------------------------------------
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup, 'launch', 'sim.launch.py')),
        launch_arguments={'use_rviz': LaunchConfiguration('use_rviz'),
                          'use_mujoco_viewer': LaunchConfiguration('use_mujoco_viewer')}.items()))

    # --- the LLM motion controller -----------------------------------------
    # === CHECKPOINT: llm-controller ===
    ld.add_action(Node(
        package='llm_integration', executable='llm_stretch', output='screen',
        parameters=[{'backend': LaunchConfiguration('backend'),
                     'model': LaunchConfiguration('model'),
                     'ollama_host': LaunchConfiguration('ollama_host')}]))
    # === END CHECKPOINT: llm-controller ===

    # --- optional: auto-publish one instruction ----------------------------
    # The Stretch sim takes a while to load; give it extra time before publishing.
    ld.add_action(TimerAction(period=12.0, actions=[ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '--once', '/llm/command',
             'std_msgs/String',
             ['data: "', LaunchConfiguration('instruction'), '"']],
        output='screen',
        condition=IfCondition(PythonExpression(
            ["'", LaunchConfiguration('instruction'), "' != ''"])))]))

    return ld
