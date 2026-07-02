# AIR26 Workshop 06 — LLM integration on the project-02 micro_ros rover (MuJoCo).
#
# One launch brings up the whole demo:
#   * the project-02 MuJoCo sim (rover + ultrasonics + odom)   <-- included from 02
#   * the LLM motion controller (llm_micro)                     <-- /llm/command -> /cmd_vel
#
#   ros2 launch llm_integration micro.launch.py
#   ros2 launch llm_integration micro.launch.py backend:=mock        # no Ollama needed
#   ros2 launch llm_integration micro.launch.py instruction:='turn left then drive forward'
#
# Then (in another terminal) publish a natural-language command:
#   ros2 topic pub --once /llm/command std_msgs/String "data: 'back up half a meter'"
#
# This file *includes* microbot_sim's mujoco.launch.py — that is the dependency on
# project 02 (see exec_depend in package.xml).

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
    micro_sim = get_package_share_directory('microbot_sim')

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument('backend', default_value='ollama',
                                        choices=['ollama', 'mock']))
    ld.add_action(DeclareLaunchArgument('model', default_value='qwen3:1.7b'))
    ld.add_action(DeclareLaunchArgument('ollama_host', default_value=''))
    ld.add_action(DeclareLaunchArgument('use_viewer', default_value='true',
                                        choices=['true', 'false']))
    ld.add_action(DeclareLaunchArgument('use_rviz', default_value='true',
                                        choices=['true', 'false']))
    ld.add_action(DeclareLaunchArgument(
        'instruction', default_value='',
        description='optional NL command auto-published once after startup'))

    # --- project 02: the rover in MuJoCo -----------------------------------
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(micro_sim, 'launch', 'mujoco.launch.py')),
        launch_arguments={'use_viewer': LaunchConfiguration('use_viewer'),
                          'use_rviz': LaunchConfiguration('use_rviz')}.items()))

    # --- the LLM motion controller -----------------------------------------
    # === CHECKPOINT: llm-controller ===
    ld.add_action(Node(
        package='llm_integration', executable='llm_micro', output='screen',
        parameters=[{'backend': LaunchConfiguration('backend'),
                     'model': LaunchConfiguration('model'),
                     'ollama_host': LaunchConfiguration('ollama_host')}]))
    # === END CHECKPOINT: llm-controller ===

    # --- optional: auto-publish one instruction ----------------------------
    # Gives the sim + model time to come up, then publishes once.
    ld.add_action(TimerAction(period=6.0, actions=[ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '--once', '/llm/command',
             'std_msgs/String',
             ['data: "', LaunchConfiguration('instruction'), '"']],
        output='screen',
        condition=IfCondition(PythonExpression(
            ["'", LaunchConfiguration('instruction'), "' != ''"])))]))

    return ld
