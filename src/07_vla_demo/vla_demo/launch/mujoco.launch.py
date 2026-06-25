# AIR26 Workshop 07 — MuJoCo target.
#   instruction -> vla_brain -> /delta_theta -> theta_integrator -> /joint_command
#   -> mujoco_driver (physics) -> /joint_states.
# Here a real simulator publishes /joint_states, so the integrator does NOT
# (publish_joint_states:=false) and the brain gets true proprioception from the sim.
#
#   ros2 launch vla_demo mujoco.launch.py                 # native MuJoCo viewer
#   ros2 launch vla_demo mujoco.launch.py use_viewer:=false use_rviz:=true   # via RViz
#   ros2 topic pub /instruction std_msgs/String "{data: circle}"

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    desc = get_package_share_directory('vla_arm_description')
    xacro_file = os.path.join(desc, 'urdf', 'arm.urdf.xacro')
    rviz_cfg = os.path.join(desc, 'rviz', 'arm.rviz')
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument('use_viewer', default_value='true',
                                        choices=['true', 'false']))
    ld.add_action(DeclareLaunchArgument('use_rviz', default_value='false',
                                        choices=['true', 'false']))
    ld.add_action(DeclareLaunchArgument('policy', default_value='scripted'))

    ld.add_action(Node(
        package='vla_demo', executable='vla_brain', output='screen',
        parameters=[{'policy': LaunchConfiguration('policy')}]))
    ld.add_action(Node(
        package='vla_demo', executable='theta_integrator', output='screen',
        parameters=[{'publish_joint_states': False}]))   # the sim owns /joint_states

    # === CHECKPOINT: mujoco ===
    ld.add_action(Node(
        package='vla_demo', executable='mujoco_driver', output='screen',
        parameters=[{'use_viewer': LaunchConfiguration('use_viewer')}]))
    # === END CHECKPOINT: mujoco ===

    # optional RViz (fed by the sim's /joint_states)
    ld.add_action(Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        condition=IfCondition(LaunchConfiguration('use_rviz'))))
    ld.add_action(Node(
        package='rviz2', executable='rviz2', output='log', arguments=['-d', rviz_cfg],
        condition=IfCondition(LaunchConfiguration('use_rviz'))))

    return ld
