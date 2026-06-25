# AIR26 Workshop 02 — MuJoCo target.
#   mujoco_driver (robot + 3 ultrasonics + odom/TF)  <->  behaviour nodes via /cmd_vel.
#   robot_state_publisher + RViz show the model, TF and the 3 range cones.
#
#   ros2 launch microbot_sim mujoco.launch.py                  # native MuJoCo viewer
#   ros2 launch microbot_sim mujoco.launch.py use_viewer:=false use_rviz:=true

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    desc = get_package_share_directory('microbot_description')
    xacro_file = os.path.join(desc, 'urdf', 'microbot.urdf.xacro')
    rviz_cfg = os.path.join(desc, 'rviz', 'microbot.rviz')
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument('use_viewer', default_value='true',
                                        choices=['true', 'false']))
    ld.add_action(DeclareLaunchArgument('use_rviz', default_value='true',
                                        choices=['true', 'false']))

    ld.add_action(Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]))

    # === CHECKPOINT: mujoco ===
    ld.add_action(Node(
        package='microbot_sim', executable='mujoco_driver', output='screen',
        parameters=[{'use_viewer': LaunchConfiguration('use_viewer')}]))
    # === END CHECKPOINT: mujoco ===

    ld.add_action(Node(
        package='rviz2', executable='rviz2', output='log', arguments=['-d', rviz_cfg],
        condition=IfCondition(LaunchConfiguration('use_rviz'))))

    return ld
