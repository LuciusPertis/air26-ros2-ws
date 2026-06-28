# AIR26 Workshop 07 — RViz target.
#   instruction -> vla_brain -> /delta_theta -> theta_integrator -> /joint_command
#   (+ /joint_states) -> robot_state_publisher -> RViz.
# No physics: the integrator fakes joint feedback so the model moves kinematically.
#
#   ros2 launch vla_demo rviz.launch.py
#   ros2 topic pub /instruction std_msgs/String "{data: wave}"
#   ros2 topic echo /delta_theta

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
    ld.add_action(DeclareLaunchArgument('use_rviz', default_value='true',
                                        choices=['true', 'false']))
    ld.add_action(DeclareLaunchArgument('policy', default_value='scripted'))

    ld.add_action(Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]))

    # === CHECKPOINT: brain ===
    ld.add_action(Node(
        package='vla_demo', executable='vla_brain', output='screen',
        parameters=[{'policy': LaunchConfiguration('policy')}]))
    # === END CHECKPOINT: brain ===

    # === CHECKPOINT: integrator ===
    # publish_joint_states:=true because RViz has no simulator feeding it back.
    ld.add_action(Node(
        package='vla_demo', executable='theta_integrator', output='screen',
        parameters=[{'publish_joint_states': True}]))
    # === END CHECKPOINT: integrator ===

    ld.add_action(Node(
        package='rviz2', executable='rviz2', output='log',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(LaunchConfiguration('use_rviz'))))

    return ld
