# AIR26 Workshop 02 — REAL ROBOT bring-up (micro-ROS / ESP32).
#
# The ESP32 firmware REPLACES mujoco_driver: over WiFi/UDP it publishes /ultrasonic/* (UInt8
# cm) and subscribes /cmd_vel, exactly like the sim. So the real-robot stack is just the
# ROS-side glue: robot_state_publisher (TF/model) + range_viz_bridge (UInt8 -> Range for RViz)
# + RViz + the behaviour nodes. The micro-ROS Agent is the bridge between the board and ROS.
#
#   # 0) source ROS, THIS workspace, AND the agent workspace (see RUN_REAL.md):
#   source /opt/ros/humble/setup.bash && source ~/air26-ros2-ws/install/setup.bash
#   source ~/uros_ws/install/setup.bash
#
#   # 1a) agent in its OWN terminal (recommended), then this launch with agent:=false:
#   ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
#   ros2 launch microbot_sim real.launch.py
#
#   # 1b) OR let the launch spawn the agent too:
#   ros2 launch microbot_sim real.launch.py agent:=true
#
# Args: agent:=false | agent_port:=8888 | use_rviz:=true | behaviors:=true

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    desc = get_package_share_directory('microbot_description')
    xacro_file = os.path.join(desc, 'urdf', 'microbot.urdf.xacro')
    rviz_cfg = os.path.join(desc, 'rviz', 'microbot_real.rviz')   # Fixed Frame = base_link
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument('agent', default_value='false',
                                        choices=['true', 'false'],
                                        description='also spawn the micro-ROS Agent here'))
    ld.add_action(DeclareLaunchArgument('agent_port', default_value='8888'))
    ld.add_action(DeclareLaunchArgument('use_rviz', default_value='true',
                                        choices=['true', 'false']))
    ld.add_action(DeclareLaunchArgument('behaviors', default_value='true',
                                        choices=['true', 'false']))

    # --- micro-ROS Agent (opt-in; needs ~/uros_ws sourced) ---
    # UDP4 on agent_port: the ESP32 firmware's AGENT_PORT must match (default 8888).
    ld.add_action(ExecuteProcess(
        cmd=['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent',
             'udp4', '--port', LaunchConfiguration('agent_port')],
        output='screen',
        condition=IfCondition(LaunchConfiguration('agent'))))

    # --- model + TF (the ESP32 has no robot_state_publisher; the host provides it) ---
    ld.add_action(Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]))

    # No odom/world frame here — RViz's Fixed Frame is base_link (microbot_real.rviz). The only
    # TF is the robot describing its OWN links (robot_state_publisher, from the URDF), which is
    # what places the model + the us_front/left/right frames the range cones attach to. The 4
    # wheel joints are 'continuous', so they need /joint_states to have a TF; the firmware sends
    # none, so joint_state_publisher pins them at 0 (wheels render but don't spin). That's it —
    # no odom, no static world transform.
    ld.add_action(Node(
        package='joint_state_publisher', executable='joint_state_publisher'))

    # --- viz-only: UInt8 cm -> sensor_msgs/Range on /ultrasonic/*/range, for RViz ---
    ld.add_action(Node(
        package='microbot_sim', executable='range_viz_bridge', output='log',
        condition=IfCondition(LaunchConfiguration('use_rviz'))))

    ld.add_action(Node(
        package='rviz2', executable='rviz2', output='log', arguments=['-d', rviz_cfg],
        condition=IfCondition(LaunchConfiguration('use_rviz'))))

    # --- the behaviours (identical to the sim; the board is just another /cmd_vel driver) ---
    ld.add_action(Node(
        package='microbot_behaviors', executable='behavior_manager', output='screen',
        condition=IfCondition(LaunchConfiguration('behaviors'))))
    ld.add_action(Node(
        package='microbot_behaviors', executable='obstacle_services', output='screen',
        condition=IfCondition(LaunchConfiguration('behaviors'))))

    return ld
