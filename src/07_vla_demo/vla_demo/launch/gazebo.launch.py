# AIR26 Workshop 07 — Gazebo (Ignition Fortress) target.
#   instruction -> vla_brain -> /delta_theta -> theta_integrator -> /joint_command
#   -> gz_command_relay -> /position_controller/commands -> gz_ros2_control -> arm.
# gz_ros2_control runs the controller_manager INSIDE Gazebo and publishes the real
# /joint_states (joint_state_broadcaster), so the integrator does not.
#
#   ros2 launch vla_demo gazebo.launch.py                       # GUI for the kids
#   ros2 launch vla_demo gazebo.launch.py gz_args:='-r -s empty.sdf'   # headless
#   ros2 topic pub /instruction std_msgs/String "{data: wave}"

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    desc = get_package_share_directory('vla_arm_description')
    xacro_file = os.path.join(desc, 'urdf', 'arm.urdf.xacro')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' use_gz_control:=true']), value_type=str)
    sim_time = {'use_sim_time': True}

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument('gz_args', default_value='-r empty.sdf',
                                        description="Gazebo args (add -s for headless)."))

    # robot model
    ld.add_action(Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}, sim_time]))

    # Gazebo (Ignition Fortress)
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': LaunchConfiguration('gz_args')}.items()))

    # /clock bridge so use_sim_time works
    ld.add_action(Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock']))

    # spawn the arm from /robot_description
    spawn = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-topic', 'robot_description', '-name', 'vla_arm', '-z', '0.0'])
    ld.add_action(spawn)

    # === CHECKPOINT: controllers ===
    # gz_ros2_control starts the controller_manager once the model is spawned, so
    # the spawners are chained after the spawn (jsb first, then position controller).
    jsb = Node(package='controller_manager', executable='spawner',
               arguments=['joint_state_broadcaster'], output='screen')
    pos = Node(package='controller_manager', executable='spawner',
               arguments=['position_controller'], output='screen')
    ld.add_action(RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb])))
    ld.add_action(RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[pos])))
    # === END CHECKPOINT: controllers ===

    # the VLA pipeline (sim owns /joint_states -> integrator doesn't publish it)
    ld.add_action(Node(package='vla_demo', executable='vla_brain', output='screen',
                       parameters=[sim_time]))
    ld.add_action(Node(package='vla_demo', executable='theta_integrator', output='screen',
                       parameters=[{'publish_joint_states': False}, sim_time]))
    ld.add_action(Node(package='vla_demo', executable='gz_command_relay', output='screen',
                       parameters=[sim_time]))

    return ld
