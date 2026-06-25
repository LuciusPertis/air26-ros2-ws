# AIR26 Workshop — Stretch SE3 MuJoCo bringup (plain scene, no Robocasa).
#
# This is the workshop's own launch file. Unlike the upstream
# stretch_mujoco_driver.launch.py, it does NOT import Robocasa at parse time, so it
# runs with `ros2 launch` on machines without robosuite/robocasa installed.
#
# Run it:
#   ros2 launch stretch_se3_bringup sim.launch.py
#   ros2 launch stretch_se3_bringup sim.launch.py use_rviz:=false      # headless
#   ros2 launch stretch_se3_bringup sim.launch.py mode:=navigation     # for Nav2 (Part C)
#
# CHECKPOINT convention: comment out a block, re-run, watch the feature disappear.

import os
from platform import system

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
import launch_ros.parameter_descriptions
from launch_ros.actions import Node

if system() == 'Linux':
    # Helps RViz/Qt find its platform plugin on Ubuntu.
    os.environ['QT_QPA_PLATFORM_PLUGIN_PATH'] = (
        '/usr/lib/x86_64-linux-gnu/qt5/plugins/platforms/libqxcb.so'
    )
    os.environ['GTK_PATH'] = ''


def generate_launch_description():
    pkg = get_package_share_directory('stretch_se3_bringup')
    urdf = os.path.join(pkg, 'urdf', 'stretch_se3.urdf')
    rviz_cfg = os.path.join(pkg, 'rviz', 'architecture.rviz')

    ld = LaunchDescription()

    # --- launch arguments -------------------------------------------------------
    ld.add_action(DeclareLaunchArgument(
        'mode', default_value='position',
        choices=['position', 'navigation', 'trajectory', 'gamepad'],
        description='Driver control mode.'))
    ld.add_action(DeclareLaunchArgument(
        'use_rviz', default_value='true', choices=['true', 'false'],
        description='Launch RViz with the architecture config.'))
    ld.add_action(DeclareLaunchArgument(
        'use_mujoco_viewer', default_value='false', choices=['true', 'false'],
        description='Open the native MuJoCo viewer (needs a GPU/display; slow on CPU).'))
    ld.add_action(DeclareLaunchArgument(
        'use_cameras', default_value='false', choices=['true', 'false'],
        description='Publish RGB-D camera streams (slow without a GPU).'))
    ld.add_action(DeclareLaunchArgument(
        'broadcast_odom_tf', default_value='True', choices=['True', 'False'],
        description='Driver broadcasts the odom->base_link TF.'))

    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        Command(['xacro ', urdf]), value_type=str)

    # === CHECKPOINT: robot_model_tf ===
    # robot_state_publisher + joint_state_publisher turn /stretch/joint_states + the
    # URDF into the full link TF tree (and the visual model you see in RViz).
    # Comment this block out: RViz loses the robot model and most TF frames; only
    # the driver's odom/base frames remain.
    ld.add_action(Node(
        package='joint_state_publisher', executable='joint_state_publisher',
        output='log',
        parameters=[{'source_list': ['/stretch/joint_states'], 'rate': 30.0,
                     'robot_description': robot_description}],
        arguments=['--ros-args', '--log-level', 'error']))
    ld.add_action(Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': robot_description,
                     'publish_frequency': 30.0, 'use_sim_time': True}],
        arguments=['--ros-args', '--log-level', 'error']))
    # === END CHECKPOINT: robot_model_tf ===

    # === CHECKPOINT: simulator ===
    # The MuJoCo driver: spawns the physics sim, exposes /cmd_vel, /joint_states,
    # /scan_filtered, /odom, /mode, etc. use_robocasa is hard-false here so the plain
    # scene loads (no kitchen, no GPU needed).
    ld.add_action(Node(
        package='stretch_simulation', executable='stretch_mujoco_driver',
        emulate_tty=True, output='screen',
        remappings=[('cmd_vel', '/stretch/cmd_vel'),
                    ('joint_states', '/stretch/joint_states')],
        parameters=[{
            'rate': 30.0,
            'timeout': 0.5,
            'mode': LaunchConfiguration('mode'),
            'broadcast_odom_tf': LaunchConfiguration('broadcast_odom_tf'),
            'fail_out_of_range_goal': False,
            'use_mujoco_viewer': LaunchConfiguration('use_mujoco_viewer'),
            'use_cameras': LaunchConfiguration('use_cameras'),
            'use_robocasa': False,
        }]))
    # === END CHECKPOINT: simulator ===

    # === CHECKPOINT: rviz ===
    # Visualization only. Comment out for a pure headless run.
    ld.add_action(Node(
        package='rviz2', executable='rviz2', output='screen',
        arguments=['-d', rviz_cfg],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('use_rviz'))))
    # === END CHECKPOINT: rviz ===

    return ld
