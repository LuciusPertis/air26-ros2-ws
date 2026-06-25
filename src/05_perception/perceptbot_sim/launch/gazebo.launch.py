"""Gazebo (Ignition Fortress) embodiment of the perception rover (reference sim).

  gz sim (perceptbot.sdf)  -> DiffDrive + 3 gpu_lidars + camera
  ros_gz_bridge            -> /cmd_vel, /scan_*, /camera/*, /odom, /clock
  scan_to_range            -> /scan_* (LaserScan) -> /ultrasonic/* (Range)
  camera_processor         -> /camera/mean_intensity, /camera/mean_color
  aruco_detector           -> /aruco/* (B6 ArUco is Webots/real-cam; empty here)
  robot_state_publisher

Behaviours 1-5 run on top:  ros2 launch perceptbot_behaviors behaviors.launch.py
Headless:  ros2 launch perceptbot_sim gazebo.launch.py gz_args:='-r -s'
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    sim_share = get_package_share_directory('perceptbot_sim')
    desc_share = get_package_share_directory('perceptbot_description')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    world = os.path.join(sim_share, 'worlds', 'perceptbot.sdf')
    bridge_yaml = os.path.join(sim_share, 'config', 'gz_bridge.yaml')
    xacro_file = os.path.join(desc_share, 'urdf', 'perceptbot.urdf.xacro')
    sim_time = {'use_sim_time': True}

    gz_args = LaunchConfiguration('gz_args')

    return LaunchDescription([
        DeclareLaunchArgument('gz_args', default_value=f'-r {world}',
                              description="Gazebo args (add -s for headless)."),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': gz_args}.items()),

        Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
             parameters=[{'config_file': bridge_yaml}, sim_time]),

        Node(package='perceptbot_sim', executable='scan_to_range', output='screen',
             parameters=[sim_time]),
        Node(package='perceptbot_perception', executable='camera_processor', output='screen',
             parameters=[sim_time]),
        Node(package='perceptbot_perception', executable='aruco_detector', output='screen',
             parameters=[sim_time]),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': ParameterValue(
                 Command(['xacro ', xacro_file]), value_type=str)}, sim_time]),
    ])
