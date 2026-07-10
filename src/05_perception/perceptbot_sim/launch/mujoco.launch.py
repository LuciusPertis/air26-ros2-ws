"""MuJoCo embodiment of the perception rover (reference sim; Webots is primary).

  mujoco_driver    -> /cmd_vel -> rover -> /ultrasonic/*, /odom, /camera/image_raw
  camera_processor -> /camera/light_level, /camera/mean_color
  aruco_detector   -> /aruco/detections, /aruco/image
  robot_state_publisher (+ optional RViz)

Behaviours run separately:  ros2 launch perceptbot_behaviors behaviors.launch.py
Headless: MUJOCO_GL=egl is set below so the offscreen camera renders without a display.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    desc = get_package_share_directory('perceptbot_description')
    xacro_file = os.path.join(desc, 'urdf', 'perceptbot.urdf.xacro')

    use_viewer = LaunchConfiguration('use_viewer')
    use_rviz = LaunchConfiguration('use_rviz')
    mujoco_gl = LaunchConfiguration('mujoco_gl')

    return LaunchDescription([
        DeclareLaunchArgument('use_viewer', default_value='false',
                              description='open the MuJoCo passive viewer'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('mujoco_gl', default_value='egl',
                              description='egl=headless, glfw=on a display'),
        SetEnvironmentVariable('MUJOCO_GL', mujoco_gl),

        Node(package='perceptbot_sim', executable='mujoco_driver', output='screen',
             parameters=[{'use_viewer': use_viewer}]),
        Node(package='perceptbot_perception', executable='camera_processor', output='screen'),
        Node(package='perceptbot_perception', executable='aruco_detector', output='screen'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': ParameterValue(
                 Command(['xacro ', xacro_file]), value_type=str)}]),
        Node(package='rviz2', executable='rviz2', condition=IfCondition(use_rviz)),
    ])
