"""live.launch.py — the REAL-hardware, many-viewers live demo.

Brings up the PC-side ROS graph that turns the real rover + ESP32-CAM into something
every attendee can watch in RViz over the LAN:

  robot_state_publisher  -> URDF -> base_link->wheels/camera fixed TF (for the RViz RobotModel)
  cmd_vel_odometry       -> integrates /cmd_vel -> /odom + TF odom->base_link + /joint_states
  mjpeg_bridge           -> pulls http://<cam-ip>/stream -> /camera/image_raw
  camera_processor       -> /camera/mean_intensity, /camera/mean_color
  aruco_detector         -> /aruco/detections, /aruco/image
  rviz2 (optional)       -> preloaded view: RobotModel + TF + Camera image

What this launch does NOT start (run these yourself — see SETUP.md / NETWORKING.md):
  * the micro-ROS agent for the boards      (set agent:=true to include it here, or run
      `ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888` in its own terminal)
  * teleop to actually drive the rover       (`ros2 run teleop_twist_keyboard teleop_twist_keyboard`)

Run ONE instance of this on the "host" laptop. Every attendee just runs their own
`rviz2` (same ROS_DOMAIN_ID, same LAN) — do NOT run mjpeg_bridge per-attendee, the
ESP32-CAM only serves one HTTP client. See NETWORKING.md.

Examples:
  ros2 launch perceptlive_perception live.launch.py stream_url:=http://10.42.0.51/stream
  ros2 launch perceptlive_perception live.launch.py stream_url:=http://10.42.0.51/stream rviz:=true agent:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    desc = get_package_share_directory('perceptlive_description')
    perc = get_package_share_directory('perceptlive_perception')
    xacro_file = os.path.join(desc, 'urdf', 'perceptbot.urdf.xacro')
    rviz_cfg = os.path.join(perc, 'rviz', 'live.rviz')

    stream_url = LaunchConfiguration('stream_url')
    rviz = LaunchConfiguration('rviz')
    agent = LaunchConfiguration('agent')
    agent_port = LaunchConfiguration('agent_port')

    return LaunchDescription([
        DeclareLaunchArgument('stream_url', default_value='http://192.168.4.1/stream',
                              description='ESP32-CAM MJPEG stream URL (http://<cam-ip>/stream)'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='launch RViz with the preloaded live view'),
        DeclareLaunchArgument('agent', default_value='false',
                              description='also launch the micro-ROS agent (udp4) for the boards'),
        DeclareLaunchArgument('agent_port', default_value='8888',
                              description='micro-ROS agent UDP port'),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             output='screen',
             parameters=[{'robot_description': ParameterValue(
                 Command(['xacro ', xacro_file]), value_type=str)}]),

        Node(package='perceptlive_perception', executable='cmd_vel_odometry',
             output='screen'),

        Node(package='perceptlive_perception', executable='mjpeg_bridge', output='screen',
             parameters=[{'stream_url': stream_url}]),
        Node(package='perceptlive_perception', executable='camera_processor', output='screen'),
        Node(package='perceptlive_perception', executable='aruco_detector', output='screen'),

        Node(package='rviz2', executable='rviz2', output='screen',
             condition=IfCondition(rviz),
             arguments=['-d', rviz_cfg]),

        Node(package='micro_ros_agent', executable='micro_ros_agent', output='screen',
             condition=IfCondition(agent),
             arguments=['udp4', '--port', agent_port]),
    ])
