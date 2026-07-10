"""Run the perception stack against the REAL ESP32-CAM (no simulator).

  mjpeg_bridge     -> pulls http://<board-ip>/stream into /camera/image_raw
  camera_processor -> /camera/light_level, /camera/mean_color   (PC-side copy; the board
                      also publishes these itself over micro-ROS — either works)
  aruco_detector   -> /aruco/detections, /aruco/image

Usage:
  ros2 launch perceptbot_perception real_camera.launch.py stream_url:=http://10.185.122.251/stream
Then the behaviours run unchanged (e.g. behaviour 6 for ArUco approach).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    stream_url = LaunchConfiguration('stream_url')
    return LaunchDescription([
        DeclareLaunchArgument('stream_url', default_value='http://192.168.4.1/stream',
                              description='ESP32-CAM MJPEG stream URL'),
        Node(package='perceptbot_perception', executable='mjpeg_bridge', output='screen',
             parameters=[{'stream_url': stream_url}]),
        Node(package='perceptbot_perception', executable='camera_processor', output='screen'),
        Node(package='perceptbot_perception', executable='aruco_detector', output='screen'),
    ])
