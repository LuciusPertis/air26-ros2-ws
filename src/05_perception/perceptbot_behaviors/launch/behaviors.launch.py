"""Bring up the six switchable behaviours for the perception rover.

  behavior_manager  -> the dispatcher + B1/B2 (pub-sub), B4/B5 (vision scalars), B6 (search)
  obstacle_services -> /check_openings (srv) + /escape_obstacle (action)   [B3]
  marker_approach   -> /approach_marker (action)                           [B6]

Switch at runtime:
  ros2 service call /set_behavior perceptbot_interfaces/srv/SetBehavior "{behavior: 6}"
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    sim_time = {'use_sim_time': True}
    return LaunchDescription([
        Node(package='perceptbot_behaviors', executable='behavior_manager',
             parameters=[sim_time], output='screen'),
        Node(package='perceptbot_behaviors', executable='obstacle_services',
             parameters=[sim_time], output='screen'),
        Node(package='perceptbot_behaviors', executable='marker_approach',
             parameters=[sim_time], output='screen'),
    ])
