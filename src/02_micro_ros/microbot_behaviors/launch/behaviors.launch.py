# AIR26 Workshop 02 — the obstacle-avoider behaviours.
# Run this alongside a sim (mujoco.launch.py or gazebo.launch.py).
#
#   ros2 launch microbot_sim mujoco.launch.py
#   ros2 launch microbot_behaviors behaviors.launch.py
#   ros2 service call /set_behavior microbot_interfaces/srv/SetBehavior "{behavior: 3}"

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # the brain: random walk + B1/B2 (pub-sub) + B3 client + /set_behavior
        Node(package='microbot_behaviors', executable='behavior_manager', output='screen'),
        # the B3 service + action server
        Node(package='microbot_behaviors', executable='obstacle_services', output='screen'),
    ])
