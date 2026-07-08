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
        # 1. Forward Ultrasonic Sensor (us_front)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_us_front',
            arguments=[
                '--x', '0.09', '--y', '0.0', '--z', '0.05',
                '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                '--frame-id', 'base_link', 
                '--child-frame-id', 'us_front'
            ]
        ),

        # 2. Left Ultrasonic Sensor (us_left - pointing left: yaw = ~90 deg)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_us_left',
            arguments=[
                '--x', '0.0', '--y', '0.1', '--z', '0.05',
                '--roll', '0.0', '--pitch', '0.0', '--yaw', '1.5708',
                '--frame-id', 'base_link', 
                '--child-frame-id', 'us_left'
            ]
        ),

        # 3. Right Ultrasonic Sensor (us_right - pointing right: yaw = ~ -90 deg)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_us_right',
            arguments=[
                '--x', '0.0', '--y', '-0.1', '--z', '0.05',
                '--roll', '0.0', '--pitch', '0.0', '--yaw', '-1.5708',
                '--frame-id', 'base_link', 
                '--child-frame-id', 'us_right'
            ]
        ),
    ])
