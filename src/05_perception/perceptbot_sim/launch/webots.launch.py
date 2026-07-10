"""Bring up the Webots perception-rover sim + the driver + the perception nodes.

  webots (R2025a) world + Ros2Supervisor (/clock)
  WebotsController -> loads perceptbot_webots.urdf (sensor/camera device tags + /cmd_vel driver)
  camera_processor -> /camera/light_level, /camera/mean_color
  aruco_detector   -> /aruco/detections, /aruco/image
  robot_state_publisher (for RViz/TF), optional rviz2

Run the behaviours separately:  ros2 launch perceptbot_behaviors behaviors.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, EmitEvent
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    sim_share = get_package_share_directory('perceptbot_sim')
    desc_share = get_package_share_directory('perceptbot_description')
    world = os.path.join(sim_share, 'worlds', 'perceptbot.wbt')
    webots_urdf = os.path.join(sim_share, 'resource', 'perceptbot_webots.urdf')
    xacro_file = os.path.join(desc_share, 'urdf', 'perceptbot.urdf.xacro')

    use_rviz = LaunchConfiguration('use_rviz')
    sim_time = {'use_sim_time': True}

    webots = WebotsLauncher(world=world, ros2_supervisor=True)

    driver = WebotsController(
        robot_name='perceptbot',
        parameters=[{'robot_description': webots_urdf}, sim_time],
        # The Camera plugin appends /image_color to the device's topicName, so the `camera`
        # device publishes /camera/image_color. Every other embodiment (MuJoCo, Gazebo, the
        # real MJPEG bridge) publishes /camera/image_raw — remap so the perception nodes,
        # the behaviours and the RViz Image display see one topic name everywhere.
        remappings=[('/camera/image_color', '/camera/image_raw')],
        respawn=True,
    )

    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': ParameterValue(
            Command(['xacro ', xacro_file]), value_type=str)}, sim_time],
    )

    camera_processor = Node(
        package='perceptbot_perception', executable='camera_processor',
        parameters=[sim_time], output='screen',
    )
    aruco_detector = Node(
        package='perceptbot_perception', executable='aruco_detector',
        parameters=[sim_time], output='screen',
    )

    rviz = Node(
        package='rviz2', executable='rviz2', condition=IfCondition(use_rviz),
        parameters=[sim_time],
    )

    # when Webots quits, shut the whole launch down
    on_webots_exit = RegisterEventHandler(
        OnProcessExit(target_action=webots, on_exit=[EmitEvent(event=Shutdown())]))

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='false',
                              description='also open RViz2'),
        webots,
        webots._supervisor,
        driver,
        robot_state_publisher,
        camera_processor,
        aruco_detector,
        rviz,
        on_webots_exit,
    ])
