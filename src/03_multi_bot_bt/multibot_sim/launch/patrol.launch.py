"""Bring up the 3-unit Webots BT patrol.

  Webots (patrol.wbt) + Ros2Supervisor
  per unit r1/r2/r3 (namespaced):
    WebotsController (multibot_driver)  -> /rN/cmd_vel, /rN/ultrasonic/*, /rN/camera/*
    aruco_pose_detector + relative_localizer  -> /rN/aruco/detections, TF, /rN/peers
    patrol_bt (py_trees) + formation_anchor   -> drives /rN/cmd_vel
    robot_state_publisher (frame_prefix rN/)

Choose the patrol style for everyone:
  ros2 launch multibot_sim patrol.launch.py formation:=convoy      # column, US+ArUco fused
  ros2 launch multibot_sim patrol.launch.py formation:=parallel    # abreast, vel-match + side-US
Switch one unit live:
  ros2 service call /r2/set_formation multibot_interfaces/srv/SetFormation "{formation: parallel}"
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController

UNITS = ['r1', 'r2', 'r3']
LEADER = 'r1'
GAP_SIDE = {'r2': 'left', 'r3': 'right'}     # which side US watches the inner neighbour


def generate_launch_description():
    sim_share = get_package_share_directory('multibot_sim')
    desc_share = get_package_share_directory('multibot_description')
    world = os.path.join(desc_share, 'worlds', 'patrol.wbt')
    webots_urdf = os.path.join(sim_share, 'urdf', 'multibot_webots.urdf')
    xacro_file = os.path.join(desc_share, 'urdf', 'multibot.urdf.xacro')
    sim_time = {'use_sim_time': True}
    formation = LaunchConfiguration('formation')
    enable_recovery = LaunchConfiguration('enable_recovery')
    debug_tree = LaunchConfiguration('debug_tree')

    webots = WebotsLauncher(world=world, ros2_supervisor=True)
    actions = [
        DeclareLaunchArgument('formation', default_value='convoy',
                              description='random | convoy | parallel'),
        DeclareLaunchArgument('enable_recovery', default_value='true',
                              description='false = no lost-unit search (pre-recovery patrol)'),
        DeclareLaunchArgument('debug_tree', default_value='false',
                              description='log each unit\'s live ASCII BT once a second'),
        webots, webots._supervisor,
        RegisterEventHandler(OnProcessExit(target_action=webots,
                                           on_exit=[EmitEvent(event=Shutdown())])),
    ]

    robot_desc = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    for ns in UNITS:
        actions.append(WebotsController(
            robot_name=ns,
            parameters=[{'robot_description': webots_urdf}, sim_time],
            respawn=True))
        actions.append(Node(
            package='multibot_perception', executable='aruco_pose_detector',
            namespace=ns, parameters=[sim_time], output='log'))
        actions.append(Node(
            package='multibot_perception', executable='relative_localizer',
            namespace=ns, parameters=[sim_time], output='log'))
        actions.append(Node(
            package='multibot_bt', executable='formation_anchor',
            namespace=ns, parameters=[sim_time], output='log'))
        actions.append(Node(
            package='multibot_bt', executable='patrol_bt', namespace=ns, output='screen',
            parameters=[sim_time, {'formation': formation, 'leader_ns': LEADER,
                                   'gap_side': GAP_SIDE.get(ns, 'right'),
                                   'enable_recovery': enable_recovery,
                                   'debug_tree': debug_tree}]))
        actions.append(Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            namespace=ns, parameters=[{'robot_description': robot_desc,
                                       'frame_prefix': ns + '/'}, sim_time], output='log'))

    return LaunchDescription(actions)
