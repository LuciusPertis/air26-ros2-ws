# AIR26 Workshop — Part C, CP-C2/C3/C4: localization + autonomous navigation.
#
# Loads a saved map, localizes with AMCL, and brings up the full Nav2 stack
# (planner, controller, behaviour tree, recoveries). Run alongside the sim in
# NAVIGATION mode, set the initial pose in RViz, then send goals (RViz "Nav2
# Goal" button, or the waypoint_demo node for CP-C3).
#
#   # terminal 1 — sim in navigation mode
#   ros2 launch stretch_se3_bringup sim.launch.py mode:=navigation
#   # terminal 2 — this launch (point map:= at the file you saved in CP-C1)
#   ros2 launch stretch_se3_nav navigation.launch.py map:=<abs path>/workshop_map.yaml
#   # in RViz: "2D Pose Estimate" to seed AMCL, then "Nav2 Goal" to drive
#   # or, programmatically (CP-C3):
#   ros2 run stretch_se3_nav waypoint_demo
#
# CHECKPOINT convention: comment a block out, re-run, watch the feature vanish.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('stretch_se3_nav')
    nav2_share = get_package_share_directory('nav2_bringup')

    nav2_params = os.path.join(pkg, 'config', 'nav2_params.yaml')
    default_map = os.path.join(pkg, 'maps', 'workshop_map.yaml')
    rviz_cfg = os.path.join(nav2_share, 'rviz', 'nav2_default_view.rviz')

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument(
        'map', default_value=default_map,
        description='Absolute path to the map .yaml saved in CP-C1.'))
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time', default_value='true', choices=['true', 'false'],
        description='Use the sim /clock (keep true with the MuJoCo sim).'))
    ld.add_action(DeclareLaunchArgument(
        'params_file', default_value=nav2_params,
        description='Nav2 parameters (tuned for the sim: base_link, /scan_filtered).'))
    ld.add_action(DeclareLaunchArgument(
        'autostart', default_value='true', choices=['true', 'false'],
        description='Auto-activate the Nav2 lifecycle nodes.'))
    ld.add_action(DeclareLaunchArgument(
        'use_rviz', default_value='true', choices=['true', 'false']))

    # === CHECKPOINT: nav2_stack ===
    # The full Nav2 bringup: map_server + amcl (localization) + planner +
    # controller + bt_navigator + behaviours. slam=False -> localize on the saved
    # map (CP-C2). Comment out and nothing plans or drives.
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': LaunchConfiguration('params_file'),
            'autostart': LaunchConfiguration('autostart'),
            'slam': 'False',
            'use_composition': 'True',
        }.items()))
    # === END CHECKPOINT: nav2_stack ===

    # === CHECKPOINT: cmd_vel_relay ===
    # Bridges Nav2's /cmd_vel output -> /stretch/cmd_vel (the driver). Comment out
    # and the robot plans but never moves.
    ld.add_action(Node(
        package='stretch_se3_nav', executable='cmd_vel_relay', output='screen'))
    # === END CHECKPOINT: cmd_vel_relay ===

    # === CHECKPOINT: rviz ===
    ld.add_action(Node(
        package='rviz2', executable='rviz2', output='screen',
        arguments=['-d', rviz_cfg],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('use_rviz'))))
    # === END CHECKPOINT: rviz ===

    return ld
