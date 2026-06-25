# AIR26 Workshop — Part C, CP-C1: SLAM mapping with slam_toolbox.
#
# Build a map by driving the robot around while slam_toolbox fuses the lidar
# (/scan_filtered) into an occupancy grid. Run alongside the sim in NAVIGATION
# mode, then drive with teleop (or the Part-B square_drive) and save the map.
#
#   # terminal 1 — sim in navigation mode (so /cmd_vel drives the base)
#   ros2 launch stretch_se3_bringup sim.launch.py mode:=navigation
#   # terminal 2 — this launch
#   ros2 launch stretch_se3_nav mapping.launch.py
#   # terminal 3 — drive around to build the map
#   ros2 run teleop_twist_keyboard teleop_twist_keyboard
#   # when the map looks complete, save it (into this package's maps/ dir):
#   ros2 run nav2_map_server map_saver_cli -f ~/air26-ros2-ws/src/04_stretch-hr-se3/stretch_se3_nav/maps/workshop_map
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
    slam_share = get_package_share_directory('slam_toolbox')
    nav2_share = get_package_share_directory('nav2_bringup')

    mapper_params = os.path.join(pkg, 'config', 'mapper_params.yaml')
    rviz_cfg = os.path.join(nav2_share, 'rviz', 'nav2_default_view.rviz')

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time', default_value='true', choices=['true', 'false'],
        description='Use the sim /clock (keep true with the MuJoCo sim).'))
    ld.add_action(DeclareLaunchArgument(
        'use_rviz', default_value='true', choices=['true', 'false']))

    # === CHECKPOINT: slam ===
    # slam_toolbox in async mapping mode. Comment out and there is no map — the
    # other nodes still run but RViz shows only the live lidar scan.
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_share, 'launch', 'online_async_launch.py')),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'slam_params_file': mapper_params,
        }.items()))
    # === END CHECKPOINT: slam ===

    # === CHECKPOINT: cmd_vel_relay ===
    # Bridges /cmd_vel (teleop) -> /stretch/cmd_vel (the driver). Comment out and
    # teleop keypresses do nothing.
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
