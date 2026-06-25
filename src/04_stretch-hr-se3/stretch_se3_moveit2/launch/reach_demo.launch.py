# AIR26 Workshop — CP-D3: run the programmatic reach demo.
#
# The reach_demo node uses MoveGroupInterface, which needs the robot_description,
# SRDF and kinematics parameters on its OWN node — so it must be launched (not
# `ros2 run`) with the MoveIt config attached.
#
#   ros2 launch stretch_se3_bringup sim.launch.py
#   ros2 launch stretch_se3_moveit2 move_group.launch.py
#   ros2 launch stretch_se3_moveit2 reach_demo.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder('stretch_se3', package_name='stretch_se3_moveit2')
        .robot_description(file_path='config/stretch_se3.urdf')
        .robot_description_semantic(file_path='config/stretch_se3.srdf')
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .joint_limits(file_path='config/joint_limits.yaml')
        .planning_pipelines(pipelines=['ompl'], default_planning_pipeline='ompl')
        .to_moveit_configs()
    )

    return LaunchDescription([
        Node(
            package='stretch_se3_moveit2', executable='reach_demo', output='screen',
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.joint_limits,
                moveit_config.planning_pipelines,
                {'use_sim_time': True},
            ]),
    ])
