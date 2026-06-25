# AIR26 Workshop — Part D: bring up MoveIt2 move_group + the trajectory bridge.
#
# Plans collision-free motions for the Stretch arm and executes them on the MuJoCo
# sim. Run the sim first (default position mode — the arm executes through the
# /stretch_controller trajectory action):
#
#   ros2 launch stretch_se3_bringup sim.launch.py
#   ros2 launch stretch_se3_moveit2 move_group.launch.py
#
# Then in RViz use the MotionPlanning panel (CP-D1/D2), or run the reach_demo node
# (CP-D3). The trajectory_bridge collapses joint_arm_l0..l3 -> wrist_extension so
# the sim driver can execute MoveIt's plans.

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
import os


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder('stretch_se3', package_name='stretch_se3_moveit2')
        .robot_description(file_path='config/stretch_se3.urdf')
        .robot_description_semantic(file_path='config/stretch_se3.srdf')
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .joint_limits(file_path='config/joint_limits.yaml')
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .planning_pipelines(pipelines=['ompl'], default_planning_pipeline='ompl')
        .to_moveit_configs()
    )

    use_sim_time = {'use_sim_time': True}
    pkg = get_package_share_directory('stretch_se3_moveit2')
    rviz_cfg = os.path.join(pkg, 'config', 'moveit.rviz')

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument(
        'use_rviz', default_value='true', choices=['true', 'false']))

    # === CHECKPOINT: move_group ===
    # The MoveIt2 brain: planning scene, OMPL planner, trajectory execution.
    ld.add_action(Node(
        package='moveit_ros_move_group', executable='move_group', output='screen',
        parameters=[moveit_config.to_dict(), use_sim_time]))
    # === END CHECKPOINT: move_group ===

    # === CHECKPOINT: trajectory_bridge ===
    # Translates MoveIt's per-segment arm trajectory into the driver's aggregate
    # wrist_extension. Comment out and plans are computed but never execute on the
    # sim (the driver rejects joint_arm_lN).
    ld.add_action(Node(
        package='stretch_se3_moveit2', executable='trajectory_bridge.py',
        output='screen', parameters=[use_sim_time]))
    # === END CHECKPOINT: trajectory_bridge ===

    # === CHECKPOINT: rviz ===
    ld.add_action(Node(
        package='rviz2', executable='rviz2', output='log',
        arguments=['-d', rviz_cfg],
        parameters=[moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    moveit_config.planning_pipelines,
                    moveit_config.joint_limits,
                    use_sim_time],
        condition=IfCondition(LaunchConfiguration('use_rviz'))))
    # === END CHECKPOINT: rviz ===

    return ld
