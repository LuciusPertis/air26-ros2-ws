import time
import pytest
import rclpy

from common.launch_descriptions import stretch_driver_ld
from common.client_nodes.joint_pose_streaming import JointPosePublisher
import numpy as np
import time

@pytest.mark.launch(fixture=stretch_driver_ld)
def test_streaming_position_commands():
    joint_pose_publisher = JointPosePublisher()
    rclpy.spin_once(joint_pose_publisher)

    Idx = joint_pose_publisher.Idx
    
    joint_pose_publisher.activate_streaming_position()

    qpos = np.zeros(Idx.num_joints)
    qpos[Idx.LIFT] = 0.6
    qpos[Idx.ARM] = 0
    qpos[Idx.WRIST_PITCH] = 0
    qpos[Idx.WRIST_ROLL] = 0
    qpos[Idx.WRIST_YAW] = 0
    qpos[Idx.GRIPPER] = joint_pose_publisher.gripper_conversion.robotis_to_finger(0)
    qpos[Idx.BASE_TRANSLATE] = 0
    qpos[Idx.BASE_ROTATE] = 0
    qpos[Idx.HEAD_PAN] = 0
    qpos[Idx.HEAD_TILT] = 0
    joint_pose_publisher.publish_joint_pose(qpos)
    joint_pose_publisher.wait_until_at_setpoint(qpos)
    
    i = 0
    while i<100:
        i = 1 + i
        rclpy.spin_once(joint_pose_publisher)
        qpos = joint_pose_publisher.get_joint_status()
        qpos[Idx.LIFT] = qpos[Idx.LIFT] + 0.05
        qpos[Idx.ARM] = qpos[Idx.ARM] + 0.05
        qpos[Idx.WRIST_PITCH] = qpos[Idx.WRIST_PITCH] + 0.1
        qpos[Idx.WRIST_ROLL] = qpos[Idx.WRIST_ROLL] + 0.1
        qpos[Idx.WRIST_YAW] = qpos[Idx.WRIST_YAW] + 0.1
        qpos[Idx.HEAD_PAN] = qpos[Idx.HEAD_PAN] + 0.1
        qpos[Idx.HEAD_TILT] = qpos[Idx.HEAD_TILT] + 0.1
        qpos[Idx.GRIPPER] = qpos[Idx.GRIPPER] + 0.1
        qpos[Idx.BASE_TRANSLATE] = 0.01
        qpos[Idx.BASE_ROTATE] = 0.0
        joint_pose_publisher.publish_joint_pose(qpos)
        time.sleep(1/15)
    i = 0
    while i<100:
        i = 1 + i
        rclpy.spin_once(joint_pose_publisher)
        qpos = joint_pose_publisher.get_joint_status()
        qpos[Idx.LIFT] = qpos[Idx.LIFT] - 0.05
        qpos[Idx.ARM] = qpos[Idx.ARM] - 0.05
        qpos[Idx.WRIST_PITCH] = qpos[Idx.WRIST_PITCH] - 0.1
        qpos[Idx.WRIST_ROLL] = qpos[Idx.WRIST_ROLL] - 0.1
        qpos[Idx.WRIST_YAW] = qpos[Idx.WRIST_YAW] - 0.1
        qpos[Idx.HEAD_PAN] = qpos[Idx.HEAD_PAN] - 0.1
        qpos[Idx.HEAD_TILT] = qpos[Idx.HEAD_TILT] - 0.1
        qpos[Idx.GRIPPER] = qpos[Idx.GRIPPER] - 0.1
        qpos[Idx.BASE_ROTATE] = -0.05
        qpos[Idx.BASE_TRANSLATE] = 0.0
        joint_pose_publisher.publish_joint_pose(qpos)
        time.sleep(1/15)

    joint_pose_publisher.destroy_node()
    rclpy.shutdown()