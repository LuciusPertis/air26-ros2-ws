import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from hello_helpers.joint_qpos_conversion import get_Idx, JointStateMapping
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from hello_helpers.gripper_conversion import GripperConversion
from rclpy.callback_groups import ReentrantCallbackGroup
import numpy as np
import time



class JointPosePublisher(Node):
    def __init__(self):
        rclpy.init()
        super().__init__('float_array_publisher')
        self.reentrant_cb = ReentrantCallbackGroup()
        self.publisher_ = self.create_publisher(Float64MultiArray, 'joint_pose_cmd', 10,callback_group=self.reentrant_cb)
        # subscribe to joint states
        self.joint_state = JointState()
        self.joint_states_subscriber = self.create_subscription(JointState, 
                                                                '/stretch/joint_states', 
                                                                self.joint_states_callback, 10,callback_group=self.reentrant_cb)
        self.Idx = get_Idx('eoa_wrist_dw3_tool_sg3')
        self.switch_to_position_mode_service = self.create_client(Trigger, '/switch_to_position_mode',callback_group=self.reentrant_cb)

        self.switch_to_navigation_mode_service = self.create_client(Trigger, '/switch_to_navigation_mode',callback_group=self.reentrant_cb)
        
        self.activate_streaming_position_service = self.create_client(Trigger, '/activate_streaming_position', callback_group=self.reentrant_cb)
        self.deactivate_streaming_position_service = self.create_client(Trigger, '/deactivate_streaming_position', callback_group=self.reentrant_cb)

        while not self.switch_to_position_mode_service.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("Waiting on '/switch_to_position_mode' service...")
        self.gripper_conversion = GripperConversion()
    
    def joint_states_callback(self, msg):
        self.joint_state = msg

    def switch_to_position_mode(self):
        trigger_request = Trigger.Request()
        trigger_result = self.switch_to_position_mode_service.call_async(trigger_request)
        return

    def activate_streaming_position(self):
        trigger_request = Trigger.Request()
        trigger_result = self.activate_streaming_position_service.call_async(trigger_request)
        time.sleep(1)

    def deactivate_streaming_position(self):
        trigger_request = Trigger.Request()
        trigger_result = self.deactivate_streaming_position_service.call_async(trigger_request)
        return


    def switch_to_navigation_mode(self):
        trigger_request = Trigger.Request()
        trigger_result = self.switch_to_navigation_mode_service.call_async(trigger_request)
        return
    
    def get_joint_status(self):
        j_status =  self.parse_joint_state(self.joint_state)
        pose = np.zeros(self.Idx.num_joints)
        pose[self.Idx.LIFT] = j_status[JointStateMapping.ROS_LIFT_JOINT]
        pose[self.Idx.ARM] = sum(j_status[joint] for joint in JointStateMapping.ROS_ARM_JOINTS)
        pose[self.Idx.GRIPPER] = j_status[JointStateMapping.ROS_GRIPPER_FINGER]
        pose[self.Idx.WRIST_ROLL] = j_status[JointStateMapping.ROS_WRIST_ROLL]
        pose[self.Idx.WRIST_PITCH] = j_status[JointStateMapping.ROS_WRIST_PITCH]
        pose[self.Idx.WRIST_YAW] = j_status[JointStateMapping.ROS_WRIST_YAW]
        pose[self.Idx.HEAD_PAN] = j_status[JointStateMapping.ROS_HEAD_PAN]
        pose[self.Idx.HEAD_TILT] = j_status[JointStateMapping.ROS_HEAD_TILT]
        return pose
        
    def parse_joint_state(self, joint_state_msg):
        joint_status = {}
        for name, position in zip(joint_state_msg.name, joint_state_msg.position):
            joint_status[name] = position
        return joint_status

    def publish_joint_pose(self, joint_pose):
        msg = Float64MultiArray()
        msg.data = list(joint_pose)
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
    
    def wait_until_at_setpoint(self, goal_qpos):
        while abs(goal_qpos[:-2] - joint_pose_publisher.get_joint_status()[:-2]).mean() > 0.01:
            rclpy.spin_once(self)
            time.sleep(0.01)

if __name__ == '__main__':
    joint_pose_publisher = JointPosePublisher()
    rclpy.spin_once(joint_pose_publisher)

    Idx = get_Idx('eoa_wrist_dw3_tool_sg3')
    
    # joint_pose_publisher.switch_to_navigation_mode()
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