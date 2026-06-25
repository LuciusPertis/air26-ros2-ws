import sys
import time
import numbers
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import String
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from action_msgs.srv import CancelGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from unique_identifier_msgs.msg import UUID


class Client(Node):

    def __init__(self, name='test_node'):
        super().__init__(name)
        self.dryrun = False
        reentrant_cb = ReentrantCallbackGroup()

        # Setup FollowJointTrajectory action client
        self._fjt_client = ActionClient(self,
            FollowJointTrajectory,
            '/stretch_controller/follow_joint_trajectory',
            callback_group=reentrant_cb
        )
        server_reached = self._fjt_client.wait_for_server(timeout_sec=20.0)
        if not server_reached:
            self.get_logger().error('Unable to connect to Stretch action server. Timeout exceeded.')
            sys.exit()

        # Setup FJT cancel service
        self._fjt_cancel_service = self.create_client(
            CancelGoal, "/stretch_controller/follow_joint_trajectory/_action/cancel_goal"
        )

        # Setup subscriptions
        self.mode = None
        self._mode_subscriber = self.create_subscription(String, '/mode', self._mode_callback, 10, callback_group=reentrant_cb)
        self.q_curr = None
        self.q_full = None
        self._jointstate_subscriber = self.create_subscription(JointState, '/stretch/joint_states', self._joint_state_callback, 10, callback_group=reentrant_cb)

        # Setup background spinning
        self._spin_thread_shutdown_flag = threading.Event()
        self._spin_thread = threading.Thread(
            target=self._spinner,
            args=(self, SingleThreadedExecutor(),),
            daemon=True,
        )
        self._spin_thread.start()

        # Wait until mode is populated
        while self.mode is None or self.q_curr is None or self.q_full is None:
            time.sleep(0.1)

    def destroy_node(self):
        self._spin_thread_shutdown_flag.set()
        self._spin_thread.join()
        super().destroy_node()

    def _spinner(self, node, executor):
        while not self._spin_thread_shutdown_flag.is_set():
            rclpy.spin_once(node, executor=executor)

    def _mode_callback(self, mode_string):
        self.mode = mode_string.data

    def _joint_state_callback(self, joint_states):
        q_curr = {}
        q_full = {}
        for joint in joint_states.name:
            i = joint_states.name.index(joint)
            pos = joint_states.position[i]
            vel = joint_states.velocity[i]
            eff = joint_states.effort[i]
            q_curr[joint] = pos
            q_full[joint] = (pos, vel, eff)
        self.q_curr = q_curr
        self.q_full = q_full

    def move_to_configuration(self, q, blocking=True, custom_contact_thresholds=False, custom_full_goal=False):
        if self.dryrun:
            return

        if self.mode not in ['position', 'navigation']:
            self.get_logger().error("move_to_configuration() only works in position/position-like modes")
            return

        point = JointTrajectoryPoint()
        point.time_from_start = Duration(seconds=0).to_msg()
        fjt_goal = FollowJointTrajectory.Goal()
        fjt_goal.goal_time_tolerance = Duration(seconds=1.0).to_msg()
        fjt_goal.trajectory.joint_names = list(q.keys())
        fjt_goal.trajectory.points = [point]

        # construct goal
        if custom_full_goal:
            is_malformed_goal = not all([len(g) == 4 for g in q.values()])
            if is_malformed_goal:
                self.get_logger().error(f"move_to_configuration() received malformed goal. The 'custom_full_goal' option requires tuple with 4 values (position, velocity, acceleration, contact_threshold_effort) for each joint name, but q = {q}")
                return
            is_malformed_number = not all([isinstance(e, numbers.Real) for g in q.values() for e in g])
            if is_malformed_number:
                self.get_logger().error(f"move_to_configuration() received malformed goal. Each value must be a real number, but q = {q}")
                return
            point.positions = [g[0] for g in q.values()]
            point.velocities = [g[1] for g in q.values()]
            point.accelerations = [g[2] for g in q.values()]
            point.effort = [g[3] for g in q.values()]
        elif custom_contact_thresholds:
            is_malformed_goal = not all([len(g) == 2 for g in q.values()])
            if is_malformed_goal:
                self.get_logger().error(f"move_to_configuration() received malformed goal. The 'custom_contact_thresholds' option requires tuple with 2 values (position, contact_threshold_effort) for each joint name, but q = {q}")
                return
            is_malformed_number = not all([isinstance(e, numbers.Real) for g in q.values() for e in g])
            if is_malformed_number:
                self.get_logger().error(f"move_to_configuration() received malformed goal. Each value must be a real number, but q = {q}")
                return
            point.positions = [g[0] for g in q.values()]
            point.effort = [g[1] for g in q.values()]
        else:
            is_malformed_number = not all([isinstance(e, numbers.Real) for e in q.values()])
            if is_malformed_number:
                self.get_logger().error(f"move_to_configuration() received malformed goal. Each value must be a real number, but q = {q}")
                return
            point.positions = [e for e in q.values()]

        # send goal
        if blocking:
            return self._fjt_client.send_goal(fjt_goal)
        else:
            return self._fjt_client.send_goal_async(fjt_goal)

    def cancel_goal(self):
        cancel_msg = CancelGoal.Request()
        cancel_msg.goal_info.goal_id = UUID(
            uuid=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        )
        cancel_msg.goal_info.stamp.nanosec = 0
        cancel_msg.goal_info.stamp.sec = 0
        self._fjt_cancel_service.call(cancel_msg)
