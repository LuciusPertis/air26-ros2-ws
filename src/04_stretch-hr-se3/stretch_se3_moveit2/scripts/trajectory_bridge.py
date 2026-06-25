#!/usr/bin/env python3
"""MoveIt2 <-> MuJoCo-driver trajectory bridge for the AIR26 workshop.

WHY THIS EXISTS
---------------
MoveIt plans in terms of the URDF's joints, so its executed trajectory names the
four telescoping-arm segments individually: joint_arm_l0, l1, l2, l3. But the
MuJoCo driver's FollowJointTrajectory server (/stretch_controller/...) only
understands the *aggregate* arm joint `wrist_extension` (it raises
NotImplementedError on joint_arm_lN). On the real Stretch, stretch_core does this
collapsing for you; the simplified sim driver does not.

So this node is a shim controller:
  * It SERVES   /stretch_arm_controller/follow_joint_trajectory  (MoveIt talks here)
  * It CALLS    /stretch_controller/follow_joint_trajectory       (the driver)
  * For every trajectory point it replaces joint_arm_l0..l3 with a single
    `wrist_extension` = l0+l1+l2+l3 and passes lift / wrist / gripper / head
    joints through unchanged.

This is the one custom integration piece Part D needs in sim; the rest of the
MoveIt config is standard.
"""

import threading

import rclpy
from rclpy.action import ActionClient, ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ARM_SEGMENTS = ['joint_arm_l0', 'joint_arm_l1', 'joint_arm_l2', 'joint_arm_l3']
DRIVER_ACTION = '/stretch_controller/follow_joint_trajectory'
BRIDGE_ACTION = '/stretch_arm_controller/follow_joint_trajectory'


def translate(trajectory: JointTrajectory) -> JointTrajectory:
    """Rewrite joint_arm_l0..l3 -> single wrist_extension (sum); pass rest through."""
    names = list(trajectory.joint_names)
    seg_idx = [names.index(s) for s in ARM_SEGMENTS if s in names]
    keep_idx = [i for i in range(len(names)) if i not in seg_idx]

    out = JointTrajectory()
    out.joint_names = [names[i] for i in keep_idx]
    if seg_idx:
        out.joint_names.append('wrist_extension')

    for pt in trajectory.points:
        new_pt = JointTrajectoryPoint()
        new_pt.positions = [pt.positions[i] for i in keep_idx]
        if seg_idx:
            new_pt.positions.append(sum(pt.positions[i] for i in seg_idx))
        # velocities/accelerations are intentionally dropped: the sim driver moves
        # each actuator to the position setpoint and waits, ignoring them.
        new_pt.time_from_start = pt.time_from_start
        out.points.append(new_pt)
    return out


class TrajectoryBridge(Node):
    def __init__(self):
        super().__init__('stretch_arm_controller')
        cb = ReentrantCallbackGroup()
        self._client = ActionClient(self, FollowJointTrajectory, DRIVER_ACTION,
                                    callback_group=cb)
        self._server = ActionServer(
            self, FollowJointTrajectory, BRIDGE_ACTION,
            execute_callback=self.execute_callback, callback_group=cb)
        self.get_logger().info(f'Bridge ready: {BRIDGE_ACTION} -> {DRIVER_ACTION}')

    def execute_callback(self, goal_handle):
        req = goal_handle.request
        n_in = len(req.trajectory.joint_names)
        forward = FollowJointTrajectory.Goal()
        forward.trajectory = translate(req.trajectory)
        # The sim driver moves to each point and *waits* for the setpoint, so
        # streaming all ~50 OMPL waypoints is painfully slow on a CPU. MoveIt has
        # already validated the whole path collision-free and the workshop scene is
        # obstacle-free, so we execute just the final goal point — the arm drives
        # smoothly to the planned target. (Drop this line to follow every waypoint.)
        if len(forward.trajectory.points) > 1:
            forward.trajectory.points = [forward.trajectory.points[-1]]
        self.get_logger().info(
            f'Forwarding goal point, '
            f'{n_in} joints -> {forward.trajectory.joint_names}')

        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(f'{DRIVER_ACTION} unavailable — is the sim up?')
            goal_handle.abort()
            return FollowJointTrajectory.Result()

        done = threading.Event()
        holder = {}

        def on_result(fut):
            holder['result'] = fut.result().result
            done.set()

        def on_goal(fut):
            gh = fut.result()
            if not gh.accepted:
                done.set()
                return
            gh.get_result_async().add_done_callback(on_result)

        self._client.send_goal_async(forward).add_done_callback(on_goal)
        done.wait(timeout=180.0)

        result = holder.get('result')
        if result is None:
            self.get_logger().error('Driver goal rejected or timed out.')
            goal_handle.abort()
            return FollowJointTrajectory.Result()

        goal_handle.succeed()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryBridge()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
