"""Shared control helper for the AIR26 Stretch SE3 workshop (Part B).

Every Part-B demo node talks to the MuJoCo driver through the SAME standard
interface the real Stretch uses: the ``FollowJointTrajectory`` action at
``/stretch_controller/follow_joint_trajectory``. You send a dict of
``{joint_name: target}`` and ``move()`` blocks until the driver reports the
joints have reached the setpoint.

This works while the driver is in its default **position mode** (the bringup
launch's ``mode:=position``). That single mode covers the lift, telescoping arm,
dex wrist, head, gripper, AND incremental base moves (``translate_mobile_base`` /
``rotate_mobile_base``) — so the Part-B nodes need no mode switching.

Continuous base *velocity* (``/stretch/cmd_vel``) is a different story: the driver
only accepts a Twist while in **navigation mode**. That path is taught with
``teleop_twist_keyboard`` in the tutorial, not here.

Valid joint names (mapped to actuators by the driver's command groups):
    joint_lift, joint_arm / wrist_extension, joint_wrist_yaw/pitch/roll,
    joint_head_pan, joint_head_tilt, gripper_aperture, translate_mobile_base,
    rotate_mobile_base
"""

import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

TRAJECTORY_ACTION = '/stretch_controller/follow_joint_trajectory'
CMD_VEL_TOPIC = '/stretch/cmd_vel'   # remapped from cmd_vel in sim.launch.py

# Soft limits from the Stretch SE3 MuJoCo model (metres for prismatic joints,
# radians for revolute). Targets are clamped to these so a demo can't fault the
# driver with an out-of-range goal. Source: stretch_mujoco actuators.py.
JOINT_LIMITS = {
    'joint_lift':           (0.0,   1.10),   # tower lift travel
    'wrist_extension':      (0.0,   0.52),   # telescoping arm (sum of 4 segments)
    'joint_arm':            (0.0,   0.52),   # alias for wrist_extension
    'joint_wrist_yaw':      (-1.39, 4.42),
    'joint_wrist_pitch':    (-1.57, 0.56),
    'joint_wrist_roll':     (-3.14, 3.14),
    'joint_head_pan':       (-4.04, 1.73),
    'joint_head_tilt':      (-1.53, 0.79),
    'gripper_aperture':     (-0.35, 0.16),   # negative = closed, positive = open
    'translate_mobile_base': (-1.0, 1.0),    # relative forward(+)/back(-) metres
    'rotate_mobile_base':   (-3.14, 3.14),   # relative CCW(+)/CW(-) radians
}


def clamp(joint, value):
    """Clamp a target to the joint's soft limits (pass-through if unknown)."""
    lo, hi = JOINT_LIMITS.get(joint, (float('-inf'), float('inf')))
    return max(lo, min(hi, value))


class StretchController(Node):
    """A Node with one blocking ``move()`` over the trajectory action.

    Subclass it for a demo, or instantiate it directly. Always call
    ``wait_for_server()`` once before the first ``move()``.
    """

    def __init__(self, name):
        super().__init__(name)
        self._client = ActionClient(self, FollowJointTrajectory, TRAJECTORY_ACTION)
        self._cmd_vel = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)

    def wait_for_server(self, timeout_sec=20.0):
        self.get_logger().info(f'Waiting for {TRAJECTORY_ACTION} ...')
        if not self._client.wait_for_server(timeout_sec=timeout_sec):
            raise RuntimeError(
                f'{TRAJECTORY_ACTION} not available after {timeout_sec:.0f}s — '
                'is the sim running? (ros2 launch stretch_se3_bringup sim.launch.py)')
        self.get_logger().info('Trajectory server is up.')

    def move(self, positions, seconds=3.0):
        """Send one trajectory point and block until the driver finishes it.

        positions: {joint_name: target}. Targets are clamped to JOINT_LIMITS.
        seconds:   advisory duration written into the point (the sim driver
                   moves at its own rate and waits for the setpoint).
        Returns True on success, False if the goal was rejected/aborted.
        """
        names = list(positions.keys())
        targets = [clamp(j, float(positions[j])) for j in names]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = names
        point = JointTrajectoryPoint()
        point.positions = targets
        point.time_from_start = Duration(sec=int(seconds),
                                         nanosec=int((seconds % 1) * 1e9))
        goal.trajectory.points = [point]

        pretty = ', '.join(f'{n}={v:.3f}' for n, v in zip(names, targets))
        self.get_logger().info(f'move -> {pretty}')

        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error('Goal rejected by the driver.')
            return False

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.get_logger().info('...reached.')
        return True

    # --- base velocity control (navigation mode) ----------------------------
    # The base is driven by *velocity* on /stretch/cmd_vel, and the driver only
    # accepts a Twist while in navigation mode (the trajectory action above does
    # not move the base in sim). So base demos must switch_mode('navigation')
    # first.

    def switch_mode(self, mode):
        """Call /switch_to_<mode>_mode (position | navigation | trajectory)."""
        srv = f'/switch_to_{mode}_mode'
        client = self.create_client(Trigger, srv)
        if not client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError(f'{srv} not available — is the sim running?')
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        self.get_logger().info(f'{srv}: {result.message}')
        return result.success

    def drive(self, linear, angular, duration_s, rate_hz=20.0):
        """Publish a constant Twist for duration_s, then stop.

        linear:  m/s forward(+)/back(-).   angular: rad/s CCW(+)/CW(-).
        Requires navigation mode (call switch_mode('navigation') first). The
        driver treats a Twist as stale after ~0.5 s, so we republish at rate_hz.
        """
        twist = Twist()
        twist.linear.x = float(linear)
        twist.angular.z = float(angular)
        period = 1.0 / rate_hz
        end = time.time() + duration_s
        while time.time() < end and rclpy.ok():
            self._cmd_vel.publish(twist)
            rclpy.spin_once(self, timeout_sec=period)
        self._cmd_vel.publish(Twist())   # stop
