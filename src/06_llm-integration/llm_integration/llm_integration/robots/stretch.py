"""stretch — LLM adapter for the project-04 Stretch SE3 rover (MuJoCo).

Stretch is a mobile manipulator: a differential base plus a vertical lift, a
telescoping arm, a pan/tilt head and a gripper. The driver exposes two very
different control surfaces, so the tools span both (the "hybrid" model):

  raw base velocity   : drive(linear, angular, duration)   via /stretch/cmd_vel
                        (needs *navigation* mode)
  discrete joint goals: set_lift / set_arm / set_head / set_gripper
                        via FollowJointTrajectory (needs *position* mode)

Mode matters: the base only listens to Twist in navigation mode, and the joint
trajectory action only moves joints in position mode. So each tool switches into
the mode it needs before acting. Joint targets are clamped to the model's soft
limits (mirrored from stretch_se3_control/stretch_trajectory.py) so the LLM can't
fault the driver with an out-of-range goal.
"""

import time

from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Twist
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from llm_integration.robot_base import RobotInterface, run_node, spin_wait

CMD_VEL_TOPIC = '/stretch/cmd_vel'
JOINT_STATES_TOPIC = '/stretch/joint_states'
TRAJECTORY_ACTION = '/stretch_controller/follow_joint_trajectory'
MAX_DURATION = 15.0

# Soft limits (metres / radians). Source: stretch_se3_control/stretch_trajectory.py.
JOINT_LIMITS = {
    'joint_lift':        (0.0,   1.10),
    'wrist_extension':   (0.0,   0.52),
    'joint_head_pan':    (-4.04, 1.73),
    'joint_head_tilt':   (-1.53, 0.79),
    'gripper_aperture':  (-0.35, 0.16),   # - = closed, + = open
}
# Joints we report back to the model as "state".
STATE_JOINTS = list(JOINT_LIMITS.keys())


def clamp(joint, value):
    lo, hi = JOINT_LIMITS.get(joint, (float('-inf'), float('inf')))
    return max(lo, min(hi, float(value)))


class StretchInterface(RobotInterface):

    def __init__(self):
        super().__init__('llm_stretch')
        self.cmd_pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)
        self.traj_client = ActionClient(self, FollowJointTrajectory,
                                        TRAJECTORY_ACTION,
                                        callback_group=self.cb_group)
        self._mode_clients = {}
        self.joints = {}     # name -> position
        self.create_subscription(JointState, JOINT_STATES_TOPIC,
                                 self._on_joints, 10,
                                 callback_group=self.cb_group)

    # ---- state -------------------------------------------------------------
    def _on_joints(self, msg):
        for name, pos in zip(msg.name, msg.position):
            self.joints[name] = round(pos, 3)

    def state_json(self):
        # Only report joints we actually have a reading for (the gripper joint is
        # published under a different name, so it stays absent rather than null).
        return {'joints': {j: self.joints[j] for j in STATE_JOINTS
                           if j in self.joints}}

    # ---- description -------------------------------------------------------
    def describe(self):
        return """\
Robot: a Hello Robot "Stretch" mobile manipulator.
Body frame: x = forward, yaw = heading (+ = left/CCW).
Degrees of freedom you control:
- a differential drive base (drive forward/back, turn in place);
- a vertical lift, 0.0 m (lowest) to 1.10 m (highest);
- a telescoping arm extension, 0.0 m (retracted) to 0.52 m (fully extended);
- a head that pans and tilts (radians; pan + = left, tilt + = up, - = down);
- a gripper that opens or closes.
Environment: a flat indoor scene with the robot standing free. Move gently.
"""

    # ---- tools -------------------------------------------------------------
    def tools(self):
        return [
            _tool('drive',
                  'Drive the base with a constant velocity for a fixed time.',
                  {'linear': _num('forward m/s (+ forward, - back)'),
                   'angular': _num('turn rad/s (+ left/CCW, - right/CW)'),
                   'duration': _num('seconds')},
                  ['linear', 'angular', 'duration']),
            _tool('set_lift',
                  'Move the lift to an absolute height in metres (0.0 to 1.10).',
                  {'height': _num('target height in metres')}, ['height']),
            _tool('set_arm',
                  'Extend/retract the arm to an absolute length in metres '
                  '(0.0 retracted to 0.52 extended).',
                  {'extension': _num('target extension in metres')}, ['extension']),
            _tool('set_head',
                  'Point the head to an absolute pan and tilt, in radians.',
                  {'pan': _num('pan radians (+ left, - right)'),
                   'tilt': _num('tilt radians (+ up, - down)')}, ['pan', 'tilt']),
            _tool('set_gripper',
                  'Open or close the gripper.',
                  {'state': {'type': 'string', 'enum': ['open', 'close'],
                             'description': 'open or close'}}, ['state']),
            _tool('stop', 'Stop the base immediately.', {}, []),
        ]

    # ---- execution ---------------------------------------------------------
    def dispatch(self, name, args):
        if name == 'stop':
            self.cmd_pub.publish(Twist())
            return {'ok': True, 'action': 'stopped'}
        if name == 'drive':
            return self._drive(float(args['linear']), float(args['angular']),
                               float(args['duration']))
        if name == 'set_lift':
            return self._joint_goal({'joint_lift': args['height']})
        if name == 'set_arm':
            return self._joint_goal({'wrist_extension': args['extension']})
        if name == 'set_head':
            return self._joint_goal({'joint_head_pan': args['pan'],
                                     'joint_head_tilt': args['tilt']})
        if name == 'set_gripper':
            target = 0.15 if str(args['state']).lower() == 'open' else -0.34
            return self._joint_goal({'gripper_aperture': target},
                                    note=f"gripper {args['state']}")
        return {'ok': False, 'error': f'unknown tool {name}'}

    # ---- base velocity (navigation mode) -----------------------------------
    def _drive(self, linear, angular, duration, rate_hz=20.0):
        if not self._switch_mode('navigation'):
            return {'ok': False, 'error': 'could not enter navigation mode'}
        duration = max(0.0, min(duration, MAX_DURATION))
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        period = 1.0 / rate_hz
        end = time.time() + duration
        while time.time() < end:
            self.cmd_pub.publish(twist)
            time.sleep(period)
        self.cmd_pub.publish(Twist())
        return {'ok': True, 'action': f'drove {duration:.1f}s',
                'linear': linear, 'angular': angular}

    # ---- joints (position mode) --------------------------------------------
    def _joint_goal(self, targets, seconds=3.0, note=None):
        if not self._switch_mode('position'):
            return {'ok': False, 'error': 'could not enter position mode'}
        if not self.traj_client.wait_for_server(timeout_sec=15.0):
            return {'ok': False, 'error': f'{TRAJECTORY_ACTION} unavailable'}

        names = list(targets)
        clamped = {n: clamp(n, targets[n]) for n in names}
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory(joint_names=names)
        point = JointTrajectoryPoint()
        point.positions = [clamped[n] for n in names]
        point.time_from_start = Duration(sec=int(seconds),
                                         nanosec=int((seconds % 1) * 1e9))
        goal.trajectory.points = [point]

        send = self.traj_client.send_goal_async(goal)
        handle = spin_wait(send, timeout=20.0)
        if handle is None or not handle.accepted:
            return {'ok': False, 'error': 'goal rejected'}
        spin_wait(handle.get_result_async(), timeout=20.0)
        return {'ok': True, 'action': note or 'joint goal reached',
                'targets': clamped}

    # ---- mode switching ----------------------------------------------------
    def _switch_mode(self, mode):
        srv = f'/switch_to_{mode}_mode'
        client = self._mode_clients.get(srv)
        if client is None:
            client = self.create_client(Trigger, srv,
                                        callback_group=self.cb_group)
            self._mode_clients[srv] = client
        if not client.wait_for_service(timeout_sec=10.0):
            self.get_logger().warn(f'{srv} not available')
            return False
        result = spin_wait(client.call_async(Trigger.Request()), timeout=10.0)
        return bool(result and result.success)


def _tool(name, desc, props, required):
    return {'type': 'function',
            'function': {'name': name, 'description': desc,
                         'parameters': {'type': 'object', 'properties': props,
                                        'required': required}}}


def _num(desc):
    return {'type': 'number', 'description': desc}


def main():
    run_node(StretchInterface)
