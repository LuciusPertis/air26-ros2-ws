"""micro — LLM adapter for the project-02 micro_ros rover (MuJoCo).

The rover is a skid-steer base driven by ``/cmd_vel`` (geometry_msgs/Twist) with
three forward/left/right ultrasonic ``/ultrasonic/*`` ranges and ``/odom``. There
is no arm — movement is base-only, so the tools are:

  raw  : drive(linear, angular, duration)   — publish a Twist for a while
  disc : move_forward(distance)             — open-loop timed straight line
         turn(angle_deg)                    — open-loop timed in-place rotation
         stop()

"open-loop timed" = we convert distance/angle into a duration at a fixed nominal
speed and publish the Twist for that long. Good enough for a teaching demo; it is
the simplest thing that maps a discrete primitive onto /cmd_vel.
"""

import math
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range

from llm_integration.robot_base import RobotInterface, run_node

NOMINAL_LIN = 0.20    # m/s used to time move_forward
NOMINAL_ANG = 0.60    # rad/s used to time turn
MAX_DURATION = 15.0   # safety clamp on any single motion


class MicroInterface(RobotInterface):

    def __init__(self):
        super().__init__('llm_micro')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.ranges = {'front': None, 'left': None, 'right': None}
        self.pose = {'x': 0.0, 'y': 0.0, 'yaw_deg': 0.0}

        self.create_subscription(Odometry, '/odom', self._on_odom, 10,
                                 callback_group=self.cb_group)
        for side in self.ranges:
            self.create_subscription(
                Range, f'/ultrasonic/{side}',
                lambda m, s=side: self.ranges.__setitem__(s, round(m.range, 3)),
                10, callback_group=self.cb_group)

    # ---- state -------------------------------------------------------------
    def _on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = {'x': round(p.x, 3), 'y': round(p.y, 3),
                     'yaw_deg': round(math.degrees(yaw), 1)}

    def state_json(self):
        return {'pose': self.pose, 'ultrasonic_m': dict(self.ranges)}

    # ---- description -------------------------------------------------------
    def describe(self):
        return """\
Robot: a small four-wheel skid-steer rover ("microbot").
Body frame: x = forward, y = left, yaw = heading (degrees, + = left/CCW).
Sensors: three ultrasonic rangefinders pointing front, left and right; each
reports distance in metres to the nearest obstacle (about 2 m max).
Environment: a flat walled arena with a few box obstacles. No arm or gripper —
this robot can only drive.
"""

    # ---- tools -------------------------------------------------------------
    def tools(self):
        return [
            _tool('drive',
                  'Drive with a constant velocity for a fixed time, then stop.',
                  {'linear': _num('forward speed in m/s (+ forward, - back)'),
                   'angular': _num('turn rate in rad/s (+ left/CCW, - right/CW)'),
                   'duration': _num('how long to drive, in seconds')},
                  ['linear', 'angular', 'duration']),
            _tool('move_forward',
                  'Drive straight by a distance in metres (negative = backward).',
                  {'distance': _num('distance in metres')}, ['distance']),
            _tool('turn',
                  'Rotate in place by an angle in degrees (+ left/CCW, - right/CW).',
                  {'angle_deg': _num('angle in degrees')}, ['angle_deg']),
            _tool('stop', 'Stop the robot immediately.', {}, []),
        ]

    # ---- execution ---------------------------------------------------------
    def dispatch(self, name, args):
        if name == 'stop':
            self.cmd_pub.publish(Twist())
            return {'ok': True, 'action': 'stopped'}
        if name == 'drive':
            return self._drive(float(args['linear']), float(args['angular']),
                               float(args['duration']))
        if name == 'move_forward':
            dist = float(args['distance'])
            dur = min(abs(dist) / NOMINAL_LIN, MAX_DURATION)
            return self._drive(math.copysign(NOMINAL_LIN, dist), 0.0, dur,
                               note=f'moved {dist:+.2f} m')
        if name == 'turn':
            ang = math.radians(float(args['angle_deg']))
            dur = min(abs(ang) / NOMINAL_ANG, MAX_DURATION)
            return self._drive(0.0, math.copysign(NOMINAL_ANG, ang), dur,
                               note=f'turned {args["angle_deg"]:+.0f} deg')
        return {'ok': False, 'error': f'unknown tool {name}'}

    def _drive(self, linear, angular, duration, note=None, rate_hz=20.0):
        duration = max(0.0, min(duration, MAX_DURATION))
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        period = 1.0 / rate_hz
        end = time.time() + duration
        while time.time() < end:
            self.cmd_pub.publish(twist)
            time.sleep(period)
        self.cmd_pub.publish(Twist())   # stop
        return {'ok': True, 'action': note or f'drove {duration:.1f}s',
                'linear': linear, 'angular': angular,
                'front_range_m': self.ranges['front']}


def _tool(name, desc, props, required):
    return {'type': 'function',
            'function': {'name': name, 'description': desc,
                         'parameters': {'type': 'object', 'properties': props,
                                        'required': required}}}


def _num(desc):
    return {'type': 'number', 'description': desc}


def main():
    run_node(MicroInterface)
