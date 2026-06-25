"""perceptbot_driver — the Webots <extern> controller plugin for the rover's wheels.

webots_ros2 auto-publishes the distance sensors and camera (declared as <device> tags in
the URDF), so this plugin only needs to do the one thing Webots has no built-in for:
turn a /cmd_vel Twist into four skid-steer wheel velocities. Same /cmd_vel contract as
the project-02 MuJoCo driver and the real ESP32 — so the behaviours don't change.

Loaded via the URDF:  <plugin type="perceptbot_sim.perceptbot_driver.PerceptbotDriver" />
"""

import rclpy
from geometry_msgs.msg import Twist

WHEEL_RADIUS = 0.05      # m  (matches the xacro / .wbt wheels)
WHEEL_SEP = 0.32         # m  left-right track (2 * wheel_dy)
MAX_W = 12.0             # rad/s clamp per wheel
WHEELS = ['wheel_fl_motor', 'wheel_fr_motor', 'wheel_rl_motor', 'wheel_rr_motor']


class PerceptbotDriver:
    def init(self, webots_node, properties):
        self.robot = webots_node.robot

        # velocity-control mode: position = +inf, then command velocity
        self.motors = {}
        for name in WHEELS:
            m = self.robot.getDevice(name)
            m.setPosition(float('inf'))
            m.setVelocity(0.0)
            self.motors[name] = m

        self.target = (0.0, 0.0)   # (v, w)
        if not rclpy.ok():
            rclpy.init(args=None)
        self.node = rclpy.create_node('perceptbot_driver')
        self.node.create_subscription(Twist, '/cmd_vel', self._on_cmd, 1)

    def _on_cmd(self, msg):
        self.target = (msg.linear.x, msg.angular.z)

    def _set(self, left, right):
        left = max(-MAX_W, min(MAX_W, left))
        right = max(-MAX_W, min(MAX_W, right))
        self.motors['wheel_fl_motor'].setVelocity(left)
        self.motors['wheel_rl_motor'].setVelocity(left)
        self.motors['wheel_fr_motor'].setVelocity(right)
        self.motors['wheel_rr_motor'].setVelocity(right)

    def step(self):
        rclpy.spin_once(self.node, timeout_sec=0)
        v, w = self.target
        # skid-steer mixing -> wheel angular velocity (rad/s)
        left = (v - w * WHEEL_SEP / 2.0) / WHEEL_RADIUS
        right = (v + w * WHEEL_SEP / 2.0) / WHEEL_RADIUS
        self._set(left, right)
