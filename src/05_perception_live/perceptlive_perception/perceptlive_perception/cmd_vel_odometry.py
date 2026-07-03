"""cmd_vel_odometry — dead-reckoned odometry for the REAL rover, PC-side.

The rover's ESP32 firmware (`esp32_microbot`) only *subscribes* to /cmd_vel to drive
the L298N motors; it has no wheel encoders, so nothing on hardware publishes a pose.
Without an `odom -> base_link` transform the robot cannot move in RViz.

This node closes that gap the cheap way: it integrates the *commanded* velocity we send
to the robot. That is open-loop dead reckoning — it will drift (wheel slip, unmodelled
dynamics) and is NOT localization. It is exactly good enough to *see the robot move* in
RViz during a live demo, and it mirrors the sim's `mujoco_driver.publish_odom()` maths so
the two behave the same.

  subscribes:  /cmd_vel            (geometry_msgs/Twist)   -- the same command we drive with
  publishes:   /odom              (nav_msgs/Odometry)
               TF  odom -> base_link
               /joint_states      (sensor_msgs/JointState) -- so the 4 wheels spin in RViz

  params:
    rate          (float, 30.0)   integration/publish rate [Hz]
    cmd_timeout   (float, 0.5)    stop integrating if no /cmd_vel for this long [s]
    wheel_radius  (float, 0.05)   from the URDF (wheel_r)
    wheel_base    (float, 0.32)   left<->right wheel separation = 2*wheel_dy (0.16*2)
    odom_frame    (str, 'odom')
    base_frame    (str, 'base_link')

To reset the pose back to the origin, just restart the node.
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

# wheel link names from perceptlive_description/urdf/perceptbot.urdf.xacro
WHEELS_LEFT = ['wheel_fl_joint', 'wheel_rl_joint']
WHEELS_RIGHT = ['wheel_fr_joint', 'wheel_rr_joint']


class CmdVelOdometry(Node):

    def __init__(self):
        super().__init__('cmd_vel_odometry')
        self.declare_parameter('rate', 30.0)
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('wheel_base', 0.32)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        self.rate = self.get_parameter('rate').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value
        self.wheel_r = self.get_parameter('wheel_radius').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        # integrated state
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.wheel_l = 0.0        # accumulated left-wheel angle [rad]
        self.wheel_r_ang = 0.0    # accumulated right-wheel angle [rad]

        # latest command
        self.v = 0.0
        self.w = 0.0
        self.last_cmd = self.get_clock().now()

        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.js_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.tf = TransformBroadcaster(self)

        self.last_t = self.get_clock().now()
        self.create_timer(1.0 / self.rate, self.step)
        self.get_logger().info(
            'cmd_vel_odometry up: integrating /cmd_vel -> /odom + TF odom->base_link '
            '(dead reckoning, open-loop — expect drift).')

    def on_cmd(self, msg):
        self.v = msg.linear.x
        self.w = msg.angular.z
        self.last_cmd = self.get_clock().now()

    def step(self):
        now = self.get_clock().now()
        dt = (now - self.last_t).nanoseconds * 1e-9
        self.last_t = now
        if dt <= 0.0:
            return

        # zero out a stale command so the robot doesn't drift forever after teleop stops
        if (now - self.last_cmd).nanoseconds * 1e-9 > self.cmd_timeout:
            self.v = self.w = 0.0

        # integrate the unicycle model (exact for straight/arc segments would be nicer,
        # but the simple Euler step matches the sim driver and is fine at 30 Hz)
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw += self.w * dt

        # differential wheel speeds -> spin the wheels visually
        half = self.wheel_base / 2.0
        v_left = self.v - self.w * half
        v_right = self.v + self.w * half
        self.wheel_l += (v_left / self.wheel_r) * dt
        self.wheel_r_ang += (v_right / self.wheel_r) * dt

        stamp = now.to_msg()
        self.publish_odom(stamp)
        self.publish_joints(stamp)

    def publish_odom(self, stamp):
        qz, qw = math.sin(self.yaw / 2.0), math.cos(self.yaw / 2.0)
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf.sendTransform(t)

        od = Odometry()
        od.header.stamp = stamp
        od.header.frame_id = self.odom_frame
        od.child_frame_id = self.base_frame
        od.pose.pose.position.x = self.x
        od.pose.pose.position.y = self.y
        od.pose.pose.orientation.z = qz
        od.pose.pose.orientation.w = qw
        od.twist.twist.linear.x = self.v
        od.twist.twist.angular.z = self.w
        self.odom_pub.publish(od)

    def publish_joints(self, stamp):
        js = JointState()
        js.header.stamp = stamp
        js.name = WHEELS_LEFT + WHEELS_RIGHT
        js.position = [self.wheel_l, self.wheel_l, self.wheel_r_ang, self.wheel_r_ang]
        self.js_pub.publish(js)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
