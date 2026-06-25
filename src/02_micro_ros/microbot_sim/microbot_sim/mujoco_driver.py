"""mujoco_driver — runs the microbot in MuJoCo and exposes the ESP32-style interface.

This node stands in for the real robot (and, later, the micro-ROS ESP32): it consumes
`/cmd_vel` and publishes the three ultrasonic `/ultrasonic/*` ranges plus odometry/TF.
The behaviour nodes don't know or care that it's MuJoCo — swap this for the
micro_ros_agent + ESP32 and they keep working.

  subscribes:  /cmd_vel              (geometry_msgs/Twist)
  publishes:   /ultrasonic/front|left|right  (sensor_msgs/Range)   <- the 3 ultrasonics
               /odom (nav_msgs/Odometry) + TF odom->base_link
               /joint_states (wheels, so RViz spins them)

The base is a kinematic planar joint driven from /cmd_vel (v, w); the 3 MuJoCo
rangefinders give real ray distances to the arena obstacles.
"""

import math
import os

import mujoco
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState, Range
from tf2_ros import TransformBroadcaster

WHEELS = ['wheel_fl_joint', 'wheel_fr_joint', 'wheel_rl_joint', 'wheel_rr_joint']
RANGERS = [('rf_front', 'us_front', '/ultrasonic/front'),
           ('rf_left',  'us_left',  '/ultrasonic/left'),
           ('rf_right', 'us_right', '/ultrasonic/right')]
WHEEL_R = 0.05
MAX_RANGE = 2.0
MIN_RANGE = 0.04


class MujocoDriver(Node):

    def __init__(self):
        super().__init__('mujoco_driver')
        self.declare_parameter('use_viewer', False)
        self.declare_parameter('rate', 100.0)
        self.declare_parameter('cmd_timeout', 0.5)

        mjcf = os.path.join(get_package_share_directory('microbot_description'),
                            'mjcf', 'microbot.xml')
        self.model = mujoco.MjModel.from_xml_path(mjcf)
        self.data = mujoco.MjData(self.model)

        self.qadr = {n: self.model.jnt_qposadr[
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, n)]
            for n in ['slide_x', 'slide_y', 'yaw'] + WHEELS}
        self.vadr = {n: self.model.jnt_dofadr[
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, n)]
            for n in ['slide_x', 'slide_y', 'yaw'] + WHEELS}
        self.sadr = {s: self.model.sensor_adr[
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, s)]
            for s, _, _ in RANGERS}

        self.v = 0.0
        self.w = 0.0
        self.last_cmd = self.get_clock().now()

        self.create_subscription(Twist, 'cmd_vel', self.on_cmd, 10)
        self.range_pubs = {topic: self.create_publisher(Range, topic, 10)
                           for _, _, topic in RANGERS}
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.js_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.tf = TransformBroadcaster(self)

        self.create_timer(1.0 / self.get_parameter('rate').value, self.step)

        self.viewer = None
        if self.get_parameter('use_viewer').value:
            from mujoco import viewer as mj_viewer
            self.viewer = mj_viewer.launch_passive(self.model, self.data)
        self.get_logger().info('mujoco_driver up: /cmd_vel -> microbot -> /ultrasonic/*, /odom.')

    def on_cmd(self, msg):
        self.v = msg.linear.x
        self.w = msg.angular.z
        self.last_cmd = self.get_clock().now()

    def step(self):
        # stale command -> stop
        if (self.get_clock().now() - self.last_cmd).nanoseconds * 1e-9 > \
                self.get_parameter('cmd_timeout').value:
            self.v = self.w = 0.0

        # === CHECKPOINT: kinematic_drive ===
        yaw = self.data.qpos[self.qadr['yaw']]
        # obstacles are SOLID: the kinematic base can't drive forward into a close
        # front obstacle (a backstop in case a behaviour reacts too late).
        front = self.data.sensordata[self.sadr['rf_front']]
        v_eff = self.v
        if v_eff > 0 and 0 <= front < 0.22:
            v_eff = 0.0
        self.data.qvel[self.vadr['slide_x']] = v_eff * math.cos(yaw)
        self.data.qvel[self.vadr['slide_y']] = v_eff * math.sin(yaw)
        self.data.qvel[self.vadr['yaw']] = self.w
        for wj in WHEELS:                      # spin wheels for show
            self.data.qvel[self.vadr[wj]] = v_eff / WHEEL_R
        # === END CHECKPOINT: kinematic_drive ===

        mujoco.mj_step(self.model, self.data)
        # keep the robot inside the arena (it can never leave)
        self.data.qpos[self.qadr['slide_x']] = min(max(
            self.data.qpos[self.qadr['slide_x']], -1.4), 1.4)
        self.data.qpos[self.qadr['slide_y']] = min(max(
            self.data.qpos[self.qadr['slide_y']], -1.4), 1.4)
        if self.viewer is not None:
            self.viewer.sync()

        now = self.get_clock().now().to_msg()
        self.publish_ranges(now)
        self.publish_odom(now, yaw)
        self.publish_joints(now)

    def publish_ranges(self, stamp):
        for sname, frame, topic in RANGERS:
            d = float(self.data.sensordata[self.sadr[sname]])
            if d < 0:                          # rangefinder: -1 = no hit
                d = MAX_RANGE
            r = Range()
            r.header.stamp = stamp
            r.header.frame_id = frame
            r.radiation_type = Range.ULTRASOUND
            r.field_of_view = 0.26
            r.min_range = MIN_RANGE
            r.max_range = MAX_RANGE
            r.range = float(min(max(d, MIN_RANGE), MAX_RANGE))
            self.range_pubs[topic].publish(r)

    def publish_odom(self, stamp, yaw):
        x = float(self.data.qpos[self.qadr['slide_x']])
        y = float(self.data.qpos[self.qadr['slide_y']])
        qz, qw = math.sin(yaw / 2), math.cos(yaw / 2)
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf.sendTransform(t)
        od = Odometry()
        od.header.stamp = stamp
        od.header.frame_id = 'odom'
        od.child_frame_id = 'base_link'
        od.pose.pose.position.x = x
        od.pose.pose.position.y = y
        od.pose.pose.orientation.z = qz
        od.pose.pose.orientation.w = qw
        od.twist.twist.linear.x = self.v
        od.twist.twist.angular.z = self.w
        self.odom_pub.publish(od)

    def publish_joints(self, stamp):
        js = JointState()
        js.header.stamp = stamp
        js.name = WHEELS
        js.position = [float(self.data.qpos[self.qadr[w]]) for w in WHEELS]
        self.js_pub.publish(js)


def main(args=None):
    rclpy.init(args=args)
    node = MujocoDriver()
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
