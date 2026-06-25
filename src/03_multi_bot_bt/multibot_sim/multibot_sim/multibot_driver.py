"""multibot_driver — self-contained Webots <extern> controller for one patrol unit.

Loaded once per robot via the URDF <plugin>. It derives its **namespace from the Webots
robot's own name** (r1/r2/r3), so each unit publishes /rN/... with no launch-side namespace
juggling. It does everything itself (no webots_ros2 device tags), which keeps multi-robot
namespacing trivial and gives us the exact frame_ids the perception/BT expect:

  subscribes: cmd_vel                       -> 4 skid-steer wheels
  publishes:  ultrasonic/{front,left,right} (Range), camera/image_raw (Image, bgra8)
              + camera/camera_info (CameraInfo, K from the Webots camera FOV)
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range, Image, CameraInfo

WHEEL_RADIUS = 0.05
WHEEL_SEP = 0.32
MAX_W = 12.0
WHEELS = ['wheel_fl_motor', 'wheel_fr_motor', 'wheel_rl_motor', 'wheel_rr_motor']
RANGERS = [('us_front', 'ultrasonic/front', 'us_front'),
           ('us_left',  'ultrasonic/left',  'us_left'),
           ('us_right', 'ultrasonic/right', 'us_right')]
MAX_RANGE, MIN_RANGE = 3.0, 0.04


class MultibotDriver:
    def init(self, webots_node, properties):
        self.robot = webots_node.robot
        self.ns = self.robot.getName()                 # 'r1' / 'r2' / 'r3'
        self.ts = int(self.robot.getBasicTimeStep())

        self.motors = {}
        for name in WHEELS:
            m = self.robot.getDevice(name)
            m.setPosition(float('inf')); m.setVelocity(0.0)
            self.motors[name] = m

        self.ds = {}
        for dev, _, _ in RANGERS:
            s = self.robot.getDevice(dev); s.enable(self.ts); self.ds[dev] = s

        self.cam = self.robot.getDevice('camera'); self.cam.enable(self.ts)
        self.cw, self.ch = self.cam.getWidth(), self.cam.getHeight()
        fov = self.cam.getFov()
        fx = (self.cw / 2.0) / math.tan(fov / 2.0)
        self.K = [fx, 0.0, self.cw/2.0, 0.0, fx, self.ch/2.0, 0.0, 0.0, 1.0]

        if not rclpy.ok():
            rclpy.init(args=None)
        self.node = rclpy.create_node('multibot_driver', namespace=self.ns)
        self.target = (0.0, 0.0)
        self.node.create_subscription(Twist, 'cmd_vel', self._on_cmd, 1)
        self.range_pubs = {topic: self.node.create_publisher(Range, topic, 10)
                           for _, topic, _ in RANGERS}
        self.img_pub = self.node.create_publisher(Image, 'camera/image_raw', 10)
        self.info_pub = self.node.create_publisher(CameraInfo, 'camera/camera_info', 10)
        self._n = 0

    def _on_cmd(self, msg):
        self.target = (msg.linear.x, msg.angular.z)

    def step(self):
        rclpy.spin_once(self.node, timeout_sec=0)
        v, w = self.target
        left = (v - w * WHEEL_SEP / 2.0) / WHEEL_RADIUS
        right = (v + w * WHEEL_SEP / 2.0) / WHEEL_RADIUS
        left = max(-MAX_W, min(MAX_W, left)); right = max(-MAX_W, min(MAX_W, right))
        self.motors['wheel_fl_motor'].setVelocity(left)
        self.motors['wheel_rl_motor'].setVelocity(left)
        self.motors['wheel_fr_motor'].setVelocity(right)
        self.motors['wheel_rr_motor'].setVelocity(right)

        self._n += 1
        now = self.node.get_clock().now().to_msg()
        if self._n % 6 == 0:                            # ~10 Hz ranges
            for dev, topic, frame in RANGERS:
                d = self.ds[dev].getValue()
                if d < 0:
                    d = MAX_RANGE
                r = Range()
                r.header.stamp = now
                r.header.frame_id = f'{self.ns}/{frame}'
                r.radiation_type = Range.ULTRASOUND
                r.field_of_view = 0.26
                r.min_range, r.max_range = MIN_RANGE, MAX_RANGE
                r.range = float(min(max(d, MIN_RANGE), MAX_RANGE))
                self.range_pubs[topic].publish(r)
        if self._n % 4 == 0:                            # ~15 Hz camera
            img = Image()
            img.header.stamp = now
            img.header.frame_id = f'{self.ns}/camera_optical_frame'
            img.height, img.width = self.ch, self.cw
            img.encoding = 'bgra8'
            img.step = self.cw * 4
            img.data = self.cam.getImage()
            self.img_pub.publish(img)
            info = CameraInfo(width=self.cw, height=self.ch)
            info.header = img.header
            info.k = self.K
            info.p = [self.K[0], 0.0, self.K[2], 0.0, 0.0, self.K[4], self.K[5], 0.0,
                      0.0, 0.0, 1.0, 0.0]
            self.info_pub.publish(info)
