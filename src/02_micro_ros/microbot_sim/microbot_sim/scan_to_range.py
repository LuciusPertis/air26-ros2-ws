"""scan_to_range — Gazebo helper: 1-beam LaserScan -> std_msgs/UInt8 (cm).

Ignition has no direct ultrasonic/Range sensor, so each ultrasonic is modelled as a
narrow 1-beam gpu_lidar. The ros_gz_bridge gives us a LaserScan; this node converts
the three of them to the same /ultrasonic/* UInt8-cm topics the MuJoCo driver and the
real ESP32 firmware publish, so the behaviour nodes see an identical interface either
way. (Also a nice little "messages" lesson — converting one message type to another.)

Latency branch: the canonical ultrasonic type is std_msgs/UInt8 centimetres; Range is
viz-only (see range_viz_bridge). The name "scan_to_range" is kept for continuity.

  subscribes:  /scan_front|left|right  (sensor_msgs/LaserScan)
  publishes:   /ultrasonic/front|left|right  (std_msgs/UInt8, cm)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import UInt8

MAP = [('/scan_front', '/ultrasonic/front'),
       ('/scan_left',  '/ultrasonic/left'),
       ('/scan_right', '/ultrasonic/right')]
MAX_RANGE = 2.0
MIN_RANGE = 0.04


class ScanToRange(Node):

    def __init__(self):
        super().__init__('scan_to_range')
        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        for scan_topic, range_topic in MAP:
            pub = self.create_publisher(UInt8, range_topic, sensor_qos)
            self.create_subscription(
                LaserScan, scan_topic,
                lambda msg, p=pub: self.convert(msg, p), 10)
        self.get_logger().info('scan_to_range up: LaserScan -> /ultrasonic/* UInt8 cm.')

    def convert(self, scan, pub):
        valid = [r for r in scan.ranges if scan.range_min <= r <= scan.range_max]
        d = min(valid) if valid else MAX_RANGE
        d = min(max(d, MIN_RANGE), MAX_RANGE)
        pub.publish(UInt8(data=int(round(d * 100.0))))          # m -> cm


def main(args=None):
    rclpy.init(args=args)
    node = ScanToRange()
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
