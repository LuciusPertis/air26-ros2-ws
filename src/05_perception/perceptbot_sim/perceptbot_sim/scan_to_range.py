"""scan_to_range — Gazebo helper: 1-beam LaserScan -> sensor_msgs/Range.

Ignition/Gazebo has no direct ultrasonic Range sensor, so each ultrasonic is modelled as a
narrow 1-beam gpu_lidar. The ros_gz_bridge gives us a LaserScan; this node converts the three
of them to the same /ultrasonic/* Range topics the MuJoCo and Webots drivers publish, so the
behaviour nodes see an identical interface either way. (Carried over from project 02.)

  subscribes:  /scan_front|left|right  (sensor_msgs/LaserScan)
  publishes:   /ultrasonic/front|left|right  (sensor_msgs/Range)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Range

MAP = [('/scan_front', '/ultrasonic/front', 'us_front'),
       ('/scan_left',  '/ultrasonic/left',  'us_left'),
       ('/scan_right', '/ultrasonic/right', 'us_right')]
MAX_RANGE = 2.0
MIN_RANGE = 0.04


class ScanToRange(Node):

    def __init__(self):
        super().__init__('scan_to_range')
        for scan_topic, range_topic, frame in MAP:
            pub = self.create_publisher(Range, range_topic, 10)
            self.create_subscription(
                LaserScan, scan_topic,
                lambda msg, p=pub, f=frame: self.convert(msg, p, f), 10)
        self.get_logger().info('scan_to_range up: LaserScan -> /ultrasonic/* Range.')

    def convert(self, scan, pub, frame):
        valid = [r for r in scan.ranges if scan.range_min <= r <= scan.range_max]
        d = min(valid) if valid else MAX_RANGE
        r = Range()
        r.header.stamp = scan.header.stamp
        r.header.frame_id = frame
        r.radiation_type = Range.ULTRASOUND
        r.field_of_view = max(scan.angle_max - scan.angle_min, 0.26)
        r.min_range = MIN_RANGE
        r.max_range = MAX_RANGE
        r.range = float(min(max(d, MIN_RANGE), MAX_RANGE))
        pub.publish(r)


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
