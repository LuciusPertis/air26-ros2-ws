"""range_viz_bridge — VIZ-ONLY: std_msgs/UInt8 (cm) -> sensor_msgs/Range (m) for RViz.

Latency branch (10-fix): the ultrasonics now speak std_msgs/UInt8 centimetres end to end
(firmware + mujoco_driver + behaviours) because that is the cheapest thing to put on the
wire and all the behaviours ever do is compare the number to a threshold. But RViz's Range
display still wants sensor_msgs/Range, so this node re-inflates cm -> metres purely for the
picture. It sits OFF the control loop — nothing that drives the robot depends on it, so you
can run it, kill it, or ignore it without touching behaviour timing.

  subscribes:  /ultrasonic/front|left|right         (std_msgs/UInt8, cm)   <- best-effort
  publishes:   /ultrasonic/front|left|right/range   (sensor_msgs/Range, m) -> RViz only
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import UInt8
from sensor_msgs.msg import Range

# in-topic (UInt8 cm), out-topic (Range m), frame_id
MAP = [('/ultrasonic/front', '/ultrasonic/front/range', 'us_front'),
       ('/ultrasonic/left',  '/ultrasonic/left/range',  'us_left'),
       ('/ultrasonic/right', '/ultrasonic/right/range', 'us_right')]
MAX_RANGE = 2.0
MIN_RANGE = 0.04
FOV = 0.26


class RangeVizBridge(Node):

    def __init__(self):
        super().__init__('range_viz_bridge')
        # match the firmware/driver best-effort sensor pubs, else no data arrives.
        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        for in_topic, out_topic, frame in MAP:
            pub = self.create_publisher(Range, out_topic, 10)
            self.create_subscription(
                UInt8, in_topic,
                lambda msg, p=pub, f=frame: self.convert(msg, p, f), sensor_qos)
        self.get_logger().info('range_viz_bridge up: /ultrasonic/* UInt8 cm -> */range Range m.')

    def convert(self, msg, pub, frame):
        r = Range()
        r.header.stamp = self.get_clock().now().to_msg()
        r.header.frame_id = frame
        r.radiation_type = Range.ULTRASOUND
        r.field_of_view = FOV
        r.min_range = MIN_RANGE
        r.max_range = MAX_RANGE
        r.range = min(max(msg.data / 100.0, MIN_RANGE), MAX_RANGE)   # cm -> m, clamped
        pub.publish(r)


def main(args=None):
    rclpy.init(args=args)
    node = RangeVizBridge()
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
