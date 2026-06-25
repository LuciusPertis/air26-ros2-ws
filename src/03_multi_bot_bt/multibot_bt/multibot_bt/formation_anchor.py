"""formation_anchor — re-broadcast this unit's velocity as the formation anchor.

For parallel patrol, followers match the LEADER's velocity (frame-free formation control —
no shared map needed). This tiny node echoes the unit's own /cmd_vel onto formation/anchor
(TwistStamped). Followers subscribe to the leader's /<leader>/formation/anchor and copy its
linear/angular velocity, then trim with the side ultrasonic to hold the lateral gap.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


class FormationAnchor(Node):
    def __init__(self):
        super().__init__('formation_anchor')
        self.pub = self.create_publisher(TwistStamped, 'formation/anchor', 10)
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd, 10)

    def on_cmd(self, msg):
        ts = TwistStamped()
        ts.header.stamp = self.get_clock().now().to_msg()
        ts.twist = msg
        self.pub.publish(ts)


def main(args=None):
    rclpy.init(args=args)
    node = FormationAnchor()
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
