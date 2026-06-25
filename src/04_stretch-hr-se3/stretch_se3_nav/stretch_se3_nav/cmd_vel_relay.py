"""Relay /cmd_vel -> /stretch/cmd_vel.

Nav2's controller (and teleop_twist_keyboard) publish base velocity on the plain
``/cmd_vel`` topic, but our MuJoCo bringup remaps the driver's input to
``/stretch/cmd_vel`` (see sim.launch.py). This tiny node bridges the two.

Upstream stretch_nav2 uses `topic_tools relay` for exactly this; that package is
not installed here, so we ship a three-line node instead — and it doubles as a
visible, hackable piece of the nav stack for students.

Both nav launch files start this automatically. You normally never run it by hand.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

IN_TOPIC = '/cmd_vel'
OUT_TOPIC = '/stretch/cmd_vel'


def main(args=None):
    rclpy.init(args=args)
    node = Node('cmd_vel_relay')
    pub = node.create_publisher(Twist, OUT_TOPIC, 10)
    node.create_subscription(Twist, IN_TOPIC, lambda msg: pub.publish(msg), 10)
    node.get_logger().info(f'Relaying {IN_TOPIC} -> {OUT_TOPIC}')
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
