"""
TUTORIAL 1b — Topics: Subscriber (Listener)

Run alongside topic_talker.py.

    ros2 run basics topic_talker     (separate terminal)
    ros2 run basics topic_listener
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Listener(Node):
    def __init__(self):
        super().__init__('listener')

        # === CHECKPOINT: topics ===
        self.sub = self.create_subscription(String, 'chatter', self.on_message, 10)
        self.get_logger().info('Listener started — waiting on /chatter')
        # === END CHECKPOINT: topics ===

    # === CHECKPOINT: topics ===
    def on_message(self, msg: String):
        self.get_logger().info(f'Heard: "{msg.data}"')
    # === END CHECKPOINT: topics ===


def main(args=None):
    rclpy.init(args=args)
    node = Listener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
