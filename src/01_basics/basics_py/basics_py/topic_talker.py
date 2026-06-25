"""
TUTORIAL 1a — Topics: Publisher (Talker)

Run alongside topic_listener.py to see pub/sub in action.

    ros2 run basics topic_talker
    ros2 run basics topic_listener   (separate terminal)
    ros2 topic echo /chatter          (optional third terminal)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Talker(Node):
    def __init__(self):
        super().__init__('talker')

        # === CHECKPOINT: topics ===
        self.pub = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(1.0, self.publish_message)
        self.count = 0
        self.get_logger().info('Talker started — publishing on /chatter every 1 s')
        # === END CHECKPOINT: topics ===

    # === CHECKPOINT: topics ===
    def publish_message(self):
        msg = String()
        msg.data = f'Hello ROS2! count={self.count}'
        self.pub.publish(msg)
        self.get_logger().info(f'Published: "{msg.data}"')
        self.count += 1
    # === END CHECKPOINT: topics ===


def main(args=None):
    rclpy.init(args=args)
    node = Talker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
