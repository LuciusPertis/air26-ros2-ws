"""instruction_pub — publish the text instruction for SmolVLA (latched, repeated).

Convenience so late subscribers get it. Or just:
  ros2 topic pub /instruction std_msgs/String "{data: 'stack the blue cube on the red cube'}"
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class InstructionPub(Node):
    def __init__(self):
        super().__init__('instruction_pub')
        self.declare_parameter('instruction', 'pick up the red cube')
        self.text = self.get_parameter('instruction').value
        self.pub = self.create_publisher(String, '/instruction', 10)
        self.create_timer(1.0, self.tick)

    def tick(self):
        self.pub.publish(String(data=self.text))


def main(args=None):
    rclpy.init(args=args)
    node = InstructionPub()
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
