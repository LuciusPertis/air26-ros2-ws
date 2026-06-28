"""gz_command_relay — /joint_command -> /position_controller/commands.

The theta_integrator publishes a sim-agnostic /joint_command. The Gazebo
ros2_control position controller listens on /position_controller/commands. This
tiny relay bridges the two (same Float64MultiArray, just a different name), keeping
the integrator independent of which simulator is running.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

IN_TOPIC = 'joint_command'
OUT_TOPIC = '/position_controller/commands'


def main(args=None):
    rclpy.init(args=args)
    node = Node('gz_command_relay')
    pub = node.create_publisher(Float64MultiArray, OUT_TOPIC, 10)
    node.create_subscription(Float64MultiArray, IN_TOPIC, lambda m: pub.publish(m), 10)
    node.get_logger().info(f'relaying {IN_TOPIC} -> {OUT_TOPIC}')
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
