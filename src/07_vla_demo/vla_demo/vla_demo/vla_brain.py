"""vla_brain — the mini-VLA node. Instruction in, delta-theta out.

This is the "brain" of the demo and the source of the star topic. Every tick it
asks the policy for a delta-theta and publishes it on **/delta_theta** — the one
topic the whole workshop is about. It is pure policy->action: it does NOT move the
robot itself (the theta_integrator + a sim do that), which keeps the data flow
obvious for students.

  subscribes:  /instruction   (std_msgs/String)            the language command
               /joint_command (std_msgs/Float64MultiArray)  proprioception (commanded theta)
  publishes:   /delta_theta   (std_msgs/Float64MultiArray)  <-- THE STAR TOPIC

Proprioception here is the *commanded* theta (the integrator's /joint_command), not
the measured /joint_states. That keeps the brain<->integrator a stable closed loop:
reading the laggy measured state would let closed-loop commands (like "home")
overshoot while physics catches up. A production VLA would read measured state with
a controller closing the loop.

Try it:
  ros2 topic pub /instruction std_msgs/String "{data: wave}"
  ros2 topic echo /delta_theta
"""

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String

from vla_demo.policies import ScriptedPolicy

JOINTS = ['joint1', 'joint2', 'joint3']


def make_policy(name, dt):
    if name == 'scripted':
        return ScriptedPolicy(dt=dt)
    if name == 'smolvla':
        from vla_demo.policies.smolvla_adapter import SmolVLAPolicy
        return SmolVLAPolicy()
    raise ValueError(f'unknown policy: {name}')


class VlaBrain(Node):

    def __init__(self):
        super().__init__('vla_brain')
        self.declare_parameter('rate', 10.0)
        self.declare_parameter('policy', 'scripted')
        self.declare_parameter('instruction', 'stop')   # start idle
        rate = self.get_parameter('rate').value

        self.instruction = self.get_parameter('instruction').value
        self.theta = np.zeros(len(JOINTS))
        self.policy = make_policy(self.get_parameter('policy').value, dt=1.0 / rate)

        # === CHECKPOINT: io ===
        self.delta_pub = self.create_publisher(Float64MultiArray, 'delta_theta', 10)
        self.create_subscription(String, 'instruction', self.on_instruction, 10)
        self.create_subscription(Float64MultiArray, 'joint_command', self.on_joint_command, 10)
        # === END CHECKPOINT: io ===

        self.timer = self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info(
            f"vla_brain up: policy='{self.get_parameter('policy').value}', "
            f'{rate:.0f} Hz, publishing /delta_theta. Instruction = "{self.instruction}".')

    def on_instruction(self, msg):
        self.instruction = msg.data
        self.policy.reset()
        self.get_logger().info(f'instruction -> "{self.instruction}"')

    def on_joint_command(self, msg):
        # proprioception = the latest commanded theta (stable, lag-free)
        if len(msg.data) == len(JOINTS):
            self.theta = np.asarray(msg.data, dtype=float)

    def tick(self):
        # === CHECKPOINT: think ===
        # The whole "VLA": language (+ theta) -> action (delta-theta).
        delta = self.policy.predict(self.instruction, self.theta)
        # === END CHECKPOINT: think ===
        self.delta_pub.publish(Float64MultiArray(data=[float(x) for x in delta]))


def main(args=None):
    rclpy.init(args=args)
    node = VlaBrain()
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
