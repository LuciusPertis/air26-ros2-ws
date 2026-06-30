"""theta_integrator — turns the delta-theta stream into absolute joint commands.

The brain only ever says "nudge by this much" (delta-theta). Something has to
remember where the joints ARE and add the nudges up. That's this node: the single
source of truth for current theta.

  subscribes:  /delta_theta   (std_msgs/Float64MultiArray)   from vla_brain
  publishes:   /joint_command (std_msgs/Float64MultiArray)   absolute [t1,t2,t3]
               /joint_states  (sensor_msgs/JointState)       ONLY in viz mode

In RViz-only mode there is no physics, so we fake feedback by publishing
/joint_states ourselves (publish_joint_states:=true). With Gazebo or MuJoCo the
simulator publishes /joint_states, so we leave it alone (publish_joint_states:=false).
"""

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

JOINTS = ['joint1', 'joint2', 'joint3']
# soft limits — must match urdf/arm.urdf.xacro and mjcf/arm.xml
LOWER = np.array([-3.14, -1.5, -2.5])
UPPER = np.array([3.14, 1.5, 2.5])


class ThetaIntegrator(Node):

    def __init__(self):
        super().__init__('theta_integrator')
        self.declare_parameter('publish_joint_states', False)  # True for RViz-only
        self.declare_parameter('rate', 50.0)
        self.viz = self.get_parameter('publish_joint_states').value

        self.theta = np.zeros(len(JOINTS))
        self.cmd_pub = self.create_publisher(Float64MultiArray, 'joint_command', 10)
        self.create_subscription(Float64MultiArray, 'delta_theta', self.on_delta, 10)

        # === CHECKPOINT: viz_feedback ===
        # Only used when no simulator is providing /joint_states (RViz-only).
        if self.viz:
            self.js_pub = self.create_publisher(JointState, 'joint_states', 10)
            self.create_timer(1.0 / self.get_parameter('rate').value, self.publish_js)
        # === END CHECKPOINT: viz_feedback ===

        self.get_logger().info(
            f'theta_integrator up: integrating /delta_theta -> /joint_command'
            f"{' (+/joint_states, viz mode)' if self.viz else ''}.")

    def on_delta(self, msg):
        delta = np.asarray(msg.data, dtype=float)
        if delta.shape[0] != len(JOINTS):
            return
        # === CHECKPOINT: integrate ===
        self.theta = np.clip(self.theta + delta, LOWER, UPPER)   # theta += delta
        # === END CHECKPOINT: integrate ===
        self.cmd_pub.publish(Float64MultiArray(data=[float(x) for x in self.theta]))

    def publish_js(self):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = JOINTS
        js.position = [float(x) for x in self.theta]
        self.js_pub.publish(js)


def main(args=None):
    rclpy.init(args=args)
    node = ThetaIntegrator()
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
