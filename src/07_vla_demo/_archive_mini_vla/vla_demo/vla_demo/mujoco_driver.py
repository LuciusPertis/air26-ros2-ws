"""mujoco_driver — runs the arm in MuJoCo and applies /joint_command.

The MuJoCo end of the pipeline. It loads mjcf/arm.xml, drives the position
actuators from the integrated joint commands, steps the physics, and publishes
the resulting /joint_states (so the brain still gets real proprioception).

  subscribes:  /joint_command (std_msgs/Float64MultiArray)  [t1, t2, t3]
  publishes:   /joint_states  (sensor_msgs/JointState)      from the sim

Open the native viewer with use_viewer:=true (needs a display); headless/offscreen
otherwise (set MUJOCO_GL=egl on a GPU-less box).
"""

import os

import mujoco
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

JOINTS = ['joint1', 'joint2', 'joint3']


class MujocoDriver(Node):

    def __init__(self):
        super().__init__('mujoco_driver')
        self.declare_parameter('use_viewer', False)
        self.declare_parameter('rate', 100.0)

        mjcf = os.path.join(get_package_share_directory('vla_arm_description'),
                            'mjcf', 'arm.xml')
        self.model = mujoco.MjModel.from_xml_path(mjcf)
        self.data = mujoco.MjData(self.model)
        # actuator index per joint (actuators are named act_<joint>)
        self.act = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, f'act_{j}')
                    for j in JOINTS]
        self.qadr = [self.model.jnt_qposadr[
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, j)] for j in JOINTS]

        self.create_subscription(Float64MultiArray, 'joint_command', self.on_cmd, 10)
        self.js_pub = self.create_publisher(JointState, 'joint_states', 10)

        self.viewer = None
        if self.get_parameter('use_viewer').value:
            # aliased import so we don't rebind the module-level name `mujoco`
            from mujoco import viewer as mj_viewer
            self.viewer = mj_viewer.launch_passive(self.model, self.data)

        dt = 1.0 / self.get_parameter('rate').value
        self.create_timer(dt, self.step)
        self.get_logger().info('mujoco_driver up: /joint_command -> MuJoCo -> /joint_states.')

    def on_cmd(self, msg):
        # === CHECKPOINT: apply_command ===
        for i, target in enumerate(msg.data[:len(self.act)]):
            self.data.ctrl[self.act[i]] = float(target)
        # === END CHECKPOINT: apply_command ===

    def step(self):
        mujoco.mj_step(self.model, self.data)
        if self.viewer is not None:
            self.viewer.sync()
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = JOINTS
        js.position = [float(self.data.qpos[a]) for a in self.qadr]
        self.js_pub.publish(js)


def main(args=None):
    rclpy.init(args=args)
    node = MujocoDriver()
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
