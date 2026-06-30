"""mujoco_driver — runs the SO-101 tabletop in MuJoCo and exposes the VLA interface.

Plain ROS python (mujoco + rclpy + cv_bridge) — NOT the venv. It publishes what the SmolVLA
policy needs (the front camera + the joint state) and applies the joint targets the policy
produces, so the policy node stays a pure brain.

  subscribes:  /joint_command       (std_msgs/Float64MultiArray, 6 position targets)
  publishes:   /joint_states        (sensor_msgs/JointState, 6 SO-101 joints)
               /camera/front/image_raw (+ camera_info)   <- the VLA's eye
  param:       scene (mjcf path), cam_width/height, MUJOCO_GL via launch (egl headless)
"""

import os
import math

import mujoco
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState, Image, CameraInfo
from cv_bridge import CvBridge

JOINTS = ['Rotation', 'Pitch', 'Elbow', 'Wrist_Pitch', 'Wrist_Roll', 'Jaw']
HOME = [0.0, -1.57, 1.57, 1.57, -1.57, 0.0]


class MujocoDriver(Node):
    def __init__(self):
        super().__init__('mujoco_driver')
        self.declare_parameter('scene', '')
        self.declare_parameter('cam_name', 'front')
        self.declare_parameter('cam_width', 320)
        self.declare_parameter('cam_height', 240)
        self.declare_parameter('rate', 50.0)
        self.declare_parameter('cam_rate', 10.0)

        scene = self.get_parameter('scene').value or os.path.join(
            get_package_share_directory('vla_so101_description'), 'mjcf', 'tabletop_scene.xml')
        self.model = mujoco.MjModel.from_xml_path(scene)
        self.data = mujoco.MjData(self.model)
        self.data.qpos[:6] = HOME
        self.data.ctrl[:6] = HOME
        mujoco.mj_forward(self.model, self.data)
        self.target = list(HOME)

        self.cam = self.get_parameter('cam_name').value
        self.cw = self.get_parameter('cam_width').value
        self.ch = self.get_parameter('cam_height').value
        self.renderer = mujoco.Renderer(self.model, self.ch, self.cw)
        self.bridge = CvBridge()
        self.cam_info = self._caminfo()

        self.create_subscription(Float64MultiArray, '/joint_command', self.on_cmd, 10)
        self.js_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.img_pub = self.create_publisher(Image, '/camera/front/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/front/camera_info', 10)

        rate = self.get_parameter('rate').value
        self.substeps = max(1, int((1.0 / rate) / self.model.opt.timestep))
        self.cam_every = max(1, int(rate / self.get_parameter('cam_rate').value))
        self._n = 0
        self.create_timer(1.0 / rate, self.step)
        self.get_logger().info(f'mujoco_driver up: scene loaded, {self.model.nu} actuators, '
                               f'cam={self.cam} {self.cw}x{self.ch}')

    def _caminfo(self):
        cid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, self.cam)
        fovy = math.radians(self.model.cam_fovy[cid]) if cid >= 0 else math.radians(55)
        fy = self.ch / (2 * math.tan(fovy / 2)); fx = fy
        ci = CameraInfo(width=self.cw, height=self.ch)
        ci.k = [fx, 0., self.cw/2., 0., fy, self.ch/2., 0., 0., 1.]
        ci.p = [fx, 0., self.cw/2., 0., 0., fy, self.ch/2., 0., 0., 0., 1., 0.]
        return ci

    def on_cmd(self, msg):
        if len(msg.data) >= 6:
            self.target = list(msg.data[:6])

    def step(self):
        self.data.ctrl[:6] = self.target
        for _ in range(self.substeps):
            mujoco.mj_step(self.model, self.data)
        now = self.get_clock().now().to_msg()

        js = JointState()
        js.header.stamp = now
        js.name = JOINTS
        js.position = [float(self.data.qpos[i]) for i in range(6)]
        self.js_pub.publish(js)

        self._n += 1
        if self._n % self.cam_every == 0:
            self.renderer.update_scene(self.data, camera=self.cam)
            rgb = self.renderer.render()
            msg = self.bridge.cv2_to_imgmsg(np.ascontiguousarray(rgb), encoding='rgb8')
            msg.header.stamp = now
            msg.header.frame_id = 'front_camera'
            self.img_pub.publish(msg)
            self.cam_info.header = msg.header
            self.info_pub.publish(self.cam_info)


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
