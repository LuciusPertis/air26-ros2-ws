"""smolvla_node — the REAL SmolVLA-450M policy, plumbed through ROS 2.

Runs in the isolated venv (torch + lerobot). Each inference it builds the observation the
checkpoint declares (camera image(s) + joint state + the text instruction), runs SmolVLA's
preprocessor (tokenises the instruction + normalises), asks the policy for an action, and
publishes it as 6 SO-101 joint targets.

  subscribes: /camera/front/image_raw (Image), /joint_states (JointState), /instruction (String)
  publishes:  /joint_command (std_msgs/Float64MultiArray, 6 position targets)
  params:     checkpoint (lerobot/smolvla_base), instruction, inference_rate, device

smolvla_base wants state(6) + THREE camera images(3x256x256) + action(6); we feed the front
render to all three image slots. It ships no normalisation stats (base model), so STATE/ACTION
use identity stats (VISUAL is IDENTITY). It is a GENERALIST base model: it runs end-to-end and
moves the arm from real text+vision, but is not fine-tuned for this scene, so it won't reliably
grasp. Outputs are clamped to the SO-101 joint limits.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String, Float64MultiArray
from cv_bridge import CvBridge

import cv2
import torch

JOINT_LIMITS = [(-1.92, 1.92), (-3.32, 0.174), (-0.174, 3.14),
                (-1.66, 1.66), (-2.79, 2.79), (-0.174, 1.75)]


class SmolVLANode(Node):
    def __init__(self):
        super().__init__('smolvla_node')
        self.declare_parameter('checkpoint', 'lerobot/smolvla_base')
        self.declare_parameter('instruction', 'pick up the red cube')
        self.declare_parameter('inference_rate', 5.0)
        self.declare_parameter('device', 'cpu')
        self.instruction = self.get_parameter('instruction').value
        self.device = self.get_parameter('device').value

        self.bridge = CvBridge()
        self.last_rgb = None
        self.last_state = None
        self._load_policy(self.get_parameter('checkpoint').value)

        self.create_subscription(Image, '/camera/front/image_raw', self.on_image, 10)
        self.create_subscription(JointState, '/joint_states', self.on_state, 10)
        self.create_subscription(String, '/instruction', self.on_instruction, 10)
        self.cmd_pub = self.create_publisher(Float64MultiArray, '/joint_command', 10)

        self.create_timer(1.0 / self.get_parameter('inference_rate').value, self.infer)
        self.get_logger().info(
            f'smolvla_node up: image_keys={self.image_keys}, state_dim={self.state_dim}, '
            f'img={self.img_hw}, instruction="{self.instruction}"')

    def _load_policy(self, checkpoint):
        from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy
        from lerobot.policies.smolvla.processor_smolvla import make_smolvla_pre_post_processors
        from lerobot.configs.types import FeatureType
        self.get_logger().info(f'loading SmolVLA from {checkpoint} (CPU; first load is slow)...')
        self.policy = SmolVLAPolicy.from_pretrained(checkpoint)
        self.policy.config.device = self.device
        self.policy.to(self.device); self.policy.eval(); self.policy.reset()

        feats = self.policy.config.input_features
        self.image_keys = [k for k, f in feats.items() if f.type == FeatureType.VISUAL]
        state_keys = [k for k, f in feats.items() if f.type == FeatureType.STATE]
        self.state_key = state_keys[0] if state_keys else None
        self.state_dim = feats[self.state_key].shape[0] if self.state_key else 6
        sh = feats[self.image_keys[0]].shape          # (C,H,W)
        self.img_hw = (sh[1], sh[2])
        # base model ships no stats -> identity for the MEAN_STD features (VISUAL=IDENTITY)
        stats = {self.state_key: {'mean': torch.zeros(self.state_dim),
                                  'std': torch.ones(self.state_dim)},
                 'action': {'mean': torch.zeros(6), 'std': torch.ones(6)}}
        self.pre, self.post = make_smolvla_pre_post_processors(self.policy.config, dataset_stats=stats)

    def on_image(self, msg):
        self.last_rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')

    def on_state(self, msg):
        if len(msg.position) >= 6:
            self.last_state = np.array(msg.position[:6], dtype=np.float32)

    def on_instruction(self, msg):
        self.instruction = msg.data
        self.get_logger().info(f'instruction -> "{self.instruction}"')

    def _build_obs(self):
        h, w = self.img_hw
        img = cv2.resize(self.last_rgb, (w, h))
        t = torch.from_numpy(img).float().permute(2, 0, 1) / 255.0   # (3,H,W) in [0,1]
        state = np.zeros(self.state_dim, np.float32)
        n = min(self.state_dim, len(self.last_state))
        state[:n] = self.last_state[:n]
        obs = {k: t.clone() for k in self.image_keys}     # feed front to all camera slots
        obs[self.state_key] = torch.from_numpy(state)
        obs['task'] = self.instruction
        return obs

    def infer(self):
        if self.last_rgb is None or self.last_state is None:
            return
        with torch.no_grad():
            action = self.post(self.policy.select_action(self.pre(self._build_obs())))
        a = np.asarray(action).reshape(-1)
        cmd = list(self.last_state)
        for i in range(min(6, len(a))):
            lo, hi = JOINT_LIMITS[i]
            cmd[i] = float(max(lo, min(hi, float(a[i]))))
        self.cmd_pub.publish(Float64MultiArray(data=cmd))


def main(args=None):
    rclpy.init(args=args)
    node = SmolVLANode()
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
