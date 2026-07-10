"""camera_processor — turn a camera image into a few cheap, behaviour-friendly numbers.

This is the software twin of what the ESP32-CAM firmware computes on-board: reduce a
whole frame to (a) how much bright light is in view, (b) the average colour, and (c) how
much of the target colour (green) is in view. Behaviours 4 and 5 condition on these
scalars instead of the ultrasonics.

  subscribes:  /camera/image_raw     (sensor_msgs/Image)
  publishes:   /camera/light_level (std_msgs/Float32)   0.0 (no light) .. 1.0 (all bright)
               /camera/mean_color     (std_msgs/ColorRGBA)  mean R,G,B in 0..1, a=1
               /camera/color_level (std_msgs/Float32)   0.0 (no green) .. 1.0 (all green)

All are normalised to 0..1 so the SAME numbers come out of the sim and the real ESP32-CAM
(the firmware publishes the identical topics). That is the whole point of project 05's
"same interface, swappable embodiment" design.

Both light_level and color_level are PIXEL-FRACTIONS, not frame means: a mean is dominated
by the bright sky / the walls and never isolates a small bright panel or a small green box
(§ the checkpoint comments below). Counting matching pixels gives a clean ~0 baseline that
rises only when the real target is in view — and stays cheap for the firmware.

Note on /camera/light_level: it is the *fraction of near-white pixels*, NOT a plain
grayscale mean. A grayscale mean is dominated by the always-bright textured sky (the
arena walls are short and the camera sits low), so it never isolates the light panel:
facing the panel it barely rises above the sky baseline. Counting only saturated pixels
gives a clean ~0 baseline that spikes only when a genuine light source (the luminous
panel) fills part of the view — and it is just as cheap for the firmware to compute
(count pixels over a brightness cut, divide by the total).
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, ColorRGBA
from cv_bridge import CvBridge
import cv2

# a pixel brighter than this (0..255 grayscale) counts as "lit" for /camera/light_level
BRIGHT_CUT = 200
# a pixel whose green channel beats BOTH red and blue by this much (0..255) counts as "green"
# for /camera/color_level (the B5 target colour). A relative test -> tolerant of brightness.
GREEN_MARGIN = 40


class CameraProcessor(Node):

    def __init__(self):
        super().__init__('camera_processor')
        self.declare_parameter('image_topic', '/camera/image_raw')
        topic = self.get_parameter('image_topic').value
        self.bridge = CvBridge()

        self.create_subscription(Image, topic, self.on_image, 10)
        # === CHECKPOINT: light_level ===
        self.light_pub = self.create_publisher(Float32, '/camera/light_level', 10)
        # === END CHECKPOINT: light_level ===
        # === CHECKPOINT: mean_color ===
        self.color_pub = self.create_publisher(ColorRGBA, '/camera/mean_color', 10)
        # === END CHECKPOINT: mean_color ===
        # === CHECKPOINT: color_level ===
        self.color_level_pub = self.create_publisher(Float32, '/camera/color_level', 10)
        # === END CHECKPOINT: color_level ===

        self.get_logger().info(f'camera_processor up, reading {topic} -> '
                               '/camera/light_level + /camera/mean_color + /camera/color_level')

    def on_image(self, msg):
        # bgr8: handles Webots bgra8 and a real cam's rgb/jpeg alike via cv_bridge
        bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # === CHECKPOINT: light_level ===
        # fraction of near-white pixels (0..1) — "how much bright light is in view".
        # Same cheap maths the firmware does: threshold the grayscale frame and count.
        # A plain frame mean would just track the bright sky; counting saturated pixels
        # stays ~0 until a real light source (the luminous panel) is in view.
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        bright_frac = float((gray > BRIGHT_CUT).mean())
        self.light_pub.publish(Float32(data=bright_frac))
        # === END CHECKPOINT: light_level ===

        # === CHECKPOINT: mean_color ===
        # average B,G,R over the frame -> ColorRGBA (note cv2 is BGR, ROS wants R,G,B)
        b, g, r = bgr.reshape(-1, 3).mean(axis=0) / 255.0
        self.color_pub.publish(ColorRGBA(r=float(r), g=float(g), b=float(b), a=1.0))
        # === END CHECKPOINT: mean_color ===

        # === CHECKPOINT: color_level ===
        # fraction of clearly-green pixels (0..1) — "how much of the target colour is in view".
        # The B5 twin of light_level. A whole-frame mean colour is dominated by walls/floor and
        # a small green box barely moves it (a neutral grey wall can even outscore it), so B5
        # could never separate green from grey. Counting only pixels where green clearly beats
        # red AND blue isolates the box: ~0 baseline, rising as the box fills the view. It is a
        # relative test (g vs r,b) so it survives brightness changes, and it is just as cheap for
        # the firmware to compute. Change GREEN_MARGIN / the channel test to chase another colour.
        bc, gc, rc = cv2.split(bgr.astype('int16'))
        green_mask = (gc - rc > GREEN_MARGIN) & (gc - bc > GREEN_MARGIN)
        self.color_level_pub.publish(Float32(data=float(green_mask.mean())))
        # === END CHECKPOINT: color_level ===


def main(args=None):
    rclpy.init(args=args)
    node = CameraProcessor()
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
