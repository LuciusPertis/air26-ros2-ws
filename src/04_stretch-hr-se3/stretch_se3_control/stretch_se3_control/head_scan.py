"""CP-B3 — Head: pan & tilt to aim the cameras.

Sweeps the head pan left/right and tilts down/up, then re-centres. The head
carries the D435i camera, so this is how you point perception at a target.

Pan and tilt are separate feature blocks.

Run (with the sim already up):
    ros2 run stretch_se3_control head_scan
"""

import rclpy

from stretch_se3_control.stretch_trajectory import StretchController


def main(args=None):
    rclpy.init(args=args)
    node = StretchController('head_scan')
    try:
        node.wait_for_server()

        # === CHECKPOINT: head_pan ===
        # Rotate the head left then right (radians; 0 = forward).
        node.move({'joint_head_pan': 1.2}, seconds=3.0)    # look left
        node.move({'joint_head_pan': -1.2}, seconds=3.0)   # look right
        node.move({'joint_head_pan': 0.0}, seconds=3.0)    # centre
        # === END CHECKPOINT: head_pan ===

        # === CHECKPOINT: head_tilt ===
        # Tilt the head down (toward the gripper) then back up.
        node.move({'joint_head_tilt': -0.9}, seconds=3.0)  # look down
        node.move({'joint_head_tilt': 0.2}, seconds=3.0)   # look up
        node.move({'joint_head_tilt': 0.0}, seconds=3.0)   # level
        # === END CHECKPOINT: head_tilt ===

        node.get_logger().info('Head scan complete.')
    except (KeyboardInterrupt, RuntimeError) as exc:
        node.get_logger().error(str(exc))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
