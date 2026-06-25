"""CP-B4 — Dex wrist (yaw/pitch/roll) + gripper.

Exercises the three wrist degrees of freedom that put the "SE3" in Stretch SE3 —
yaw, pitch, roll — then opens and closes the gripper. Each is an independent
feature block.

Run (with the sim already up):
    ros2 run stretch_se3_control wrist_gripper
"""

import rclpy

from stretch_se3_control.stretch_trajectory import StretchController


def main(args=None):
    rclpy.init(args=args)
    node = StretchController('wrist_gripper')
    try:
        node.wait_for_server()

        # === CHECKPOINT: wrist_yaw ===
        node.move({'joint_wrist_yaw': 1.0}, seconds=3.0)
        node.move({'joint_wrist_yaw': 0.0}, seconds=3.0)
        # === END CHECKPOINT: wrist_yaw ===

        # === CHECKPOINT: wrist_pitch ===
        node.move({'joint_wrist_pitch': -0.8}, seconds=3.0)
        node.move({'joint_wrist_pitch': 0.0}, seconds=3.0)
        # === END CHECKPOINT: wrist_pitch ===

        # === CHECKPOINT: wrist_roll ===
        node.move({'joint_wrist_roll': 1.5}, seconds=3.0)
        node.move({'joint_wrist_roll': 0.0}, seconds=3.0)
        # === END CHECKPOINT: wrist_roll ===

        # === CHECKPOINT: gripper ===
        # gripper_aperture: negative = closed, positive = open.
        node.move({'gripper_aperture': 0.16}, seconds=2.0)   # open
        node.move({'gripper_aperture': -0.35}, seconds=2.0)  # close
        # === END CHECKPOINT: gripper ===

        node.get_logger().info('Wrist & gripper demo complete.')
    except (KeyboardInterrupt, RuntimeError) as exc:
        node.get_logger().error(str(exc))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
