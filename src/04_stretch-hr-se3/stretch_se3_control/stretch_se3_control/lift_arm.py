"""CP-B2 — Lift & telescoping arm.

Raises the lift, extends the telescoping arm, then retracts and lowers. The arm
is commanded as a single aggregate joint ``wrist_extension`` (0 = retracted,
0.52 m = fully out); the driver splits that across the four arm segments.

Two independent feature blocks (lift / arm) so students can comment out either
one and watch only the other move.

Run (with the sim already up):
    ros2 run stretch_se3_control lift_arm
"""

import rclpy

from stretch_se3_control.stretch_trajectory import StretchController


def main(args=None):
    rclpy.init(args=args)
    node = StretchController('lift_arm')
    try:
        node.wait_for_server()

        # === CHECKPOINT: lift ===
        # The vertical tower travel (0.0 = bottom, 1.10 m = top).
        node.move({'joint_lift': 0.9}, seconds=4.0)
        # === END CHECKPOINT: lift ===

        # === CHECKPOINT: arm ===
        # The telescoping arm reach (0.0 = retracted, 0.52 m = fully extended).
        node.move({'wrist_extension': 0.4}, seconds=4.0)
        node.move({'wrist_extension': 0.0}, seconds=4.0)
        # === END CHECKPOINT: arm ===

        # Return the lift to a low, safe height.
        node.move({'joint_lift': 0.4}, seconds=4.0)
        node.get_logger().info('Lift & arm demo complete.')
    except (KeyboardInterrupt, RuntimeError) as exc:
        node.get_logger().error(str(exc))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
