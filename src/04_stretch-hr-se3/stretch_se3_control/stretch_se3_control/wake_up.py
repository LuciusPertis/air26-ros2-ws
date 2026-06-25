"""CP-B5 — Combined pose: stow -> wake-up.

Ties B2-B4 together by commanding several joints in a SINGLE trajectory point, so
the whole upper body moves to a named pose at once (the driver waits for every
joint to arrive before the goal completes).

  * stow    — arm retracted, lift low, wrist folded in, gripper closed, head fwd.
  * wake-up — lift raised, arm slightly out, wrist level, gripper open, head
              tilted down to look at the workspace.

Comment out either pose block to keep the robot in the other one.

Run (with the sim already up):
    ros2 run stretch_se3_control wake_up
"""

import rclpy

from stretch_se3_control.stretch_trajectory import StretchController

STOW_POSE = {
    'joint_lift': 0.35,
    'wrist_extension': 0.0,
    'joint_wrist_yaw': 3.3,        # folded across the body
    'joint_wrist_pitch': 0.0,
    'joint_wrist_roll': 0.0,
    'gripper_aperture': -0.35,     # closed
    'joint_head_pan': 0.0,
    'joint_head_tilt': 0.0,
}

WAKE_UP_POSE = {
    'joint_lift': 0.8,
    'wrist_extension': 0.25,
    'joint_wrist_yaw': 0.0,        # pointing forward
    'joint_wrist_pitch': 0.0,
    'joint_wrist_roll': 0.0,
    'gripper_aperture': 0.16,      # open
    'joint_head_pan': 0.0,
    'joint_head_tilt': -0.6,       # look down at the workspace
}


def main(args=None):
    rclpy.init(args=args)
    node = StretchController('wake_up')
    try:
        node.wait_for_server()

        # === CHECKPOINT: stow ===
        node.get_logger().info('Stowing...')
        node.move(STOW_POSE, seconds=5.0)
        # === END CHECKPOINT: stow ===

        # === CHECKPOINT: wake_up ===
        node.get_logger().info('Waking up...')
        node.move(WAKE_UP_POSE, seconds=5.0)
        # === END CHECKPOINT: wake_up ===

        node.get_logger().info('Wake-up sequence complete.')
    except (KeyboardInterrupt, RuntimeError) as exc:
        node.get_logger().error(str(exc))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
