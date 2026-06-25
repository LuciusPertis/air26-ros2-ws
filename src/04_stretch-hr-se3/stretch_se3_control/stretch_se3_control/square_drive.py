"""CP-B1 — Base: drive a square.

Traces an (approximate) square by alternating a forward drive and an in-place
90-degree turn, four times, using base *velocity* on /stretch/cmd_vel.

Key teaching point: the base is the one part of Stretch you drive with a
**velocity** command (a Twist), and the driver only accepts it in **navigation
mode** — unlike the lift/arm/head/wrist, which take position goals through the
trajectory action in position mode. So this node switches to navigation mode
first, then restores position mode at the end.

For *interactive* teleoperation, see the tutorial's teleop_twist_keyboard step
(same /stretch/cmd_vel, same navigation-mode requirement).

The sim runs slower than real time on CPU, so the square is approximate; tune the
speeds/durations below to taste.

Run (with the sim already up):
    ros2 run stretch_se3_control square_drive
"""

import rclpy

from stretch_se3_control.stretch_trajectory import StretchController

_pi = 3.14159265
_sidelen = 1.0

FORWARD_SPEED = 0.10    # m/s
TURN_SPEED = 1          # rad/s
SIDE_SECONDS = 10       #_sidelen / FORWARD_SPEED  # drive forward this long per side
TURN_SECONDS = 17       #_pi / (2 * TURN_SPEED)  # turn ~90 deg (TURN_SPEED * TURN_SECONDS ~= pi/2)

d_version = 1.0

def main(args=None):
    rclpy.init(args=args)
    node = StretchController('square_drive')
    try:
        node.get_logger().info(f'Square drive demo v{d_version:.1f} starting...')
        node.switch_mode('navigation')

        # === CHECKPOINT: square_drive ===
        # Comment out this loop (or shrink range(4)) to change the path. Each
        # iteration drives one side, then turns ~90 degrees.
        for corner in range(4):
            node.get_logger().info(f'Side {corner + 1}/4')
            node.drive(FORWARD_SPEED, 0.0, SIDE_SECONDS) # side stride
            node.drive(0.0, TURN_SPEED, TURN_SECONDS) # in-place turn
        # === END CHECKPOINT: square_drive ===

        node.get_logger().info('Square complete; restoring position mode.')
        node.switch_mode('position')
    except (KeyboardInterrupt, RuntimeError) as exc:
        node.get_logger().error(str(exc))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
