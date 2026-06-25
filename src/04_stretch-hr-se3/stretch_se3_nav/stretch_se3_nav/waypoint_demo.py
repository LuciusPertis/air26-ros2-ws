"""CP-C3 — Programmatic navigation with the Nav2 Simple Commander.

Sends the robot through a list of waypoints using ``nav2_simple_commander``'s
``BasicNavigator`` — the same Python API you'd use on the real Stretch. This is
the scripted equivalent of clicking "Nav2 Goal" in RViz.

Prerequisites (separate terminals):
  1. sim in navigation mode:  ros2 launch stretch_se3_bringup sim.launch.py mode:=navigation
  2. localization + Nav2:      ros2 launch stretch_se3_nav navigation.launch.py map:=<your_map>.yaml
  3. set the initial pose in RViz (2D Pose Estimate) so AMCL knows where the robot is

Then:
  ros2 run stretch_se3_nav waypoint_demo

Adapted from nav2_simple_commander's example_waypoint_follower demo.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# Waypoints in the map frame: (x, y, yaw_z, yaw_w). Edit for your saved map —
# pick reachable free-space points (check the costmap in RViz first).
WAYPOINTS = [
    (1.0, 0.0, 0.0, 1.0),
    (1.0, 1.0, 0.707, 0.707),
    (0.0, 0.0, 1.0, 0.0),
]


def make_pose(navigator, x, y, qz, qw):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.orientation.z = float(qz)
    pose.pose.orientation.w = float(qw)
    return pose


def main(args=None):
    rclpy.init(args=args)
    navigator = BasicNavigator()

    navigator.get_logger().info('Waiting for Nav2 to become active...')
    navigator.waitUntilNav2Active()   # blocks until amcl + bt_navigator are up

    # === CHECKPOINT: waypoints ===
    # Comment out this block to bring up the stack without driving anywhere.
    goals = [make_pose(navigator, *wp) for wp in WAYPOINTS]
    navigator.followWaypoints(goals)

    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback:
            navigator.get_logger().info(
                f'On waypoint {feedback.current_waypoint + 1}/{len(goals)}')
    # === END CHECKPOINT: waypoints ===

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        navigator.get_logger().info('All waypoints reached.')
    elif result == TaskResult.CANCELED:
        navigator.get_logger().warn('Navigation canceled.')
    else:
        navigator.get_logger().error(f'Navigation failed: {result}')

    navigator.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
