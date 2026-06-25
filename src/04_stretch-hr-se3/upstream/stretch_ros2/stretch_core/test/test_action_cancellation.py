import time
import pytest
import rclpy

from common.launch_descriptions import stretch_driver_ld
from common.client_nodes.stretch_driver import Client


@pytest.mark.launch(fixture=stretch_driver_ld)
def test_position_mode_cancellation():
    rclpy.init()
    node = Client("test_fjt_cancel")

    # Move the arm lift all the way up
    print(node.mode)
    node.move_to_configuration({"joint_lift": 1.1})

    # Move the lift part way down
    node.get_logger().info("Moving lift down...")
    node.move_to_configuration({"joint_lift": 0.5}, blocking=False)

    # Sleep for 0.5s, then cancel
    time.sleep(0.5)
    node.get_logger().info("Cancelling the goal")
    node.cancel_goal()

    # Lift position should not be near 0.5
    time.sleep(2)
    assert node.q_curr["joint_lift"] > 1.0

    # Clean up
    node.destroy_node()
    rclpy.shutdown()
