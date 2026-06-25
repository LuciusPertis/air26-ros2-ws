// AIR26 Workshop — CP-D3: programmatic motion planning with MoveGroupInterface.
//
// The C++ equivalent of dragging the arm in RViz and clicking Plan & Execute.
// Plans to a couple of named targets (defined in the SRDF) and a joint-space
// goal, executing each on the MuJoCo sim through move_group + the trajectory
// bridge.
//
// Prerequisites (separate terminals):
//   ros2 launch stretch_se3_bringup sim.launch.py
//   ros2 launch stretch_se3_moveit2 move_group.launch.py
// Then (a launch, NOT `ros2 run` — MoveGroupInterface needs the SRDF/URDF params
// on this node):
//   ros2 launch stretch_se3_moveit2 reach_demo.launch.py
//
// Adapted from the galactic hello-robot stretch_moveit2 movegroup_test.cpp.

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

using moveit::planning_interface::MoveGroupInterface;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
      "reach_demo",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Spin in a background thread so MoveGroupInterface can query current state.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  auto logger = node->get_logger();
  MoveGroupInterface arm(node, "stretch_arm");
  arm.setPlanningPipelineId("ompl");
  arm.setMaxVelocityScalingFactor(0.4);
  arm.setMaxAccelerationScalingFactor(0.4);
  arm.setPlanningTime(10.0);

  auto plan_and_execute = [&](const std::string& label) {
    MoveGroupInterface::Plan plan;
    if (arm.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(logger, "[%s] plan found, executing...", label.c_str());
      arm.execute(plan);
      RCLCPP_INFO(logger, "[%s] done.", label.c_str());
    } else {
      RCLCPP_ERROR(logger, "[%s] planning FAILED.", label.c_str());
    }
    // Let the (slower-than-real-time) sim arm settle before the next plan.
    rclcpp::sleep_for(std::chrono::seconds(2));
  };

  // === CHECKPOINT: named_targets ===
  // Named targets come straight from the SRDF group_states (see stretch_se3.srdf).
  RCLCPP_INFO(logger, "Moving to 'ready'...");
  arm.setNamedTarget("ready");
  plan_and_execute("ready");

  RCLCPP_INFO(logger, "Moving to 'stow'...");
  arm.setNamedTarget("stow");
  plan_and_execute("stow");
  // === END CHECKPOINT: named_targets ===

  // === CHECKPOINT: joint_goal ===
  // A joint-space goal: raise the lift and extend the arm a little.
  RCLCPP_INFO(logger, "Moving to a custom joint goal...");
  std::map<std::string, double> target{
      {"joint_lift", 0.9},
      {"joint_arm_l0", 0.05}, {"joint_arm_l1", 0.05},
      {"joint_arm_l2", 0.05}, {"joint_arm_l3", 0.05},
      {"joint_wrist_yaw", 0.0}};
  arm.setJointValueTarget(target);
  plan_and_execute("joint_goal");
  // === END CHECKPOINT: joint_goal ===

  RCLCPP_INFO(logger, "Reach demo complete.");
  rclcpp::shutdown();
  spinner.join();
  return 0;
}
