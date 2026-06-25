// TUTORIAL 3a — Actions: Server
//
// ros2 run basics_cpp action_server
// ros2 run basics_cpp action_client    (separate terminal)

#include <memory>
#include <thread>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "example_interfaces/action/fibonacci.hpp"

using Fibonacci = example_interfaces::action::Fibonacci;
using GoalHandle = rclcpp_action::ServerGoalHandle<Fibonacci>;
using namespace std::chrono_literals;

class CountServer : public rclcpp::Node
{
public:
  CountServer() : Node("count_server")
  {
    // === CHECKPOINT: actions ===
    action_server_ = rclcpp_action::create_server<Fibonacci>(
      this, "count_up",
      [](const rclcpp_action::GoalUUID &, std::shared_ptr<const Fibonacci::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](std::shared_ptr<GoalHandle>) {
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](std::shared_ptr<GoalHandle> gh) {
        std::thread([this, gh]() { execute(gh); }).detach();
      });
    RCLCPP_INFO(get_logger(), "Action server /count_up ready");
    // === END CHECKPOINT: actions ===
  }

private:
  // === CHECKPOINT: actions ===
  void execute(std::shared_ptr<GoalHandle> goal_handle)
  {
    int target = goal_handle->get_goal()->order;
    RCLCPP_INFO(get_logger(), "Goal received: count to %d", target);

    auto feedback = std::make_shared<Fibonacci::Feedback>();
    for (int i = 0; i <= target; ++i) {
      if (goal_handle->is_canceling()) {
        goal_handle->canceled(std::make_shared<Fibonacci::Result>());
        return;
      }
      feedback->sequence = {i};
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(get_logger(), "Feedback: %d/%d", i, target);
      std::this_thread::sleep_for(1s);
    }

    auto result = std::make_shared<Fibonacci::Result>();
    result->sequence = {target};
    goal_handle->succeed(result);
    RCLCPP_INFO(get_logger(), "Goal succeeded — reached %d", target);
  }

  rclcpp_action::Server<Fibonacci>::SharedPtr action_server_;
  // === END CHECKPOINT: actions ===
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CountServer>());
  rclcpp::shutdown();
}
