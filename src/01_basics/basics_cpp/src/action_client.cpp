// TUTORIAL 3b — Actions: Client
//
// ros2 run basics_cpp action_server    (separate terminal first)
// ros2 run basics_cpp action_client

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "example_interfaces/action/fibonacci.hpp"

using Fibonacci = example_interfaces::action::Fibonacci;
using GoalHandle = rclcpp_action::ClientGoalHandle<Fibonacci>;

class CountClient : public rclcpp::Node
{
public:
  CountClient() : Node("count_client")
  {
    // === CHECKPOINT: actions ===
    client_ = rclcpp_action::create_client<Fibonacci>(this, "count_up");
    // === END CHECKPOINT: actions ===
  }

  // === CHECKPOINT: actions ===
  void send_goal(int target)
  {
    client_->wait_for_action_server();
    RCLCPP_INFO(get_logger(), "Sending goal: count to %d", target);

    auto goal = Fibonacci::Goal();
    goal.order = target;

    auto opts = rclcpp_action::Client<Fibonacci>::SendGoalOptions();

    opts.feedback_callback = [this](GoalHandle::SharedPtr,
      const std::shared_ptr<const Fibonacci::Feedback> fb)
    {
      RCLCPP_INFO(get_logger(), "Feedback: %d", fb->sequence[0]);
    };

    opts.result_callback = [this](const GoalHandle::WrappedResult & res)
    {
      RCLCPP_INFO(get_logger(), "Result: reached %d", res.result->sequence[0]);
      rclcpp::shutdown();
    };

    client_->async_send_goal(goal, opts);
  }

private:
  rclcpp_action::Client<Fibonacci>::SharedPtr client_;
  // === END CHECKPOINT: actions ===
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CountClient>();

  // === CHECKPOINT: actions ===
  node->send_goal(5);
  // === END CHECKPOINT: actions ===

  rclcpp::spin(node);
}
