// CROSS-LANGUAGE — Actions
// C++ client that sends a goal to the Python action server.
//
// Terminal 1: ros2 run basics_cross py_action_server.py
// Terminal 2: ros2 run basics_cross cpp_action_client

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "example_interfaces/action/fibonacci.hpp"

using Fibonacci  = example_interfaces::action::Fibonacci;
using GoalHandle = rclcpp_action::ClientGoalHandle<Fibonacci>;

class CppActionClient : public rclcpp::Node
{
public:
  CppActionClient() : Node("cpp_action_client")
  {
    client_ = rclcpp_action::create_client<Fibonacci>(this, "count_up");
  }

  void send_goal(int target)
  {
    client_->wait_for_action_server();
    RCLCPP_INFO(get_logger(), "[C++ client] Sending goal: count to %d", target);

    auto goal = Fibonacci::Goal();
    goal.order = target;

    auto opts = rclcpp_action::Client<Fibonacci>::SendGoalOptions();

    opts.feedback_callback = [this](GoalHandle::SharedPtr,
      const std::shared_ptr<const Fibonacci::Feedback> fb)
    {
      RCLCPP_INFO(get_logger(), "[C++ client] Feedback from Python server: %d", fb->sequence[0]);
    };

    opts.result_callback = [this](const GoalHandle::WrappedResult & res)
    {
      RCLCPP_INFO(get_logger(), "[C++ client] Result from Python server: reached %d",
        res.result->sequence[0]);
      rclcpp::shutdown();
    };

    client_->async_send_goal(goal, opts);
  }

private:
  rclcpp_action::Client<Fibonacci>::SharedPtr client_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CppActionClient>();
  node->send_goal(5);
  rclcpp::spin(node);
}
