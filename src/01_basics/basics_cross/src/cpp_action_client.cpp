// CROSS-LANGUAGE — Actions
// C++ client that sends a goal to the Python action server.
//
// Terminal 1: ros2 run basics_cross py_action_server.py
// Terminal 2: ros2 run basics_cross cpp_action_client

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "basics_cross/action/count_up.hpp"

using CountUp    = basics_cross::action::CountUp;
using GoalHandle = rclcpp_action::ClientGoalHandle<CountUp>;

class CppActionClient : public rclcpp::Node
{
public:
  CppActionClient() : Node("cpp_action_client")
  {
    client_ = rclcpp_action::create_client<CountUp>(this, "count_up");
  }

  void send_goal(int target)
  {
    client_->wait_for_action_server();
    RCLCPP_INFO(get_logger(), "[C++ client] Sending goal: count to %d", target);

    auto goal = CountUp::Goal();
    goal.order = target;

    auto opts = rclcpp_action::Client<CountUp>::SendGoalOptions();

    opts.feedback_callback = [this](GoalHandle::SharedPtr,
      const std::shared_ptr<const CountUp::Feedback> fb)
    {
      RCLCPP_INFO(get_logger(), "[C++ client] Feedback from Python server: %ld", fb->sequence[0]);
    };

    opts.result_callback = [this](const GoalHandle::WrappedResult & res)
    {
      RCLCPP_INFO(get_logger(), "[C++ client] Result from Python server: reached %ld",
        res.result->sequence[0]);
      rclcpp::shutdown();
    };

    client_->async_send_goal(goal, opts);
  }

private:
  rclcpp_action::Client<CountUp>::SharedPtr client_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CppActionClient>();
  node->send_goal(5);
  rclcpp::spin(node);
}
