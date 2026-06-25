// TUTORIAL 4 — Combined Node (C++)
//
// One node: topics + services + actions simultaneously.
// Comment out any CHECKPOINT block, rebuild, and observe.
//
// ros2 run basics_cpp combined_node
//
// Interact:
//   ros2 topic echo /chatter
//   ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 4, b: 6}"
//   ros2 run basics_cpp action_client

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"
#include "example_interfaces/action/fibonacci.hpp"

using namespace std::chrono_literals;
using AddTwoInts = example_interfaces::srv::AddTwoInts;
using Fibonacci  = example_interfaces::action::Fibonacci;
using GoalHandle = rclcpp_action::ServerGoalHandle<Fibonacci>;

class CombinedNode : public rclcpp::Node
{
public:
  CombinedNode() : Node("combined_node"), count_(0)
  {
    // === CHECKPOINT: topics ===
    pub_   = create_publisher<std_msgs::msg::String>("chatter", 10);
    timer_ = create_wall_timer(1s, [this]() { publish_message(); });
    RCLCPP_INFO(get_logger(), "Topic publisher on /chatter — every 1 s");
    // === END CHECKPOINT: topics ===

    // === CHECKPOINT: services ===
    srv_ = create_service<AddTwoInts>(
      "add_two_ints",
      [this](const AddTwoInts::Request::SharedPtr req,
             AddTwoInts::Response::SharedPtr res)
      {
        res->sum = req->a + req->b;
        RCLCPP_INFO(get_logger(), "Service: %ld + %ld = %ld", req->a, req->b, res->sum);
      });
    RCLCPP_INFO(get_logger(), "Service /add_two_ints ready");
    // === END CHECKPOINT: services ===

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
        std::thread([this, gh]() { execute_count(gh); }).detach();
      });
    RCLCPP_INFO(get_logger(), "Action server /count_up ready");
    // === END CHECKPOINT: actions ===
  }

private:
  // === CHECKPOINT: topics ===
  void publish_message()
  {
    auto msg = std_msgs::msg::String();
    msg.data = "Combined node alive — tick " + std::to_string(count_++);
    pub_->publish(msg);
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;
  // === END CHECKPOINT: topics ===

  // === CHECKPOINT: services ===
  rclcpp::Service<AddTwoInts>::SharedPtr srv_;
  // === END CHECKPOINT: services ===

  // === CHECKPOINT: actions ===
  void execute_count(std::shared_ptr<GoalHandle> goal_handle)
  {
    int target = goal_handle->get_goal()->order;
    auto feedback = std::make_shared<Fibonacci::Feedback>();
    for (int i = 0; i <= target; ++i) {
      feedback->sequence = {i};
      goal_handle->publish_feedback(feedback);
      std::this_thread::sleep_for(1s);
    }
    auto result = std::make_shared<Fibonacci::Result>();
    result->sequence = {target};
    goal_handle->succeed(result);
  }

  rclcpp_action::Server<Fibonacci>::SharedPtr action_server_;
  // === END CHECKPOINT: actions ===
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CombinedNode>());
  rclcpp::shutdown();
}
