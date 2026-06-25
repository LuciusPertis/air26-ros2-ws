// CROSS-LANGUAGE — Topics
// C++ subscriber that receives from the Python talker.
//
// Terminal 1: ros2 run basics_cross py_talker.py
// Terminal 2: ros2 run basics_cross cpp_listener

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class CppListener : public rclcpp::Node
{
public:
  CppListener() : Node("cpp_listener")
  {
    sub_ = create_subscription<std_msgs::msg::String>(
      "chatter", 10,
      [this](const std_msgs::msg::String & msg) {
        RCLCPP_INFO(get_logger(), "[C++ heard from Python] \"%s\"", msg.data.c_str());
      });
    RCLCPP_INFO(get_logger(), "C++ listener ready on /chatter");
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CppListener>());
  rclcpp::shutdown();
}
