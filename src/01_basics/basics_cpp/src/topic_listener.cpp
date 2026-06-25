// TUTORIAL 1b — Topics: Subscriber (Listener)
//
// ros2 run basics_cpp topic_talker     (separate terminal)
// ros2 run basics_cpp topic_listener

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class Listener : public rclcpp::Node
{
public:
  Listener() : Node("listener")
  {
    // === CHECKPOINT: topics ===
    sub_ = create_subscription<std_msgs::msg::String>(
      "chatter", 10,
      [this](const std_msgs::msg::String & msg) {
        RCLCPP_INFO(get_logger(), "Heard: \"%s\"", msg.data.c_str());
      });
    RCLCPP_INFO(get_logger(), "Listener started — waiting on /chatter");
    // === END CHECKPOINT: topics ===
  }

private:
  // === CHECKPOINT: topics ===
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  // === END CHECKPOINT: topics ===
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Listener>());
  rclcpp::shutdown();
}
