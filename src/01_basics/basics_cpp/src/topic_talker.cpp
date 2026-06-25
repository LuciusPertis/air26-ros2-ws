// TUTORIAL 1a — Topics: Publisher (Talker)
//
// ros2 run basics_cpp topic_talker
// ros2 run basics_cpp topic_listener   (separate terminal)
// ros2 topic echo /chatter

#include <chrono>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class Talker : public rclcpp::Node
{
public:
  Talker() : Node("talker"), count_(0)
  {
    // === CHECKPOINT: topics ===
    pub_ = create_publisher<std_msgs::msg::String>("chatter", 10);
    timer_ = create_wall_timer(1s, [this]() { publish_message(); });
    RCLCPP_INFO(get_logger(), "Talker started — publishing on /chatter every 1 s");
    // === END CHECKPOINT: topics ===
  }

private:
  // === CHECKPOINT: topics ===
  void publish_message()
  {
    auto msg = std_msgs::msg::String();
    msg.data = "Hello ROS2! count=" + std::to_string(count_++);
    pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "Published: \"%s\"", msg.data.c_str());
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;
  // === END CHECKPOINT: topics ===
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Talker>());
  rclcpp::shutdown();
}
