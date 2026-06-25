// TUTORIAL 2b — Services: Client
//
// ros2 run basics_cpp service_server   (separate terminal first)
// ros2 run basics_cpp service_client

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

using AddTwoInts = example_interfaces::srv::AddTwoInts;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("add_client");

  // === CHECKPOINT: services ===
  auto client = node->create_client<AddTwoInts>("add_two_ints");

  while (!client->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_INFO(node->get_logger(), "Waiting for /add_two_ints service...");
  }

  auto request = std::make_shared<AddTwoInts::Request>();
  request->a = 3;
  request->b = 5;

  auto future = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, future) == rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(node->get_logger(), "Result: %ld + %ld = %ld",
      request->a, request->b, future.get()->sum);
  }
  // === END CHECKPOINT: services ===

  rclcpp::shutdown();
}
