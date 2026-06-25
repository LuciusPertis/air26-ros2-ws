// TUTORIAL 2a — Services: Server
//
// ros2 run basics_cpp service_server
// ros2 run basics_cpp service_client   (separate terminal)
// ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 3, b: 5}"

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

using AddTwoInts = example_interfaces::srv::AddTwoInts;

class AddServer : public rclcpp::Node
{
public:
  AddServer() : Node("add_server")
  {
    // === CHECKPOINT: services ===
    srv_ = create_service<AddTwoInts>(
      "add_two_ints",
      [this](const AddTwoInts::Request::SharedPtr req,
             AddTwoInts::Response::SharedPtr res)
      {
        res->sum = req->a + req->b;
        RCLCPP_INFO(get_logger(), "Request: %ld + %ld = %ld", req->a, req->b, res->sum);
      });
    RCLCPP_INFO(get_logger(), "Service /add_two_ints ready");
    // === END CHECKPOINT: services ===
  }

private:
  // === CHECKPOINT: services ===
  rclcpp::Service<AddTwoInts>::SharedPtr srv_;
  // === END CHECKPOINT: services ===
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AddServer>());
  rclcpp::shutdown();
}
