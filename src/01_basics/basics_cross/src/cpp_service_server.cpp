// CROSS-LANGUAGE — Services
// C++ server called by the Python client.
//
// Terminal 1: ros2 run basics_cross cpp_service_server
// Terminal 2: ros2 run basics_cross py_service_client.py

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "basics_cross/srv/add_two_ints.hpp"

using AddTwoInts = basics_cross::srv::AddTwoInts;

class CppServiceServer : public rclcpp::Node
{
public:
  CppServiceServer() : Node("cpp_service_server")
  {
    srv_ = create_service<AddTwoInts>(
      "add_two_ints",
      [this](const AddTwoInts::Request::SharedPtr req,
             AddTwoInts::Response::SharedPtr res)
      {
        res->sum = req->a + req->b;
        RCLCPP_INFO(get_logger(), "[C++ server] %ld + %ld = %ld", req->a, req->b, res->sum);
      });
    RCLCPP_INFO(get_logger(), "C++ service server /add_two_ints ready");
  }

private:
  rclcpp::Service<AddTwoInts>::SharedPtr srv_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CppServiceServer>());
  rclcpp::shutdown();
}
