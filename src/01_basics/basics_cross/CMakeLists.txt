cmake_minimum_required(VERSION 3.8)
project(basics_cross)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(example_interfaces REQUIRED)

# ── C++ nodes ──────────────────────────────────────────────────────────────

# Topic: C++ listener that receives from a Python talker
add_executable(cpp_listener src/cpp_listener.cpp)
ament_target_dependencies(cpp_listener rclcpp std_msgs)

# Service: C++ server called by a Python client
add_executable(cpp_service_server src/cpp_service_server.cpp)
ament_target_dependencies(cpp_service_server rclcpp example_interfaces)

# Action: C++ client that talks to a Python action server
add_executable(cpp_action_client src/cpp_action_client.cpp)
ament_target_dependencies(cpp_action_client rclcpp rclcpp_action example_interfaces)

install(TARGETS
  cpp_listener
  cpp_service_server
  cpp_action_client
  DESTINATION lib/${PROJECT_NAME}
)

# ── Python nodes ───────────────────────────────────────────────────────────

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  basics_cross/py_talker.py
  basics_cross/py_service_client.py
  basics_cross/py_action_server.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
