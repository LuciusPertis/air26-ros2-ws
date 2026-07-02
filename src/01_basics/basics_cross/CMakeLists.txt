cmake_minimum_required(VERSION 3.8)
project(basics_cross)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ── Interfaces ──────────────────────────────────────────────────────────────
# Defined right here so the package depends on nothing but rcl. rosidl also
# generates the `basics_cross` Python module (basics_cross.msg/srv/action) that
# the Python nodes below import — which is why we do NOT also call
# ament_python_install_package (it would fight rosidl for the same namespace).
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Greeting.msg"
  "srv/AddTwoInts.srv"
  "action/CountUp.action"
)
# Handle to the C++ typesupport just generated, so our own C++ nodes can link it.
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# ── C++ nodes ──────────────────────────────────────────────────────────────

# Topic: C++ listener that receives from a Python talker
add_executable(cpp_listener src/cpp_listener.cpp)
ament_target_dependencies(cpp_listener rclcpp)
target_link_libraries(cpp_listener "${cpp_typesupport_target}")

# Service: C++ server called by a Python client
add_executable(cpp_service_server src/cpp_service_server.cpp)
ament_target_dependencies(cpp_service_server rclcpp)
target_link_libraries(cpp_service_server "${cpp_typesupport_target}")

# Action: C++ client that talks to a Python action server
add_executable(cpp_action_client src/cpp_action_client.cpp)
ament_target_dependencies(cpp_action_client rclcpp rclcpp_action)
target_link_libraries(cpp_action_client "${cpp_typesupport_target}")

install(TARGETS
  cpp_listener
  cpp_service_server
  cpp_action_client
  DESTINATION lib/${PROJECT_NAME}
)

# ── Python nodes ───────────────────────────────────────────────────────────
# Installed as plain scripts; at run time they import the rosidl-generated
# `basics_cross` interfaces (on the PYTHONPATH after sourcing install/).
install(PROGRAMS
  basics_cross/py_talker.py
  basics_cross/py_service_client.py
  basics_cross/py_action_server.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
