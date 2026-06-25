cmake_minimum_required(VERSION 3.8)
project(basics_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(example_interfaces REQUIRED)

set(DEPS rclcpp std_msgs example_interfaces)
set(DEPS_ACTION rclcpp rclcpp_action example_interfaces)

# === CHECKPOINT: topics ===
add_executable(topic_talker   src/topic_talker.cpp)
add_executable(topic_listener src/topic_listener.cpp)
ament_target_dependencies(topic_talker   ${DEPS})
ament_target_dependencies(topic_listener ${DEPS})
# === END CHECKPOINT: topics ===

# === CHECKPOINT: services ===
add_executable(service_server src/service_server.cpp)
add_executable(service_client src/service_client.cpp)
ament_target_dependencies(service_server ${DEPS})
ament_target_dependencies(service_client ${DEPS})
# === END CHECKPOINT: services ===

# === CHECKPOINT: actions ===
add_executable(action_server src/action_server.cpp)
add_executable(action_client src/action_client.cpp)
ament_target_dependencies(action_server ${DEPS_ACTION})
ament_target_dependencies(action_client ${DEPS_ACTION})
# === END CHECKPOINT: actions ===

add_executable(combined_node src/combined_node.cpp)
ament_target_dependencies(combined_node rclcpp rclcpp_action std_msgs example_interfaces)

install(TARGETS
  topic_talker
  topic_listener
  service_server
  service_client
  action_server
  action_client
  combined_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
