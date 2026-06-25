cmake_minimum_required(VERSION 3.8)
project(microbot_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetBehavior.srv"
  "srv/CheckOpenings.srv"
  "action/EscapeObstacle.action"
  DEPENDENCIES action_msgs
)

ament_package()
