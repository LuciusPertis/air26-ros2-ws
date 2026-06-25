cmake_minimum_required(VERSION 3.8)
project(stretch_se3_moveit2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

# CP-D3 programmatic reach demo (C++ MoveGroupInterface)
add_executable(reach_demo src/reach_demo.cpp)
ament_target_dependencies(reach_demo rclcpp moveit_ros_planning_interface)
install(TARGETS reach_demo DESTINATION lib/${PROJECT_NAME})

# trajectory bridge (Python node) + config/launch
install(PROGRAMS scripts/trajectory_bridge.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
