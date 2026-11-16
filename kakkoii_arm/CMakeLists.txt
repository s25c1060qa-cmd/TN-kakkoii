cmake_minimum_required(VERSION 3.8)
project(kakkoii_arm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(angles REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(pick_and_place src/pick_and_place_node.cpp)
ament_target_dependencies(pick_and_place
  rclcpp
  moveit_ros_planning_interface
  moveit_msgs
  geometry_msgs
  angles
  tf2_geometry_msgs
)

install(TARGETS pick_and_place
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
