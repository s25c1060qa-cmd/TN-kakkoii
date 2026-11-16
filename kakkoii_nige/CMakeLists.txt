cmake_minimum_required(VERSION 3.8)
project(kakkoii_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(angles REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(crane_x7_description REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 実行ファイルの追加
add_executable(kakkoii scripts/kakkoii.cpp)
ament_target_dependencies(kakkoii
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  angles
  tf2_geometry_msgs
  crane_x7_description
)

# インストール設定（ビルド後に実行ファイルとlaunchを所定の位置に配置する）
install(TARGETS
  kakkoii
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
