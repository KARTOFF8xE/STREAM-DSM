cmake_minimum_required(VERSION 3.8)
project(lifecycle_publisher)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(talker src/lifecyclePub.cpp)

ament_target_dependencies(talker
  rclcpp
  rclcpp_lifecycle
  std_msgs
)

install(TARGETS
  talker
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()