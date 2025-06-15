cmake_minimum_required(VERSION 3.5)
project(cpp_pubsub)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp std_msgs)
add_executable(multitalker src/multi_publisher_member_function.cpp)
ament_target_dependencies(multitalker rclcpp std_msgs)
add_executable(timestamptalker src/timestamp_publisher.cpp)
ament_target_dependencies(timestamptalker rclcpp std_msgs)
add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp std_msgs)
add_executable(timestamplistener src/timestamp_subscriber.cpp)
ament_target_dependencies(timestamplistener rclcpp std_msgs)

install(TARGETS
  talker
  timestamptalker
  multitalker
  listener
  timestamplistener
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()