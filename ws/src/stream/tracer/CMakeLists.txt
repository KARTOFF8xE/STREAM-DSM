cmake_minimum_required(VERSION 3.5)
project(tracer)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_library(babeltrace2_LIBRARIES NAMES babeltrace2 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(PkgConfig 0.29 REQUIRED)
pkg_check_modules(
  lttng
  IMPORTED_TARGET
    lttng-ctl
  REQUIRED
)

add_executable(structural src/structural.cpp)
add_executable(continuous src/continuous.cpp)


target_link_libraries(structural PRIVATE ${babeltrace2_LIBRARIES})
target_link_libraries(continuous PRIVATE ${babeltrace2_LIBRARIES})

install(TARGETS
  structural
  continuous
  DESTINATION lib/${PROJECT_NAME})

ament_package()