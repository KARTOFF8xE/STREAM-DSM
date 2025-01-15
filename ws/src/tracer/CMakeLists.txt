cmake_minimum_required(VERSION 3.5)
project(tracer)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find_library(LTTNG_LIBRARIES NAMES lttng-ctl REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(PkgConfig 0.29 REQUIRED)
pkg_check_modules(
  lttng
  IMPORTED_TARGET
    lttng-ctl
  REQUIRED
)

add_executable(start src/startTracer.cpp)
add_executable(stop src/stopTracer.cpp)
ament_target_dependencies(start)
ament_target_dependencies(stop)

install(TARGETS
  start
  stop
  DESTINATION lib/${PROJECT_NAME})

target_link_libraries(start PRIVATE ${lttng_LIBRARIES})
target_link_libraries(stop PRIVATE ${lttng_LIBRARIES})

ament_package()