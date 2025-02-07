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

add_executable(babel src/babelTrace.cpp)


target_link_libraries(babel PRIVATE ${babeltrace2_LIBRARIES})

install(TARGETS
  babel
  DESTINATION lib/${PROJECT_NAME})

ament_package()