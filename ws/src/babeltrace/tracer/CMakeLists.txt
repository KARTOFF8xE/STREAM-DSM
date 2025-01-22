cmake_minimum_required(VERSION 3.5)
project(tracer)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
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
ament_target_dependencies(babel)

install(TARGETS
  babel
  DESTINATION lib/${PROJECT_NAME})

target_link_libraries(babel PRIVATE ${babeltrace2_LIBRARIES})

ament_package()