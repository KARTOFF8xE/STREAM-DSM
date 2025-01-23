cmake_minimum_required(VERSION 3.5)
project(neo4j)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_library(curl_LIBRARIES NAMES curl REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(fmt REQUIRED)

add_executable(foo src/main.cpp)
ament_target_dependencies(foo)

install(TARGETS
  foo
  DESTINATION lib/${PROJECT_NAME})

target_link_libraries(foo PRIVATE ${curl_LIBRARIES})
target_link_libraries(foo PRIVATE fmt::fmt)

ament_package()