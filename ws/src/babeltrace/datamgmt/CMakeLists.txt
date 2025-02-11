cmake_minimum_required(VERSION 3.5)
project(datamgmt)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)

add_executable(datamgmt src/datamgmt.cpp)

install(TARGETS
  datamgmt
  DESTINATION lib/${PROJECT_NAME})

ament_package()