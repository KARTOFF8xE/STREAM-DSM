cmake_minimum_required(VERSION 3.16)
project(primary)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(ipc REQUIRED)

add_executable(primary src/primary.cpp)

target_link_libraries(primary
  PRIVATE
    ipc::ipc_lib
)

install(TARGETS
  primary
  DESTINATION lib/${PROJECT_NAME})

ament_package()