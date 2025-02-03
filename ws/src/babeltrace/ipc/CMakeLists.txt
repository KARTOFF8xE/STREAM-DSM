cmake_minimum_required(VERSION 3.16)
project(ipc)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

SET(CMAKE_BUILD_TYPE RelWithDebInfo)

find_package(ament_cmake REQUIRED)

add_library(ipc_lib SHARED src/ipc-client.cpp)
add_executable(primary src/primary.cpp src/ipc-client.cpp)
add_executable(secondary src/secondary.cpp src/ipc-client.cpp)

target_include_directories(ipc_lib PRIVATE include)
target_include_directories(primary PRIVATE include)
target_include_directories(secondary PRIVATE include)

target_compile_options(primary PRIVATE -fno-stack-protector)

ament_export_targets(ipc_lib HAS_LIBRARY_TARGET)

install(DIRECTORY include/ipc DESTINATION include)
install(
  TARGETS
    ipc_lib
  EXPORT ipc_lib
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)

install(
  TARGETS
    primary
    secondary
  RUNTIME DESTINATION bin
)


ament_package()