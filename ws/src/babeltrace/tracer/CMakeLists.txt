# cmake_minimum_required(VERSION 3.5)
# project(tracer)

# # Default to C++17
# if(NOT CMAKE_CXX_STANDARD)
#   set(CMAKE_CXX_STANDARD 17)
# endif()


# find_package(ament_cmake REQUIRED)
# find_package(ipc REQUIRED)

# add_executable(primary src/primary.cpp)
# add_executable(secondary src/secondary.cpp)

# target_link_libraries(primary ipc::ipc_lib)
# target_link_libraries(secondary ipc::ipc_lib)


# ament_package()

cmake_minimum_required(VERSION 3.5)
project(tracer)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_library(babeltrace2_LIBRARIES NAMES babeltrace2 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ipc REQUIRED)
find_package(PkgConfig 0.29 REQUIRED)
pkg_check_modules(
  lttng
  IMPORTED_TARGET
    lttng-ctl
  REQUIRED
)

add_executable(babel src/babelTrace.cpp)
add_executable(primary src/primary.cpp)
add_executable(secondary src/secondary.cpp)

target_link_libraries(primary ipc::ipc_lib)
target_link_libraries(secondary ipc::ipc_lib)

target_link_libraries(babel PRIVATE ${babeltrace2_LIBRARIES})
target_include_directories(primary PRIVATE include)
target_include_directories(secondary PRIVATE include)

install(TARGETS
  babel
  primary
  secondary
  DESTINATION lib/${PROJECT_NAME})

ament_package()