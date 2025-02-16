cmake_minimum_required(VERSION 3.16)
project(datamgmt)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(ipc REQUIRED)
find_package(neo4j REQUIRED)

add_executable(datamgmt
  src/datamgmt/datamgmt.cpp
  src/util.cpp
  src/pipe/pipe.cpp
)
target_include_directories(datamgmt PRIVATE include/)

target_link_libraries(datamgmt
  PRIVATE
    ipc::ipc_lib
    neo4j::neo4j_lib
)
# target_link_libraries(sink PRIVATE neo4j::neo4j_lib)

install(TARGETS
  datamgmt
  DESTINATION lib/${PROJECT_NAME})

ament_package()