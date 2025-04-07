cmake_minimum_required(VERSION 3.16)
project(datamgmt)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(ipc REQUIRED)
find_package(dbs REQUIRED)
find_package(nlohmann_json 3.11.3 REQUIRED)

add_executable(datamgmt
  src/datamgmt/datamgmt.cpp
  src/datamgmt/utils.cpp
  src/datamgmt/nodeandtopicobserver/nodeandtopicobserver.cpp
  src/datamgmt/processobserver/processobserver.cpp
  src/datamgmt/relationmgmt/relationmgmt.cpp
  src/pipe/pipe.cpp
)
target_include_directories(datamgmt PRIVATE include/)

target_link_libraries(datamgmt
  PRIVATE
    ipc::ipc_lib
    dbs::dbs_lib
    nlohmann_json::nlohmann_json
)

install(TARGETS
  datamgmt
  DESTINATION lib/${PROJECT_NAME})

ament_package()