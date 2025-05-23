cmake_minimum_required(VERSION 3.16)
project(babeltrace_plugins)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_library(curl_LIBRARIES NAMES curl REQUIRED)
find_package(ipc REQUIRED)
find_package(fmt REQUIRED)
find_package(dbs REQUIRED)
find_library(babeltrace2_LIBRARIES NAMES babeltrace2 REQUIRED)

add_library(sink SHARED
  src/participants.cpp
  src/sink.cpp
)

target_include_directories(sink PRIVATE include)
target_link_libraries(sink
  PRIVATE
    fmt::fmt
    ${babeltrace2_LIBRARIES}
    dbs::dbs_lib
    ipc::ipc_lib
)

install(TARGETS sink DESTINATION "$ENV{HOME}/.local/lib/babeltrace2/plugins")

file(APPEND "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/local_setup.zsh" "")