cmake_minimum_required(VERSION 3.16)
project(babeltrace_plugins)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_library(curl_LIBRARIES NAMES curl REQUIRED)
find_library(babeltrace2_LIBRARIES NAMES babeltrace2 REQUIRED)
find_package(fmt REQUIRED)
find_package(ipc REQUIRED)
find_package(neo4j REQUIRED)

add_library(sink SHARED
    src/sink.cpp
    src/participantFactory.cpp
    src/participants/client.cpp
    src/participants/node.cpp
    src/participants/dummy.cpp
    src/participants/service.cpp
    src/participants/subscriber.cpp
    src/participants/publisher.cpp
    src/participants/subscriber.cpp
)

target_include_directories(sink PRIVATE include)
target_link_libraries(sink PRIVATE fmt::fmt)
target_link_libraries(sink PRIVATE ${babeltrace2_LIBRARIES})
target_link_libraries(sink PRIVATE ipc::ipc_lib)
target_link_libraries(sink PRIVATE neo4j::neo4j_lib)

install(TARGETS sink DESTINATION "$ENV{HOME}/.local/lib/babeltrace2/plugins")

file(APPEND "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/local_setup.zsh" "")