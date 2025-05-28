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

add_library(structuralsink SHARED
  src/structural/participants.cpp
  src/structural/sink.cpp
)

add_library(continuoussink SHARED
  src/continuous/participants.cpp
  src/continuous/sink.cpp
)

target_include_directories(structuralsink PRIVATE include)
target_link_libraries(structuralsink
  PRIVATE
    fmt::fmt
    ${babeltrace2_LIBRARIES}
    dbs::dbs_lib
    ipc::ipc_lib
)

target_include_directories(continuoussink PRIVATE include)
target_link_libraries(continuoussink
  PRIVATE
    fmt::fmt
    ${babeltrace2_LIBRARIES}
    dbs::dbs_lib
    ipc::ipc_lib
)

install(TARGETS structuralsink DESTINATION "$ENV{HOME}/.local/lib/babeltrace2/plugins")
install(TARGETS continuoussink DESTINATION "$ENV{HOME}/.local/lib/babeltrace2/plugins")

file(APPEND "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/local_setup.zsh" "")