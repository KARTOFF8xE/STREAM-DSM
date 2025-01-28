cmake_minimum_required(VERSION 3.5)
project(babeltrace_plugins)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_library(curl_LIBRARIES NAMES curl REQUIRED)
find_library(babeltrace2_LIBRARIES NAMES babeltrace2 REQUIRED)
find_package(fmt REQUIRED)

add_library(simple_sink SHARED
    src/simple_sink.cpp
)

target_include_directories(simple_sink PRIVATE include)
target_link_libraries(simple_sink PRIVATE fmt::fmt)
target_link_libraries(simple_sink PRIVATE ${curl_LIBRARIES})
target_link_libraries(simple_sink PRIVATE ${babeltrace2_LIBRARIES})

install(TARGETS simple_sink DESTINATION "$ENV{HOME}/.local/lib/babeltrace2/plugins")
if(NOT EXISTS "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/local_setup.zsh")
  file(TOUCH "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/local_setup.zsh") # dummy setup.zsh, otherwise its annoying
endif()