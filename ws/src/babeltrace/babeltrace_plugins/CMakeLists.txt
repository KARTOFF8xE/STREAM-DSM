cmake_minimum_required(VERSION 3.5)

project(babeltrace_plugins)

find_library(babeltrace2_LIBRARIES NAMES babeltrace2 REQUIRED)

add_library(simple_sink SHARED
    src/simple_sink.c
)

target_include_directories(simple_sink PRIVATE include)
target_link_libraries(simple_sink PRIVATE ${babeltrace2_LIBRARIES})

list(APPEND ENV{BABELTRACE_PLUGIN_PATH} ${CMAKE_BINARY_DIR})

install(TARGETS simple_sink DESTINATION "$ENV{HOME}/.local/lib/babeltrace2/plugins")