cmake_minimum_required(VERSION 3.10)
project(foo)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LTTNG REQUIRED lttng-ctl)

add_executable(bar src/main.cpp)

target_include_directories(bar PRIVATE ${LTTNG_INCLUDE_DIRS})
target_link_libraries(bar PRIVATE ${LTTNG_LIBRARIES})