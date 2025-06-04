cmake_minimum_required(VERSION 3.5)
project(tracer)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_library(babeltrace2_LIBRARIES NAMES babeltrace2 REQUIRED)
find_package(ipc REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(PkgConfig 0.29 REQUIRED)
pkg_check_modules(LTTNG REQUIRED lttng-ctl)


add_executable(structural src/structural/tracer.cpp)
add_executable(structuralLO
  src/structuralLO/babeltrace.cpp
  src/structuralLO/tracer.cpp
  src/structuralLO/threadPool.cpp)
target_include_directories(structuralLO PRIVATE include/)
add_executable(continuous
  src/continuous/babeltrace.cpp
  src/continuous/tracer.cpp
  src/continuous/threadPool.cpp)
target_include_directories(continuous PRIVATE include/)


target_include_directories(continuous PRIVATE ${LTTNG_INCLUDE_DIRS})
target_link_libraries(continuous PRIVATE ${LTTNG_LIBRARIES})
target_include_directories(structural PRIVATE ${LTTNG_INCLUDE_DIRS})
target_link_libraries(structural PRIVATE ${LTTNG_LIBRARIES})
target_include_directories(structuralLO PRIVATE ${LTTNG_INCLUDE_DIRS})
target_link_libraries(structuralLO PRIVATE
  ${LTTNG_LIBRARIES}
  ipc::ipc_lib  
)

target_link_libraries(structuralLO PRIVATE ${babeltrace2_LIBRARIES})
target_link_libraries(structural PRIVATE ${babeltrace2_LIBRARIES})
target_link_libraries(continuous PRIVATE ${babeltrace2_LIBRARIES})

install(TARGETS
  structuralLO
  structural
  continuous
  DESTINATION lib/${PROJECT_NAME})

ament_package()