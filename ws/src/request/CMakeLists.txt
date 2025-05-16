cmake_minimum_required(VERSION 3.16)
project(request)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(ipc REQUIRED)

add_executable(node src/node.cpp)
add_executable(topic src/topic.cpp)
add_executable(standardSingleAttribute src/standardSingleAttribute.cpp)
add_executable(standardAggregatedAttribute src/standardAggregatedAttribute.cpp)
add_executable(customAttribute src/customAttribute.cpp)
add_executable(aggregatedMember src/aggregatedMember.cpp)
add_executable(customMember src/customMember.cpp)


target_link_libraries(node
  PRIVATE
    ipc::ipc_lib
)

target_link_libraries(topic
  PRIVATE
    ipc::ipc_lib
)

target_link_libraries(standardSingleAttribute
  PRIVATE
    ipc::ipc_lib
)

target_link_libraries(standardAggregatedAttribute
  PRIVATE
    ipc::ipc_lib
)

target_link_libraries(customAttribute
  PRIVATE
    ipc::ipc_lib
)

target_link_libraries(aggregatedMember
  PRIVATE
    ipc::ipc_lib
)

target_link_libraries(customMember
  PRIVATE
    ipc::ipc_lib
)


install(TARGETS
  node
  topic
  standardSingleAttribute
  standardAggregatedAttribute
  customAttribute
  aggregatedMember
  customMember
  DESTINATION lib/${PROJECT_NAME})

ament_package()