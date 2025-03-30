cmake_minimum_required(VERSION 3.16)
project(dbs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(CURL REQUIRED)
find_package(fmt REQUIRED)

add_library(dbs_lib SHARED
    src/curl/myCurl.cpp
    src/neo4j/node/node.cpp
    src/neo4j/publisher/publisher.cpp
    src/neo4j/subscriber/subscriber.cpp
    src/neo4j/service/service.cpp
    src/neo4j/client/client.cpp
    src/neo4j/actionservice/actionservice.cpp
    src/neo4j/actionclient/actionclient.cpp
    src/neo4j/topic/topic.cpp
    src/neo4j/roots/roots.cpp
)

target_include_directories(dbs_lib PRIVATE include)

target_link_libraries(dbs_lib
  PRIVATE
    CURL::libcurl
    fmt::fmt
)

ament_export_targets(dbs_lib HAS_LIBRARY_TARGET)

install(
  DIRECTORY include/neo4j
  DESTINATION include
)
install(
  DIRECTORY include/curl
  DESTINATION include
)
install(
  TARGETS dbs_lib
  EXPORT dbs_lib
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)

ament_package()