cmake_minimum_required(VERSION 3.8)
project(consumer_demo)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ring_buffer_lib REQUIRED)

add_library(consumer_node SHARED
  src/consumer_node.cpp
)
target_include_directories(consumer_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(consumer_node rclcpp rclcpp_components ring_buffer_lib)

rclcpp_components_register_nodes(consumer_node "ConsumerNode")

install(TARGETS consumer_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
