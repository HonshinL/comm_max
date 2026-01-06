cmake_minimum_required(VERSION 3.8)
project(producer_demo)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ringbuffer_lib REQUIRED)

add_library(producer_node SHARED
  src/producer_node.cpp
)
target_include_directories(producer_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(producer_node rclcpp rclcpp_components ringbuffer_lib)

rclcpp_components_register_nodes(producer_node "ProducerNode")

install(TARGETS producer_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
