cmake_minimum_required(VERSION 3.8)
project(producer_with_dub)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(double_buffer_lib REQUIRED)

add_library(producer_component SHARED
  src/producer_component.cpp
)
target_include_directories(producer_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(producer_component
  rclcpp rclcpp_components std_msgs double_buffer_lib
)
rclcpp_components_register_nodes(producer_component "ProducerNode")

install(TARGETS producer_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
