cmake_minimum_required(VERSION 3.8)
project(sub_demo)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)

add_library(sub_node_component SHARED src/subscriber_node.cpp)
ament_target_dependencies(sub_node_component rclcpp rclcpp_components std_msgs)

rclcpp_components_register_nodes(sub_node_component "SubscriberNode")

install(TARGETS sub_node_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
