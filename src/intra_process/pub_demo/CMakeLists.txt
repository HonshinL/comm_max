cmake_minimum_required(VERSION 3.8)
project(pub_demo)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)

# 编译为共享库
add_library(pub_node_component SHARED src/publisher_node.cpp)
ament_target_dependencies(pub_node_component rclcpp rclcpp_components std_msgs)

# 注册组件
rclcpp_components_register_nodes(pub_node_component "PublisherNode")

install(TARGETS pub_node_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
