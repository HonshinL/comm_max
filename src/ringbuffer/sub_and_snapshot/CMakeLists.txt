cmake_minimum_required(VERSION 3.5)
project(sub_and_snapshot)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(sub_and_snapshot src/main.cpp)
ament_target_dependencies(sub_and_snapshot rclcpp std_msgs)

install(TARGETS
  sub_and_snapshot
  DESTINATION lib/${PROJECT_NAME})

ament_package()
