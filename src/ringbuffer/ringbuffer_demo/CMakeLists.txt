cmake_minimum_required(VERSION 3.5)
project(ringbuffer_demo)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(ringbuffer_demo src/main.cpp)
ament_target_dependencies(ringbuffer_demo rclcpp std_msgs)

install(TARGETS
  ringbuffer_demo
  DESTINATION lib/${PROJECT_NAME})

ament_package()
