cmake_minimum_required(VERSION 3.5)
project(condition_variable_demo)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(cond_var_demo src/main.cpp)
ament_target_dependencies(cond_var_demo rclcpp std_msgs)

install(TARGETS
  cond_var_demo
  DESTINATION lib/${PROJECT_NAME})

ament_package()
