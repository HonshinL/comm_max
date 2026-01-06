cmake_minimum_required(VERSION 3.5)
project(intra_demo)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(pub_sub_intra src/main.cpp)
ament_target_dependencies(pub_sub_intra rclcpp std_msgs)
install(TARGETS pub_sub_intra DESTINATION lib/${PROJECT_NAME})

ament_package()
