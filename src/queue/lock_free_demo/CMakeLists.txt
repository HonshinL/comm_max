cmake_minimum_required(VERSION 3.5)
project(lock_free_demo)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Boost REQUIRED)  # 不要写 lockfree

add_executable(lock_free_demo src/main.cpp)
ament_target_dependencies(lock_free_demo rclcpp std_msgs)

target_include_directories(lock_free_demo PUBLIC ${Boost_INCLUDE_DIRS})

install(TARGETS
  lock_free_demo
  DESTINATION lib/${PROJECT_NAME})

ament_package()
