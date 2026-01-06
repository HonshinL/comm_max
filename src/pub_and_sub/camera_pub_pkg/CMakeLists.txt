cmake_minimum_required(VERSION 3.8)
project(camera_pub_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(camera_pub src/camera_pub.cpp)
ament_target_dependencies(camera_pub
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV)

add_executable(camera_pub_with_stamp src/camera_pub_with_stamp.cpp)
ament_target_dependencies(camera_pub_with_stamp
  rclcpp
  sensor_msgs)

install(TARGETS 
  camera_pub
  camera_pub_with_stamp
  DESTINATION lib/${PROJECT_NAME})

ament_package()
