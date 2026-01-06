cmake_minimum_required(VERSION 3.8)
project(prod_and_coms)

# 默认使用 C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ringbuffer_lib REQUIRED)

# Producer 节点
add_executable(producer_node src/producer_node.cpp)
ament_target_dependencies(producer_node rclcpp ringbuffer_lib)

# Consumer 节点
add_executable(consumer_node src/consumer_node.cpp)
ament_target_dependencies(consumer_node rclcpp ringbuffer_lib)

# 组合运行的 main.cpp
add_executable(main_exec src/main.cpp)
ament_target_dependencies(main_exec rclcpp ringbuffer_lib)

# 关键：为 main_exec 添加头文件搜索路径
target_include_directories(main_exec PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 安装可执行文件
install(TARGETS
  producer_node
  consumer_node
  main_exec
  DESTINATION lib/${PROJECT_NAME}
)

# 安装头文件
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_include_directories(include)

ament_package()
