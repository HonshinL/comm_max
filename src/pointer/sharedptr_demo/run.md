cd ~/ros2_ws
colcon build --packages-select sharedptr_demo
source install/setup.bash
ros2 run sharedptr_demo sharedptr_demo
ros2 topic pub /chatter std_msgs/msg/String "data: 'Hello from ROS2!'"
