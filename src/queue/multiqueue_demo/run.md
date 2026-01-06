cd ~/ros2_ws
colcon build --packages-select multiqueue_demo
source install/setup.bash
ros2 run multiqueue_demo multiqueue_demo
ros2 topic pub /chatter std_msgs/msg/String "data: 'Hello from ROS2'"
