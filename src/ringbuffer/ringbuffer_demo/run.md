cd ~/ros2_ws
colcon build --packages-select ringbuffer_demo
source install/setup.bash
ros2 run ringbuffer_demo ringbuffer_demo
ros2 topic pub /chatter std_msgs/msg/String "data: 'Hello from ROS2'"
