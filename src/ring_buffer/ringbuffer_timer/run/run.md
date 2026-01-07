cd ~/ros2_ws
colcon build --packages-select ringbuffer_timer
source install/setup.bash
ros2 run ringbuffer_timer producer_node
ros2 run ringbuffer_timer consumer_node
ros2 run ringbuffer_timer main_exec
ros2 topic pub /chatter std_msgs/msg/String "data: 'Hello from ROS2'"

ros2 launch ringbuffer_timer main_exec.launch.py
