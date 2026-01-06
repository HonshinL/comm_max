cd ~/ros2_ws
colcon build --packages-select ringbuffer_demo
source install/setup.bash
ros2 run prod_and_coms producer_node
ros2 run prod_and_coms consumer_node
ros2 run prod_and_coms main_exec
ros2 topic pub /chatter std_msgs/msg/String "data: 'Hello from ROS2'"

ros2 launch prod_and_coms main_exec.launch.py
