cd ~/ros2_ws
colcon build --packages-select sub_and_snapshot
source install/setup.bash
ros2 run sub_and_snapshot sub_and_snapshot
ros2 topic pub /chatter std_msgs/msg/String "data: 'Hello from ROS2'"

ros2 launch sub_and_snapshot snapshot.launch.py
