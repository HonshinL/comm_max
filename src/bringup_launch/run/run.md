ros2 launch ringbuffer_launch main_exec.launch.py
ros2 launch ringbuffer_launch event_driven.launch.py


ros2 service call /produce_once std_srvs/srv/Trigger {}



ros2 launch bringup_launch roudi_with_nodes.launch.py
