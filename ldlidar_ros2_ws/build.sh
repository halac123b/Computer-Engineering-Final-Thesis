#!/bin/bash
colcon build
source install/setup.bash
echo source ~/ldlidar_ros2_ws/install/setup.bash >> ~/.bashrc
source ~/.bashrc
ros2 launch ldlidar_stl_ros2 viewer_ld19.launch.py
