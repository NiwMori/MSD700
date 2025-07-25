#!/bin/bash
export ROS_DOMAIN_ID=70
source /opt/ros/foxy/setup.bash
source /home/msd700/ros2_ws/install/setup.bash
# start launch file
ros2 launch msd_remote remote_launch.py
