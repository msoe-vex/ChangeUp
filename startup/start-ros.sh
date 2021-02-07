#!/bin/bash
echo "Starting ROS..."
cd /home/ubuntu/ChangeUp
source ./devel/setup.bash
roslaunch change_up_driver_control 15_inch_robot.launch
