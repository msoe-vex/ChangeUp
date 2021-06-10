#!/bin/bash
# Run 'crontab -e' and add this script with a @reboot tag to run on startup
source /opt/ros/noetic/setup.sh
source /home/ubuntu/ChangeUp/devel/setup.bash
roslaunch /home/ubuntu/ChangeUp/src/change_up_driver_control/launch/15_inch_robot.launch
