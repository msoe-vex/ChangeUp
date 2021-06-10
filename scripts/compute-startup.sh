#!/bin/bash
# Run 'crontab -e' and add this script with a @reboot tag to run on startup. Make sure the
# crontab file ends with an empty line
sleep 30
source /opt/ros/noetic/setup.bash
source /home/ubuntu/ChangeUp/devel/setup.bash
roslaunch /home/ubuntu/ChangeUp/src/change_up_driver_control/launch/15_inch_robot.launch

