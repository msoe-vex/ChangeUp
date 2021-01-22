#include <ros/ros.h>
#include <ros/console.h>
#include "std_msgs/Int8.h"
#include "v5_hal/V5Controller.h"

#include <sstream>

std_msgs::Int8 msg1;

void primaryJoystickCallback(const v5_hal::V5Controller::ConstPtr& joystick ) {
    ROS_INFO("%i", msg->btn_right);
    msg1.data = msg->analog_left_y; 
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");

  ros::NodeHandle nh;

  ros::Subscriber driverJoystick = nh.subscribe("/controller/primary", 100, primaryJoystickCallback);

   ros::Publisher chatter_pub = nh.advertise<std_msgs::Int8>("/motor/leftSwerve1/moveMotorVoltage", 100);


  ros::Rate loop_rate(100);  

  int count = 0;
  while (ros::ok())
  {


    chatter_pub.publish(msg1);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return 0;
}