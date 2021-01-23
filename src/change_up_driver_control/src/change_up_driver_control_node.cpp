#include <ros/ros.h>
#include <ros/console.h>
#include "std_msgs/Int8.h"
#include "v5_hal/V5Controller.h"

#include <sstream>

std_msgs::Int8 msg1;

void primaryJoystickCallback(const v5_hal::V5Controller::ConstPtr& joystick) {
    msg1.data = msg->analog_left_y; 
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "controllers/changeUpDriverControl");

  ros::NodeHandle handle;

  ros::Subscriber primary_joystick_sub = handle.subscribe("/controller/primary", 100, primaryJoystickCallback);

  ros::Publisher left_intake_pub = handle.advertise<std_msgs::Int8>("/motor/leftIntake/moveMotorVoltage", 100);
  ros::Publisher right_intake_pub = handle.advertise<std_msgs::Int8>("/motor/rightIntake/moveMotorVoltage", 100);
  
  ros::Publisher bottom_roller_pub = handle.advertise<std_msgs::Int8>("/motor/bottomRollers/moveMotorVoltage", 100);
  ros::Publisher top_rollers_pub = handle.advertise<std_msgs::Int8>("/motor/topRollers/moveMotorVoltage", 100);


  ros::Rate loop_rate(50);  

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