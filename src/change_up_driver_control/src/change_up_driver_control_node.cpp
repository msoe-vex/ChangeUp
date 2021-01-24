#include <ros/ros.h>
#include <ros/console.h>
#include "std_msgs/Int8.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Vector3.h"
#include "v5_hal/V5Controller.h"

#include <sstream>

std_msgs::Int8 left_intake_msg;
std_msgs::Int8 right_intake_msg;
std_msgs::Int8 bottom_rollers_msg;
std_msgs::Int8 top_rollers_msg;
geometry_msgs::Vector3 robot_target_velocity_msg;
std_msgs::Float32 robot_target_rotation_velocity_msg;

/**
 * Spins the front intakes of the robot at the specified voltage
 * Parameters:
 * 		voltage: The voltage, from -127 to 127
 */
void spinIntakesVoltage(int voltage) {
	left_intake_msg.data = voltage;
	right_intake_msg.data = voltage;
}

/**
 * Spins the bottom set of rollers (from a single motor) at the specified voltage
 * Parameters:
 * 		voltage: The voltage, from -127 to 127
 */
void spinBottomRollersVoltage(int voltage) {
	bottom_rollers_msg.data = voltage;
}

/**
 * Spins the top set of rollers (from a single motor) at the specified voltage
 * Parameters:
 * 		voltage: The voltage, from -127 to 127
 */
void spinTopRollersVoltage(int voltage) {
	top_rollers_msg.data = voltage;
}

/**
 * Callback function that runs when a new joystick message is received
 * Parameters:
 * 		joystick: Message containing data from the joystick
 */
void primaryJoystickCallback(const v5_hal::V5Controller& msg) {
	robot_target_velocity_msg.x = msg.analog_left_x;
	robot_target_velocity_msg.y = msg.analog_left_y;

	robot_target_rotation_velocity_msg.data = msg.analog_right_x;

	int intake_voltage = 0;
	int bottom_rollers_voltage = 0;
	int top_rollers_voltage = 0;

	if (msg.btn_r1) {
		intake_voltage = 127;
		bottom_rollers_voltage = 127;
		top_rollers_voltage = 127;

		if (msg.btn_l1) {
			top_rollers_voltage *= -1;
		}
	}
	else if (msg.btn_r2) {
		intake_voltage = -127;
		bottom_rollers_voltage = -127;
		top_rollers_voltage = -127;
	}

	spinIntakesVoltage(intake_voltage);
	spinBottomRollersVoltage(bottom_rollers_voltage);
	spinTopRollersVoltage(top_rollers_voltage);
}

/**
 * Main function to execute the ROS code for creating and publishing messages.
 * Populating messages takes place in the callback function(s).
 * Parameters:
 * 		argc: Number of arguments to the program
 * 		argv: Array of arguments to the program
 */
int main(int argc, char** argv) {
	ros::init(argc, argv, "changeUpDriverControl");

	ros::NodeHandle handle;

	// Create subscribers and link to callbacks
	ros::Subscriber primary_joystick_sub = handle.subscribe("/controller/primary", 100, primaryJoystickCallback);

	// Create publishers
	ros::Publisher left_intake_pub = handle.advertise<std_msgs::Int8>("/motor/leftIntake/moveMotorVoltage", 100);
	ros::Publisher right_intake_pub = handle.advertise<std_msgs::Int8>("/motor/rightIntake/moveMotorVoltage", 100);
	
	ros::Publisher bottom_rollers_pub = handle.advertise<std_msgs::Int8>("/motor/bottomRollers/moveMotorVoltage", 100);
	ros::Publisher top_rollers_pub = handle.advertise<std_msgs::Int8>("/motor/topRollers/moveMotorVoltage", 100);

	ros::Publisher robot_target_velocity_pub = handle.advertise<geometry_msgs::Vector3>("swerveCommandJoystick", 100);
	ros::Publisher robot_target_rotation_velocity_pub = handle.advertise<std_msgs::Float32>("swerveCommandRotate", 100);

	ros::Rate loop_rate(50);

	while (ros::ok())
	{
		left_intake_pub.publish(left_intake_msg);
		right_intake_pub.publish(right_intake_msg);
		bottom_rollers_pub.publish(bottom_rollers_msg);
		top_rollers_pub.publish(top_rollers_msg);
		robot_target_velocity_pub.publish(robot_target_velocity_msg);
		robot_target_rotation_velocity_pub.publish(robot_target_rotation_velocity_msg);

		ros::spinOnce();

		loop_rate.sleep();
	}

	return 0;
}