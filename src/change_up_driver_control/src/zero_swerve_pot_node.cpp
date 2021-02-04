#include <ros/console.h>
#include <ros/ros.h>

#include <sstream>
#include <math.h>

#include "std_msgs/Bool.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float32.h"

std_msgs::Float32 pot_msg;
ros::NodeHandle* handle;

int offset;
int pot_value;

/**
 * Callback function that runs when a new potentiometer message is received
 * Updates and publishes the new potentiometer value with the offset
 * Parameters:
 * 		msg: Message containing the current potentiometer value
 */
void potentiometerCallback(const std_msgs::Int16& msg) {
    pot_value = msg.data;
    float adjusted_pot_value = pot_value - offset;
    pot_msg.data = (adjusted_pot_value / 4096.0) * (2 * M_PI);
}

/**
 * Callback function sets the potentiometer offset value to the current value of the potentiometer
 * Zeros the offset value.
 * Parameters:
 * 		msg: Message bool - if true zero the offset
 */
void zeroPotCallback(const std_msgs::Bool& msg) {
    if (msg.data) {
        handle->setParam("pot_offset", pot_value);
        offset = pot_value;
    }
}

/**
 * Main function to execute the ROS code for creating and publishing messages.
 * Populating messages takes place in the callback function(s).
 * Parameters:
 * 		argc: Number of arguments to the program
 * 		argv: Array of arguments to the program
 */
int main(int argc, char** argv) {
    ros::init(argc, argv, "zeroSwervePot");

    handle = new ros::NodeHandle();

    // Grab offest value from memory
    handle->param("pot_offset", offset, 0);

    // Create subscribers and link to callbacks
    ros::Subscriber motor_pot_sub = handle->subscribe("inputPot", 100, potentiometerCallback);
    ros::Subscriber zero_pot_sub = handle->subscribe("zeroPot", 100, zeroPotCallback);

    // Create publishers
    ros::Publisher motor_pot_pub = handle->advertise<std_msgs::Float32>("/outputPot", 100);

    ros::Rate loop_rate(100);

	while (ros::ok())
	{
		motor_pot_pub.publish(pot_msg);

		ros::spinOnce();

		loop_rate.sleep();
	}

    return 0;
}