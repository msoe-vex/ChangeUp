#include <ros/console.h>
#include <ros/ros.h>

#include <sstream>

#include "std_msgs/Bool.h"
#include "std_msgs/Int16.h"

std_msgs::Int16 pot_msg;
ros::NodeHandle handle;
ros::Publisher motor_pot_pub;

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
    pot_msg.data = pot_value - offset;
    motor_pot_pub.publish(pot_msg);
}

/**
 * Callback function sets the potentiometer offset value to the current value of the potentiometer
 * Zeros the offset value.
 * Parameters:
 * 		msg: Message bool - if true zero the offset
 */
void zeroPotCallback(const std_msgs::Bool& msg) {
    if (msg.data) {
        handle.setParam("pot_offset", pot_value);
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

    // Grab offest value from memory
    handle.getParam("pot_offset", offset);

    // Create subscribers and link to callbacks
    ros::Subscriber motor_pot_sub = handle.subscribe("input_pot", 100, potentiometerCallback);
    ros::Subscriber zero_pot_sub = handle.subscribe("zero_pot", 100, zeroPotCallback);

    // Create publishers
    motor_pot_pub = handle.advertise<std_msgs::Int16>("output_pot", 100);

    ros::spin();

    return 0;
}