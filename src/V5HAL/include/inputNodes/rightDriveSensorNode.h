#ifndef _RIGHT_DRIVE_SENSOR_NODE_H_
#define _RIGHT_DRIVE_SENSOR_NODE_H_

#include "api.h"
#include "constants.h"
#include "ros_lib/ros.h"
#include "ros_lib/std_msgs/Int16.h"
#include "ros_lib/V5Publisher.h"

class RightDriveSensorNode : public V5Publisher<std_msgs::Int16> {
private:
    ros::NodeHandle* _handle;
    std_msgs::Int16* _encoder_pos;
    std_msgs::Int16* _encoder_vel;
public:
    RightDriveSensorNode();

    void setup();

    void publish();

    ~RightDriveSensorNode();
};

#endif