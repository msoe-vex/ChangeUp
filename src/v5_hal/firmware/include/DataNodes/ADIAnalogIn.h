#pragma once

#include "api.h"
#include "NodeManager.h"
#include "ros_lib/ros.h"
#include "ros_lib/v5_hal/ADIAnalogIn.h"

class ADIAnalogIn : public Node {
private:
    pros::ADIAnalogIn* m_analog_in;
    v5_hal::ADIAnalogIn* m_analog_in_msg;
    ros::NodeHandle* m_handle;
    ros::Publisher* m_publisher;
    std::string m_handle_name;

    void populateAnalogInMsg();

public:
    ADIAnalogIn(NodeManager* nodeManager, int port, std::string handleName);

    void initialize();

    void periodic();

    ~ADIAnalogIn();
};
