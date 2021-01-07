#pragma once

#include "api.h"
#include "NodeManager.h"
#include "ros_lib/ros.h"
#include "ros_lib/v5_hal/ADIAnalogInData.h"

class ADIAnalogInNode : public Node {
private:
    pros::ADIAnalogIn* m_analog_in;
    v5_hal::ADIAnalogInData* m_analog_in_msg;
    ros::NodeHandle* m_handle;
    ros::Publisher* m_publisher;
    std::string m_handle_name;

    void populateAnalogInMsg();

public:
    ADIAnalogInNode(NodeManager* nodeManager, int port, std::string handleName);

    void initialize();

    void periodic();

    ~ADIAnalogInNode();
};
