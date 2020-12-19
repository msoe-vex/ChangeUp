#pragma once

#include "api.h"
#include "NodeManager.h"
#include "ros_lib/ros.h"
#include "ros_lib/v5_hal/ADIDigitalIn.h"

class ADIDigitalIn : public Node {
private:
    pros::ADIDigitalIn* m_digital_in;
    v5_hal::ADIDigitalIn* m_digital_in_msg;
    ros::NodeHandle* m_handle;
    ros::Publisher* m_publisher;
    std::string m_handle_name;

    void populateDigitalInMsg();

public:
    ADIDigitalIn(NodeManager* nodeManager, int port, std::string handleName);

    void initialize();

    void periodic();

    ~ADIDigitalIn();
};
