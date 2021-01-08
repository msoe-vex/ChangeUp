#pragma once

#include "api.h"
#include "NodeManager.h"
#include "ros_lib/ros.h"
#include "ros_lib/v5_hal/ADIDigitalInData.h"

class ADIDigitalInNode : public Node {
private:
    pros::ADIDigitalIn* m_digital_in;
    v5_hal::ADIDigitalInData* m_digital_in_msg;
    ros::NodeHandle* m_handle;
    ros::Publisher* m_publisher;
    std::string m_handle_name;

    void populateDigitalInMsg();

public:
    ADIDigitalInNode(NodeManager* nodeManager, int port, std::string handleName);

    void initialize();

    void periodic();

    ~ADIDigitalInNode();
};
