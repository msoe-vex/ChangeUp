#pragma once

#include "NodeManager.h"
#include "api.h"
#include "ros_lib/ros.h"
#include "ros_lib/std_msgs/Bool.h"

class ADIDigitalInNode : public Node {
private:
    pros::ADIDigitalIn m_digital_in;
    std_msgs::Bool m_digital_in_msg;
    std::string m_handle_name;
    ros::Publisher* m_publisher;

    void m_populateMessage();

public:
    ADIDigitalInNode(NodeManager* node_manager, int port,
        std::string* handle_name);

    void initialize();

    void periodic();

    ~ADIDigitalInNode();
};
