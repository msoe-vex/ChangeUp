#pragma once

#include "NodeManager.h"
#include "api.h"
#include "ros_lib/ros.h"
#include "ros_lib/std_msgs/Int16.h"

class ADIUltrasonicNode : public Node {
private:
    pros::ADIUltrasonic m_ultrasonic;
    std_msgs::Int16 m_ultrasonic_msg;
    std::string m_handle_name;
    ros::Publisher* m_publisher;

    void m_populateMessage();

public:
    ADIUltrasonicNode(NodeManager* node_manager, int port_ping, int port_echo,
        std::string* handle_name);

    void initialize();

    void periodic();

    ~ADIUltrasonicNode();
};
