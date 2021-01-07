#pragma once

#include "api.h"
#include "NodeManager.h"
#include "ros_lib/ros.h"
#include "ros_lib/v5_hal/ADIUltrasonicData.h"

class ADIUltrasonicNode : public Node {
private:
    pros::ADIUltrasonic* m_ultrasonic;
    v5_hal::ADIUltrasonicData* m_ultrasonic_msg;
    ros::NodeHandle* m_handle;
    ros::Publisher* m_publisher;
    std::string m_handle_name;

    void populateUltrasonicMsg();

public:
    ADIUltrasonicNode(NodeManager* nodeManager,  int port_ping, int port_echo, std::string handleName);

    void initialize();

    void periodic();

    ~ADIUltrasonicNode();
};
