#pragma once

#include "NodeManager.h"
#include "api.h"
#include "ros_lib/ros.h"
#include "ros_lib/std_msgs/Int32.h"

class ADIEncoderNode : public Node {
private:
    pros::ADIEncoder m_encoder;
    std_msgs::Int32 m_encoder_msg;
    std::string m_handle_name;
    ros::Publisher* m_publisher;

    void m_populateMessage();

public:
    ADIEncoderNode(NodeManager* node_manager, int port_top, int port_bottom,
        std::string handle_name, bool reverse=false);

    void initialize();

    int getValue();

    void periodic();

    ~ADIEncoderNode();
};
