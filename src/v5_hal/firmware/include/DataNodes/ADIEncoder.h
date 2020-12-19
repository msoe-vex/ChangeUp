#pragma once

#include "api.h"
#include "NodeManager.h"
#include "ros_lib/ros.h"
#include "ros_lib/v5_hal/ADIEncoder.h"

class ADIEncoder : public Node {
private:
    pros::ADIEncoder* m_encoder;
    v5_hal::ADIEncoder* m_encoder_msg;
    ros::NodeHandle* m_handle;
    ros::Publisher* m_publisher;
    std::string m_handle_name;

    void populateEncoderMsg();

public:
    ADIEncoder(NodeManager* nodeManager, int port_top, int port_bottom, bool reverse, std::string handleName);

    void initialize();

    void periodic();

    ~ADIEncoder();
};
