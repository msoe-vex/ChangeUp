#pragma once

#include "api.h"
#include "NodeManager.h"
#include "ros_lib/ros.h"
#include "ros_lib/v5_hal/ProsTime.h"

class ProsTimeNode : public Node {
private:
    ros::NodeHandle* m_handle;
    ros::Publisher* m_publisher;
    std::string m_handle_name;
    v5_hal::ProsTime* m_prostime_msg;

public:
    ProsTimeNode(NodeManager* nodeHandle, std::string handleName);

    void initialize();
    
    void periodic();

    void populateProsTimeMsg();

    ~ProsTimeNode();
};