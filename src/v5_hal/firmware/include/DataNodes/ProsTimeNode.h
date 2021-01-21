#pragma once

#include "NodeManager.h"
#include "api.h"
#include "ros_lib/ros.h"
#include "ros_lib/v5_hal/ProsTime.h"

class ProsTimeNode : public Node {
private:
    ros::Publisher m_publisher;
    v5_hal::ProsTime m_prostime_msg;
    std::strin*g m_handle_name;

    void populateyMessage();
    
public:
    ProsTimeNode(NodeManager* nodeHandle, std::string handleName);

    void initialize();
    
    void periodic();

    void populateProsTimeMsg();

    ~ProsTimeNode();
};